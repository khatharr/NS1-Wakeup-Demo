#include "esp_sdp_api.h"
#include "swpro_link.h"

#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd.h"
#include "esp_hidd_api.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "link_state.h"
#include "swpro_hid.h"
#include "esp_sdp_api.h"
#include "swpro_link.h"
#include "swpro_pairing.h"
#include "baseband.h"
#include "wakeup_code.h"
#include "wake_cycle.h"
#include "wake_session.h"
#include "ns2_wake.h"

#define TAG "swpro"

#define RECONNECT_INTERVAL_US UINT64_C(200000)
// Wait 800 ms for the Switch to release the previous ACL/L2CAP session and become ready for HID initialization.
#define WAKE_PAGE_GAP_US INT64_C(800000)
#define STATUS_INTERVAL_US UINT64_C(5000000)
#define AUTO_A_DELAY_US INT64_C(2000000)
#define AUTO_A_PRESS_US INT64_C(100000)
#define AUTO_A_RELEASE_US INT64_C(100000)
// Bound the duration of a wake boot if Bluetooth events never arrive.
#define WAKE_BOOT_TIMEOUT_US INT64_C(30000000)

typedef enum {
    AUTO_A_IDLE,
    AUTO_A_WAIT,
    AUTO_A_PRESSED,
    AUTO_A_RELEASED,
    AUTO_A_DONE,
} auto_a_state_t;

static volatile bool s_hid_started;
static volatile uint32_t s_hid_connection_generation;
volatile bool g_hid_connected;
volatile bool g_acl_connected;
volatile bool g_pairing;
static uint32_t s_reconnect_attempts;
int64_t g_next_reconnect_us;
// Clock_Offset passed to HCI Create Connection; bit 15 is the valid flag.
uint16_t g_hci_clock_offset;
// Difference between native clocks on the Switch and ESP32. Direct ACL registration passes a 27-bit value to the control structure.
uint32_t g_direct_clock_delta;
uint32_t g_direct_rx_clk;
uint16_t g_direct_rx_bitoff;
uint8_t g_direct_lt_addr;
volatile bool g_direct_clock_delta_valid;
volatile bool g_direct_sync_valid;
volatile bool g_direct_handoff_started;
// When CMD_WAKE_START payload[27] is 1, skip ACL registration and start normal paging 20 ms after Phase 3 transmission.
volatile bool g_direct_handoff_disabled;
volatile bool g_direct_handoff_active;
volatile esp_err_t g_direct_handoff_start_err;
volatile int64_t g_direct_handoff_deadline_us;
volatile int64_t g_direct_handoff_quiet_until_us;
volatile bool g_reconnect_paused;
// Prevent normal paging before CSB transmission because ongoing paging causes CSB enable to fail with status 0x0b.
volatile bool g_reconnect_disabled;

// A zero wait time disconnects ACL links with no dynamic channels.
extern uint8_t L2CA_SetIdleTimeoutByBdAddr(uint8_t *bd_addr, uint16_t timeout,
                                           uint8_t transport);
#define BT_TRANSPORT_BR_EDR_LOCAL 1

// The supervision timeout is measured in 0.625 ms slots.
extern void BTM_SetDefaultLinkSuperTout(uint16_t timeout);

// Send HCI Disconnect whether or not dynamic channels are present.
extern uint8_t btm_remove_acl(uint8_t *bd_addr, uint8_t transport);

extern uint8_t BTM_GetRole(uint8_t *bd_addr, uint8_t *role);

volatile bool g_cycle_enabled;

volatile bool g_wake_page_pending;
volatile int64_t g_wake_page_deadline_us;
volatile uint8_t g_wake_disconnect_tries;
#define WAKE_DISCONNECT_TRIES 2
#define WAKE_DISCONNECT_RETRY_US INT64_C(200000)
#define WAKE_DISCONNECT_FIRST_US INT64_C(300000)
volatile bool g_wake_hid_connect_pending;
volatile bool g_wake_stop_requested;


int64_t g_connect_deadline_us;
uint32_t g_connect_failures;
#define CONNECT_FAILURE_LIMIT 20

// Bluedroid power management (bta_dm_pm) repeatedly requests sniff and unsniff for HID connections without application direction, colliding with Switch-side link control and LMP.
// Disable all ESP32-initiated power-mode changes via the linker's --wrap and follow Switch-initiated link control. Returning BTM_SUCCESS prevents the caller from retrying.
static volatile uint32_t s_pm_suppressed;

extern uint8_t __real_BTM_SetDeviceClass(uint8_t dev_class[3]);

uint8_t __wrap_BTM_SetDeviceClass(uint8_t dev_class[3])
{
    uint8_t cod[3];
    memcpy(cod, dev_class, sizeof(cod));
    // Preserve CoD 0x002508, which Switch 2 recognizes, even when switching to General Discoverable.
    cod[1] |= 0x20;
    return __real_BTM_SetDeviceClass(cod);
}

uint8_t __wrap_BTM_SetPowerMode(uint8_t pm_id, uint8_t *remote_bda, void *p_mode)
{
    (void)pm_id;
    (void)remote_bda;
    (void)p_mode;
    s_pm_suppressed++;
    return 0; // BTM_SUCCESS
}


// Stop automatic connections only once to avoid repeated reconnects and disconnects immediately after sleep.
volatile bool g_stay_disconnected;

bool g_wake_local_release_tried;

int64_t g_idle_restart_due_us;


// Restarting with an active ACL link does not notify the Switch of disconnection, so the Switch retains the link until its supervision timeout.
// A connection cycle would repeatedly restart while leaving the ACL link active; always finish disconnecting ACL before restarting.
static void disconnect_before_restart(void)
{
    esp_bd_addr_t host;
    if (g_acl_connected && swpro_host_snapshot(host)) {
        ESP_LOGI(TAG, "Requesting ACL disconnect before reboot");
        btm_remove_acl(host, BT_TRANSPORT_BR_EDR_LOCAL);
        const int64_t until = esp_timer_get_time() + 500000;
        while (g_acl_connected && esp_timer_get_time() < until) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (g_acl_connected) {
            ESP_LOGW(TAG, "Rebooting without confirming ACL disconnect completion");
        }
    }
}

void swpro_restart_after_disconnect(uint32_t restart_marker)
{
    disconnect_before_restart();
    g_restart_marker = restart_marker;
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_restart();
}

void swpro_finish_wake(wake_result_t result)
{
    // Prevent another connection while finishing the current operation.
    g_wake_stop_requested = true;
    g_cycle_enabled = false;
    swpro_stop_auto_connect("Wake operation finished");
    disconnect_before_restart();
    vTaskDelay(pdMS_TO_TICKS(20));
    boot_return_wifi(result);
}

void swpro_stop_auto_connect(const char *reason)
{
    const bool first_request = !g_stay_disconnected;
    const bool hid_active = g_hid_connected || g_connect_deadline_us != 0;
    g_stay_disconnected = true;
    if (first_request) {
        ESP_LOGI(TAG, "%s: automatic page start and connectable mode stopped; press HOME to start connection attempts",
                 reason);
    }
    g_reconnect_disabled = true;
    g_connect_deadline_us = 0;
    // Cancel scheduled paging. Simply setting g_reconnect_disabled still allows one paging request already authorized by switch1_hci_clock_offset_set().
    switch1_hci_page_cancel();
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    if (hid_active) {
        const esp_err_t err = esp_bt_hid_device_disconnect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "HID disconnect request failed: %s", esp_err_to_name(err));
        }
    }
    // Also send HCI Disconnect to guarantee ACL disconnection even when an HID disconnect request is in progress.
    if (g_acl_connected) {
        esp_bd_addr_t host;
        if (swpro_host_snapshot(host)) {
            btm_remove_acl(host, BT_TRANSPORT_BR_EDR_LOCAL);
        }
    }
}

void wake_handoff_finished(void)
{
    g_direct_handoff_active = false;
    g_direct_handoff_deadline_us = 0;
    if (!g_wake_tx_active) {
        g_reconnect_paused = false;
        g_next_reconnect_us = 0;
    }
}

void swpro_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Authentication succeeded: %s", param->auth_cmpl.device_name);
            swpro_host_store(param->auth_cmpl.bda);
            esp_bt_hid_device_connect(param->auth_cmpl.bda);
        } else {
            ESP_LOGW(TAG, "Authentication failed: status=%d", param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_CFM_REQ_EVT:
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;

    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT: {
        uint8_t acl_role = 0xff;
        BTM_GetRole(param->acl_conn_cmpl_stat.bda, &acl_role);
        ESP_LOGI(TAG,
                 "ACL connection complete: status=0x%x role=%s(%u) peer=" ESP_BD_ADDR_STR
                 " auto_page_stopped=%u in_progress=%u",
                 param->acl_conn_cmpl_stat.stat,
                 acl_role == 0   ? "central/ESP32 paging"
                 : acl_role == 1 ? "peripheral/Switch paging"
                                 : "unknown",
                 acl_role, ESP_BD_ADDR_HEX(param->acl_conn_cmpl_stat.bda),
                 g_reconnect_disabled ? 1U : 0U, g_cycle_enabled ? 1U : 0U);
        if (param->acl_conn_cmpl_stat.stat == ESP_BT_STATUS_SUCCESS) {
            g_acl_connected = true;
            // Stop CSB transmission after connection so it cannot interfere with ACL traffic.
            if (g_wake_tx_active) {
                g_wake_stop_requested = true;
            }
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE,
                                     ESP_BT_NON_DISCOVERABLE);
        } else {
            g_acl_connected = false;
            esp_bt_gap_set_scan_mode(
                ESP_BT_CONNECTABLE,
                (g_pairing || !swpro_host_exists()) ? ESP_BT_GENERAL_DISCOVERABLE
                                               : ESP_BT_NON_DISCOVERABLE);
            g_connect_deadline_us = 0;
            g_next_reconnect_us = esp_timer_get_time() + 300000;
        }
        if (g_direct_handoff_active) {
            esp_bd_addr_t host;
            if (swpro_host_snapshot(host) &&
                memcmp(host, param->acl_conn_cmpl_stat.bda, sizeof(host)) == 0) {
                if (param->acl_conn_cmpl_stat.stat == ESP_BT_STATUS_SUCCESS) {
                    g_wake_hid_connect_pending = true;
                }
                wake_handoff_finished();
            }
        }
        break;
    }

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        ESP_LOGI(TAG, "ACL disconnection complete: reason=0x%x", param->acl_disconn_cmpl_stat.reason);
        g_acl_connected = false;
        // Stop sending input reports if the ACL link disconnects before an HID disconnect notification.
        if (g_hid_connected) {
            esp_bd_addr_t host;
            if (swpro_host_snapshot(host) &&
                memcmp(host, param->acl_disconn_cmpl_stat.bda,
                       sizeof(host)) == 0) {
                ESP_LOGI(TAG, "Detected Switch-side ACL disconnection; clearing HID connection state");
                g_hid_connected = false;
                g_connect_deadline_us = 0;
                swpro_hid_set_report_interval_us(SWPRO_REPORT_INTERVAL_DEFAULT_US);
                esp_bt_hid_device_disconnect();
                if (!g_cycle_enabled) {
                    swpro_stop_auto_connect("ACL disconnection");
                }
            }
        }
        if (!g_reconnect_disabled) {
            esp_bt_gap_set_scan_mode(
                ESP_BT_CONNECTABLE,
                (g_pairing || !swpro_host_exists()) ? ESP_BT_GENERAL_DISCOVERABLE
                                               : ESP_BT_NON_DISCOVERABLE);
        }
        if (g_direct_handoff_active) {
            esp_bd_addr_t host;
            if (swpro_host_snapshot(host) &&
                memcmp(host, param->acl_disconn_cmpl_stat.bda, sizeof(host)) == 0) {
                wake_handoff_finished();
            }
        }
        break;

    case ESP_BT_GAP_MODE_CHG_EVT:
        // In sniff mode, match the report interval to the sniff interval to avoid overflowing the transmit queue.
        if (param->mode_chg.mode == ESP_BT_PM_MD_SNIFF) {
            uint32_t interval_us = (uint32_t)param->mode_chg.interval * 625;
            if (interval_us < SWPRO_REPORT_INTERVAL_DEFAULT_US) {
                interval_us = SWPRO_REPORT_INTERVAL_DEFAULT_US;
            }
            swpro_hid_set_report_interval_us(interval_us);
        } else {
            swpro_hid_set_report_interval_us(SWPRO_REPORT_INTERVAL_DEFAULT_US);
        }
        ESP_LOGI(TAG, "Power mode changed: mode=%d interval=%u", param->mode_chg.mode,
                 param->mode_chg.interval);
        break;

    default:
        break;
    }
}

#if CONFIG_BT_SDP_COMMON_ENABLED
void swpro_sdp_callback(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param)
{
    switch (event) {
    case ESP_SDP_INIT_EVT:
        if (param->init.status == ESP_SDP_SUCCESS) {
            esp_bluetooth_sdp_dip_record_t dip_record = {
                .hdr = {
                    .type = ESP_SDP_TYPE_DIP_SERVER,
                },
                .vendor = 0x057E,
                .vendor_id_source = 0x0002, // USB Implementers Forum
                .product = 0x2009,
                .version = 0x0100,
                .primary_record = true,
            };
            esp_sdp_create_record((esp_bluetooth_sdp_record_t *)&dip_record);
        } else {
            ESP_LOGE(TAG, "SDP initialization failed: %d", param->init.status);
        }
        break;
    case ESP_SDP_CREATE_RECORD_COMP_EVT:
        ESP_LOGI(TAG, "Device ID record registered: status=%d", param->create_record.status);
        break;
    default:
        break;
    }
}
#endif

void swpro_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id,
                                void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDD_START_EVENT:
        if (param->start.status != ESP_OK) {
            ESP_LOGE(TAG, "HID device start failed: %d", param->start.status);
            break;
        }
        s_hid_started = true;
        if (g_cycle_resume_after_boot) {
            connect_cycle_set(true);
            g_cycle_resume_after_boot = false;
            // Do not start normal paging until the wake-up command is sent; paging started at boot would prevent CSB from enabling and transmitting.
            g_reconnect_disabled = true;
            g_cycle_next_us = esp_timer_get_time() + 200000;
        }
        if (g_pairing || !swpro_host_exists()) {
            ESP_LOGI(TAG, "Accepting pairing requests");
            {
                const esp_err_t serr = esp_bt_gap_set_scan_mode(
                    ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                ESP_LOGI(TAG, "  Set discoverable/connectable mode: %s", esp_err_to_name(serr));
            }
        } else {
            ESP_LOGI(TAG, "Waiting to reconnect to saved Switch");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            if (g_page_right_after_boot) {
                ESP_LOGI(TAG, "Booted immediately after wake-up; starting normal page");
                g_next_reconnect_us = esp_timer_get_time() + 50000;
            } else {
                // The esp_hid wrapper attempts an automatic connection using Virtual Cable information; defer manual reconnection by one cycle to avoid a collision.
                g_next_reconnect_us =
                    esp_timer_get_time() + RECONNECT_INTERVAL_US;
            }
        }
        break;

    case ESP_HIDD_CONNECT_EVENT:
        g_connect_deadline_us = 0;
        if (param->connect.status == ESP_OK && g_stay_disconnected) {
            ESP_LOGI(TAG, "HID connection from Switch while automatic connection is stopped; keeping connection");
            g_stay_disconnected = false;
        }
        if (param->connect.status == ESP_OK) {
            swpro_hid_reset_session();
            swpro_hid_set_report_interval_us(SWPRO_REPORT_INTERVAL_DEFAULT_US);
            s_reconnect_attempts = 0;
            g_pairing = false;
            g_hid_connected = true;
            g_connect_failures = 0;
            s_hid_connection_generation++;
            ESP_LOGI(TAG, "HID OPEN");
        } else {
            g_connect_failures++;
            ESP_LOGW(TAG, "HID connection failed: %d (consecutive=%lu)",
                     param->connect.status, (unsigned long)g_connect_failures);
        }
        if (g_direct_handoff_active) {
            wake_handoff_finished();
        }
        break;

    case ESP_HIDD_OUTPUT_EVENT:
        swpro_hid_handle_output(param->output.report_id, param->output.data,
                                param->output.length);
        break;

    case ESP_HIDD_FEATURE_EVENT:
        ESP_LOGI(TAG, "FEATURE: trans=%d type=%d id=0x%02x len=%u",
                 param->feature.trans_type, param->feature.report_type,
                 param->feature.report_id, param->feature.length);
        if (param->feature.trans_type == ESP_HID_TRANS_SET_REPORT &&
            param->feature.report_type == ESP_HID_REPORT_TYPE_OUTPUT) {
            esp_bt_hid_device_report_error(ESP_HID_PAR_HANDSHAKE_RSP_SUCCESS);
            swpro_hid_handle_output(param->feature.report_id,
                                    param->feature.data,
                                    param->feature.length);
        }
        break;

    case ESP_HIDD_PROTOCOL_MODE_EVENT:
        ESP_LOGI(TAG, "Protocol mode: %s",
                 param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;

    case ESP_HIDD_DISCONNECT_EVENT:
        // Start ACL disconnection before logging to ensure disconnection begins before the Switch reconnects HID.
        if (g_acl_connected && !g_cycle_enabled && !g_pairing &&
            !g_stay_disconnected && swpro_host_exists()) {
            esp_bd_addr_t host;
            if (swpro_host_snapshot(host)) {
                btm_remove_acl(host, BT_TRANSPORT_BR_EDR_LOCAL);
            }
        }
        g_hid_connected = false;
        g_connect_deadline_us = 0;
        swpro_hid_set_report_interval_us(SWPRO_REPORT_INTERVAL_DEFAULT_US);
        ESP_LOGI(TAG, "HID disconnected: reason=%d", param->disconnect.reason);
        if (g_acl_connected) {
            esp_bd_addr_t host;
            if (swpro_host_snapshot(host) &&
                L2CA_SetIdleTimeoutByBdAddr(host, 0,
                                            BT_TRANSPORT_BR_EDR_LOCAL)) {
                ESP_LOGI(TAG, "Requesting disconnect of remaining ACL link");
            }
        }
        if (!g_cycle_enabled && !g_pairing && swpro_host_exists()) {
            swpro_stop_auto_connect("HID disconnection");
            if (switch1_csb_wake_events() != 0) {
                g_idle_restart_due_us =
                    esp_timer_get_time() + IDLE_RESTART_DELAY_US;
            }
        } else if (g_pairing || !swpro_host_exists()) {
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            g_next_reconnect_us =
                esp_timer_get_time() + (int64_t)RECONNECT_INTERVAL_US;
        }
        if (g_direct_handoff_active) {
            wake_handoff_finished();
        }
        break;

    case ESP_HIDD_STOP_EVENT:
        s_hid_started = false;
        ESP_LOGI(TAG, "HID device stopped");
        break;

    default:
        break;
    }
}

void try_reconnect(void)
{
    esp_bd_addr_t host;
    // Start an HID connection request even on an ACL link established by the wake-up process.
    if (g_reconnect_disabled || !s_hid_started || g_pairing || g_hid_connected ||
        !swpro_host_snapshot(host)) {
        return;
    }

    if (g_connect_failures >= CONNECT_FAILURE_LIMIT) {
        ESP_LOGE(TAG, "HID connection failed %u times; returning to Wi-Fi",
                 (unsigned)g_connect_failures);
        swpro_finish_wake(WAKE_ERR_HID);
    }

    const int64_t now = esp_timer_get_time();
    if (g_connect_deadline_us != 0) {
        if (now < g_connect_deadline_us) {
            return;
        }
        ESP_LOGW(TAG, "HID connection request timed out; disconnecting HID and clearing state");
        esp_bt_hid_device_disconnect();
        g_connect_deadline_us = 0;
        return;
    }

    s_reconnect_attempts++;
    g_connect_deadline_us = now + (int64_t)CONNECT_TIMEOUT_US;
    switch1_hci_clock_offset_set(host, g_hci_clock_offset);
    const esp_err_t err = esp_bt_hid_device_connect(host);
    if (err != ESP_OK) {
        switch1_hci_page_cancel();
        g_connect_deadline_us = 0;
        ESP_LOGW(TAG, "Reconnect request failed: %s", esp_err_to_name(err));
    }
}

static void log_status(void)
{
    {
        const int bonds = esp_bt_gap_get_bond_device_num();
        ESP_LOGI(TAG, "  Stored pairing records: %d", bonds);
        if (bonds > 0) {
            esp_bd_addr_t list[4];
            int listed = bonds < 4 ? bonds : 4;
            if (esp_bt_gap_get_bond_device_list(&listed, list) == ESP_OK) {
                for (int i = 0; i < listed; ++i) {
                    ESP_LOGI(TAG, "    Peer " ESP_BD_ADDR_STR,
                             ESP_BD_ADDR_HEX(list[i]));
                }
            }
        }
    }
    esp_bd_addr_t host;
    if (!swpro_host_snapshot(host)) {
        ESP_LOGI(TAG,
                  "status: connected=%u pairing=%u host=none frames=%lu retries=%lu",
                  g_hid_connected, g_pairing,
                  (unsigned long)swpro_hid_input_frames(), (unsigned long)s_reconnect_attempts);
        return;
    }

    uint8_t wakeup_code[4];
    switch1_wakeup_code(host, wakeup_code);
    ESP_LOGI(TAG,
              "status: connected=%u pairing=%u host=" ESP_BD_ADDR_STR
              " wake-up code=%02X%02X%02X%02X wake_tx=%s csb_starts=%lu"
              " interval=%luus frames=%lu retries=%lu pm_sup=%lu",
              g_hid_connected, g_pairing, ESP_BD_ADDR_HEX(host),
              wakeup_code[0], wakeup_code[1], wakeup_code[2], wakeup_code[3],
              g_wake_tx_active ? "active" : "idle",
              (unsigned long)switch1_csb_wake_events(),
              (unsigned long)swpro_hid_report_interval_us(),
              (unsigned long)swpro_hid_input_frames(), (unsigned long)s_reconnect_attempts,
              (unsigned long)s_pm_suppressed);
}


static void report_bridge_status(void)
{
    // Log state lines at WARN level so the input bridge can read them during connection attempts.
    esp_bd_addr_t host;
    if (swpro_host_snapshot(host)) {
        ESP_LOGW(TAG, "#S c=%u p=%u y=%u f=%lu h=" ESP_BD_ADDR_STR,
                 g_hid_connected ? 1U : 0U, g_pairing ? 1U : 0U,
                 g_cycle_enabled ? 1U : 0U,
                 (unsigned long)swpro_hid_input_frames(),
                 ESP_BD_ADDR_HEX(host));
    } else {
        ESP_LOGW(TAG, "#S c=%u p=%u y=%u f=%lu h=none",
                 g_hid_connected ? 1U : 0U, g_pairing ? 1U : 0U,
                 g_cycle_enabled ? 1U : 0U,
                 (unsigned long)swpro_hid_input_frames());
    }
}

void swpro_protocol_task(void *argument)
{
    (void)argument;
    TickType_t previous = xTaskGetTickCount();
    int64_t next_send_us = 0;
    int64_t next_status_us = esp_timer_get_time() + STATUS_INTERVAL_US;

    int64_t next_bridge_us = 0;
    bool last_connected = false;
    bool last_cycle = false;
    uint32_t last_auto_a_generation = 0;
    int64_t auto_a_deadline_us = 0;
    auto_a_state_t auto_a_state = AUTO_A_IDLE;
    bool a_press_sent = false;
    bool a_release_sent = false;

    while (true) {
        vTaskDelayUntil(&previous, 1);
        const int64_t now = esp_timer_get_time();

        if (now >= WAKE_BOOT_TIMEOUT_US) {
            swpro_finish_wake(WAKE_ERR_TIMEOUT);
        }

        idle_restart_step(now);
        connect_cycle_step(now);
        ns2_wake_step(!g_wake_tx_active && !g_direct_handoff_active &&
                      !g_wake_page_pending && !g_acl_connected &&
                      !g_hid_connected && g_connect_deadline_us == 0);

        // Generate a single A press after each successful HID connection.
        // Never block the protocol task: it also drives the normal input reports.
        if (!g_hid_connected || swpro_hid_host_stopped()) {
            if (auto_a_state != AUTO_A_IDLE) {
                swpro_hid_override_a(false, false);
                swpro_finish_wake(WAKE_ERR_INPUT);
            }
        } else if (last_auto_a_generation != s_hid_connection_generation) {
            if (auto_a_state != AUTO_A_IDLE) {
                swpro_finish_wake(WAKE_ERR_INPUT);
            }
            last_auto_a_generation = s_hid_connection_generation;
            swpro_hid_override_a(false, false);
            auto_a_deadline_us = now + AUTO_A_DELAY_US;
            auto_a_state = AUTO_A_WAIT;
            ESP_LOGI(TAG, "Auto-unlock: A press scheduled 2 seconds after HID connection");
        } else if (auto_a_state == AUTO_A_WAIT && now >= auto_a_deadline_us) {
            swpro_hid_override_a(true, true);
            auto_a_deadline_us = now + AUTO_A_PRESS_US;
            auto_a_state = AUTO_A_PRESSED;
            ESP_LOGI(TAG, "Auto-unlock: A pressed");
        } else if (auto_a_state == AUTO_A_PRESSED && now >= auto_a_deadline_us) {
            swpro_hid_override_a(true, false);
            auto_a_deadline_us = now + AUTO_A_RELEASE_US;
            auto_a_state = AUTO_A_RELEASED;
            ESP_LOGI(TAG, "Auto-unlock: A released");
        } else if (auto_a_state == AUTO_A_RELEASED && now >= auto_a_deadline_us) {
            swpro_hid_override_a(false, false);
            // Require successful report submission during both phases.
            swpro_finish_wake(a_press_sent && a_release_sent
                              ? WAKE_OK : WAKE_ERR_INPUT);
        }

        // Suppress state lines during CSB transmission so UART output cannot disrupt the CSB transmission period.
        if (!g_wake_tx_active &&
            (now >= next_bridge_us || last_connected != g_hid_connected ||
             last_cycle != g_cycle_enabled)) {
            next_bridge_us = now + 1000000;
            last_connected = g_hid_connected;
            last_cycle = g_cycle_enabled;
            report_bridge_status();
        }

        if (g_direct_handoff_active &&
            now >= g_direct_handoff_quiet_until_us) {
            esp_err_t result = ESP_ERR_TIMEOUT;
            uint8_t link = 0xff;
            if (switch1_acl_handoff_status(&result, &link) &&
                result != ESP_OK) {
                ESP_LOGW(TAG, "Failed to register ACL link from wake-up response: %s link=%u",
                         esp_err_to_name(result), link);
                wake_handoff_finished();
            } else if (g_direct_handoff_deadline_us != 0 &&
                       now >= g_direct_handoff_deadline_us) {
                // Reinitialize the Bluetooth controller because it may still be registered with the Bluetooth host and link manager.
                ESP_LOGE(TAG, "Rebooting because ACL link registration from wake-up response did not complete");
                swpro_restart_after_disconnect(0);
            }
        }

        if (g_wake_hid_connect_pending) {
            g_wake_hid_connect_pending = false;
            if (g_acl_connected && !g_hid_connected) {
                g_reconnect_paused = false;
                g_next_reconnect_us = now + (int64_t)RECONNECT_INTERVAL_US;
                {
                    // Request ACL disconnection after stopping CSB to avoid contention between CSB transmission and ACL disconnection.
                    if (g_wake_tx_active) {
                        g_wake_stop_requested = true;
                        const int64_t stop_until =
                            esp_timer_get_time() + 500000;
                        while (g_wake_tx_active &&
                               esp_timer_get_time() < stop_until) {
                            vTaskDelay(pdMS_TO_TICKS(1));
                        }
                        if (g_wake_tx_active) {
                            ESP_LOGW(TAG,
                                     "Wake-up succeeded: CSB stop was not confirmed; "
                                     "requesting disconnect");
                        }
                    }
                    esp_bd_addr_t host;
                    if (swpro_host_snapshot(host) &&
                        L2CA_SetIdleTimeoutByBdAddr(host, 0,
                                                    BT_TRANSPORT_BR_EDR_LOCAL)) {
                        ESP_LOGI(TAG,
                                 "Wake-up succeeded: starting normal page after ACL disconnect");
                        g_wake_page_pending = true;
                        g_wake_disconnect_tries = 0;
                        g_wake_local_release_tried = false;
                        // Disable normal paging until ACL disconnection finishes so it cannot reconnect to an ACL link being torn down.
                        g_reconnect_disabled = true;
                        // Cancel any pending HID connection request to avoid leaving an old L2CAP request in Bluedroid.
                        if (g_connect_deadline_us != 0) {
                            ESP_LOGI(TAG,
                                     "Wake-up succeeded: cancelling in-progress HID "
                                     "connection request");
                            esp_bt_hid_device_disconnect();
                        }
                        g_connect_deadline_us = 0;
                        switch1_hci_page_cancel();
                        g_wake_page_deadline_us =
                            esp_timer_get_time() + WAKE_DISCONNECT_FIRST_US;
                    } else {
                        ESP_LOGW(TAG,
                                 "Wake-up succeeded: cannot request disconnect; rebooting (%s)",
                                 g_cycle_enabled ? "continue connection cycle"
                                                 : "start normal page after boot");
                        swpro_restart_after_disconnect(g_cycle_enabled
                                                     ? CYCLE_RESTART_MARKER
                                                     : WAKE_RESTART_MARKER);
                    }
                }
            }
        }

        // Start normal paging only after CSB transmission and ACL disconnection have both completed.
        if (g_wake_page_pending && !g_wake_tx_active &&
            (!g_acl_connected || now >= g_wake_page_deadline_us)) {
            if (g_acl_connected &&
                g_wake_disconnect_tries < WAKE_DISCONNECT_TRIES) {
                esp_bd_addr_t host;
                g_wake_disconnect_tries++;
                const uint8_t ke_before = switch1_acl_lc_state();
                if (ke_before != 5 && (ke_before < 25 || ke_before > 27)) {
                    (void)switch1_acl_force_idle();
                }
                const esp_err_t derr = switch1_acl_detach();
                ESP_LOGW(TAG,
                         "ACL disconnect incomplete; requesting lower-layer detach"
                         " (%u/%u): %s link_state=%u",
                         g_wake_disconnect_tries, WAKE_DISCONNECT_TRIES,
                         esp_err_to_name(derr), switch1_acl_lc_state());
                if (derr != ESP_OK && swpro_host_snapshot(host)) {
                    btm_remove_acl(host, BT_TRANSPORT_BR_EDR_LOCAL);
                }
                g_wake_page_deadline_us =
                    esp_timer_get_time() + WAKE_DISCONNECT_RETRY_US;
            } else if (g_acl_connected) {
                if (!g_wake_local_release_tried) {
                    g_wake_local_release_tried = true;
                    const esp_err_t rerr = switch1_acl_release();
                    ESP_LOGW(TAG, "Releasing ESP32-side ACL state only: %s",
                             esp_err_to_name(rerr));
                    if (rerr == ESP_OK) {
                        g_wake_page_deadline_us = now + 400000;
                        goto wake_page_done;
                    }
                }
                g_wake_page_pending = false;
                ESP_LOGW(TAG,
                         "Rebooting because ACL disconnect did not complete (%s)",
                         g_cycle_enabled ? "continue connection cycle"
                                         : "start normal page after boot");
                swpro_restart_after_disconnect(g_cycle_enabled ? CYCLE_RESTART_MARKER
                                                         : WAKE_RESTART_MARKER);
            } else {
                g_wake_page_pending = false;
                g_reconnect_paused = false;
                g_reconnect_disabled = false;
                g_next_reconnect_us =
                    esp_timer_get_time() + WAKE_PAGE_GAP_US;
                ESP_LOGI(TAG,
                         "ACL disconnected; reconnecting with normal page after %dms",
                         (int)(WAKE_PAGE_GAP_US / 1000));
            }
        }
wake_page_done:

        if (g_hid_connected && !swpro_hid_host_stopped()) {
            if (now >= next_send_us) {
                next_send_us = now + swpro_hid_report_interval_us();
                if (swpro_hid_send_report() == ESP_OK) {
                    if (auto_a_state == AUTO_A_PRESSED) a_press_sent = true;
                    if (auto_a_state == AUTO_A_RELEASED) a_release_sent = true;
                }
            }
        } else {
            next_send_us = 0;
            if (!g_reconnect_paused && now >= g_next_reconnect_us) {
                g_next_reconnect_us = now + (int64_t)RECONNECT_INTERVAL_US;
                try_reconnect();
            }
        }


        if (now >= next_status_us) {
            next_status_us = now + STATUS_INTERVAL_US;
            log_status();
        }
    }
}
