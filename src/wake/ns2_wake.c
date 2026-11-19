#include "ns2_wake.h"

#include <string.h>

#include "esp_bt_device.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#include "wake_cycle.h"

#define TAG "ns2_wake"

#define NS2_WAKE_MAGIC UINT32_C(0x4E533258)
#define NS2_WAKE_SCAN_SECONDS 30
#define NS2_WAKE_ADVERTISE_MS 2000
#define NS2_WAKE_WAIT_MS 2000
#define NS2_NINTENDO_COMPANY_ID UINT16_C(0x0553)
#define NS2_WAKE_TYPE UINT8_C(0x01)

typedef struct {
    uint32_t magic;
    uint8_t address_type;
    uint8_t controller_address[6];
    uint16_t product_id;
    uint8_t console_address[6];
} ns2_wake_identity_t;

static nvs_handle_t s_nvs;
static ns2_wake_identity_t s_identity;
static ns2_wake_identity_t s_pending_identity;
static portMUX_TYPE s_identity_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_save_pending;
static volatile bool s_save_in_progress;
static volatile bool s_scan_pending;
static volatile bool s_scan_requested;
static volatile bool s_scan_active;
static volatile bool s_scan_cancel_pending;
static volatile bool s_adv_data_ready;
static volatile bool s_adv_started;
static volatile bool s_adv_stopped;
static volatile bool s_adv_failed;

static const esp_ble_scan_params_t s_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x0030,
    .scan_window = 0x0030,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
};

static const esp_ble_adv_params_t s_adv_params = {
    .adv_int_min = 0x0030,
    .adv_int_max = 0x0060,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static bool identity_valid(const ns2_wake_identity_t *identity)
{
    return identity != NULL && identity->magic == NS2_WAKE_MAGIC;
}

static bool parse_manufacturer(const uint8_t *data, size_t length,
                               uint16_t *product_id,
                               uint8_t console_address[6])
{
    if (data == NULL || product_id == NULL || console_address == NULL ||
        length < 18) {
        return false;
    }
    const uint16_t company_id =
        (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    if (company_id != NS2_NINTENDO_COMPANY_ID || data[2] != NS2_WAKE_TYPE) {
        return false;
    }
    *product_id = (uint16_t)data[7] | ((uint16_t)data[8] << 8);
    for (size_t i = 0; i < 6; ++i) {
        console_address[i] = data[17 - i];
    }
    return true;
}

static bool parse_advertisement(const uint8_t *data, size_t length,
                                uint16_t *product_id,
                                uint8_t console_address[6])
{
    uint8_t manufacturer_length = 0;
    uint8_t *manufacturer = esp_ble_resolve_adv_data_by_type(
        (uint8_t *)data, (uint16_t)length,
        ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &manufacturer_length);
    return parse_manufacturer(manufacturer, manufacturer_length,
                              product_id, console_address);
}

static void build_advertisement(const ns2_wake_identity_t *identity,
                                uint8_t data[31])
{
    memset(data, 0, 31);
    data[0] = 2;
    data[1] = ESP_BLE_AD_TYPE_FLAG;
    data[2] = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;
    data[3] = 27;
    data[4] = ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE;
    data[5] = 0x53;
    data[6] = 0x05;
    data[7] = NS2_WAKE_TYPE;
    data[8] = 0x00;
    data[9] = 0x03;
    data[10] = 0x7E;
    data[11] = 0x05;
    data[12] = (uint8_t)identity->product_id;
    data[13] = (uint8_t)(identity->product_id >> 8);
    data[14] = 0x00;
    data[15] = 0x01;
    data[16] = 0x81;
    for (size_t i = 0; i < 6; ++i) {
        data[17 + i] = identity->console_address[5 - i];
    }
    data[23] = 0x0F;
}

esp_err_t ns2_wake_init(void)
{
    esp_err_t err = nvs_open("swpro", NVS_READWRITE, &s_nvs);
    if (err != ESP_OK) {
        return err;
    }

    size_t length = sizeof(s_identity);
    memset(&s_identity, 0, sizeof(s_identity));
    err = nvs_get_blob(s_nvs, "ns2wake", &s_identity, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK || length != sizeof(s_identity) ||
        !identity_valid(&s_identity)) {
        ESP_LOGW(TAG, "Ignoring invalid NS2 wake identity");
        memset(&s_identity, 0, sizeof(s_identity));
        return ESP_OK;
    }
    return ESP_OK;
}

bool ns2_wake_has_identity(void)
{
    bool valid;
    portENTER_CRITICAL(&s_identity_lock);
    valid = !s_save_pending && !s_save_in_progress &&
            identity_valid(&s_identity);
    portEXIT_CRITICAL(&s_identity_lock);
    return valid;
}

esp_err_t ns2_wake_scan_start(void)
{
    esp_err_t result = ESP_OK;
    portENTER_CRITICAL(&s_identity_lock);
    if (s_scan_pending || s_scan_requested || s_scan_active) {
        result = ESP_ERR_INVALID_STATE;
    } else {
        s_scan_pending = true;
    }
    portEXIT_CRITICAL(&s_identity_lock);
    return result;
}

void ns2_wake_scan_stop(void)
{
    bool stop;
    portENTER_CRITICAL(&s_identity_lock);
    s_scan_pending = false;
    if (s_scan_requested) {
        s_scan_cancel_pending = true;
    }
    stop = s_scan_active;
    portEXIT_CRITICAL(&s_identity_lock);
    if (stop) {
        const esp_err_t err = esp_ble_gap_stop_scanning();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop NS2 BLE scan: %s",
                     esp_err_to_name(err));
        }
    }
}

void ns2_wake_step(bool radio_idle)
{
    ns2_wake_identity_t identity;
    bool save;
    bool start_scan = false;

    portENTER_CRITICAL(&s_identity_lock);
    save = s_save_pending;
    if (save) {
        identity = s_pending_identity;
        s_save_pending = false;
        s_save_in_progress = true;
    }
    if (radio_idle && s_scan_pending && !s_scan_requested && !s_scan_active) {
        s_scan_pending = false;
        s_scan_requested = true;
        s_scan_cancel_pending = false;
        start_scan = true;
    }
    portEXIT_CRITICAL(&s_identity_lock);

    if (start_scan) {
        const esp_err_t err = esp_ble_gap_set_scan_params(
            (esp_ble_scan_params_t *)&s_scan_params);
        if (err != ESP_OK) {
            portENTER_CRITICAL(&s_identity_lock);
            s_scan_requested = false;
            portEXIT_CRITICAL(&s_identity_lock);
            ESP_LOGE(TAG, "#N NS2 BLE scan parameter request failed: %s",
                     esp_err_to_name(err));
        }
    }
    if (!save) {
        return;
    }

    esp_err_t err = nvs_set_blob(s_nvs, "ns2wake", &identity,
                                 sizeof(identity));
    if (err == ESP_OK) {
        err = nvs_commit(s_nvs);
    }
    if (err == ESP_OK) {
        portENTER_CRITICAL(&s_identity_lock);
        s_identity = identity;
        s_save_in_progress = false;
        portEXIT_CRITICAL(&s_identity_lock);
        ESP_LOGW(TAG,
                 "#N NS2 wake data saved: controller=" ESP_BD_ADDR_STR
                 " product=0x%04x console=" ESP_BD_ADDR_STR,
                 ESP_BD_ADDR_HEX(identity.controller_address),
                 identity.product_id,
                 ESP_BD_ADDR_HEX(identity.console_address));
    } else {
        portENTER_CRITICAL(&s_identity_lock);
        s_save_in_progress = false;
        portEXIT_CRITICAL(&s_identity_lock);
        ESP_LOGE(TAG, "#N NS2 wake data save failed: %s",
                 esp_err_to_name(err));
    }
}

bool ns2_wake_prepare_controller_address(void)
{
    if (!identity_valid(&s_identity)) {
        ESP_LOGE(TAG, "NS2 wake identity is unavailable");
        return false;
    }
    const esp_err_t err = esp_iface_mac_addr_set(
        s_identity.controller_address, ESP_MAC_BT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set NS2 wake controller address: %s",
                 esp_err_to_name(err));
        return false;
    }
    return true;
}

static bool wait_flag(volatile bool *flag, uint32_t timeout_ms)
{
    for (uint32_t elapsed = 0; elapsed < timeout_ms; elapsed += 10) {
        if (*flag || s_adv_failed) {
            return *flag;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

void ns2_wake_advertise_and_restart(void)
{
    static uint8_t advertisement[31];
    const uint8_t *local_address = esp_bt_dev_get_address();
    if (local_address == NULL ||
        memcmp(local_address, s_identity.controller_address, 6) != 0) {
        ESP_LOGE(TAG, "NS2 wake controller address was not applied");
        goto restart;
    }

    build_advertisement(&s_identity, advertisement);
    s_adv_data_ready = false;
    s_adv_started = false;
    s_adv_stopped = false;
    s_adv_failed = false;

    esp_err_t err = esp_ble_gap_config_adv_data_raw(advertisement,
                                                    sizeof(advertisement));
    if (err != ESP_OK || !wait_flag(&s_adv_data_ready, NS2_WAKE_WAIT_MS)) {
        ESP_LOGE(TAG, "Failed to configure NS2 wake advertisement%s%s",
                 err == ESP_OK ? "" : ": ",
                 err == ESP_OK ? "" : esp_err_to_name(err));
        goto restart;
    }
    err = esp_ble_gap_start_advertising((esp_ble_adv_params_t *)&s_adv_params);
    if (err != ESP_OK || !wait_flag(&s_adv_started, NS2_WAKE_WAIT_MS)) {
        ESP_LOGE(TAG, "Failed to start NS2 wake advertisement%s%s",
                 err == ESP_OK ? "" : ": ",
                 err == ESP_OK ? "" : esp_err_to_name(err));
        goto restart;
    }

    ESP_LOGI(TAG,
             "Sending NS2 wake advertisement for %u ms: controller="
             ESP_BD_ADDR_STR " product=0x%04x console=" ESP_BD_ADDR_STR,
             NS2_WAKE_ADVERTISE_MS,
             ESP_BD_ADDR_HEX(s_identity.controller_address),
             s_identity.product_id,
             ESP_BD_ADDR_HEX(s_identity.console_address));
    vTaskDelay(pdMS_TO_TICKS(NS2_WAKE_ADVERTISE_MS));
    err = esp_ble_gap_stop_advertising();
    if (err != ESP_OK || !wait_flag(&s_adv_stopped, NS2_WAKE_WAIT_MS)) {
        ESP_LOGW(TAG, "NS2 wake advertisement stop was not confirmed%s%s",
                 err == ESP_OK ? "" : ": ",
                 err == ESP_OK ? "" : esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "NS2 wake advertisement finished");
    }
    // Restore the normal public BD_ADDR and resume paging the saved Switch.
    g_restart_marker = 0;

restart:
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
    __builtin_unreachable();
}

void ns2_wake_gap_callback(esp_gap_ble_cb_event_t event,
                           esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        bool start = false;
        portENTER_CRITICAL(&s_identity_lock);
        if (s_scan_requested) {
            if (param->scan_param_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                s_scan_requested = false;
            } else if (s_scan_cancel_pending) {
                s_scan_requested = false;
                s_scan_cancel_pending = false;
            } else {
                start = true;
            }
        }
        portEXIT_CRITICAL(&s_identity_lock);
        if (param->scan_param_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "#N NS2 BLE scan parameter setup failed: status=%u",
                     param->scan_param_cmpl.status);
            break;
        }
        if (start) {
            const esp_err_t err =
                esp_ble_gap_start_scanning(NS2_WAKE_SCAN_SECONDS);
            if (err != ESP_OK) {
                portENTER_CRITICAL(&s_identity_lock);
                s_scan_requested = false;
                portEXIT_CRITICAL(&s_identity_lock);
                ESP_LOGE(TAG, "#N NS2 BLE scan start request failed: %s",
                         esp_err_to_name(err));
            }
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
        const bool succeeded =
            param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS;
        bool stop;
        portENTER_CRITICAL(&s_identity_lock);
        s_scan_requested = false;
        stop = succeeded && s_scan_cancel_pending;
        s_scan_active = succeeded && !s_scan_cancel_pending;
        s_scan_cancel_pending = false;
        portEXIT_CRITICAL(&s_identity_lock);
        if (stop) {
            const esp_err_t err = esp_ble_gap_stop_scanning();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "#N NS2 BLE scan cancel failed: %s",
                         esp_err_to_name(err));
            }
        } else if (succeeded) {
            ESP_LOGW(TAG,
                     "#N NS2 BLE scan started for %u seconds; press HOME on a paired "
                     "Joy-Con 2 or Pro Controller 2",
                     NS2_WAKE_SCAN_SECONDS);
        } else {
            ESP_LOGE(TAG, "#N NS2 BLE scan start failed: status=%u",
                     param->scan_start_cmpl.status);
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            bool active;
            portENTER_CRITICAL(&s_identity_lock);
            active = s_scan_active;
            portEXIT_CRITICAL(&s_identity_lock);
            if (!active) {
                break;
            }
            ns2_wake_identity_t identity = {0};
            identity.magic = NS2_WAKE_MAGIC;
            if (!parse_advertisement(param->scan_rst.ble_adv,
                                     param->scan_rst.adv_data_len,
                                     &identity.product_id,
                                     identity.console_address)) {
                break;
            }
            identity.address_type = param->scan_rst.ble_addr_type;
            memcpy(identity.controller_address, param->scan_rst.bda, 6);
            bool captured = false;
            portENTER_CRITICAL(&s_identity_lock);
            if (s_scan_active) {
                s_pending_identity = identity;
                s_save_pending = true;
                s_scan_active = false;
                captured = true;
            }
            portEXIT_CRITICAL(&s_identity_lock);
            if (captured) {
                (void)esp_ble_gap_stop_scanning();
            }
        } else if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
            bool timed_out;
            portENTER_CRITICAL(&s_identity_lock);
            timed_out = s_scan_active;
            s_scan_requested = false;
            s_scan_active = false;
            s_scan_cancel_pending = false;
            portEXIT_CRITICAL(&s_identity_lock);
            if (timed_out) {
                ESP_LOGW(TAG,
                         "#N NS2 BLE scan timed out without a matching wake packet");
            }
        }
        break;

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        portENTER_CRITICAL(&s_identity_lock);
        s_scan_requested = false;
        s_scan_active = false;
        s_scan_cancel_pending = false;
        portEXIT_CRITICAL(&s_identity_lock);
        break;

    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        s_adv_data_ready =
            param->adv_data_raw_cmpl.status == ESP_BT_STATUS_SUCCESS;
        s_adv_failed = !s_adv_data_ready;
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        s_adv_started = param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS;
        s_adv_failed = !s_adv_started;
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        s_adv_stopped = param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS;
        break;

    default:
        break;
    }
}
