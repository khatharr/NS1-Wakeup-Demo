#include "swpro_uart.h"
#include "wake_cycle.h"

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
#include "swpro_link.h"
#include "swpro_pairing.h"
#include "baseband.h"
#include "wakeup_code.h"
#include "swpro_uart.h"
#include "wake_cycle.h"
#include "wake_session.h"

#define TAG "swpro"

// CSB受信状態を初期化するため、3回ごとに再起動する
#define CYCLE_SOFT_RETRY 3

#define CYCLE_SOFT_RETRY_GAP_US INT64_C(2500000)

// 起動直後のハードウェアエラーを避けるため、1.5秒待機する
#define CYCLE_WAKE_MIN_UPTIME_US INT64_C(1500000)

RTC_NOINIT_ATTR uint32_t g_restart_marker;
RTC_NOINIT_ATTR uint32_t g_cycle_wake_tries;
volatile bool g_page_right_after_boot;
volatile bool g_cycle_resume_after_boot;
int64_t g_cycle_next_us;

static const uint8_t k_cycle_wake_payload[15] = {
    0, 3, 0xFF, 6, 45, 10, 4, 3, 4, 0, 3, 140, 0, 15, 1};

// CMD_CSB_RX payload: 22スロット周期、6箇所、1箇所あたり160ms
static const uint8_t k_cycle_rx_payload[18] = {
    1, 22, 14, 0, 0, 140, 0, 1, 1, 1, 6, 16, 7, 1, 0, 0, 4, 0};

typedef enum {
    CYCLE_IDLE = 0,
    CYCLE_WAKE_START,
    CYCLE_WAKE_RX,
    CYCLE_WAKE_WAIT,
    CYCLE_PAGE_WAIT,
    CYCLE_HANDOFF_WAIT,
} connect_cycle_state_t;

static connect_cycle_state_t s_cycle_state;
static uint32_t s_cycle_rounds;

void connect_cycle_set(bool enabled)
{
    if (g_cycle_enabled == enabled) {
        return;
    }
    g_cycle_enabled = enabled;
    s_cycle_state = enabled ? CYCLE_WAKE_START : CYCLE_IDLE;
    g_cycle_next_us = 0;
    if (enabled) {
        s_cycle_rounds = 0;
        // RTCメモリは電源投入時の値が不定なので初期化する
        if (!g_cycle_resume_after_boot) {
            g_cycle_wake_tries = 0;
        }
        // Switch側だけでペアリング情報を削除された場合も再検出できるよう、検出可能にする
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                 ESP_BT_GENERAL_DISCOVERABLE);
        g_stay_disconnected = false;
        switch1_set_report_verbose(false);
        esp_log_level_set("wake_csb", ESP_LOG_WARN);
        esp_log_level_set("wake_link", ESP_LOG_WARN);
        ESP_LOGI(TAG, "Connection cycle started");
    } else {
        if (!g_pairing && swpro_host_exists()) {
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                     ESP_BT_NON_DISCOVERABLE);
        }
        ESP_LOGI(TAG, "Connection cycle stopped");
    }
}

void idle_restart_step(int64_t now)
{
    if (g_idle_restart_due_us == 0 || now < g_idle_restart_due_us) {
        return;
    }
    g_idle_restart_due_us = 0;
    if (g_hid_connected || g_acl_connected || g_cycle_enabled || g_pairing ||
        g_wake_tx_active) {
        return;
    }
    ESP_LOGI(TAG,
             "Restarting because no connection processing is active. The next attempt "
             "will start immediately after boot");
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_restart();
}

void connect_cycle_step(int64_t now)
{
    if (!g_cycle_enabled) {
        return;
    }
    if (g_hid_connected) {
        connect_cycle_set(false);
        return;
    }
    if (g_wake_page_pending || g_acl_connected) {
        // 進行中のACL切断とページ接続を中断しない
        s_cycle_state = CYCLE_HANDOFF_WAIT;
        g_cycle_next_us = now + 6000000;
        return;
    }

    if (s_cycle_state == CYCLE_HANDOFF_WAIT) {
        if (now >= g_cycle_next_us) {
            ESP_LOGI(TAG, "Connection attempt: page request did not establish a link; moving to the next cycle");
            s_cycle_state = CYCLE_WAKE_START;
            g_cycle_next_us = now;
        } else {
            return;
        }
    }

    if (s_cycle_state == CYCLE_WAKE_WAIT && !g_wake_tx_active &&
        !g_direct_handoff_active && !g_acl_connected && !g_wake_page_pending) {
        // 送信とリンク処理が完了した場合は待機期限を短縮する
        g_cycle_next_us = now;
    }
    if (now < g_cycle_next_us) {
        return;
    }

    switch (s_cycle_state) {
    case CYCLE_WAKE_START:
        if (now < CYCLE_WAKE_MIN_UPTIME_US) {
            g_cycle_next_us = CYCLE_WAKE_MIN_UPTIME_US;
            break;
        }
        // 進行中のページがあるとCSBの有効化がstatus 0x0bで拒否される
        g_reconnect_disabled = true;
        g_reconnect_paused = true;
        if (g_wake_tx_active || g_direct_handoff_active ||
            g_connect_deadline_us != 0) {
            static uint8_t defer_count;
            if ((defer_count++ % 3) == 0) {
                ESP_LOGW(TAG,
                         "Connection attempt delayed: transmitting=%u ACL link registration=%u "
                         "connection request deadline in %lldms",
                         g_wake_tx_active ? 1U : 0U,
                         g_direct_handoff_active ? 1U : 0U,
                         g_connect_deadline_us == 0
                             ? 0LL
                             : (long long)((g_connect_deadline_us - now) / 1000));
            }
            g_cycle_next_us = now + 200000;
            break;
        }
        s_cycle_rounds++;
        g_cycle_wake_tries++;
        ESP_LOGW(TAG, "Connection attempt: wake-up %lu/%u",
                 (unsigned long)g_cycle_wake_tries, CYCLE_WAKE_TRIES);
        apply_uart_frame(CMD_WAKE_START, k_cycle_wake_payload,
                         sizeof(k_cycle_wake_payload));
        g_cycle_next_us = now + 300000;
        s_cycle_state = CYCLE_WAKE_RX;
        break;

    case CYCLE_WAKE_RX:
        if (!g_wake_tx_active) {
            ESP_LOGW(TAG, "Connection attempt: wake-up transmission did not start; restarting");
            swpro_restart_after_disconnect(CYCLE_RESTART_MARKER);
        }
        apply_uart_frame(CMD_CSB_RX, k_cycle_rx_payload,
                         sizeof(k_cycle_rx_payload));
        g_cycle_next_us = now + 2200000;
        s_cycle_state = CYCLE_WAKE_WAIT;
        break;

    case CYCLE_WAKE_WAIT:
        if (g_cycle_wake_tries < CYCLE_WAKE_TRIES) {
#if CYCLE_SOFT_RETRY
            if ((g_cycle_wake_tries % CYCLE_SOFT_RETRY) != 0) {
                ESP_LOGW(TAG,
                         "Connection attempt: no response; retrying without restart "
                         "(attempt %lu)",
                         (unsigned long)g_cycle_wake_tries);
                s_cycle_state = CYCLE_WAKE_START;
                g_cycle_next_us = now + CYCLE_SOFT_RETRY_GAP_US;
                break;
            }
#endif
            ESP_LOGW(TAG, "Connection attempt: no response; restarting before retry");
            swpro_restart_after_disconnect(CYCLE_RESTART_MARKER);
            break;
        }
        // スリープ中のSwitchはページ接続要求を受信しないため、1回だけ送信する
        ESP_LOGW(TAG, "Connection attempt: sending one page connection request");
        g_reconnect_disabled = false;
        g_reconnect_paused = false;
        g_next_reconnect_us = 0;
        g_cycle_next_us = now + 3600000;
        s_cycle_state = CYCLE_PAGE_WAIT;
        break;

    case CYCLE_PAGE_WAIT:
        // 起動後の自動再接続を防ぐため、ページ接続要求を停止してから再起動する
        ESP_LOGI(TAG, "Connection attempt: stopping the page request, restarting, and moving to the next cycle");
        g_reconnect_disabled = true;
        g_reconnect_paused = true;
        g_cycle_wake_tries = 0;
        swpro_restart_after_disconnect(CYCLE_RESTART_MARKER);
        break;

    case CYCLE_HANDOFF_WAIT:
    case CYCLE_IDLE:
    default:
        break;
    }
}
