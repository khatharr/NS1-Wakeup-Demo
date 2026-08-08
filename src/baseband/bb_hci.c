// このbasebandモジュールはbluedroidを介さずにHCIコマンドを送信するため、送信処理と完了待ちをここにまとめる。
// bluedroidが処理しないCSB受信イベントを転送し、HCI Create Connectionへ渡すページ接続のClock_Offsetをリンカの--wrapで差し替える。

#include "baseband.h"

#include "bb_internal.h"
#include "bb_rom.h"

#include <stdbool.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "common/bt_target.h"
#include "hci/hci_layer.h"
#include "stack/bt_types.h"
#include "stack/hcidefs.h"
#include "stack/hcimsgs.h"

static const char *TAG = "wake_hci";

// HCIコマンドの失敗を許容する試行で、エラーログ出力が監視処理を遅延させる場合だけtrueに設定
volatile bool g_suppress_command_error_log;

volatile bool g_command_pending;
volatile bool g_command_timed_out;

volatile uint8_t g_command_status;
static volatile uint16_t s_command_opcode;
// Command Completeイベントのp[5]以降を最大8バイト保存
// HCI_SET_CLBが採用した送信間隔の確認に使用
uint8_t g_command_return[8];
volatile uint8_t g_command_return_length;
// bluedroidの接続ステートマシンは維持し、HCI Create Connectionコマンドを送信する直前に設定するClock_Offsetだけを差し替える
static uint8_t s_hci_page_target[6];
static volatile uint16_t s_hci_page_clock_offset;
static volatile bool s_hci_page_armed;
static portMUX_TYPE s_hci_page_lock = portMUX_INITIALIZER_UNLOCKED;

void switch1_hci_page_cancel(void)
{
    portENTER_CRITICAL(&s_hci_page_lock);
    s_hci_page_armed = false;
    portEXIT_CRITICAL(&s_hci_page_lock);
}
void switch1_hci_clock_offset_set(const uint8_t bd_addr[6], uint16_t offset)
{
    portENTER_CRITICAL(&s_hci_page_lock);
    s_hci_page_armed = false;
    memcpy(s_hci_page_target, bd_addr, sizeof(s_hci_page_target));
    s_hci_page_clock_offset = offset;
    s_hci_page_armed = true;
    portEXIT_CRITICAL(&s_hci_page_lock);
}

extern BOOLEAN __real_btsnd_hcic_create_conn(BD_ADDR dest,
                                               UINT16 packet_types,
                                               UINT8 page_scan_rep_mode,
                                               UINT8 page_scan_mode,
                                               UINT16 clock_offset,
                                               UINT8 allow_switch);

BOOLEAN __wrap_btsnd_hcic_create_conn(BD_ADDR dest, UINT16 packet_types,
                                      UINT8 page_scan_rep_mode,
                                      UINT8 page_scan_mode,
                                      UINT16 clock_offset, UINT8 allow_switch)
{
    uint16_t configured_offset;
    bool target_armed;
    const uint16_t requested_offset = clock_offset;
    portENTER_CRITICAL(&s_hci_page_lock);
    configured_offset = s_hci_page_clock_offset;
    target_armed = s_hci_page_armed &&
                memcmp(dest, s_hci_page_target,
                       sizeof(s_hci_page_target)) == 0;
    if (target_armed) {
        s_hci_page_armed = false;
    }
    portEXIT_CRITICAL(&s_hci_page_lock);
    if (!target_armed) {
        ESP_LOGI(TAG,
                 "HCI Create Connection not sent: "
                 "target=%02x:%02x:%02x:%02x:%02x:%02x one-time permission not armed",
                 dest[0], dest[1], dest[2], dest[3], dest[4], dest[5]);
        return FALSE;
    }
    const bool offset_configured = configured_offset != 0;
    if (offset_configured) {
        clock_offset = configured_offset;
        page_scan_rep_mode = 1;
    }
    ESP_LOGI(TAG,
             "HCI Create Connection: target=%02x:%02x:%02x:%02x:%02x:%02x "
             "Clock_Offset %04x -> %04x, PSRM=%u, PSM=%u, offset_configured=%u",
             dest[0], dest[1], dest[2], dest[3], dest[4], dest[5], requested_offset,
             clock_offset, page_scan_rep_mode, page_scan_mode,
             offset_configured ? 1u : 0u);
    return __real_btsnd_hcic_create_conn(dest, packet_types,
                                          page_scan_rep_mode, page_scan_mode,
                                          clock_offset, allow_switch);
}

extern void __real_btu_hcif_process_event(uint8_t controller_id,
                                          BT_HDR *p_msg);

void __wrap_btu_hcif_process_event(uint8_t controller_id, BT_HDR *p_msg)
{
    bb_csb_rx_on_event(p_msg);
    __real_btu_hcif_process_event(controller_id, p_msg);
}

static void command_complete(BT_HDR *response, void *context)
{
    (void)context;
    const uint8_t *const p = response->data + response->offset;
    g_command_return_length = 0;
    if (response->len >= 6 && p[0] == HCI_COMMAND_COMPLETE_EVT &&
        (uint16_t)(p[3] | ((uint16_t)p[4] << 8)) == s_command_opcode) {
        g_command_status = p[5];
        // HCI_SET_CLBは採用したLT_ADDRと送信間隔をCommand Completeイベントで返す。
        // HCI_SET_CLBに指定した送信間隔とControllerが返した送信間隔を比較するため、p[5]以降のデータを保存する。
        uint16_t extra = (uint16_t)(response->len - 5);
        if (extra > sizeof(g_command_return)) {
            extra = (uint16_t)sizeof(g_command_return);
        }
        memcpy(g_command_return, p + 5, extra);
        g_command_return_length = (uint8_t)extra;
    } else {
        g_command_status = HCI_ERR_UNSPECIFIED;
    }
    g_command_pending = false;
    osi_free(response);
}

static void command_status(uint8_t status, BT_HDR *command, void *context)
{
    (void)context;
    g_command_status = status;
    g_command_pending = false;
    // hci_layerはcallbackから戻った後もmetadataを参照する。
    // callbackをNULLへ変更し、hci_layer自身にcommandを解放させる。
    hci_cmd_metadata_t *const metadata = HCI_GET_CMD_METAMSG(command);
    metadata->command_status_cb = NULL;
}

esp_err_t send_command_wait(uint16_t opcode, const uint8_t *params,
                            uint8_t length)
{
    if (g_command_pending || g_command_timed_out) {
        return ESP_ERR_INVALID_STATE;
    }
    BT_HDR *p = HCI_GET_CMD_BUF(length);
    if (p == NULL) {
        return ESP_ERR_NO_MEM;
    }

    uint8_t *pp = (uint8_t *)(p + 1);
    p->len = HCIC_PREAMBLE_SIZE + length;
    p->offset = 0;
    UINT16_TO_STREAM(pp, opcode);
    UINT8_TO_STREAM(pp, length);
    if (length != 0) {
        memcpy(pp, params, length);
    }

    hci_cmd_metadata_t *const metadata = HCI_GET_CMD_METAMSG(p);
    metadata->opcode = opcode;
    metadata->command_complete_cb = command_complete;
    metadata->command_status_cb = command_status;
    s_command_opcode = opcode;
    g_command_status = HCI_ERR_UNSPECIFIED;
    g_command_pending = true;
    hci_layer_get_interface()->transmit_command(
        p, command_complete, command_status, NULL);

    // ESP-IDF hci_layerのCOMMAND_PENDING_TIMEOUTと同じ8秒を上限にする
    for (unsigned i = 0; i < 8000 && g_command_pending; ++i) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (g_command_pending) {
        // 遅延callbackが到着しても、HCIコマンドがController内部へ反映されたかは判定できない
        g_command_timed_out = true;
        return ESP_ERR_TIMEOUT;
    }
    if (g_command_status != HCI_SUCCESS) {
        if (!g_suppress_command_error_log) {
            ESP_LOGE(TAG, "HCI command failed: opcode=%04x status=%02x",
                     opcode, g_command_status);
        }
        return ESP_FAIL;
    }
    return ESP_OK;
}
