/*
 * 前回接続時のsm4が残っていると、btm_sec_l2cap_access_reqがSwitchの機能情報を取得せずに認証を開始する。
 * その場合はL2CAP接続応答を送信できないため、ウェイクアップ処理の開始前にsm4を未取得状態へ設定する。
 *
 * 対象フィールドはtBTM_SEC_DEV_RECのsec_stateとsm4である。
 * 構造体の配置はsdkconfigの設定で変わるため、オフセットを手計算せず、内部ヘッダーの型定義を介してメンバー名でアクセスする。
 * 必要なインクルードパスはsrc/CMakeLists.txtに設定済み。
 */
#include "host_sec_state.h"

#include <string.h>

#include "common/bt_target.h"
#include "stack/btm_api.h"
#include "btm_int.h"

#include "esp_log.h"

static const char *const TAG = "host_sec";

bool host_sec_state_read(const uint8_t bd_addr[6], host_sec_state_t *out)
{
    if (bd_addr == NULL || out == NULL) {
        return false;
    }
    tBTM_SEC_DEV_REC *const device_record = btm_find_dev((uint8_t *)bd_addr);
    if (device_record == NULL) {
        return false;
    }
    out->sec_state = device_record->sec_state;
    out->sm4 = device_record->sm4;
    out->sec_flags = device_record->sec_flags;
    out->security_required = device_record->security_required;
    out->link_key_type = device_record->link_key_type;
    out->is_originator = device_record->is_originator ? 1u : 0u;
    return true;
}

void host_sec_state_report(const uint8_t bd_addr[6], const char *log_label)
{
    host_sec_state_t state;
    if (!host_sec_state_read(bd_addr, &state)) {
        ESP_LOGI(TAG, "%s: device record is not registered",
                 log_label != NULL ? log_label : "");
        return;
    }
    ESP_LOGI(TAG,
             "%s: sec_state=%u sm4=0x%02x sec_flags=0x%04x "
             "security_required=0x%04x link_key_type=%u originator=%u",
             log_label != NULL ? log_label : "", state.sec_state, state.sm4,
             state.sec_flags, state.security_required, state.link_key_type,
             state.is_originator);
}

bool host_sec_state_reset_to_page_state(const uint8_t bd_addr[6])
{
    if (bd_addr == NULL) {
        return false;
    }
    tBTM_SEC_DEV_REC *const device_record = btm_find_dev((uint8_t *)bd_addr);
    if (device_record == NULL) {
        return false;
    }
    const uint8_t sec_state_before = device_record->sec_state;
    const uint8_t sm4_before = device_record->sm4;

    // デバイス名取得待ちで停止した状態を解消する。
    // btm_sec_start_get_nameのタイムアウト値は0であるため、デバイス名が返らない限り状態が解除されない。
    // btm_sec_execute_procedureはsec_stateがアイドル状態以外であれば、処理を開始せずに終了する。
    if (device_record->sec_state != BTM_SEC_STATE_IDLE) {
        device_record->sec_state = BTM_SEC_STATE_IDLE;
    }

    // Switchの機能情報を未取得状態に設定する。
    // sm4にBTM_SM4_TRUEが設定されていると、btm_sec_l2cap_access_reqが保留せず認証と暗号化を要求する。
    // 機能情報を取得してから処理を再開し、ページ接続時と同じ順序になる。
    device_record->sm4 = BTM_SM4_UNKNOWN;

    ESP_LOGI(TAG,
             "Set device record to post-page state: "
             "sec_state %u -> %u  "
             "sm4 0x%02x -> 0x%02x",
             sec_state_before, device_record->sec_state, sm4_before,
             device_record->sm4);
    return true;
}
