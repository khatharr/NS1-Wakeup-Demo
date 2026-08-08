#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t sec_state;
    uint8_t sm4;
    uint16_t sec_flags;
    uint16_t security_required;
    uint8_t link_key_type;
    uint8_t is_originator;
} host_sec_state_t;

/**
 * @brief Switchのデバイス記録から接続可否に関わるフィールドを読み出す
 * @param bd_addr SwitchのBD_ADDR
 * @param out 読み出した状態の格納先
 * @return デバイス記録が登録されていればtrue、なければfalse
 */
bool host_sec_state_read(const uint8_t bd_addr[6], host_sec_state_t *out);

/**
 * @brief Switchのデバイス記録にある接続状態をログへ出力する
 * @param bd_addr SwitchのBD_ADDR
 * @param log_label ログ行に付けるラベル。NULLなら空文字列を使用する
 */
void host_sec_state_report(const uint8_t bd_addr[6], const char *log_label);

/**
 * @brief デバイス記録のsec_stateとsm4をページ接続直後の状態へ設定する
 * @param bd_addr SwitchのBD_ADDR
 * @return デバイス記録が見つかり設定できればtrue、なければfalse
 * @note デバイス名取得待ちで停止したsec_stateを解除し、Switchの機能情報を未取得状態に設定する。
 *       リンクキーは変更しないため、ペアリング情報は保持される。
 */
bool host_sec_state_reset_to_page_state(const uint8_t bd_addr[6]);

#ifdef __cplusplus
}
#endif
