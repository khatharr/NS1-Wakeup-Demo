#ifndef SWPRO_LINK_H
#define SWPRO_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_event.h"
#include "esp_gap_bt_api.h"
#include "esp_sdp_api.h"

/**
 * @brief 保存済みSwitchアドレスがない状態
 * @note 検出可能・接続可能モードでペアリング受付を行う
 */
extern volatile bool g_pairing;

/**
 * @brief 自動ページ開始と接続可能モードを停止した状態
 * @note 切断通知のたびに停止処理を繰り返すと、Switchの再接続とESP32の切断要求が相互に繰り返されるため、停止処理は1回だけ実行する。
 */
extern volatile bool g_stay_disconnected;

/**
 * @brief HID接続要求の連続失敗回数
 * @note 一定回数に達したら再起動してBluetooth接続状態を初期化する
 */
extern uint32_t g_connect_failures;

/** @brief リンク監視タイムアウトのスロット数 */
#define LINK_SUPERVISION_TIMEOUT_SLOTS 3200
/**
 * @brief ページ時間切れのスロット数
 * @note 1スロットは0.625msで、0x1400は3.2秒。
 */
#define WAKE_PAGE_TIMEOUT_SLOTS UINT16_C(0x1400)
extern void BTM_SetDefaultLinkSuperTout(uint16_t timeout);

void swpro_gap_callback(esp_bt_gap_cb_event_t event,
                        esp_bt_gap_cb_param_t *param);
void swpro_hidd_event_callback(void *handler_args, esp_event_base_t base,
                               int32_t id, void *event_data);
#if CONFIG_BT_SDP_COMMON_ENABLED
void swpro_sdp_callback(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param);
#endif

/**
 * @brief ウェイクアップ処理で生成したACLリンクをホスト側Bluetoothスタックへ登録したあと、ACLを切断して通常ページを開始するまでの進行状態
 */
extern volatile bool g_wake_page_pending;
extern volatile int64_t g_wake_page_deadline_us;
extern volatile bool g_wake_hid_connect_pending;
extern volatile uint8_t g_wake_disconnect_tries;
extern bool g_wake_local_release_tried;

/**
 * @brief HID接続とウェイクアップ処理が停止している期間に再起動する予定時刻
 * @note 0は再起動を予約していない状態を示す
 */
extern int64_t g_idle_restart_due_us;

void swpro_protocol_task(void *argument);

/**
 * @brief ACL切断完了を待ってから再起動する
 * @param restart_marker 再起動後の処理を選択する識別値
 * @note ACLが残った状態で再起動すると、Switch側にリンクが残るため避ける
 */
void swpro_restart_after_disconnect(uint32_t restart_marker);

/**
 * @brief 自動ページ開始と接続可能モードを停止する
 * @param reason ログに出力する停止理由
 * @note HOME入力で接続試行を開始するまで、自動接続を再開しない
 */
void swpro_stop_auto_connect(const char *reason);

#endif
