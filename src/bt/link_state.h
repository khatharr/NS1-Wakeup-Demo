/**
 * @brief 複数モジュールが共有するBluetooth接続状態
 * @note 定義はswpro_link.cにある
 * @note ウェイクアップモジュールと接続サイクル処理が参照する変数だけを公開する
 * @note 別コアのタスクから参照または更新する状態にはvolatileを付ける
 */
#ifndef LINK_STATE_H
#define LINK_STATE_H

#include <stdbool.h>
#include <stdint.h>

extern volatile bool g_hid_connected;

extern volatile bool g_acl_connected;

/**
 * @brief 自動ページ開始を禁止する指定
 * @note ページは1回あたり3.2秒の時間切れを待ち、その間ずっとページ用の電波を送信する。
 *       進行中のページがあるとCSBの有効化がstatus 0x0bで拒否されるため、ウェイクアップ用CSBを送信する前に自動ページ開始を停止する。
 */
extern volatile bool g_reconnect_disabled;

/**
 * @brief 自動ページ開始を一時停止する指定
 * @note ウェイクアップ用CSB送信中にtrueにする
 */
extern volatile bool g_reconnect_paused;

extern int64_t g_next_reconnect_us;

/**
 * @brief HID接続要求の完了期限
 * @note 0は進行中のHID接続要求がない状態を示す
 */
extern int64_t g_connect_deadline_us;

extern volatile bool g_cycle_enabled;

/**
 * @brief 1回の接続サイクルにおけるウェイクアップ試行回数
 * @note 再起動後も保持するためRTCメモリに保存する
 */
extern uint32_t g_cycle_wake_tries;

/**
 * @brief ページへ移る前にウェイクアップを試す回数
 * @note HOMEで接続試行を始めると、Switchがスリープへ移行する前に入力されることがある
 *       この場合はウェイクアップが成立しないため、スリープ移行完了まで再試行する。
 *       1回あたり再起動1.2秒とウェイクアップ処理2秒が必要で、6回で約20秒になる。
 */
#define CYCLE_WAKE_TRIES 3

/** @brief HCI Create ConnectionのClock_Offsetで値が有効であることを示すビット */
#define HCI_CLOCK_OFFSET_VALID UINT16_C(0x8000)

/**
 * @brief HID接続要求を開始してから結果イベントを受信するまでの期限
 * @note BluedroidのHID状態がCONNECTINGから遷移しない場合、以降の要求はBUSYで拒否される。
 *       期限を超えたらHID切断要求を送って内部状態を解除する。
 * @note ページの時間切れ3.2秒より少し長い値にする
 */
#define CONNECT_TIMEOUT_US UINT64_C(4000000)

void try_reconnect(void);

#endif
