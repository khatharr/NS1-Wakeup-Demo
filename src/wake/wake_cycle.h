#ifndef WAKE_CYCLE_H
#define WAKE_CYCLE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 次のサイクル処理を開始できる時刻
 */
extern int64_t g_cycle_next_us;

/**
 * @brief 接続処理サイクルを開始または停止する
 * @param enabled trueで開始し、falseで停止する
 */
void connect_cycle_set(bool enabled);

/**
 * @brief 接続処理サイクルを1段階進める
 * @param now 現在時刻をマイクロ秒で指定する
 *
 * 1msごとに呼び出す。
 */
void connect_cycle_step(int64_t now);

/**
 * @brief 接続処理が停止中に予約された再起動を実行する
 * @param now 現在時刻をマイクロ秒で指定する
 *
 * HID接続が切断され、他の接続処理が動作していない場合だけ再起動する。
 * 次の試行を起動直後の状態から開始するために使用する。
 */
void idle_restart_step(int64_t now);

/**
 * @brief 再起動前後で保持し、app_mainが起動時に読み出す値
 */
#define WAKE_RESTART_MARKER UINT32_C(0x57414B45)
#define CYCLE_RESTART_MARKER UINT32_C(0x43594331)
#define NS2_WAKE_RESTART_MARKER UINT32_C(0x4E533257)
/**
 * @brief 再起動理由をapp_mainへ渡すマーカー
 */
extern uint32_t g_restart_marker;

/**
 * @brief ウェイクアップ直後の再起動から起動したことを示すフラグ
 */
extern volatile bool g_page_right_after_boot;

/**
 * @brief 接続試行を継続した状態で再起動したことを示すフラグ
 */
extern volatile bool g_cycle_resume_after_boot;

/**
 * @brief HID接続の切断後に再起動するまでの待機時間
 */
#define IDLE_RESTART_DELAY_US INT64_C(3000000)

#endif
