/* 送信タスクとUARTタスクは別コアで動作する。タスク間で共有する値にはvolatileを使用する。 */
#ifndef WAKE_INTERNAL_H
#define WAKE_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "baseband.h"
#include "wake_payload.h"
#include "link_state.h"
#include "wake_session.h"

typedef enum {
    WAKE_BURST_WAIT_CLEAR,
    WAKE_BURST_WAIT_START,
    WAKE_BURST_WAIT_GAP,
} wake_state_t;

// 64スロット周期ではピコネットクロックのCLK[6:1]が変化せず、白色化の初期値が一定になる
#define WAKE_CSB_RAW_INTERVAL UINT16_C(0x0040)
#define WAKE_SECONDS_MIN UINT8_C(1)
#define WAKE_SECONDS_MAX UINT8_C(30)
#define WAKE_WHITEN_SEED_MAX UINT8_C(127)
#define WAKE_WHITEN_SEED_NONE UINT8_C(0xFF)
#define WAKE_CHANNEL_MAX UINT8_C(78)
#define WAKE_VARIANT_START_MAX UINT8_C(31)
#define WAKE_VARIANT_START_NONE UINT8_C(0xFF)
// 0xFEは白色化初期値0~31の順次切り替えを指定する
#define WAKE_VARIANT_SWEEP UINT8_C(0xFE)
#define WAKE_SWEEP_STEP_US UINT32_C(750000)
// descriptor検出から4スロット後に送信を開始するため、受信開始時刻を最後のバーストから15 CLKN後に設定する
#define WAKE_RX_OFFSET_DEFAULT UINT8_C(15)
// 318.75ms周期の応答バーストを2周期分監視する
#define WAKE_RX_WATCH_US INT64_C(700000)
// CSB送信とCSB受信を同時に登録できるよう、送信イベントの予約時間を250us短縮する
#define CSB_TX_DECLARED_MARGIN_US 250
#define WAKE_LEAD_SLOTS_DEFAULT UINT8_C(4)
#define WAKE_LEAD_SLOTS_MAX UINT8_C(32)
// TYPE=3(DM1)、FLOW=1、ARQN=0、SEQN=1
#define WAKE_PHASE3_HEADER UINT16_C(0x0298)
#define WAKE_PHASE3_LLID_DEFAULT UINT8_C(2)
// control structure +0x00のbit0~4へ指定するフレームフォーマット。24ではFLOW、ARQN、SEQNが0になる
#define WAKE_PHASE3_FRAME_FORMAT 24
// DM1のアクセスコード72us、ヘッダー54us、descriptor確認周期の半分100usを加算する
#define WAKE_RX_DETECT_LATENCY_US 226
// Switchの応答開始を基準とするフェーズ3の送信時刻
#define WAKE_PHASE3_TARGET_US 634
#define WAKE_PHASE3_BIAS_US 25
// フェーズ3と直後のページスキャン受信がスケジューラ上で重ならないようにする
#define WAKE_PHASE3_AIRTIME_US 400
// CSB停止前にフェーズ1の送信が再開しないよう、次の送信を1秒遅延させる
#define WAKE_PHASE3_QUIET_SLOTS UINT16_C(1600)
#define WAKE_CLOCK_SAMPLES 12

extern uint8_t g_wake_payload[SWITCH1_CSB_DATA_MAX];
extern uint8_t g_wake_target[6];
extern uint64_t g_wake_target_bch;
extern uint8_t g_wake_channel;
extern uint16_t g_wake_interval;
extern uint16_t g_wake_real_interval;
extern uint16_t g_wake_length;
extern volatile bool g_wake_started;
extern volatile int64_t g_wake_deadline_us;
extern uint8_t g_wake_variants[32][SWITCH1_CSB_DATA_MAX];
extern uint8_t g_wake_phase3[32][WAKE_PHASE3_LENGTH];
extern uint8_t g_wake_phase3_plain[WAKE_PHASE3_LENGTH];
extern bool g_wake_phase3_raw;
extern uint16_t g_wake_phase3_ms;
extern uint16_t g_wake_phase3_header;
extern uint16_t g_wake_phase3_pheader;
extern volatile uint32_t g_wake_phase3_body_writes;
extern bool g_wake_phase3_format_done;
extern uint16_t g_wake_phase1_header;
extern uint16_t g_wake_phase1_pheader;
extern bool g_wake_in_phase3;
extern uint32_t g_wake_phase3_bursts;
extern volatile bool g_wake_phase3_pending;
extern volatile uint32_t g_wake_phase3_schedule_slot;
extern volatile uint32_t g_wake_phase3_actual_slot;
extern volatile uint8_t g_wake_phase3_limit;
extern volatile uint8_t g_wake_phase3_step;
extern volatile uint8_t g_wake_phase3_rounds;
extern volatile uint8_t g_wake_phase3_rounds_done;
extern volatile uint16_t g_wake_phase3_bitoff;
extern volatile uint8_t g_wake_phase3_target_slot_adjustment;
extern volatile uint32_t g_wake_phase3_target_slot;
extern volatile uint8_t g_wake_phase3_gap;
extern volatile uint16_t g_wake_phase3_hold_ms;
extern volatile int64_t g_wake_phase3_done_us;
extern volatile int64_t g_wake_rx_detect_us;
extern volatile esp_err_t g_wake_rx_stop_err;
extern volatile uint16_t g_wake_shift_interval;
extern volatile uint16_t g_wake_shift_report;
extern bool g_wake_variant_enabled;
extern volatile uint8_t g_wake_variant_start;
extern uint32_t g_wake_burst_index;
extern wake_state_t g_wake_state;
extern int64_t g_wake_burst_start_us;
extern uint8_t g_wake_next_index;
extern uint16_t g_wake_payload_offset;
extern uint32_t g_wake_write_delay_us;
extern uint8_t g_wake_lead_slots;
extern bool g_wake_sweep;
extern int64_t g_wake_start_us;
extern uint32_t g_wake_clock_log[WAKE_CLOCK_SAMPLES];
extern uint8_t g_wake_clock_count;
extern uint16_t g_wake_gap_log[WAKE_CLOCK_SAMPLES];
extern uint8_t g_wake_gap_count;
extern uint16_t g_wake_offset_log[WAKE_CLOCK_SAMPLES];
extern uint8_t g_wake_index_log[WAKE_CLOCK_SAMPLES];
extern uint32_t g_wake_offset_changes;
extern uint32_t g_wake_forced_starts;
extern uint32_t g_wake_start_failures;
extern uint32_t g_wake_total_ms;
extern bool g_wake_receive_after;
extern uint8_t g_wake_rx_offset;
extern uint32_t g_wake_last_clkn;
extern uint32_t g_wake_clock_failures;
extern uint32_t g_wake_clock_failures_run;
extern uint32_t g_wake_write_failures;

void wake_try_receive(const uint8_t target[6], uint16_t interval);

#endif
