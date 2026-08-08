#include "wake_session.h"

#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd_api.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "link_state.h"
#include "swpro_pairing.h"
#include "host_sec_state.h"
#include "wakeup_code.h"
#include "wake_internal.h"

#define TAG "wake"

void wake_tx_task(void *argument)
{
    (void)argument;
    int64_t last_task_yield_us = 0;

    while (true) {
        if (g_wake_tx_active) {
            if (g_wake_stop_requested) {
                wake_finish_tx("UART stop requested");
                continue;
            }
            // 送信開始前にも期限を判定する。接続処理の完了イベントを受信できない場合に送信開始を待ち続けないよう、期限到達時は送信処理を終了する。
            if (esp_timer_get_time() >= g_wake_deadline_us) {
                const bool receive_after_stop =
                    g_wake_started && g_wake_receive_after;
                uint8_t target_address[6];
                uint16_t tx_interval = g_wake_interval;
                memcpy(target_address, g_wake_target, sizeof(target_address));
                wake_finish_tx(g_wake_started ? "TX deadline reached"
                                              : "start deadline reached");
                if (receive_after_stop) {
                    wake_try_receive(target_address, tx_interval);
                }
                continue;
            }
            if (!g_wake_started) {
                // 接続処理が進行中の場合はLT_ADDRの予約がstatus 0x0bで拒否されるため、接続処理の完了後に送信を開始する。
                if (g_connect_deadline_us != 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                const esp_err_t start_error = switch1_csb_raw_start(
                    g_wake_target, g_wake_target_bch,
                    g_wake_payload, g_wake_length,
                    g_wake_interval, g_wake_channel);
                if (start_error != ESP_OK) {
                    // 再接続のページが進行中の場合はLT_ADDRの予約またはCSBの有効化が拒否される。期限まで待機しながら送信開始を再試行する。
                    if (g_wake_start_failures < 3) {
                        ESP_LOGW(TAG, "Wake TX start failed; retrying: %s",
                                 esp_err_to_name(start_error));
                    }
                    g_wake_start_failures++;
                    vTaskDelay(pdMS_TO_TICKS(200));
                    continue;
                }
                /*
                 * HCIはパケットのスロット数+3未満の送信間隔を受け付けない。
                 * - DH5では8スロット(5ms)が下限である。
                 * - 送信間隔を6スロット(3.75ms)にするため、開始後にスケジュール要素の送信周期を直接変更する。
                 */
                if (g_wake_real_interval != 0) {
                    uint16_t current_interval = 0;
                    if (switch1_csb_tx_read_schedule(NULL, &current_interval) &&
                        current_interval != g_wake_real_interval) {
                        const esp_err_t interval_error = switch1_csb_tx_set_interval(
                            current_interval, g_wake_real_interval);
                        ESP_LOGI(TAG,
                                 "Wake TX: interval changed from %u to %u slots: %s",
                                 current_interval, g_wake_real_interval,
                                 esp_err_to_name(interval_error));
                        if (interval_error == ESP_OK) {
                            g_wake_interval = g_wake_real_interval;
                        }
                    }
                }
                g_wake_started = true;
                g_wake_start_us = esp_timer_get_time();
                g_wake_deadline_us =
                    g_wake_start_us + (int64_t)g_wake_total_ms * 1000;
                g_wake_burst_start_us = g_wake_start_us;
                last_task_yield_us = g_wake_start_us;
            }

            // 使用する配列要素はバースト番号ではなくクロックから毎回算出するため、区切りを取りこぼしても後続バーストとの対応がずれない。
            if (g_wake_variant_enabled) {
                const bool descriptor_present =
                    switch1_csb_descriptor_offset() != 0;
                const int64_t now_us = esp_timer_get_time();
                bool payload_written = false;

                /*
                 * - 応答を検出したら、フェーズ3が応答の634us後に送信されるよう送信イベントを移動する。
                 * - 応答と送信はともに6スロット周期であるため、目標時刻へ周期の倍数を加えても位置は変わらない。
                 * - 分解能は1スロット(625us)で、最大312usの誤差が生じる。
                 */
                if (g_wake_phase3_pending && g_wake_shift_interval == 0 &&
                    g_wake_rx_detect_us != 0 &&
                    g_wake_burst_start_us != 0 &&
                    now_us >= g_wake_rx_detect_us +
                               (int64_t)g_wake_phase3_hold_ms * 1000) {
                    /*
                     * - 受信時刻を読み出せない場合は送信間隔を7スロットに変更してフェーズ3を連続送信する。
                     * - 応答は6スロットごとに続くため、応答に対する送信位置が1バーストごとに1スロットずつ進む。
                     * - 6回で6通りの位置をすべて通過する。
                     */
                    int32_t phase3_interval_slots = g_wake_phase3_step;
                    uint32_t scheduled_tx_slot = UINT32_MAX;
                    const bool phase3_duration_ready =
                        switch1_csb_tx_set_duration(WAKE_PHASE3_AIRTIME_US) ==
                        ESP_OK;
                    esp_err_t reschedule_error;
                    if (g_wake_phase3_target_slot != 0) {
                        // 受信処理との競合で先送りされた既存イベントを、受信時刻から算出した直近の同位相へControllerタスク内で移動する。
                        phase3_interval_slots = g_wake_interval;
                        reschedule_error = switch1_csb_tx_reschedule(
                            g_wake_phase3_target_slot,
                            g_wake_phase3_bitoff,
                            g_wake_phase3_gap, &scheduled_tx_slot);
                    } else {
                        // 受信時刻を取得できない診断経路では、送信間隔を1回だけ変更して応答に対する送信位置を走査する。
                        uint32_t next_tx_slot = 0;
                        if (switch1_csb_tx_read_schedule(&next_tx_slot, NULL)) {
                            scheduled_tx_slot = next_tx_slot;
                        }
                        if (g_wake_phase3_bitoff != 0) {
                            switch1_csb_tx_set_bit_offset(
                                g_wake_phase3_bitoff);
                        }
                        reschedule_error = switch1_csb_tx_set_interval(
                            g_wake_interval, (uint16_t)phase3_interval_slots);
                        if (reschedule_error == ESP_OK &&
                            scheduled_tx_slot != UINT32_MAX) {
                            scheduled_tx_slot =
                                (scheduled_tx_slot + (uint32_t)phase3_interval_slots) &
                                UINT32_C(0x07ffffff);
                        }
                    }
                    g_wake_shift_report = (uint16_t)phase3_interval_slots;
                    if (reschedule_error == ESP_OK) {
                        g_wake_shift_interval = (uint16_t)phase3_interval_slots;
                        if (scheduled_tx_slot != UINT32_MAX) {
                            g_wake_phase3_schedule_slot = scheduled_tx_slot;
                            /*
                             * - フェーズ3の送信時刻が確定した時点で、直後のページスキャン受信も登録する。
                             * - 3スロット以上先でない場合はスケジューラが開始時刻を補正する。
                             * - ACLリンクがすでに成立している場合は同じlink 0を再作成することになるため、ACLリンクの直接登録を開始しない。
                             */
                            if (!g_direct_handoff_disabled &&
                                !g_acl_connected &&
                                phase3_duration_ready &&
                                !g_direct_handoff_started &&
                                g_direct_clock_delta_valid &&
                                g_direct_sync_valid &&
                                g_direct_lt_addr >= 1 &&
                                g_direct_lt_addr <= 7) {
                                const int64_t handoff_us = esp_timer_get_time();
                                g_direct_handoff_started = true;
                                g_direct_handoff_active = true;
                                g_direct_handoff_deadline_us =
                                    handoff_us + (int64_t)CONNECT_TIMEOUT_US;
                                g_direct_handoff_quiet_until_us =
                                    handoff_us + 40000;
                                g_direct_handoff_start_err =
                                    switch1_acl_handoff_start(
                                        g_wake_target,
                                        switch1_parity34(g_wake_target),
                                        g_direct_lt_addr,
                                        g_direct_clock_delta,
                                        g_direct_rx_clk,
                                        g_direct_rx_bitoff,
                                        g_wake_phase3_schedule_slot);
                                if (g_direct_handoff_start_err != ESP_OK) {
                                    g_direct_handoff_active = false;
                                    g_direct_handoff_deadline_us = 0;
                                }
                            }
                        }
                    } else {
                        g_wake_shift_report = 0;
                        if (reschedule_error == ESP_FAIL) {
                            ESP_LOGE(TAG,
                                     "Wake TX: failed to reschedule TX event");
                            wake_finish_tx("TX event rescheduling failed");
                            continue;
                        }
                        g_wake_shift_interval = UINT16_MAX;
                        ESP_LOGW(TAG,
                                 "Wake TX: cannot change interval: %s",
                                 esp_err_to_name(reschedule_error));
                    }
                }

                switch (g_wake_state) {
                case WAKE_BURST_WAIT_CLEAR:
                    /*
                     * - CSB開始直後はdescriptorがすでに非ゼロの場合がある。
                     * - descriptorが0になったことを1回確認してから、次の立ち上がりを検出する。
                     * - 0への遷移を取りこぼした場合は、送信間隔が経過した時点で次の状態へ移行する。
                     */
                    if (!descriptor_present ||
                        now_us - g_wake_burst_start_us >=
                            (int64_t)g_wake_interval * 625) {
                        g_wake_state = WAKE_BURST_WAIT_START;
                    }
                    break;

                case WAKE_BURST_WAIT_START:
                    /*
                     * - descriptorの立ち上がりを取りこぼすと、バーストは直前のバーストのペイロードで送信される。
                     * - 送信間隔が経過しても立ち上がりを検出できない場合は、経過時間をバースト境界として扱う。
                     * - 添字はその時点で読み出したクロックから算出するため、この経路でも対応関係は維持される。
                     */
                    if (!descriptor_present &&
                        now_us - g_wake_burst_start_us <
                            (int64_t)g_wake_interval * 625) {
                        break;
                    }
                    /*
                     * - フェーズ3を送信するバーストでは、経過時間を境界とする経路を使用しない。
                     * - descriptorがない状態で送信すると、ヘッダーとペイロードヘッダーを更新できず、長さとLLIDがフェーズ1の設定で送信される。
                     * - descriptorが非ゼロになるまで待機する。
                     */
                    if (!descriptor_present && g_wake_phase3_pending) {
                        break;
                    }
                    if (!descriptor_present) {
                        g_wake_forced_starts++;
                    }
                    /*
                     * - descriptorが非ゼロになった時点を、このバーストの送信開始時刻として扱う。
                     * - ROMがウェイクアップ用parityとチャンネルを再設定するため、ここで再適用する。
                     * - 再適用しない場合、アクセスコードが通常値に再設定される。
                     */
                    switch1_csb_wake_refresh();
                    g_wake_burst_start_us = now_us;
                    g_wake_burst_index++;
                    if (g_wake_phase1_header == 0) {
                        switch1_csb_read_descriptor(&g_wake_phase1_header,
                                                    &g_wake_phase1_pheader);
                    }
                    bool phase3_schedule_ready = g_wake_phase3_pending;
                    if (phase3_schedule_ready &&
                        g_wake_phase3_schedule_slot != UINT32_MAX) {
                        /*
                         * - 判定は目標時刻との一致ではなく、目標時刻以後とする。
                         * - 受信との競合で送信が1回取り消されると、次回時刻が目標時刻を越えて一致判定できなくなる。
                         * - 加算するスロット数は送信周期そのものなので、目標時刻を越えてもスロット位相は維持される。
                         * - 応答も同じ周期で続くため、目標時刻以後の最初のバーストでも送信位置は変わらない。
                         */
                        uint32_t current_slot = 0;
                        if (!switch1_csb_tx_read_schedule(&current_slot, NULL)) {
                            phase3_schedule_ready = false;
                        } else {
                            const uint32_t delta =
                                ((current_slot & UINT32_C(0x07ffffff)) -
                                 g_wake_phase3_schedule_slot) &
                                UINT32_C(0x07ffffff);
                            phase3_schedule_ready =
                                delta < UINT32_C(0x04000000);
                            if (phase3_schedule_ready) {
                                g_wake_phase3_actual_slot =
                                    current_slot & UINT32_C(0x07ffffff);
                            }
                        }
                    }
                    g_wake_in_phase3 =
                        phase3_schedule_ready ||
                        (g_wake_phase3_ms != 0 &&
                         now_us >= g_wake_deadline_us -
                                    (int64_t)g_wake_phase3_ms * 1000);
                    if (g_wake_in_phase3 && g_wake_phase3_raw) {
                        /*
                         * - ハードウェアはヘッダーとペイロードヘッダーも白色化する。
                         * - control structureで白色化を無効にし、書き込んだペイロードを白色化せず送信する。
                         * - ペイロードには白色化を相殺した内容ではなく、未処理の内容を書き込む。
                         */
                        switch1_csb_tx_set_whitening_disable(true);
                         // ここではUARTへ出力しない。フェーズ3を送信するまでの経路にUART出力を挿入すると、その処理時間だけ送信が遅れる。
                        if (!g_wake_phase3_format_done) {
                            g_wake_phase3_format_done = true;
                            (void)switch1_csb_tx_set_frame_format(
                                WAKE_PHASE3_FRAME_FORMAT, NULL);
                        }
                    }
                    if (g_wake_in_phase3) {
                        /*
                         * - ペイロードヘッダーはハードウェアが白色化するため、送信側で相殺できない。
                         * - 長さフィールドは実際に送信するバイト数を決める。
                         * - LLIDフィールドを1または2以外にすると、baseband側のASSERT_PARAM(8 0)でパニックする。
                         * - その結果、電波上ではペイロードヘッダーだけに白色化が適用される。
                         */
                        switch1_csb_write_descriptor(g_wake_phase3_header,
                                                     g_wake_phase3_pheader);
                        (void)switch1_csb_tx_set_duration(
                            WAKE_PHASE3_AIRTIME_US);
                        g_wake_phase3_bursts++;
                        /*
                         * - フェーズ3の送信完了後に送信間隔を変更する。途中では変更しない。
                         * - 変更した間隔を維持すると、応答に対する送信位置が1バーストごとに1スロットずつ進む。
                         */
                        if (g_wake_shift_interval != 0 &&
                            g_wake_shift_interval != UINT16_MAX &&
                            g_wake_phase3_bursts >=
                                g_wake_phase3_limit) {
                            /*
                             * - フェーズ3の後は送信せず、ページ開始までの21msはCSB送信を停止する。
                             * - CSBの停止にはHCIの往復で数msかかり、その間にフェーズ1が再送されるため、次回の送信時刻を十分先へ移動する。
                             */
                            if (switch1_csb_tx_set_interval(
                                    g_wake_shift_interval,
                                    WAKE_PHASE3_QUIET_SLOTS) == ESP_OK) {
                                g_wake_shift_interval =
                                    WAKE_PHASE3_QUIET_SLOTS;
                            }
                        }
                    } else if (g_wake_phase1_header != 0) {
                        switch1_csb_write_descriptor(g_wake_phase1_header,
                                                     g_wake_phase1_pheader);
                    }
                    {
                        const uint16_t offset = switch1_csb_payload_offset();
                        if (offset != 0) {
                            if (g_wake_payload_offset != 0 &&
                                g_wake_payload_offset != offset) {
                                g_wake_offset_changes++;
                            }
                            g_wake_payload_offset = offset;
                        }
                        if (g_wake_clock_count < WAKE_CLOCK_SAMPLES) {
                            g_wake_offset_log[g_wake_clock_count] = offset;
                        }
                    }
                    {
                        uint32_t clkn = 0;
                        if (switch1_csb_read_clkn(&clkn)) {
                            g_wake_clock_failures_run = 0;
                            /*
                             * - 書き込み対象の送信はdescriptor出現からlead_slots後に始まるため、CLKNにその分を加えて添字を算出する。
                             * - CLKNの単位は312.5usであり、1スロットは2カウントに相当する。
                             */
                            uint8_t basis = g_wake_variant_start;
                            if (g_wake_sweep) {
                                basis = (uint8_t)(((now_us - g_wake_start_us) /
                                                   WAKE_SWEEP_STEP_US) &
                                                  0x1F);
                            }
                            g_wake_next_index = wake_whiten_variant_index(
                                clkn + 2u * g_wake_lead_slots, basis);
                            g_wake_last_clkn = clkn;
                            if (g_wake_clock_count < WAKE_CLOCK_SAMPLES) {
                                g_wake_index_log[g_wake_clock_count] =
                                    g_wake_next_index;
                                g_wake_clock_log[g_wake_clock_count++] =
                                    clkn;
                            }
                            g_wake_state = WAKE_BURST_WAIT_GAP;
                        } else {
                             // サンプル完了ビットが連続してクリアされない場合は、レジスタの取り扱いが誤っている可能性があるため、送信を終了して報告する。
                            g_wake_clock_failures++;
                            if (++g_wake_clock_failures_run >= 4) {
                                ESP_LOGE(TAG,
                                         "Wake TX: unable to read the clock repeatedly");
                                wake_finish_tx("clock read failed");
                                continue;
                            }
                            g_wake_state = WAKE_BURST_WAIT_CLEAR;
                        }
                    }
                    break;

                case WAKE_BURST_WAIT_GAP: {
                    /*
                     * - 送信中のバーストへ書き込むと、そのバーストの途中で内容が変化する。
                     * - 電波を送信していない区間まで待機する。
                     * - descriptorが消える時刻は送信終了時刻と一致しないため、区間の判定にはdescriptorではなく経過時間を使用する。
                     */
                    const int64_t elapsed = now_us - g_wake_burst_start_us;
                    if (elapsed < g_wake_write_delay_us) {
                        break;
                    }
                    if (g_wake_gap_count < WAKE_CLOCK_SAMPLES) {
                        g_wake_gap_log[g_wake_gap_count++] =
                            (uint16_t)(elapsed > 65535 ? 65535 : elapsed);
                    }
                    /*
                     * - フラグがtrueになった時点でペイロード本体をフェーズ3の内容へ切り替える。
                     * - ヘッダーは現在のバーストに反映されるが、本体は次のバーストから反映される。
                     * - 先に書き込まない場合、1回だけ送信するフェーズ3の本体がフェーズ1の内容になる。
                     */
                    const bool phase3_body =
                        g_wake_in_phase3 || g_wake_phase3_pending;
                    const uint8_t *const body =
                        phase3_body
                            ? (g_wake_phase3_raw
                                   ? g_wake_phase3_plain
                                   : g_wake_phase3[g_wake_next_index])
                            : g_wake_variants[g_wake_next_index];
                    const uint16_t body_length =
                        phase3_body ? WAKE_PHASE3_LENGTH
                                     : g_wake_length;
                    if (switch1_csb_write_payload_at(g_wake_payload_offset,
                                                     body, body_length) !=
                        ESP_OK) {
                        g_wake_write_failures++;
                    } else if (phase3_body) {
                        g_wake_phase3_body_writes++;
                    }
                    /*
                     * - フェーズ3のバーストではdescriptorが解放されるまで待機する。
                     * - descriptorはControllerが送信を登録した後に解放されるため、待たずに状態を次の状態へ変更すると、解放前のdescriptorを次のバーストの立ち上がりと誤検出する。
                     * - 待機上限は4msとする。
                     */
                    if (g_wake_in_phase3 && g_wake_phase3_pending) {
                        const int64_t limit = esp_timer_get_time() + 4000;
                        while (esp_timer_get_time() < limit) {
                            uint16_t header = 0;
                            uint16_t pheader = 0;
                            if (switch1_csb_read_descriptor(&header,
                                                            &pheader) != ESP_OK) {
                                break;
                            }
                        }
                    }
                    /*
                     * - 応答を検出して切り替えたフェーズ3が、必要回数だけ送信される完了時刻を記録する。
                     * - ここで書き込んだ内容はdescriptor出現からlead_slots後に送信され、DM1の12バイトを送信する時間は約400usである。
                     * - UARTタスクはこの時刻の経過後に受信を停止し、ページ処理へ移行する。
                     */
                    if (g_wake_in_phase3 && g_wake_phase3_pending &&
                        g_wake_phase3_done_us == 0 &&
                        g_wake_phase3_bursts >= g_wake_phase3_limit) {
                        g_wake_phase3_rounds_done++;
                        if (g_wake_phase3_rounds_done <
                            g_wake_phase3_rounds) {
                             // 掃引中に応答の連続送信が終了する場合がある。フェーズ1の送信へ切り替えて次の応答列を待ち、応答を検出したら再度掃引する。
                            g_wake_phase3_pending = false;
                            g_wake_phase3_bursts = 0;
                            g_wake_shift_interval = 0;
                            g_wake_in_phase3 = false;
                            break;
                        }
                        /*
                         * - 送信完了時刻は、実際に登録されている送信スロットから算出する。
                         * - 応答検出時点で次の送信スロットが30スロット先にある場合もあり、lead_slots後と固定すると、送信前に送信タスクを終了することになる。
                         */
                        int64_t lead_us =
                            (int64_t)g_wake_lead_slots * 625;
                        if (g_wake_phase3_actual_slot != 0) {
                            const uint32_t now_slot =
                                (g_wake_last_clkn >> 1) &
                                UINT32_C(0x07ffffff);
                            const uint32_t ahead =
                                (g_wake_phase3_actual_slot - now_slot) &
                                UINT32_C(0x07ffffff);
                             // 応答列は約50msしか継続しないため、それより遠い送信スロット値は読み違いと判断して採用しない。
                            if (ahead >= g_wake_lead_slots && ahead <= 80) {
                                lead_us = (int64_t)ahead * 625;
                            }
                        }
                        g_wake_phase3_done_us =
                            g_wake_burst_start_us + lead_us +
                            WAKE_PHASE3_AIRTIME_US;
                        /*
                         * - フェーズ3の送信完了時刻に送信処理を終了する。
                         * - フェーズ3を1回送信した後に停止する。
                         * - 書き込んだ内容が送信されるのはlead_slots後であるため、直ちに停止するとフェーズ3自体が送信されない。
                         */
                        g_wake_deadline_us = g_wake_phase3_done_us;
                        /*
                         * - ACLリンクの直接登録を開始している場合は、送信完了後にウェイクアップ用受信を再登録しない。
                         * - 受信descriptorのリングバッファと添字をACL受信とCSB受信で共有しているため、ウェイクアップ受信を継続するとACLリンクの受信と競合する。
                         */
                        if (g_direct_handoff_started) {
                            g_wake_receive_after = false;
                        }
                    }
                    payload_written = true;
                    g_wake_state = WAKE_BURST_WAIT_CLEAR;
                    break;
                }

                default:
                    g_wake_state = WAKE_BURST_WAIT_CLEAR;
                    break;
                }

                /*
                 * - 書き込み直後は次のバーストまで時間があるため、他のタスクへCPUを譲る。
                 * - フェーズ3のフラグがtrueの間はCPUの実行権を他のタスクへ移さない。1ms待機すると、descriptorが非ゼロになる625usの区間を越えてバーストを取りこぼす。
                 */
                if (g_wake_phase3_pending && g_wake_phase3_done_us == 0) {
                    esp_rom_delay_us(20);
                } else if (payload_written || now_us - last_task_yield_us >= 10000) {
                    last_task_yield_us = now_us;
                    vTaskDelay(pdMS_TO_TICKS(1));
                } else {
                    esp_rom_delay_us(20);
                }
                continue;
            }

            const esp_err_t refresh_error = switch1_csb_wake_refresh();
            if (refresh_error != ESP_OK) {
                ESP_LOGE(TAG, "Wake TX: failed to reapply descriptor: %s",
                         esp_err_to_name(refresh_error));
                wake_finish_tx("descriptor reapply failed");
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
