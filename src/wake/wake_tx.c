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
            // Also check the deadline before transmission begins. If no connection-complete event arrives, stop at the deadline instead of waiting indefinitely to start.
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
                // An active connection attempt causes LT_ADDR reservation to fail with status 0x0b; start transmission after connection processing completes.
                if (g_connect_deadline_us != 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                const esp_err_t start_error = switch1_csb_raw_start(
                    g_wake_target, g_wake_target_bch,
                    g_wake_payload, g_wake_length,
                    g_wake_interval, g_wake_channel);
                if (start_error != ESP_OK) {
                    // If reconnect paging is in progress, LT_ADDR reservation or CSB enabling is rejected; retry starting transmission until the deadline.
                    if (g_wake_start_failures < 3) {
                        ESP_LOGW(TAG, "Wake TX start failed; retrying: %s",
                                 esp_err_to_name(start_error));
                    }
                    g_wake_start_failures++;
                    vTaskDelay(pdMS_TO_TICKS(200));
                    continue;
                }
                /*
                 * HCIHCI rejects transmit intervals shorter than packet slot count + 3.
                 * - DH5For DH5, the minimum is eight slots (5 ms).
                 * - To use six slots (3.75 ms), directly change the schedule-element transmit period after starting.
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

            // Calculate the array index from the clock for each burst, not from the burst counter, so missing a boundary does not misalign later variants.
            if (g_wake_variant_enabled) {
                const bool descriptor_present =
                    switch1_csb_descriptor_offset() != 0;
                const int64_t now_us = esp_timer_get_time();
                bool payload_written = false;

                /*
                 * - On response detection, move the transmit event so Phase 3 is sent 634 us after the response.
                 * - Both response and transmission repeat every six slots, so adding multiples of that period does not change relative phase.
                 * - Resolution is one slot (625 us), allowing up to 312 us of error.
                 */
                if (g_wake_phase3_pending && g_wake_shift_interval == 0 &&
                    g_wake_rx_detect_us != 0 &&
                    g_wake_burst_start_us != 0 &&
                    now_us >= g_wake_rx_detect_us +
                               (int64_t)g_wake_phase3_hold_ms * 1000) {
                    /*
                     * - If the receive timestamp is unavailable, change the transmit interval to seven slots and send successive Phase 3 bursts.
                     * - Responses recur every six slots, so the transmit position relative to the response advances one slot per burst.
                     * - 6Six bursts traverse all six positions.
                     */
                    int32_t phase3_interval_slots = g_wake_phase3_step;
                    uint32_t scheduled_tx_slot = UINT32_MAX;
                    const bool phase3_duration_ready =
                        switch1_csb_tx_set_duration(WAKE_PHASE3_AIRTIME_US) ==
                        ESP_OK;
                    esp_err_t reschedule_error;
                    if (g_wake_phase3_target_slot != 0) {
                        // Move a pending event delayed by receive contention inside the controller task to the nearest matching phase calculated from the receive timestamp.
                        phase3_interval_slots = g_wake_interval;
                        reschedule_error = switch1_csb_tx_reschedule(
                            g_wake_phase3_target_slot,
                            g_wake_phase3_bitoff,
                            g_wake_phase3_gap, &scheduled_tx_slot);
                    } else {
                        // In the diagnostic path without a receive timestamp, change the transmit period once to scan transmit positions relative to the response.
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
                             * - When Phase 3 timing is fixed, also register page-scan reception immediately afterward.
                             * - 3If the event is not at least three slots ahead, the scheduler adjusts the start time.
                             * - ACLIf an ACL link is already established, do not start direct ACL registration because it would recreate the same link 0.
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
                     * - CSBImmediately after CSB starts, the descriptor may already be nonzero.
                     * - descriptorFirst observe it becoming zero, then detect the next rising edge.
                     * - 0If the transition to zero is missed, advance to the next state when the transmit interval elapses.
                     */
                    if (!descriptor_present ||
                        now_us - g_wake_burst_start_us >=
                            (int64_t)g_wake_interval * 625) {
                        g_wake_state = WAKE_BURST_WAIT_START;
                    }
                    break;

                case WAKE_BURST_WAIT_START:
                    /*
                     * - descriptorIf the descriptor rising edge is missed, that burst is sent with the previous burst's payload.
                     * - If no rising edge is detected within one transmit interval, treat elapsed time as the burst boundary.
                     * - The index is still calculated from the clock read at that point, preserving variant alignment.
                     */
                    if (!descriptor_present &&
                        now_us - g_wake_burst_start_us <
                            (int64_t)g_wake_interval * 625) {
                        break;
                    }
                    /*
                     * - Do not use the elapsed-time boundary path for a Phase 3 burst.
                     * - descriptorSending without a descriptor prevents updates to the header and payload header, so length and LLID would remain from Phase 1.
                     * - descriptorWait until the descriptor becomes nonzero.
                     */
                    if (!descriptor_present && g_wake_phase3_pending) {
                        break;
                    }
                    if (!descriptor_present) {
                        g_wake_forced_starts++;
                    }
                    /*
                     * - descriptorTreat the descriptor becoming nonzero as the start time for this burst.
                     * - ROMROM reapplies normal parity and channel, so reapply wake-up settings here.
                     * - Otherwise the access code is reset to its normal value.
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
                         * - Check at or after the target time, not only for exact equality.
                         * - If receive contention cancels a transmission, the next event passes the target and would not match an equality test.
                         * - The number of slots added is the transmit period, so slot phase is preserved even after the target.
                         * - Responses recur with the same period, so relative transmit position is unchanged on the first burst after the target.
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
                         * - Hardware also whitens the packet header and payload header.
                         * - control structureDisable whitening in the control structure to send the written payload unchanged.
                         * - Write the plain payload rather than the pre-whitened cancellation variant.
                         */
                        switch1_csb_tx_set_whitening_disable(true);
                         // Do not log to UART here; adding UART output before Phase 3 would delay transmission by the logging duration.
                        if (!g_wake_phase3_format_done) {
                            g_wake_phase3_format_done = true;
                            (void)switch1_csb_tx_set_frame_format(
                                WAKE_PHASE3_FRAME_FORMAT, NULL);
                        }
                    }
                    if (g_wake_in_phase3) {
                        /*
                         * - Hardware whitens the payload header, which cannot be cancelled from the transmitter side.
                         * - The length field determines the number of bytes actually transmitted.
                         * - LLIDSetting LLID to anything other than 1 or 2 triggers baseband ASSERT_PARAM(8 0).
                         * - As a result, only the payload header is whitened over the air.
                         */
                        switch1_csb_write_descriptor(g_wake_phase3_header,
                                                     g_wake_phase3_pheader);
                        (void)switch1_csb_tx_set_duration(
                            WAKE_PHASE3_AIRTIME_US);
                        g_wake_phase3_bursts++;
                        /*
                         * - Change the transmit interval after the Phase 3 burst finishes, never during it.
                         * - Keeping the changed interval advances the transmit position relative to the response by one slot per burst.
                         */
                        if (g_wake_shift_interval != 0 &&
                            g_wake_shift_interval != UINT16_MAX &&
                            g_wake_phase3_bursts >=
                                g_wake_phase3_limit) {
                            /*
                             * - After Phase 3, stop sending; keep CSB transmission stopped during the 21 ms before paging begins.
                             * - CSBStopping CSB takes several ms of HCI exchanges, during which Phase 1 may retransmit; move the next transmit time sufficiently far ahead.
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
                             * - The targeted transmission begins lead_slots after descriptor appearance; add that to CLKN when calculating the index.
                             * - CLKNCLKN is measured in 312.5 us units, so one slot corresponds to two counts.
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
                             // If the sample completion bit repeatedly fails to clear, the register handling may be wrong; stop transmission and report it.
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
                     * - Writing to a burst while it is transmitting changes its payload midway through that burst.
                     * - Wait until the radio is not transmitting.
                     * - descriptorDescriptor disappearance does not coincide with transmission end, so use elapsed time rather than the descriptor to identify the quiet period.
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
                     * - When the flag becomes true, switch the payload body to the Phase 3 content.
                     * - Header changes apply to the current burst, but body changes apply starting with the next burst.
                     * - Without writing beforehand, the single transmitted Phase 3 would contain the Phase 1 body.
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
                     * - During the Phase 3 burst, wait until the descriptor has been freed.
                     * - descriptorThe controller frees the descriptor after scheduling the transmission; advancing too early would mistake that existing descriptor for the next burst's rising edge.
                     * - Limit the wait to 4 ms.
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
                     * - Record the completion time after the required number of Phase 3 bursts triggered by a detected response.
                     * - Data written here is transmitted lead_slots after descriptor appearance; transmitting a 12-byte DM1 takes about 400 us.
                     * - UARTAfter that timestamp, the UART task stops reception and moves to paging.
                     */
                    if (g_wake_in_phase3 && g_wake_phase3_pending &&
                        g_wake_phase3_done_us == 0 &&
                        g_wake_phase3_bursts >= g_wake_phase3_limit) {
                        g_wake_phase3_rounds_done++;
                        if (g_wake_phase3_rounds_done <
                            g_wake_phase3_rounds) {
                             // The response burst train can end during a sweep; switch back to Phase 1, await the next response train, then sweep again when a response is detected.
                            g_wake_phase3_pending = false;
                            g_wake_phase3_bursts = 0;
                            g_wake_shift_interval = 0;
                            g_wake_in_phase3 = false;
                            break;
                        }
                        /*
                         * - Calculate the transmit-completion time from the actual scheduled transmit slot.
                         * - At response detection the next transmit slot can be 30 slots away; assuming lead_slots would terminate the transmit task before transmission.
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
                             // The response train lasts only about 50 ms, so treat a scheduled slot farther away as a misread and reject it.
                            if (ahead >= g_wake_lead_slots && ahead <= 80) {
                                lead_us = (int64_t)ahead * 625;
                            }
                        }
                        g_wake_phase3_done_us =
                            g_wake_burst_start_us + lead_us +
                            WAKE_PHASE3_AIRTIME_US;
                        /*
                         * - Terminate transmission at the Phase 3 completion timestamp.
                         * - Stop after sending Phase 3 once.
                         * - The written body is transmitted lead_slots later, so stopping immediately would prevent Phase 3 transmission.
                         */
                        g_wake_deadline_us = g_wake_phase3_done_us;
                        /*
                         * - ACLIf direct ACL registration has started, do not re-register wake-up reception after transmission completes.
                         * - ACL and CSB reception share the receive-descriptor ring buffer and index, so continuing wake-up reception would conflict with ACL reception.
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
                 * - Yield CPU time to other tasks after writing because there is time before the next burst.
                 * - While the Phase 3 flag is true, do not yield CPU; a 1 ms wait could miss the 625 us window in which the descriptor is nonzero.
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
