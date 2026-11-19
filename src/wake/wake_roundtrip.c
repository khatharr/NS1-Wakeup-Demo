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

void wake_try_receive(const uint8_t target[6], uint16_t interval)
{
    // Starting CSB reception before the transmit schedule element has been freed triggers an assertion in ld_csb_tx.c.
    // Wait 80 ms after stopping transmission before starting reception.
    vTaskDelay(pdMS_TO_TICKS(80));
    uint32_t clkn = 0;
    switch1_csb_read_clkn(&clkn);
    // Specify the expected receive time as an offset from the clock of the last burst.
    // The unit is a slot (625 us), matching the clock used by ld_csb_rx_sched.
    //
    // The Switch responds 2248 us after the ESP32 burst starts.
    // Actual phase is the remainder after division by Interval, so searching every phase from 0 to Interval-1 covers all possibilities.
    const uint32_t clk = clkn >> 1;
    uint32_t next = (g_wake_last_clkn >> 1) + g_wake_rx_offset;
    while ((int32_t)(next - clk) < 4) {
        next += interval;
    }
    const esp_err_t err = switch1_csb_rx_start(
        target, switch1_wakeup_parity34(target), interval, 0,
        next & UINT32_C(0x0fffffff), g_wake_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to start CSB receive: %s", esp_err_to_name(err));
        return;
    }
    // After enabling reception, update the transmit duration in the transmit schedule element.
    // If the duration set at startup is not retained, the transmit interval stretches from 3.75 ms to 15 ms.
    switch1_csb_tx_set_duration(
        (uint16_t)((g_wake_length > 183 ? 5 : 3) * 625 -
                   CSB_TX_DECLARED_MARGIN_US));
    ESP_LOGI(TAG,
             "CSB receive started: offset=%u slots last burst=%08x(slot %08x) "
             "scheduled receive=%08x current=%08x(slots)",
             (unsigned)g_wake_rx_offset, (unsigned)g_wake_last_clkn,
             (unsigned)(g_wake_last_clkn >> 1), (unsigned)next,
             (unsigned)clk);
    // Response bursts repeat every 318.75 ms. Wait two cycles because one can miss responses.
    const int64_t until = esp_timer_get_time() + WAKE_RX_WATCH_US;
    uint32_t received = 0;
    while (esp_timer_get_time() < until) {
        uint8_t addr[6];
        uint32_t clock = 0;
        uint8_t status = 0;
        uint8_t data[32];
        uint8_t length = 0;
        if (switch1_csb_rx_take(addr, &clock, &status, data, &length)) {
            received++;
            char text[32 * 3 + 1];
            int used = 0;
            for (uint8_t i = 0; i < length && used < (int)sizeof(text) - 3;
                 ++i) {
                used += snprintf(text + used, sizeof(text) - used, "%02x ",
                                 data[i]);
            }
            ESP_LOGI(TAG,
                     "Switch response received: clock=%08x status=%02x length=%u payload %s",
                     (unsigned)clock, status, length, text);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (received == 0) {
        ESP_LOGI(TAG, "CSB receive: no event detected in %dms (offset=%u)",
                 (int)(WAKE_RX_WATCH_US / 1000),
                 (unsigned)g_wake_rx_offset);
    }
    switch1_csb_rx_stop(target, interval);
}

// Stop CSB reception and restart it offset slots after the clock of the last burst.
//
// A start request with only Enable=1 succeeds only if CSB reception is already running.
// Because the schedule element is freed on supervision timeout, the second and later searches were rejected with status 0x0c.
// Inserting a stop step allows restarting from either state.
static esp_err_t wake_restart_rx(uint16_t interval, uint8_t offset)
{
    uint32_t clkn = 0;
    if (!switch1_csb_read_clkn(&clkn)) {
        return ESP_FAIL;
    }
    const uint32_t clk = clkn >> 1;
    uint32_t next = (g_wake_last_clkn >> 1) + offset;
    while ((int32_t)(next - clk) < 4) {
        next += interval;
    }
    return switch1_csb_rx_restart(interval, next & UINT32_C(0x0fffffff));
}

// Delay between CSB receive restarts; two HCI round trips are required, so do not issue them back-to-back.
#define RX_RESTART_INTERVAL_US 20000

static void take_phase2_capture(void)
{
    esp_err_t result = ESP_ERR_TIMEOUT;
    uint8_t lt_addr = 0;
    uint32_t clock_delta = 0;
    uint32_t rx_clk = 0;
    uint16_t rx_bitoff = 0;
    if (!switch1_phase2_capture_take(&result, &lt_addr, &clock_delta, &rx_clk,
                                     &rx_bitoff)) {
        return;
    }
    if (result != ESP_OK || lt_addr == 0 || lt_addr > 7) {
        ESP_LOGW(TAG, "Unable to register phase 2 payload with the controller: %s LT_ADDR=%u",
                 esp_err_to_name(result), lt_addr);
        return;
    }

    g_direct_lt_addr = lt_addr;
    g_direct_clock_delta = clock_delta;
    g_direct_rx_clk = rx_clk;
    g_direct_rx_bitoff = rx_bitoff;
    g_hci_clock_offset =
        (uint16_t)(HCI_CLOCK_OFFSET_VALID |
                   ((clock_delta >> 1) & UINT32_C(0x7fff)));
    g_direct_clock_delta_valid = true;
    __asm__ __volatile__("memw" ::: "memory");
    g_direct_sync_valid = true;
}

// Wait for the Switch response.
// Transmit Phase 3, stop transmission, and begin paging.
// Detect responses by directly reading receive descriptors.
// Waiting for HCI event round trips cannot meet the 634 us deadline from response to Phase 3.
// Because the transmit task switches to Phase 3, set a transmit-request flag and wait for completion.
static uint32_t wake_roundtrip_run(const uint8_t target[6], uint16_t interval,
                                  uint8_t trigger, uint8_t bursts,
                                  uint32_t timeout_ms, bool page_after,
                                  uint8_t base_offset, uint8_t positions,
                                  uint32_t dwell_ms)
{
    // Receptions already recorded in descriptors at monitoring start may be stale, occurring between CSB receive startup and logging its state.
    // Using them would delay Phase 3 far beyond the response timestamp; snapshot current values and trigger only on new receptions.
    switch1_csb_rx_poll_prime();
    switch1_csb_rx_event_reset();
    g_wake_phase3_limit = (bursts != 0) ? bursts : 1;
    g_wake_phase3_done_us = 0;
    g_wake_phase3_pending = false;
    g_wake_phase3_schedule_slot = UINT32_MAX;
    g_wake_phase3_actual_slot = 0;
    g_direct_sync_valid = false;
    g_wake_rx_stop_err = ESP_ERR_NOT_FOUND;

    uint32_t hits = 0;
    uint16_t first_status = 0;
    uint16_t first_header = 0;
    uint16_t first_pheader = 0;
    int64_t first_us = 0;
    int64_t detected_us = 0;
    const int64_t started = esp_timer_get_time();
    const int64_t until = started + (int64_t)timeout_ms * 1000;
    // The phase of the Switch response relative to the ESP32 transmission changes on each attempt.
    // Using one fixed receive start position makes some attempts succeed and others fail.
    // Search by changing the receive position sequentially within one attempt.
    const uint8_t position_count = (positions != 0) ? positions : 1;
    uint8_t used_offset = base_offset;
    uint8_t tried = 0;
    uint8_t move_failures = 0;
    bool direct_capture_complete = false;
    int64_t yielded_us = started;
    // Continually read transmit and receive schedule elements throughout monitoring.
    // Receive priority only increases with scheduling failures; there is no path that decreases it.
    // When priority reaches 11, transmission cannot be scheduled, so check for priority >= 11.
    switch1_csb_sched_t sched = {0};
    switch1_csb_sched_t rx_live = {0};
    uint8_t rx_priority_max = 0;
    uint8_t tx_priority_max = 0;
    uint32_t sched_samples = 0;
    uint32_t rx_freed_samples = 0;
    // Restart CSB reception; space out requests because they require two HCI round trips.
    int64_t rx_restart_next_us = 0;
    uint32_t rx_restart_calls = 0;
    uint32_t rx_restart_ok = 0;
    for (uint8_t p = 0; p < position_count; ++p) {
        if (p != 0) {
            used_offset = (uint8_t)(base_offset + p);
            const esp_err_t move_err = wake_restart_rx(interval, used_offset);
            if (move_err != ESP_OK) {
                // If the receive position cannot be changed, keep receiving at the current position and stop only the position search.
                ESP_LOGW(TAG, "Wake-up round trip: unable to move the receive position to %u: %s",
                         used_offset, esp_err_to_name(move_err));
                move_failures++;
                used_offset = (uint8_t)(used_offset - 1);
            }
            switch1_csb_rx_poll_prime();
        }
        tried++;
        const int64_t dwell_until =
            esp_timer_get_time() + (int64_t)dwell_ms * 1000;
        // Descriptor status persists until the descriptor is reused.
        // With a six-slot receive period, four descriptors make a 15 ms ring.
        // Polling every 200 us prevents missing updates.
        // Yield CPU time to other tasks every 10 ms to avoid starving them.
        while (esp_timer_get_time() < dwell_until &&
               esp_timer_get_time() < until) {
            switch1_csb_sched_read(&sched);
            sched_samples++;
            if (sched.rx_element == 0) {
                rx_freed_samples++;
                // Receive schedule elements are freed when supervision timeout expires (ld_csb_rx_sched 0x400391f0).
                // No receive events can be captured while the element is freed.
                // Restart CSB reception here because events cannot be captured after the element is freed.
                const int64_t now_us = esp_timer_get_time();
                if (now_us >= rx_restart_next_us) {
                    rx_restart_next_us = now_us + RX_RESTART_INTERVAL_US;
                    const esp_err_t restart_err =
                        wake_restart_rx(interval, used_offset);
                    rx_restart_calls++;
                    if (restart_err == ESP_OK) {
                        rx_restart_ok++;
                        switch1_csb_rx_poll_prime();
                    }
                }
            } else {
                rx_live = sched;
            }
            if (sched.rx_priority > rx_priority_max) {
                rx_priority_max = sched.rx_priority;
            }
            if (sched.tx_priority > tx_priority_max) {
                tx_priority_max = sched.tx_priority;
            }
            take_phase2_capture();
            const bool direct_response =
                g_direct_clock_delta_valid && g_direct_sync_valid &&
                g_direct_lt_addr >= 1 && g_direct_lt_addr <= 7;
            const bool direct_trigger = direct_response && trigger <= 1;
            // Use descriptor status only for recording the reception state.
            // Set the flag using the HCI event count.
            // Consecutive successful receptions store the same status, so counting status changes does not work.
            uint16_t status = 0;
            uint16_t header = 0;
            uint16_t pheader = 0;
            if (switch1_csb_rx_poll(&status, &header, &pheader)) {
                if (first_status == 0) {
                    first_status = status;
                    first_header = header;
                    first_pheader = pheader;
                }
            }
            // Set the flag when status changes to a value that passes the validity checks.
            // The beginning of a response burst train always changes from 6801 to 6800.
            // The HCI event clock field is the scheduled receive time, not the arrival time, so it cannot be used for positioning.
            if ((status != 0 || direct_trigger) &&
                !g_wake_phase3_pending) {
                if (first_us == 0) {
                    first_us = esp_timer_get_time();
                }
                if (direct_trigger ||
                    (g_direct_clock_delta_valid &&
                     switch1_csb_rx_event_count() + 1 >= trigger)) {
                    // The transmit interval cannot be changed while CSB reception is enabled.
                    // Receive events reserve six slots around candidate transmit times; choosing a transmit period not divisible by six makes scheduling fail and stops transmission.
                    // Stop CSB reception as soon as the response is received.
                    // Read the hardware-measured receive time before stopping CSB reception.
                    uint32_t rx_clk = g_direct_rx_clk;
                    uint16_t rx_bitoff = g_direct_rx_bitoff;
                    uint16_t rx_raw = g_direct_rx_bitoff;
                    const bool have_sync =
                        direct_response || switch1_csb_rx_read_sync(
                                               &rx_clk, &rx_bitoff, &rx_raw);
                    if (have_sync && g_direct_clock_delta_valid &&
                        !g_direct_sync_valid &&
                        g_direct_lt_addr >= 1 && g_direct_lt_addr <= 7) {
                        g_direct_rx_clk = rx_clk;
                        g_direct_rx_bitoff = rx_bitoff;
                        __asm__ __volatile__("memw" ::: "memory");
                        g_direct_sync_valid = true;
                    }
                    uint32_t tx_slot = 0;
                    uint16_t tx_interval_now = 0;
                    switch1_csb_tx_read_schedule(&tx_slot, &tx_interval_now);
                    const int64_t before = esp_timer_get_time();
                    // If stop is rejected with status=0x0C, transition to Phase 3 while CSB reception is still active.
                    // Stopping again would cost 17 ms in HCI round trips and shift Phase 3 away from the response burst; just record the result and continue.
                    // The next attempt's switch1_csb_rx_start handles stopping CSB reception.
                    g_wake_rx_stop_err = switch1_csb_rx_stop(target,
                                                                 interval);
                    detected_us = esp_timer_get_time();
                    // If a response timestamp was captured, schedule Phase 3 at timestamp + 634 us.
                    // The transmit period determines the slot, and control structure +0x02 determines the position within the slot.
                    // Responses repeat every six slots, so add multiples of six until the target is at least four slots after the next transmission.
                    uint32_t target_slot = 0;
                    uint16_t target_bitoff = 0;
                    const bool phase3_target_available =
                        have_sync && tx_interval_now == g_wake_interval;
                    if (phase3_target_available) {
                        // Apply correction based on measured values.
                        const uint32_t total = rx_bitoff +
                                               WAKE_PHASE3_TARGET_US -
                                               WAKE_PHASE3_BIAS_US;
                        target_slot = rx_clk + total / 625u +
                                      g_wake_phase3_target_slot_adjustment;
                        target_bitoff = (uint16_t)(total % 625u);
                        g_wake_phase3_target_slot = target_slot;
                        g_wake_phase3_bitoff = target_bitoff;
                    }
                    g_wake_rx_detect_us = detected_us;
                    // Write the Phase 3 body to shared memory before setting the flag.
                    // Changes to shared memory take effect starting with the next burst; writing from the transmit task would delay application by one burst.
                    if (g_wake_phase3_raw &&
                        switch1_csb_write_payload_at(
                            g_wake_payload_offset,
                            g_wake_phase3_plain,
                            WAKE_PHASE3_LENGTH) == ESP_OK) {
                        g_wake_phase3_body_writes++;
                    }
                    __asm__ __volatile__("memw" ::: "memory");
                    g_wake_phase3_pending = true;
                    __asm__ __volatile__("memw" ::: "memory");
                    // Set the flag, then rewrite the body.
                    // If the transmit task executes WAIT_GAP between the write and flag setting, it overwrites the body with Phase 1 data because the flag has not propagated yet.
                    // Rewriting after setting the flag ensures shared memory contains the correct body in either task interleaving.
                    if (g_wake_phase3_raw &&
                        switch1_csb_write_payload_at(
                            g_wake_payload_offset,
                            g_wake_phase3_plain,
                            WAKE_PHASE3_LENGTH) == ESP_OK) {
                        g_wake_phase3_body_writes++;
                    }
                    direct_capture_complete =
                        direct_trigger && g_wake_phase3_rounds == 1;
                    // Log only after setting the flag.
                    // UART output at 115200 bps takes about 10 ms per line, which would all add to the response-to-Phase-3 delay if logged before setting the flag.
                    // The transmit task runs on a different core, so logging here does not block Phase 3 transmission.
                    ESP_LOGI(TAG,
                             "  Response detected; stopping receive (%lldus) "
                             "sync=%s clk=%08x in-slot=%uus(raw %u) "
                             "next transmission=%08x interval=%u",
                             (long long)(detected_us - before),
                              have_sync ? "valid" : "invalid",
                             (unsigned)rx_clk, rx_bitoff, rx_raw,
                             (unsigned)tx_slot, tx_interval_now);
                    if (phase3_target_available) {
                        ESP_LOGI(TAG,
                                  "  Phase 3 target time: slot %08x offset %uus",
                                 (unsigned)target_slot, target_bitoff);
                    }
                }
            }
            hits = switch1_csb_rx_event_count();
            if (direct_response && hits == 0) {
                hits = 1;
            }
            // Once the ESP32 has captured the body and receive time, do not move the receive position until Phase 3 uses those values.
            if (direct_capture_complete || g_wake_phase3_done_us != 0) {
                break;
            }
            const int64_t now = esp_timer_get_time();
            if (now - yielded_us >= 10000) {
                yielded_us = now;
                vTaskDelay(1);
            } else {
                esp_rom_delay_us(200);
            }
        }
        if (direct_capture_complete || g_wake_phase3_done_us != 0 ||
            esp_timer_get_time() >= until) {
            break;
        }
    }
    // After setting the flag, wait for Phase 3 transmission to complete.
    // The transmit task selects the transmission time, so this function only waits.
    // Allow 400 ms for completion; transmit events can be skipped by the scheduler due to reception conflicts, delaying the next burst by tens of ms.
    while (g_wake_phase3_pending && esp_timer_get_time() < until + 400000) {
        const int64_t done = g_wake_phase3_done_us;
        if (done != 0 && esp_timer_get_time() >= done) {
            break;
        }
        vTaskDelay(1);
    }
    // Stop CSB reception before returning to the ROM callback.
    // Capturing the Phase 2 body uses the receive context of link 0 allocated by page scan, so its completed descriptor is not freed.
    // Returning to the ROM callback without freeing the completed descriptor causes an assertion because activity is not 13.
    if (switch1_csb_rx_is_active()) {
        switch1_csb_rx_stop(target, interval);
    }
    switch1_phase2_capture_cancel();
    take_phase2_capture();
    bool direct_path = false;
    bool direct_result_ready = false;
    esp_err_t direct_result = ESP_ERR_TIMEOUT;
    uint8_t direct_link = 0xff;
    const int64_t sent_us = g_wake_phase3_done_us;

    // After Phase 3, start paging before logging.
    // Logging earlier costs about 9 ms per line at 115200 bps and can delay paging by more than 100 ms across several lines.
    // Print the saved records after starting paging.
    int64_t t_page = 0;
    int64_t t_rx_stopped = 0;
    int64_t t_tx_stopped = 0;
    if (page_after && sent_us != 0) {
        const int64_t t0 = esp_timer_get_time();
        // Reception was stopped when the response was detected; stopping again would add 17 ms of HCI round trips.
        if (!g_wake_phase3_pending) {
            switch1_csb_rx_stop(target, interval);
        }
        t_rx_stopped = esp_timer_get_time();
        if (g_wake_tx_active) {
            g_wake_stop_requested = true;
        }
        const int64_t stop_until = esp_timer_get_time() + 1000000;
        while (g_wake_tx_active && esp_timer_get_time() < stop_until) {
            esp_rom_delay_us(100);
        }
        t_tx_stopped = esp_timer_get_time();
        __asm__ __volatile__("memw" ::: "memory");
        direct_path =
            g_direct_handoff_started && g_direct_handoff_start_err == ESP_OK;
        if (direct_path) {
            direct_result_ready =
                switch1_acl_handoff_status(&direct_result, &direct_link);
            if (direct_result_ready && direct_result != ESP_OK) {
                direct_path = false;
                g_direct_handoff_active = false;
                g_direct_handoff_deadline_us = 0;
            }
        }
        // Do not start paging if direct ACL connection initialization succeeded.
        // Paging is only a fallback used when the deadline expires.
        if (!direct_path) {
            const int64_t page_not_before = sent_us + 20000;
            while (esp_timer_get_time() < page_not_before) {
                esp_rom_delay_us(100);
            }
            try_reconnect();
            t_page = esp_timer_get_time();
        }
        (void)t0;
    }

    switch1_phase2_capture_report();
    if (g_direct_handoff_started) {
        if (g_direct_handoff_start_err != ESP_OK) {
            ESP_LOGW(TAG, "Unable to queue ACL link registration: %s",
                     esp_err_to_name(g_direct_handoff_start_err));
        } else if (!direct_result_ready) {
            ESP_LOGI(TAG, "ACL link registration: waiting for Controller processing");
        } else {
            ESP_LOGI(TAG, "ACL link registration: %s link=%u LT_ADDR=%u",
                     esp_err_to_name(direct_result), direct_link,
                     g_direct_lt_addr);
        }
        uint8_t lc_state_after = 0;
        uint8_t lc_flags = 0;
        if (switch1_lc_start_status(&lc_state_after, &lc_flags)) {
            ESP_LOGI(TAG,
                     "  r_lc_start: link state=%u(completion state is 5 when argument 2 is 1) "
                     "lc_env=%u ld_acl_env=%u llc_env=%u BD_ADDR registered=%u",
                     lc_state_after, (lc_flags & 1u) ? 1u : 0u,
                     (lc_flags & 2u) ? 1u : 0u, (lc_flags & 4u) ? 1u : 0u,
                     (lc_flags & 8u) ? 1u : 0u);
        } else {
            ESP_LOGI(TAG, "  r_lc_start: not called");
        }
    }
    ESP_LOGI(TAG, "  Phase 3 position search: %u/%u rounds", g_wake_phase3_rounds_done,
             g_wake_phase3_rounds);

    ESP_LOGI(TAG,
             "Wake-up round trip: responses=%lu phase3_pending=%s "
             "first response=%lldms header=%03x payload_header=%02x "
             "status=%04x receive_position=%u(attempts=%u move_failures=%u)",
             (unsigned long)hits,
              g_wake_phase3_pending ? "yes" : "no",
             (long long)((first_us != 0 ? first_us - started : 0) / 1000),
             first_header & 0x3FF, first_pheader & 0xFF, first_status,
             used_offset, tried, move_failures);
    ESP_LOGI(TAG,
             "  Schedule elements: RX max_priority=%u state=%u interval=%u slots "
             "RX reservation=%uus freed=%lu%%(%lu samples)/"
             "TX max_priority=%u TX reservation=%uus/RX stop %s",
             rx_priority_max, rx_live.rx_state, rx_live.rx_interval_slots,
             rx_live.rx_duration_us,
             sched_samples != 0
                 ? (unsigned long)(rx_freed_samples * 100u / sched_samples)
                 : 0ul,
             (unsigned long)sched_samples, tx_priority_max,
             sched.tx_duration_us, esp_err_to_name(g_wake_rx_stop_err));
    ESP_LOGI(TAG, "  CSB receive restarts: %lu calls, %lu succeeded",
             (unsigned long)rx_restart_calls, (unsigned long)rx_restart_ok);
    switch1_csb_rx_poll_report();
    if (sent_us != 0) {
        ESP_LOGI(TAG,
                 "  Phase 3 transmission: interval=%u slots bursts=%lu "
                 "(response-time offset changes by %d slots per burst)",
                 g_wake_shift_report,
                 (unsigned long)g_wake_phase3_bursts,
                 (int)g_wake_shift_report - (int)g_wake_interval);
        ESP_LOGI(TAG,
                 "  Phase 3 position: target=%08x scheduled=%08x actual=%08x "
                 "(%ld periods late at %u-slot interval)",
                 (unsigned)g_wake_phase3_target_slot,
                 (unsigned)g_wake_phase3_schedule_slot,
                 (unsigned)g_wake_phase3_actual_slot,
                 g_wake_interval,
                 g_wake_phase3_schedule_slot == UINT32_MAX
                     ? -1L
                     : (long)(((g_wake_phase3_actual_slot -
                                g_wake_phase3_schedule_slot) &
                               UINT32_C(0x07ffffff)) /
                              g_wake_interval));
        ESP_LOGI(TAG,
                 "  Phase 3 details: flag-to-completion=%lldus "
                 "bursts=%lu header=%04x pheader=%04x payload_writes=%lu "
                 "offset=%04x",
                 (long long)(sent_us - detected_us),
                 (long long)(sent_us - started),
                 (unsigned long)g_wake_phase3_bursts,
                 g_wake_phase3_header, g_wake_phase3_pheader,
                 (unsigned long)g_wake_phase3_body_writes,
                 g_wake_payload_offset);
    }

    if (t_page != 0) {
        ESP_LOGI(TAG,
                 "Wake-up round trip: started page processing %lldus after Phase 3 "
                 "(RX stop=%lldus TX stop=%lldus request=%lldus "
                 "Clock_Offset=%04x)",
                 (long long)(t_page - sent_us),
                 (long long)(t_rx_stopped - sent_us),
                 (long long)(t_tx_stopped - t_rx_stopped),
                 (long long)(t_page - t_tx_stopped), g_hci_clock_offset);
        return hits;
    }
    if (direct_path) {
        return hits;
    }
    switch1_csb_rx_stop(target, interval);
    if (!page_after) {
        return hits;
    }
    // The transmit task stops when Phase 3 transmission completes.
    // Request a stop only if the transmit task is still active.
    if (g_wake_tx_active) {
        g_wake_stop_requested = true;
    }
    // Wait until the transmit task stops.
    // g_wake_tx_active becomes false immediately after stop confirmation, so there is no need to wait for subsequent logging.
    const int64_t stop_until = esp_timer_get_time() + 3000000;
    while (g_wake_tx_active && esp_timer_get_time() < stop_until) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (g_wake_tx_active) {
        ESP_LOGW(TAG, "Wake-up round trip: transmission task did not stop");
        return hits;
    }
    // Do not start paging here while further attempts remain.
    // Active paging would cause LT_ADDR reservation and CSB enabling to fail with status 0x0b, making the next attempt fail to receive.
    // After the specified attempt count, CYCLE_PAGE_WAIT starts paging.
    if (g_cycle_enabled && g_cycle_wake_tries < CYCLE_WAKE_TRIES) {
        // Also prevent periodic reconnection from starting paging.
        // CMD_CSB_RX paging options clear the reconnection-disable flag, so restore it here or paging starts a few hundred ms later.
        g_reconnect_disabled = true;
        g_reconnect_paused = true;
        ESP_LOGI(TAG,
                 "Wake-up round trip: Phase 3 transmission failed; ending processing "
                 "(page processing deferred because another attempt remains)");
        return hits;
    }
    ESP_LOGI(TAG, "Wake-up round trip: Phase 3 transmission failed; starting page processing");
    try_reconnect();
    return hits;
}

// Handle UART command CMD_CSB_RX.
//
// Start CSB reception.
// On detecting a Switch response, move to Phase 3 transmission and direct ACL link initialization.
// Monitor reception settings and status because the transmit task handles the response-to-Phase-3 transition.
void wake_cmd_receive(const uint8_t *payload, uint8_t length)
{
    if (length < 1 || length > 23) {
        return;
    }
    g_hci_clock_offset = 0;
    g_direct_clock_delta = 0;
    g_direct_lt_addr = 0;
    g_direct_clock_delta_valid = false;
    g_direct_sync_valid = false;
    esp_bd_addr_t host;
    if (!swpro_host_snapshot(host)) {
        ESP_LOGW(TAG, "CSB receive: peer BD_ADDR is not configured");
        return;
    }
    const uint16_t interval =
        (length >= 2 && payload[1] >= 2) ? payload[1] : 6;
    uint16_t before[8];
    uint16_t after[8];
    switch1_csb_rx_dump(0, 8, before);
    esp_err_t err;
    if (payload[0] != 0) {
        uint32_t clkn = 0;
        const bool clkn_ok = switch1_csb_read_clkn(&clkn);
        if (length >= 8 && payload[7] != 0 && g_wake_started) {
            const esp_err_t capture_err = switch1_phase2_capture_start(
                host, switch1_wakeup_parity34(host));
            ESP_LOGI(TAG, "Phase 2 payload capture preparation: %s",
                     esp_err_to_name(capture_err));
        }
        if (length >= 22 && clkn_ok) {
            const uint32_t remote_slot =
                (uint32_t)payload[18] |
                ((uint32_t)payload[19] << 8) |
                ((uint32_t)payload[20] << 16) |
                ((uint32_t)payload[21] << 24);
            const uint32_t local_slot = clkn >> 1;
            const uint32_t d =
                (remote_slot - local_slot) & UINT32_C(0x07ffffff);
            g_direct_clock_delta = d;
            g_direct_clock_delta_valid = true;
            g_hci_clock_offset =
                (uint16_t)(HCI_CLOCK_OFFSET_VALID |
                           ((d >> 1) & UINT32_C(0x7fff)));
        }
        if (length >= 23 && payload[22] >= 1 && payload[22] <= 7) {
            g_direct_lt_addr = payload[22];
        }
        // Pass the receive start time in slots.
        // ld_csb_rx_sched derives receive phase from the remainder modulo Interval.
        // During wake-up transmission, specify an offset from the last burst's clock.
        // Responses track transmit bursts, so matching receive position is impossible without this offset.
        const uint32_t clk = clkn >> 1;
        uint32_t next = (clk + 16u) & UINT32_C(0x0fffffff);
        uint32_t offset = 0;
        if (length >= 3 && g_wake_started) {
            next = (g_wake_last_clkn >> 1) + payload[2];
            while ((int32_t)(next - clk) < 4) {
                next += interval;
            }
            // If payload[3] is nonzero, shift the piconet clock so the whitening seed becomes zero.
            // The seed is the low six bits of the slot clock, so align to a multiple of 64.
            // Receive timing is unchanged because phase includes Clock_Offset.
            if (length >= 4 && payload[3] != 0) {
                // payload[6] is a correction term; permit searching values 0-63 if the calculated offset does not match.
                const uint32_t correction =
                    (length >= 7) ? (payload[6] & 0x3Fu) : 0u;
                offset = (64u - (next & 0x3Fu) + correction) & 0x3Fu;
                next += offset;
            }
            next &= UINT32_C(0x0fffffff);
            ESP_LOGI(TAG,
                     "CSB receive start: offset=%u slots "
                     "last burst=%08x(slot %08x) "
                     "scheduled=%08x current=%08x correction=%u "
                     "whitening seed=%u(all in slots)",
                     payload[2], (unsigned)g_wake_last_clkn,
                     (unsigned)(g_wake_last_clkn >> 1),
                     (unsigned)next, (unsigned)clk, (unsigned)offset,
                     (unsigned)(next & 0x3Fu));
        }
        err = switch1_csb_rx_start(host, switch1_wakeup_parity34(host),
                                   interval, offset, next, 0);
        if (err == ESP_OK) {
            // Read the CSB receive schedule element immediately after starting.
            // ld_csb_rx_sched (0x400391f0) frees the element once elapsed time reaches supervision timeout; knowing the reference clock lets us calculate when.
            switch1_csb_sched_t started_sched;
            switch1_csb_sched_read(&started_sched);
            uint32_t clkn_now = 0;
            (void)switch1_csb_read_clkn(&clkn_now);
            ESP_LOGI(TAG,
                     "CSB receive: schedule_element=%08x state=%u interval=%u slots "
                     "priority=%u RX reservation=%uus supervision_base=%08x "
                     "supervision_timeout=%u slots current=%08x(slots)",
                     (unsigned)started_sched.rx_element,
                     started_sched.rx_state,
                     started_sched.rx_interval_slots,
                     started_sched.rx_priority,
                     started_sched.rx_duration_us,
                     (unsigned)started_sched.rx_sup_base,
                     started_sched.rx_sup_timeout_slots,
                     (unsigned)(clkn_now >> 1));
        }
        if (err != ESP_OK) {
            // If reception cannot start, wake_roundtrip_run is never called; cancel Phase 2 body capture here.
            // Without clearing capture state, the next attempt's switch1_phase2_capture_start fails with ESP_ERR_INVALID_STATE until reboot.
            switch1_phase2_capture_cancel();
        }
        // After enabling reception, update the transmit schedule element's duration to the actual transmit duration.
        // If the duration configured at startup is lost, the transmit interval stretches from 3.75 ms to 15 ms.
        if (err == ESP_OK) {
            switch1_csb_tx_set_duration(
                (uint16_t)((g_wake_length > 183 ? 5 : 3) * 625 -
                           CSB_TX_DECLARED_MARGIN_US));
        }
        // payload[4] specifies LT_ADDR; default to 0 if omitted.
        // 0 matches the Switch response but is also a broadcast address, so make it configurable to test whether the receiver rejects it.
        if (err == ESP_OK && length >= 5) {
            switch1_csb_rx_write_ltaddr(payload[4]);
        }
        // Values 2-28 in payload[3] are treated as CSB receive frame formats.
        // ACL frame formats use the processing path that stores received payloads.
        if (err == ESP_OK && length >= 4 && payload[3] >= 2) {
            switch1_csb_rx_set_format(payload[3]);
            ESP_LOGI(TAG, "CSB receive frame format set to %u",
                     payload[3]);
        }
    } else {
        err = switch1_csb_rx_stop(host, interval);
    }
    // Capture the control-structure snapshot here, but do not print it until after monitoring.
    // A Switch response can arrive immediately after enabling reception; printing three UART lines (about 25 ms) would delay detection and shift Phase 3 away from its response burst.
    switch1_csb_rx_dump(0, 8, after);

    if (payload[0] != 0 && err == ESP_OK) {
        // Detect reception by reading descriptors directly.
        // This detects reception even when no HCI event is produced, and unlike UART-based reads does not miss state transitions.
        // Poll without waiting so this can also run during transmission.
        // payload[5] sets monitoring time in 10 ms units; default is 600 ms.
        const uint32_t watch_ms =
            (length >= 6 && payload[5] != 0)
                ? (uint32_t)payload[5] * 10u
                : 600u;
        // If payload[7] is nonzero, after that many responses switch to Phase 3, stop transmitting, and start paging.
        // If zero, record reception state only and leave transmit settings unchanged.
        // payload[8] is the Phase 3 burst count; [9] enables paging; [10] is the number of receive positions; [11] is the dwell time at each position, in 10 ms units.
        const uint8_t trigger = (length >= 8) ? payload[7] : 0;
        if (trigger != 0 && g_wake_started) {
            const uint8_t bursts =
                (length >= 9 && payload[8] != 0) ? payload[8] : 1;
            const bool page_after =
                (length >= 10) ? payload[9] != 0 : true;
            g_reconnect_disabled = !page_after;
            const uint8_t positions =
                (length >= 11 && payload[10] != 0) ? payload[10] : 1;
            const uint32_t dwell_ms =
                (length >= 12 && payload[11] != 0)
                    ? (uint32_t)payload[11] * 10u
                    : watch_ms;
            // payload[12] is the transmit interval during Phase 3, in slots.
            // Choosing seven, coprime with the Phase 1 period of six, shifts the response-relative timing by one slot per burst.
            g_wake_phase3_step =
                (length >= 13 && payload[12] != 0) ? payload[12] : 7;
            // payload[13] is the number of position-search repetitions.
            g_wake_phase3_rounds =
                (length >= 14 && payload[13] != 0) ? payload[13] : 1;
            g_wake_phase3_rounds_done = 0;
            // payload[14] is the slot-boundary offset in units of 4 us.
            g_wake_phase3_bitoff =
                (length >= 15) ? (uint16_t)payload[14] * 4u : 0u;
            // payload[15] is a slot-based correction relative to the response timestamp.
            g_wake_phase3_target_slot_adjustment =
                (length >= 16) ? payload[15] : 0;
            // payload[16] is the number of slots between stopping Phase 1 and transmitting Phase 3.
            g_wake_phase3_gap =
                (length >= 17 && payload[16] != 0) ? payload[16] : 4;
            // payload[17] is the maximum time Phase 1 keeps transmitting at full speed after response reception before switching to Phase 3, in 10 ms units.
            g_wake_phase3_hold_ms =
                (length >= 18) ? (uint16_t)payload[17] * 10u : 0u;
            wake_roundtrip_run(host, interval, trigger, bursts, watch_ms,
                              page_after, payload[2], positions,
                              dwell_ms);
        } else {
            const uint32_t changes = switch1_csb_rx_watch(watch_ms);
            ESP_LOGI(TAG, "  Synchronization events detected during %lums: %lu",
                     (unsigned long)watch_ms, (unsigned long)changes);
        }
    }
    ESP_LOGI(TAG, "CSB receive %s: %s interval=%u slots",
             payload[0] != 0 ? "started" : "stopped", esp_err_to_name(err),
             interval);
    char text[16 * 5 + 1];
    int used = 0;
    for (int i = 0; i < 8; ++i) {
        used += snprintf(text + used, sizeof(text) - used, "%04x ",
                         before[i]);
    }
    ESP_LOGI(TAG, "  RX control structure before: %s", text);
    used = 0;
    for (int i = 0; i < 8; ++i) {
        used += snprintf(text + used, sizeof(text) - used, "%04x ",
                         after[i]);
    }
    ESP_LOGI(TAG, "  RX control structure after: %s", text);
}
