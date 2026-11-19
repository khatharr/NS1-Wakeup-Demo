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

uint8_t g_wake_payload[SWITCH1_CSB_DATA_MAX];
uint8_t g_wake_target[6];
uint64_t g_wake_target_bch;
uint8_t g_wake_channel;
uint16_t g_wake_interval;
// Set a short transmit interval directly in the control structure when HCI cannot configure it; 0 means unchanged.
uint16_t g_wake_real_interval;
uint16_t g_wake_length;
volatile bool g_wake_tx_active;
volatile bool g_wake_started;
volatile int64_t g_wake_deadline_us;

uint8_t g_wake_variants[32][SWITCH1_CSB_DATA_MAX];
uint8_t g_wake_phase3[32][WAKE_PHASE3_LENGTH];
// Phase 3 payload used when controller whitening is disabled.
uint8_t g_wake_phase3_plain[WAKE_PHASE3_LENGTH];
bool g_wake_phase3_raw;
uint16_t g_wake_phase3_ms;
uint16_t g_wake_phase3_header;
uint16_t g_wake_phase3_pheader;
volatile uint32_t g_wake_phase3_body_writes;
bool g_wake_phase3_format_done;
// Phase 1 packet header and payload header restored after Phase 3 transmission.
uint16_t g_wake_phase1_header;
uint16_t g_wake_phase1_pheader;
bool g_wake_in_phase3;
uint32_t g_wake_phase3_bursts;
// Set by reception processing; the transmit task switches to Phase 3.
volatile bool g_wake_phase3_pending;
volatile uint32_t g_wake_phase3_schedule_slot = UINT32_MAX;
volatile uint32_t g_wake_phase3_actual_slot;
volatile uint8_t g_wake_phase3_limit;
// A transmit period coprime with 6 shifts the transmit position relative to the response by one slot per burst.
volatile uint8_t g_wake_phase3_step;
// Number of times to repeat transmit-position searching in case the response is detected just before it ends.
volatile uint8_t g_wake_phase3_rounds;
volatile uint8_t g_wake_phase3_rounds_done;
// Offset from the slot boundary (us).
volatile uint16_t g_wake_phase3_bitoff;
volatile uint8_t g_wake_phase3_target_slot_adjustment;
volatile uint32_t g_wake_phase3_target_slot;
volatile uint8_t g_wake_phase3_gap;
volatile uint16_t g_wake_phase3_hold_ms;
// Transmission completion timestamp from esp_timer_get_time(); 0 means not yet transmitted.
volatile int64_t g_wake_phase3_done_us;
volatile int64_t g_wake_rx_detect_us;
volatile esp_err_t g_wake_rx_stop_err;
volatile uint16_t g_wake_shift_interval;
volatile uint16_t g_wake_shift_report;
bool g_wake_variant_enabled;
volatile uint8_t g_wake_variant_start;
uint32_t g_wake_burst_index;
wake_state_t g_wake_state;
int64_t g_wake_burst_start_us;
uint8_t g_wake_next_index;
// Offset of the payload in shared memory, used while the descriptor is unavailable.
uint16_t g_wake_payload_offset;
// Delay between descriptor appearance and writing the payload.
uint32_t g_wake_write_delay_us;
// Number of slots between descriptor appearance and the corresponding transmission.
uint8_t g_wake_lead_slots;
bool g_wake_sweep;
int64_t g_wake_start_us;
uint32_t g_wake_clock_log[WAKE_CLOCK_SAMPLES];
uint8_t g_wake_clock_count;
uint16_t g_wake_gap_log[WAKE_CLOCK_SAMPLES];
uint8_t g_wake_gap_count;
uint16_t g_wake_offset_log[WAKE_CLOCK_SAMPLES];
uint8_t g_wake_index_log[WAKE_CLOCK_SAMPLES];
uint32_t g_wake_offset_changes;
uint32_t g_wake_forced_starts;
uint32_t g_wake_start_failures;
uint32_t g_wake_total_ms;
bool g_wake_receive_after;
// Difference between last transmit time and receive start time (in CLKN units).
uint8_t g_wake_rx_offset = WAKE_RX_OFFSET_DEFAULT;
uint32_t g_wake_last_clkn;
uint32_t g_wake_clock_failures;
uint32_t g_wake_clock_failures_run;
uint32_t g_wake_write_failures;

void wake_finish_tx(const char *reason)
{
    const bool was_active = g_wake_tx_active;
    // Do not delay resuming connection requests in case the Switch is already awake.
    if (was_active) {
        g_next_reconnect_us = 0;
    }
    if (g_wake_phase3_raw) {
        switch1_csb_tx_set_whitening_disable(false);
    }
    // Restore the transmit interval even if processing finished before updating the descriptor.
    if (g_wake_shift_interval != 0 &&
        g_wake_shift_interval != UINT16_MAX) {
        switch1_csb_tx_set_interval(g_wake_shift_interval,
                                    g_wake_interval);
        g_wake_shift_interval = 0;
    }
    if (g_wake_phase3_bitoff != 0) {
        switch1_csb_tx_set_bit_offset(0);
    }
    // Setting false before CSB is stopped would make the next start request conflict with cleanup.
    const esp_err_t stop_err = switch1_csb_wake_stop_wait();
    if (stop_err != ESP_OK) {
        ESP_LOGE(TAG, "Wake transmission: CSB stop did not complete: %s",
                 esp_err_to_name(stop_err));
        g_reconnect_paused = true;
        // After an HCI timeout, the transmit wait queue cannot be recovered through public APIs; reboot while preserving NVS.
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    }
    if (!g_direct_handoff_active) {
        g_reconnect_paused = false;
        g_next_reconnect_us = 0;
    }

    const int64_t quiet_until = g_direct_handoff_quiet_until_us;
    while (quiet_until != 0 && esp_timer_get_time() < quiet_until) {
        esp_rom_delay_us(100);
    }

    // Setting false too early would start direct ACL link registration and normal paging simultaneously.
    g_wake_started = false;
    g_wake_stop_requested = false;
    __asm__ __volatile__("memw" ::: "memory");
    g_wake_tx_active = false;

    if (was_active) {
        ESP_LOGI(TAG,
                 "Wake transmission stopped: %s, %lu bursts, "
                 "%lu clock read failures, %lu payload write failures, "
                 "%lu forced burst starts",
                 reason, (unsigned long)g_wake_burst_index,
                 (unsigned long)g_wake_clock_failures,
                 (unsigned long)g_wake_write_failures,
                 (unsigned long)g_wake_forced_starts);
        ESP_LOGI(TAG,
                 "  write delay %lu us, lead %u slots, "
                 "payload offset %04x, offset changes %lu",
                 (unsigned long)g_wake_write_delay_us,
                 g_wake_lead_slots, g_wake_payload_offset,
                 (unsigned long)g_wake_offset_changes);
        if (g_wake_phase3_ms != 0) {
            ESP_LOGI(TAG,
                     "  phase 3: last %u ms, %lu bursts, "
                     "header=%04x pheader=%04x "
                     "(phase 1 header=%04x pheader=%04x)",
                     g_wake_phase3_ms,
                     (unsigned long)g_wake_phase3_bursts,
                     g_wake_phase3_header, g_wake_phase3_pheader,
                     g_wake_phase1_header, g_wake_phase1_pheader);
        }
    }
}

void wake_cmd_start(const uint8_t *payload, uint8_t length)
{
    if (length < 3 || length > 28) {
        return;
    }
    if (g_direct_handoff_active) {
        ESP_LOGW(TAG, "Wake transmission: waiting for ACL link registration to finish");
        return;
    }
    // Starting CSB transmission during paging is rejected with status 0x0b.
    g_reconnect_disabled = true;
    g_reconnect_paused = true;
    g_next_reconnect_us = INT64_MAX;
    switch1_hci_page_cancel();
    if (g_connect_deadline_us != 0) {
        g_connect_deadline_us = 0;
        esp_bt_hid_device_disconnect();
    }
    const uint8_t channel = payload[0];
    const uint8_t seconds = payload[1];
    const uint8_t whiten_seed = payload[2];
    // payload[4]: payload length divided by 4; 0 means 339 bytes.
    const uint16_t body_length =
        (length >= 5 && payload[4] != 0)
            ? (uint16_t)(payload[4] * 4)
            : (uint16_t)SWITCH1_CSB_DATA_MAX;
    // payload[3]: transmit interval in slots; 0 means 64 slots.
    const uint16_t interval =
        (length >= 4 && payload[3] != 0) ? payload[3]
                                         : WAKE_CSB_RAW_INTERVAL;
    // payload[5]: value added to whitening-sequence index calculation; 0xFF uses payload[2] as a fixed value.
    const uint8_t variant_selector =
        (length >= 6) ? payload[5] : WAKE_VARIANT_START_NONE;
    const bool variant_sweep = variant_selector == WAKE_VARIANT_SWEEP;
    const bool variant_enabled =
        variant_selector != WAKE_VARIANT_START_NONE;
    // payload[6]: slots between descriptor appearance and transmission; 0 means four slots.
    // payload[7]: delay between descriptor appearance and writing, in 100 us units; 0 means automatic.
    const uint8_t lead_slots =
        (length >= 7 && payload[6] != 0) ? payload[6]
                                         : WAKE_LEAD_SLOTS_DEFAULT;
    const uint32_t write_delay_us =
        (length >= 8) ? (uint32_t)payload[7] * 100u : 0u;
    // payload[8]: unused.
    // payload[9]: Phase 3 transmit duration before stopping, in 10 ms units; 0 transmits only after response detection.
    // payload[10]: Phase 3 LLID; 0 means LLID 2.
    const uint16_t phase3_ms =
        (length >= 10) ? (uint16_t)payload[9] * 10u : 0u;
    const uint8_t phase3_llid =
        (length >= 11 && payload[10] != 0) ? payload[10]
                                           : WAKE_PHASE3_LLID_DEFAULT;
    // payload[11]: transmit duration in 10 ms units; if nonzero, overrides payload[1].
    const uint32_t total_ms =
        (length >= 12 && payload[11] != 0)
            ? (uint32_t)payload[11] * 10u
            : (uint32_t)seconds * 1000u;
    // payload[12]: if nonzero, receive using the wake-up access code after transmission stops.
    const bool receive_after = length >= 13 && payload[12] != 0;
    // payload[13]: receive start time offset; default if omitted.
    const uint8_t rx_offset =
        length >= 14 ? payload[13] : WAKE_RX_OFFSET_DEFAULT;

    // The controller enforces minimum intervals per packet type; only check for an even number of slots here.
    if (interval < 2 || (interval & 1) != 0) {
        ESP_LOGE(TAG, "Wake transmission: invalid interval: %u slots",
                 interval);
        return;
    }
    if (body_length > SWITCH1_CSB_DATA_MAX) {
        ESP_LOGE(TAG, "Wake transmission: payload length out of range: %u",
                 body_length);
        return;
    }
    if (channel > WAKE_CHANNEL_MAX) {
        ESP_LOGE(TAG, "Wake transmission: invalid channel: %u", channel);
        return;
    }
    if (seconds < WAKE_SECONDS_MIN || seconds > WAKE_SECONDS_MAX) {
        ESP_LOGE(TAG, "Wake transmission: duration out of range: %u seconds", seconds);
        return;
    }
    if (whiten_seed > WAKE_WHITEN_SEED_MAX &&
        whiten_seed != WAKE_WHITEN_SEED_NONE) {
        ESP_LOGE(TAG, "Wake transmission: whitening seed out of range: %u",
                 whiten_seed);
        return;
    }
    if (variant_enabled && !variant_sweep &&
        variant_selector > WAKE_VARIANT_START_MAX) {
        ESP_LOGE(TAG, "Wake transmission: variant start out of range: %u",
                 variant_selector);
        return;
    }
    if (lead_slots > WAKE_LEAD_SLOTS_MAX) {
        ESP_LOGE(TAG, "Wake transmission: lead slots out of range: %u",
                 lead_slots);
        return;
    }
    esp_bd_addr_t target_addr;
    if (!swpro_host_snapshot(target_addr)) {
        ESP_LOGW(TAG, "Wake transmission: target address unavailable");
        return;
    }
    if (g_wake_tx_active) {
        ESP_LOGW(TAG, "Wake transmission: already active");
        return;
    }
    if (g_hid_connected || g_acl_connected) {
        ESP_LOGW(TAG, "Wake transmission: not started while ACL/HID is connected");
        return;
    }

    if (phase3_llid > 3) {
        ESP_LOGE(TAG, "Wake transmission: phase 3 LLID out of range: %u",
                 phase3_llid);
        return;
    }
    const uint8_t *const local_addr = esp_bt_dev_get_address();
    if (local_addr == NULL) {
        ESP_LOGE(TAG, "Wake transmission: ESP32 BD_ADDR unavailable");
        return;
    }

    wake_build_phase1(target_addr, g_wake_payload, body_length);
    if (whiten_seed != WAKE_WHITEN_SEED_NONE) {
        wake_whiten_cancel(g_wake_payload, body_length,
                                  whiten_seed, WHITEN_SKIP_MULTISLOT);
    }
    wake_build_phase1_variants(target_addr, (uint8_t *)g_wake_variants,
                                SWITCH1_CSB_DATA_MAX, body_length);
    // payload[18] through [23]: BD_ADDR transmitted in Phase 3; default is the ESP32 BD_ADDR.
    const uint8_t *phase3_addr = local_addr;
    uint8_t phase3_addr_buf[6];
    if (length >= 24) {
        for (int byte_index = 0; byte_index < 6; ++byte_index) {
            phase3_addr_buf[byte_index] = payload[18 + byte_index];
        }
        phase3_addr = phase3_addr_buf;
        ESP_LOGI(TAG,
                 "Wake transmission: replacing phase 3 address with "
                 ESP_BD_ADDR_STR,
                 ESP_BD_ADDR_HEX(phase3_addr_buf));
    }
    wake_build_phase3(phase3_addr, g_wake_phase3,
                       g_wake_phase3_plain);
    g_wake_phase3_ms = phase3_ms;
    // For LLID 3, use LT_ADDR 0 and no whitening; for LLID 1 or 2, use LT_ADDR 1 and whitening cancellation.
    g_wake_phase3_raw =
        (length >= 15) ? (payload[14] != 0) : (phase3_llid == 3);
    g_wake_phase3_header =
        g_wake_phase3_raw
            ? WAKE_PHASE3_HEADER
            : (uint16_t)((WAKE_PHASE3_HEADER &
                          ~UINT16_C(0x0007)) | 1);
    // payload[24] and [25]: Phase 3 header word written to descriptor +0x02.
    if (length >= 26) {
        g_wake_phase3_header =
            (uint16_t)(payload[24] | ((uint16_t)payload[25] << 8));
        ESP_LOGI(TAG,
                 "Wake transmission: setting phase 3 header word to %04x",
                 g_wake_phase3_header);
    }
    g_wake_phase3_pheader =
        switch1_txphdr(WAKE_PHASE3_LENGTH,
                       (phase3_llid == 3) ? 1 : 0, phase3_llid);
    g_wake_phase1_header = 0;
    g_wake_phase1_pheader = 0;
    g_wake_phase3_format_done = false;
    g_wake_in_phase3 = false;
    g_wake_phase3_bursts = 0;
    g_wake_phase3_pending = false;
    g_wake_phase3_schedule_slot = UINT32_MAX;
    g_wake_phase3_limit = 1;
    g_wake_phase3_step = 7;
    g_wake_phase3_rounds = 1;
    g_wake_phase3_rounds_done = 0;
    g_wake_phase3_done_us = 0;
    g_wake_rx_detect_us = 0;
    g_wake_shift_interval = 0;
    g_wake_shift_report = 0;
    g_wake_phase3_target_slot = 0;
    g_wake_phase3_body_writes = 0;
    g_wake_length = body_length;
    memcpy(g_wake_target, target_addr, sizeof(g_wake_target));
    g_wake_target_bch = switch1_wakeup_parity34(target_addr);
    g_wake_channel = channel;
    g_wake_interval = interval;
    // payload[26]: transmit period set in control structure after starting; 0 means unchanged.
    g_wake_real_interval = (length >= 27) ? payload[26] : 0;
    // payload[27]: if nonzero, skip ACL registration and start paging 20 ms after Phase 3.
    g_direct_handoff_disabled = (length >= 28) && (payload[27] != 0);
    g_wake_variant_enabled = variant_enabled;
    g_wake_sweep = variant_sweep;
    g_wake_variant_start =
        (variant_enabled && !variant_sweep) ? variant_selector : 0;
    g_wake_burst_index = 0;
    g_wake_state = WAKE_BURST_WAIT_CLEAR;
    g_wake_payload_offset = 0;
    g_wake_next_index = 0;
    g_wake_clock_count = 0;
    g_wake_gap_count = 0;
    g_wake_clock_failures = 0;
    g_wake_clock_failures_run = 0;
    g_wake_write_failures = 0;
    g_wake_offset_changes = 0;
    g_wake_forced_starts = 0;
    g_wake_start_failures = 0;
    g_wake_lead_slots = lead_slots;
    // At 1 Mbps, packet bit count equals transmit time in us; update payload midway between transmissions.
    if (write_delay_us != 0) {
        g_wake_write_delay_us = write_delay_us;
    } else {
        const int32_t burst_us = (int32_t)(158u + 8u * body_length);
        const int32_t previous_end =
            625 * ((int32_t)lead_slots - (int32_t)interval) + burst_us;
        const int32_t next_start = 625 * (int32_t)lead_slots;
        const int32_t middle = (previous_end + next_start) / 2;
        g_wake_write_delay_us = middle > 0 ? (uint32_t)middle : 0;
    }
    g_wake_started = false;
    g_wake_stop_requested = false;
    g_reconnect_paused = true;
    switch1_hci_page_cancel();
    if (g_connect_deadline_us != 0) {
        esp_bt_hid_device_disconnect();
    }
    // Update the deadline before setting g_wake_tx_active so the transmit task cannot use the previous deadline.
    // Three seconds is used while waiting for CSB transmission to start; recalculate the deadline at transmission start.
    g_wake_total_ms = total_ms;
    g_wake_receive_after = receive_after;
    g_wake_rx_offset = rx_offset;
    g_hci_clock_offset = 0;
    g_direct_clock_delta = 0;
    g_direct_rx_clk = 0;
    g_direct_rx_bitoff = 0;
    g_direct_lt_addr = 0;
    g_direct_clock_delta_valid = false;
    g_direct_sync_valid = false;
    g_direct_handoff_started = false;
    g_direct_handoff_start_err = ESP_ERR_INVALID_STATE;
    g_direct_handoff_deadline_us = 0;
    g_direct_handoff_quiet_until_us = 0;
    g_wake_deadline_us =
        esp_timer_get_time() + (int64_t)total_ms * 1000 + 3000000;
    g_wake_tx_active = true;
    // If security state sm4 persists, btm_acl_created skips feature retrieval and L2CAP never leaves its pending state.
    // Clear only sm4 while keeping the link key.
    host_sec_state_report(target_addr, "before wake-up start");
    host_sec_state_reset_to_page_state(target_addr);
    ESP_LOGI(TAG,
             "Wake transmission start requested: target=" ESP_BD_ADDR_STR
             " channel=%u seconds=%u whiten_seed=%d variant=%s"
             " variant_start=%u connection_mode=%s",
             ESP_BD_ADDR_HEX(target_addr), channel, seconds,
             whiten_seed == WAKE_WHITEN_SEED_NONE
                 ? -1
                 : (int)whiten_seed,
             variant_sweep ? "sweep"
                           : (variant_enabled ? "enabled" : "disabled"),
             (unsigned)g_wake_variant_start,
              g_direct_handoff_disabled ? "page" : "ACL link registration");
}
