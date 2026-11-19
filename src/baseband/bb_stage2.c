/*
 * ROMReplace the ROM CSB receive callback before it releases the receive descriptor.
 * Use the Switch native clock in the received payload to calculate the clock difference when registering the ACL link.
 */

#include "baseband.h"

#include "bb_internal.h"
#include "bb_rom.h"

#include <stdbool.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "common/bt_target.h"
#include "hci/hci_layer.h"
#include "stack/bt_types.h"
#include "stack/hcidefs.h"
#include "stack/hcimsgs.h"

static const char *TAG = "wake_link";

static bool s_phase2_report_pending;
static phase2_prog_push_t s_phase2_prog_push_original;

phase2_capture_t g_phase2_capture;

static void phase2_prog_push(uint32_t timestamp, uint32_t activity,
                             uint32_t mode, uint32_t arg,
                             phase2_frame_cb_t callback);

void IRAM_ATTR phase2_restore_hook(void)
{
    phase2_prog_push_t const original_prog_push =
        s_phase2_prog_push_original;
    void **const ip_functions = *(void ***)PHASE2_ROM_IP_FUNCS_PTR_ADDR;
    if (original_prog_push != NULL && ip_functions != NULL &&
        ip_functions[PHASE2_FM_PROG_PUSH_INDEX] == (void *)phase2_prog_push) {
        ip_functions[PHASE2_FM_PROG_PUSH_INDEX] = (void *)original_prog_push;
    }
    __asm__ __volatile__("memw" ::: "memory");
    s_phase2_prog_push_original = NULL;
}

bool phase2_install_hook(void)
{
    void **const ip_functions = *(void ***)PHASE2_ROM_IP_FUNCS_PTR_ADDR;
    if (ip_functions == NULL) {
        return false;
    }
    phase2_prog_push_t const installed_prog_push =
        (phase2_prog_push_t)ip_functions[PHASE2_FM_PROG_PUSH_INDEX];
    if (installed_prog_push == phase2_prog_push) {
        return s_phase2_prog_push_original != NULL;
    }
    if (installed_prog_push == NULL ||
        (s_phase2_prog_push_original != NULL &&
         s_phase2_prog_push_original != installed_prog_push)) {
        return false;
    }
    s_phase2_prog_push_original = installed_prog_push;
    ip_functions[PHASE2_FM_PROG_PUSH_INDEX] = (void *)phase2_prog_push;
    __asm__ __volatile__("memw" ::: "memory");
    return true;
}

static void IRAM_ATTR phase2_csb_frame(uint32_t timestamp,
                                      uint32_t activity, uint32_t arg,
                                      uint32_t reason)
{
    (void)arg;
    if (g_phase2_capture.active) {
        const uint8_t reason_index = reason < 3 ? (uint8_t)reason : 3;
        g_phase2_capture.reasons[reason_index]++;
    }
    bool capture_completed = false;
    if (reason == 1 && g_phase2_capture.active &&
        activity == g_phase2_capture.link) {
        const uint8_t descriptor_index = ld_env[0x1aa] & 3;
        uint16_t descriptor_snapshot[PHASE2_DESC_WORDS];
        const volatile uint16_t *const descriptor =
            (const volatile uint16_t *)(PHASE2_DESC_ADDR +
                                        2u * PHASE2_DESC_WORDS *
                                            descriptor_index);
        for (uint8_t i = 0; i < PHASE2_DESC_WORDS; ++i) {
            descriptor_snapshot[i] = descriptor[i];
            g_phase2_capture.desc[i] = descriptor_snapshot[i];
        }
        g_phase2_capture.desc_index = descriptor_index;
        if ((descriptor_snapshot[0] & PHASE2_DESC_DONE) != 0 &&
            (descriptor_snapshot[1] & PHASE2_STATUS_FAIL_MASK) == 0 &&
            ((descriptor_snapshot[1] >> 11) & UINT16_C(0x1f)) == activity &&
            (descriptor_snapshot[2] & UINT16_C(0x03ff)) == PHASE2_HEADER &&
            descriptor_snapshot[3] == PHASE2_PHEADER &&
            descriptor_snapshot[6] != 0 &&
            descriptor_snapshot[6] <= EM_SIZE - PHASE2_LENGTH) {
            volatile uint16_t *const control_structure =
                (volatile uint16_t *)(ACL_CS_BASE +
                                      ACL_CS_STRIDE * activity);
            const uint16_t rx_clock_high =
                control_structure[PHASE2_RXCLK_HI_WORD];
            const volatile uint8_t *const payload_source =
                (const volatile uint8_t *)(EM_BASE + descriptor_snapshot[6]);
            bool payload_matches =
                rx_clock_high < UINT16_C(0x0800) && (payload_source[0] & 7) != 0;
            for (uint8_t i = 0; i < PHASE2_LENGTH; ++i) {
                g_phase2_capture.payload[i] = payload_source[i];
            }
            for (uint8_t i = 0; payload_matches && i < 6; ++i) {
                payload_matches = g_phase2_capture.payload[1 + i] ==
                                  g_phase2_capture.target[5 - i];
            }
            if (payload_matches) {
                uint32_t received_clock =
                    ((uint32_t)rx_clock_high << 16) |
                    control_structure[PHASE2_RXCLK_LO_WORD];
                int32_t received_bit_offset =
                    (int32_t)(control_structure[PHASE2_RXBITOFF_WORD] &
                              UINT16_C(0x03ff)) -
                    PHASE2_RXBITOFF_CORRECTION;
                if (received_bit_offset < 0) {
                    received_bit_offset += 625;
                    received_clock =
                        (received_clock - 1) & UINT32_C(0x07ffffff);
                }
                g_phase2_capture.payload_offset = descriptor_snapshot[6];
                g_phase2_capture.rx_clk = received_clock;
                g_phase2_capture.rx_bitoff = (uint16_t)received_bit_offset;
                g_phase2_capture.active = false;
                g_phase2_capture.result = ESP_OK;
                __asm__ __volatile__("memw" ::: "memory");
                g_phase2_capture.state = PHASE2_CAPTURE_DONE;
                capture_completed = true;
            }
        }
    }

    /*
     * ROMPreserve cleanup done by ROM ld_csb_rx_frm_cbk (0x400393a4) with reason=1.
     * - If schedule element +0x3E is PHASE2_ELT_STATE_STOPPING (2), receive cleanup frees the EA schedule element.
     * - On synchronization detection, write the registration timestamp to EA schedule element +0x2C.
     * - +0x2CUsing that +0x2C timestamp, the receive window reservation is computed as 2*drift+1923 us.
     * - timestampWithout updating timestamp, the receive-window reservation continually grows from 1933 to 3037 us.
     */
    if (reason == 1) {
        volatile uint8_t *const schedule_element =
            *(volatile uint8_t *volatile *)CSB_RX_ENV_ADDR;
        if (schedule_element != NULL) {
            (void)r_ea_elt_remove((void *)schedule_element);
            if (schedule_element[PHASE2_ELT_STATE] ==
                PHASE2_ELT_STATE_STOPPING) {
                ((phase2_csb_rx_end_t)PHASE2_ROM_CSB_RX_END_ADDR)();
                if (capture_completed) {
                    phase2_restore_hook();
                }
                return;
            }
            const volatile uint16_t *const control_structure =
                (const volatile uint16_t *)PHASE2_CSB_CS_ADDR;
            if ((control_structure[PHASE2_RXBITOFF_WORD] &
                 UINT16_C(0x8000)) != 0) {
                *(volatile uint32_t *)(schedule_element + PHASE2_ELT_SUP_BASE) =
                    timestamp;
            }
        }
    }
    /*
     * Also schedule the next frame by calling ROM's CSB receive scheduler from phase2_csb_frame.
     * - Use the page-scan link while capturing the payload.
     * - Resume ordinary CSB reception after capture completes.
     */
    ((phase2_csb_rx_sched_t)PHASE2_ROM_CSB_RX_SCHED_ADDR)(0);
    if (capture_completed) {
        phase2_restore_hook();
    }
}

static void IRAM_ATTR pscan_rx_frame(uint32_t timestamp,
                                         uint32_t activity, uint32_t arg,
                                         uint32_t reason)
{
    if (reason == 1) {
        /*
         * Frame ManagerFrame Manager constraint:
         * - callbackApply the preliminary correction only when the callback equals ROM page-scan callback (PHASE2_PSCAN_CALLBACK_ADDR).
         * - For the replacement callback, perform the same correction as ROM's page-scan callback in pscan_rx_frame.
         */
        ld_pscan_assert_err_handle();

        uint8_t *const page_scan_env = ld_pscan_env;
        if (g_acl_handoff.pscan_rx_active && page_scan_env != NULL &&
            activity == g_acl_handoff.reserved_link &&
            page_scan_env[0x3d] == (uint8_t)activity) {
            const uint8_t descriptor_index = ld_env[0x1aa] & 3;
            volatile uint16_t *const descriptor =
                (volatile uint16_t *)(PHASE2_DESC_ADDR +
                                      2u * PHASE2_DESC_WORDS * descriptor_index);
            const uint16_t descriptor_status = descriptor[1];
            const uint16_t descriptor_header = descriptor[2];
            const uint8_t packet_type =
                (uint8_t)((descriptor_header >> 3) & 0x0f);
            if ((descriptor[0] & PHASE2_DESC_DONE) != 0 &&
                (descriptor_status & PHASE2_STATUS_FAIL_MASK) == 0 &&
                ((descriptor_status >> 11) & UINT16_C(0x1f)) == activity &&
                (descriptor_header & 7) == g_acl_handoff.lt_addr &&
                (packet_type == 1 || packet_type == 3)) {
                g_acl_handoff.pscan_rx_status = descriptor_status;
                g_acl_handoff.pscan_rx_header = descriptor_header;
                g_acl_handoff.pscan_rx_pheader = descriptor[3];
                if (packet_type == 3) {
                    /*
                     * ROMROM's page-scan Phase 2 recognizes only TYPE=1 as an established connection.
                     * - PHASE2_STATUS_FAIL_MASK=0Change the TYPE field of a DM1 with PHASE2_STATUS_FAIL_MASK=0 to POLL.
                     * - TYPEPass the descriptor with TYPE changed to POLL to the ROM page-scan callback immediately afterward.
                     */
                    descriptor[2] =
                        (uint16_t)((descriptor_header & ~UINT16_C(0x0078)) |
                                   UINT16_C(0x0008));
                    g_acl_handoff.pscan_rx_dm1 = true;
                }
            }
        }
    }

    ((phase2_frame_cb_t)PHASE2_PSCAN_CALLBACK_ADDR)(timestamp, activity, arg,
                                                    reason);
    g_acl_handoff.pscan_rx_active = false;
    __asm__ __volatile__("memw" ::: "memory");
    phase2_restore_hook();
}

static void IRAM_ATTR phase2_prog_push(uint32_t timestamp,
                                       uint32_t activity, uint32_t mode,
                                       uint32_t arg,
                                       phase2_frame_cb_t callback)
{
    phase2_prog_push_t const original_prog_push = s_phase2_prog_push_original;
    if (original_prog_push == NULL) {
        return;
    }
    if (g_phase2_capture.active) {
        g_phase2_capture.push_count++;
        g_phase2_capture.last_activity = activity;
        g_phase2_capture.last_callback = (uintptr_t)callback;
        if (activity == PHASE2_CSB_ACTIVITY &&
            (uintptr_t)callback == PHASE2_CSB_CALLBACK_ADDR) {
            const uint8_t capture_link = g_phase2_capture.link;
            volatile uint16_t *const control_structure =
                (volatile uint16_t *)(ACL_CS_BASE + ACL_CS_STRIDE * capture_link);
            if (!g_phase2_capture.tail_first_taken) {
                for (uint8_t i = 0; i < 4; ++i) {
                    g_phase2_capture.tail_first[i] = control_structure[47 + i];
                }
                g_phase2_capture.tail_first_taken = true;
            }
            for (uint8_t i = 0; i < 4; ++i) {
                g_phase2_capture.tail_last[i] = control_structure[47 + i];
            }
            control_structure[0] =
                (control_structure[0] & ~UINT16_C(0x001f)) |
                PHASE2_DIRECT_FORMAT;
            control_structure[4] =
                (uint16_t)((control_structure[4] &
                            ~(CS_LTADDR_MASK | PHASE2_LINKCNTL_EDR |
                              PHASE2_LINKCNTL_AFH)) |
                           CS_LINKCNTL_WHITENING_DSB);
            control_structure[5] =
                (uint16_t)(g_phase2_capture.target[5] |
                           ((uint16_t)g_phase2_capture.target[4] << 8));
            control_structure[6] =
                (uint16_t)(g_phase2_capture.target[3] |
                           ((uint16_t)g_phase2_capture.target[2] << 8));
            control_structure[7] =
                (uint16_t)(g_phase2_capture.target[1] |
                           ((uint16_t)g_phase2_capture.target[0] << 8));
            control_structure[8] =
                (uint16_t)(g_phase2_capture.bch34 & UINT64_C(0xffff));
            control_structure[9] =
                (uint16_t)((g_phase2_capture.bch34 >> 16) & UINT64_C(0xffff));
            control_structure[10] =
                (uint16_t)(CS_BCH_RX_FLAGS |
                           ((g_phase2_capture.bch34 >> 32) & 3));
            control_structure[11] &=
                (uint16_t)~(CS_HOPCNTL_FH_EN | CS_HOPCNTL_CHANNEL_MASK);
            for (uint8_t i = 0; i < PHASE2_CS_WORDS; ++i) {
                g_phase2_capture.control[i] = control_structure[i];
            }
            g_phase2_capture.last_format =
                control_structure[0] & UINT16_C(0x001f);
            g_phase2_capture.programmed++;
            original_prog_push(timestamp, capture_link, 0, 0, phase2_csb_frame);
        } else {
            volatile uint16_t *const control_structure =
                (volatile uint16_t *)PHASE2_CSB_CS_ADDR;
            g_phase2_capture.last_format =
                control_structure[0] & UINT16_C(0x001f);
            original_prog_push(timestamp, activity, mode, arg, callback);
        }
        return;
    }
    if (g_acl_handoff.pscan_rx_active &&
        activity == g_acl_handoff.reserved_link &&
        (uintptr_t)callback == PHASE2_PSCAN_CALLBACK_ADDR) {
        original_prog_push(timestamp, activity, mode, arg, pscan_rx_frame);
        return;
    }
    original_prog_push(timestamp, activity, mode, arg, callback);
}

static void phase2_capture_start_controller(void *arg)
{
    phase2_capture_t *const capture = arg;
    if (capture->state != PHASE2_CAPTURE_STARTING) {
        return;
    }

    if (!phase2_install_hook()) {
        capture->result = ESP_ERR_INVALID_STATE;
        capture->state = PHASE2_CAPTURE_DONE;
        return;
    }

    uint8_t *const page_scan_env = ld_pscan_env;
    if (page_scan_env == NULL || page_scan_env[0x3d] >= ACL_LINK_COUNT ||
        lm_env[60 + 9 * page_scan_env[0x3d]] != 2 ||
        !acl_link_context_clear(page_scan_env[0x3d])) {
        phase2_restore_hook();
        capture->result = ESP_ERR_INVALID_STATE;
        capture->state = PHASE2_CAPTURE_DONE;
        return;
    }

    capture->link = page_scan_env[0x3d];
    g_acl_handoff.reserved_link = capture->link;
    g_acl_handoff.prepare_result = ESP_OK;
    g_acl_handoff.prepare_state = ACL_PREPARE_DONE;
    /*
     * Reuse the ACL link and page_scan_env allocated by page scan on TYPE=3 received just after Phase 3.
     * - CSBRemove only the EA schedule element that conflicts with CSB transmission via r_ea_elt_remove.
     */
    (void)r_ea_elt_remove(page_scan_env);
    page_scan_env[0x3e] = 0;
    page_scan_env[0x3f] = 0;
    capture->active = true;
    capture->state = PHASE2_CAPTURE_WAITING;
    __asm__ __volatile__("memw" ::: "memory");
}

esp_err_t switch1_phase2_capture_start(const uint8_t bd_addr[6], uint64_t bch34)
{
    if (bd_addr == NULL || g_phase2_capture.state == PHASE2_CAPTURE_STARTING ||
        g_phase2_capture.state == PHASE2_CAPTURE_WAITING ||
        g_acl_handoff.state == ACL_HANDOFF_PENDING) {
        return ESP_ERR_INVALID_STATE;
    }

    memcpy(g_phase2_capture.target, bd_addr, sizeof(g_phase2_capture.target));
    g_phase2_capture.bch34 = bch34;
    g_phase2_capture.active = false;
    g_phase2_capture.result = ESP_ERR_TIMEOUT;
    g_phase2_capture.link = UINT8_MAX;
    g_phase2_capture.desc_index = UINT8_MAX;
    g_phase2_capture.programmed = 0;
    g_phase2_capture.push_count = 0;
    g_phase2_capture.last_activity = 0;
    g_phase2_capture.last_callback = 0;
    g_phase2_capture.last_format = 0;
    memset((void *)g_phase2_capture.reasons, 0,
           sizeof(g_phase2_capture.reasons));
    memset(g_phase2_capture.desc, 0, sizeof(g_phase2_capture.desc));
    memset(g_phase2_capture.control, 0, sizeof(g_phase2_capture.control));
    g_phase2_capture.payload_offset = 0;
    memset(g_phase2_capture.payload, 0, sizeof(g_phase2_capture.payload));
    g_acl_handoff.reserved_link = UINT8_MAX;
    g_acl_handoff.prepare_result = ESP_ERR_TIMEOUT;
    g_acl_handoff.prepare_state = ACL_PREPARE_IDLE;
    g_phase2_capture.state = PHASE2_CAPTURE_STARTING;
    __asm__ __volatile__("memw" ::: "memory");

    if (btdm_dispatch_work_to_controller(phase2_capture_start_controller,
                                         &g_phase2_capture, true) != 0) {
        g_phase2_capture.result = ESP_FAIL;
        g_phase2_capture.state = PHASE2_CAPTURE_DONE;
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void phase2_capture_cancel_controller(void *arg)
{
    phase2_capture_t *const capture = arg;
    capture->active = false;
    phase2_restore_hook();
    __asm__ __volatile__("memw" ::: "memory");
    if (capture->state == PHASE2_CAPTURE_STARTING ||
        capture->state == PHASE2_CAPTURE_WAITING) {
        capture->result = ESP_ERR_TIMEOUT;
        capture->state = PHASE2_CAPTURE_DONE;
    }
}

void switch1_phase2_capture_cancel(void)
{
    const uint8_t state_before_cancel = g_phase2_capture.state;
    if (g_phase2_capture.state != PHASE2_CAPTURE_STARTING &&
        g_phase2_capture.state != PHASE2_CAPTURE_WAITING) {
        ESP_LOGI(TAG,
                 "Phase 2 capture stopped: state=%u active=%u schedule_count=%lu",
                 state_before_cancel, g_phase2_capture.active,
                 (unsigned long)g_phase2_capture.push_count);
        return;
    }
    const int dispatch_result = btdm_dispatch_work_to_controller(
        phase2_capture_cancel_controller, &g_phase2_capture, true);
    if (dispatch_result != 0) {
        g_phase2_capture.active = false;
        g_phase2_capture.result = ESP_FAIL;
        g_phase2_capture.state = PHASE2_CAPTURE_DONE;
    }
    ESP_LOGI(TAG,
             "Phase 2 capture stopped: dispatch=%d state=%u->%u active=%u "
             "schedule_count=%lu last_activity=%lu/%08lx format=%u",
             dispatch_result, state_before_cancel, g_phase2_capture.state,
             g_phase2_capture.active,
             (unsigned long)g_phase2_capture.push_count,
             (unsigned long)g_phase2_capture.last_activity,
             (unsigned long)g_phase2_capture.last_callback,
             g_phase2_capture.last_format);
}

bool switch1_phase2_capture_take(esp_err_t *out_result, uint8_t *out_lt_addr,
                                 uint32_t *out_clock_delta,
                                 uint32_t *out_rx_clk,
                                 uint16_t *out_rx_bitoff)
{
    if (g_phase2_capture.state != PHASE2_CAPTURE_DONE) {
        return false;
    }
    __asm__ __volatile__("memw" ::: "memory");

    uint8_t payload[PHASE2_LENGTH];
    for (uint8_t i = 0; i < PHASE2_LENGTH; ++i) {
        payload[i] = g_phase2_capture.payload[i];
    }
    esp_err_t result = g_phase2_capture.result;
    uint8_t lt_addr = 0;
    uint32_t clock_delta = 0;
    if (result == ESP_OK) {
        lt_addr = payload[0] & 7;
        result = lt_addr != 0 ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
        for (uint8_t i = 0; result == ESP_OK && i < 6; ++i) {
            if (payload[1 + i] != g_phase2_capture.target[5 - i]) {
                result = ESP_ERR_INVALID_RESPONSE;
            }
        }
        const uint32_t packed_remote_clock =
            (uint32_t)payload[13] | ((uint32_t)payload[14] << 8) |
            ((uint32_t)payload[15] << 16) |
            (((uint32_t)payload[16] & UINT32_C(0x1f)) << 24);
        const uint32_t remote_clock = packed_remote_clock >> 2;
        clock_delta = (remote_clock - g_phase2_capture.rx_clk) &
                      UINT32_C(0x07ffffff);
    }

    g_phase2_capture.result = result;
    /*
     * - UARTAbout 20 ms of UART output for three lines adds delay between Phase 2 response and Phase 3 transmission.
     * - g_phase2_captureg_phase2_capture fields desc, control, payload, rx_clk, and rx_bitoff are unchanged until the next capture begins.
     */
    s_phase2_report_pending = true;

    if (out_result != NULL) {
        *out_result = result;
    }
    if (out_lt_addr != NULL) {
        *out_lt_addr = lt_addr;
    }
    if (out_clock_delta != NULL) {
        *out_clock_delta = clock_delta;
    }
    if (out_rx_clk != NULL) {
        *out_rx_clk = g_phase2_capture.rx_clk;
    }
    if (out_rx_bitoff != NULL) {
        *out_rx_bitoff = g_phase2_capture.rx_bitoff;
    }
    g_phase2_capture.state = PHASE2_CAPTURE_IDLE;
    return true;
}

volatile bool g_report_verbose = true;


void switch1_set_report_verbose(bool enabled)
{
    g_report_verbose = enabled;
}

void switch1_phase2_capture_report(void)
{
    if (!g_report_verbose) {
        return;
    }
    if (!s_phase2_report_pending) {
        return;
    }
    s_phase2_report_pending = false;

    uint8_t payload[PHASE2_LENGTH];
    for (uint8_t i = 0; i < PHASE2_LENGTH; ++i) {
        payload[i] = g_phase2_capture.payload[i];
    }
    ESP_LOGI(TAG,
             "Phase 2 capture result: result=%s link=%u programmed=%lu "
             "callback_calls=%lu last_activity=%lu/%08lx format=%u "
             "reason[0..3]=%lu/%lu/%lu/%lu descriptor[%u]=%04x %04x %04x %04x %04x "
             "%04x %04x",
             esp_err_to_name(g_phase2_capture.result), g_phase2_capture.link,
             (unsigned long)g_phase2_capture.programmed,
             (unsigned long)g_phase2_capture.push_count,
             (unsigned long)g_phase2_capture.last_activity,
             (unsigned long)g_phase2_capture.last_callback,
             g_phase2_capture.last_format,
             (unsigned long)g_phase2_capture.reasons[0],
             (unsigned long)g_phase2_capture.reasons[1],
             (unsigned long)g_phase2_capture.reasons[2],
             (unsigned long)g_phase2_capture.reasons[3],
             g_phase2_capture.desc_index, g_phase2_capture.desc[0],
             g_phase2_capture.desc[1], g_phase2_capture.desc[2],
             g_phase2_capture.desc[3], g_phase2_capture.desc[4],
             g_phase2_capture.desc[5], g_phase2_capture.desc[6]);
    if (g_phase2_capture.payload_offset != 0) {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, payload, sizeof(payload), ESP_LOG_INFO);
    }
    ESP_LOGI(TAG,
             "  Phase 2 control structure: %04x %04x %04x %04x %04x %04x %04x %04x "
             "%04x %04x %04x %04x %04x %04x %04x %04x",
             g_phase2_capture.control[0], g_phase2_capture.control[1],
             g_phase2_capture.control[2], g_phase2_capture.control[3],
             g_phase2_capture.control[4], g_phase2_capture.control[5],
             g_phase2_capture.control[6], g_phase2_capture.control[7],
             g_phase2_capture.control[8], g_phase2_capture.control[9],
             g_phase2_capture.control[10], g_phase2_capture.control[11],
             g_phase2_capture.control[12], g_phase2_capture.control[13],
             g_phase2_capture.control[14], g_phase2_capture.control[15]);
    ESP_LOGI(TAG,
             "  Phase 2 control-structure tail: first +5E=%04x +60=%04x +62=%04x +64=%04x  "
             "latest +5E=%04x +60=%04x +62=%04x +64=%04x",
             g_phase2_capture.tail_first[0], g_phase2_capture.tail_first[1],
             g_phase2_capture.tail_first[2], g_phase2_capture.tail_first[3],
             g_phase2_capture.tail_last[0], g_phase2_capture.tail_last[1],
             g_phase2_capture.tail_last[2], g_phase2_capture.tail_last[3]);
}
