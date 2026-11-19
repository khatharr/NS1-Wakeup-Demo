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


void lmp_not_accepted_hook_install(void);
void lmp_not_accepted_hook_remove(void);
void packet_type_hook_install(void);
void packet_type_hook_remove(void);

acl_handoff_request_t g_acl_handoff;
bool acl_link_context_clear(uint8_t link_id);

bool acl_link_context_clear(uint8_t link_id)
{
    const uint8_t *const hci_record = &hci_env[16 + 7 * link_id];
    return hci_record[0] == 0 && lc_state[link_id] == 0 &&
           lc_env[link_id] == NULL && ld_acl_env[link_id] == NULL &&
           llc_env[link_id] == NULL;
}

static void acl_handoff_controller(void *handoff_arg)
{
    acl_handoff_request_t *const request = handoff_arg;
    const uint8_t link_id = request->reserved_link;
    uint8_t *const pscan_env = ld_pscan_env;

    for (uint8_t i = 0; i < ACL_LINK_COUNT; ++i) {
        const uint8_t *const hci_record = &hci_env[16 + 7 * i];
        if (hci_record[0] == 0) {
            continue;
        }
        /*
         * - hci_env[17 + 7*i]r_hci_bt_acl_bdaddr_register copies the six-byte BD_ADDR into hci_env[17 + 7*i] least-significant byte first.
         * - request->targetrequest->target is stored in display order, so reverse the byte order for comparison.
         */
        bool same_peer = true;
        for (uint8_t j = 0; j < 6; ++j) {
            if (hci_record[1 + j] != request->target[5 - j]) {
                same_peer = false;
                break;
            }
        }
        if (same_peer) {
            request->result = ESP_ERR_INVALID_STATE;
            __asm__ __volatile__("memw" ::: "memory");
            request->state = ACL_HANDOFF_DONE;
            return;
        }
    }

    // Only the schedule element was removed during Phase 2 capture; re-register frame-format-3 reception.
    uint16_t reject_flags = 0;
    reject_flags |= request->prepare_state != ACL_PREPARE_DONE ? UINT16_C(1) << 0 : 0;
    reject_flags |= request->prepare_result != ESP_OK ? UINT16_C(1) << 1 : 0;
    reject_flags |= link_id >= ACL_LINK_COUNT ? UINT16_C(1) << 2 : 0;
    reject_flags |= pscan_env == NULL ? UINT16_C(1) << 3 : 0;
    if (pscan_env != NULL) {
        reject_flags |= pscan_env[0x3d] != link_id ? UINT16_C(1) << 4 : 0;
        reject_flags |= pscan_env[0x3e] != 0 ? UINT16_C(1) << 5 : 0;
        reject_flags |= pscan_env[0x3f] != 0 ? UINT16_C(1) << 6 : 0;
    }
    if (link_id < ACL_LINK_COUNT) {
        reject_flags |= lm_env[60 + 9 * link_id] != 2 ? UINT16_C(1) << 7 : 0;
        reject_flags |= !acl_link_context_clear(link_id) ? UINT16_C(1) << 8 : 0;
    }
    request->pscan_rx_reject = reject_flags;
    if (reject_flags != 0) {
        request->result = ESP_ERR_INVALID_STATE;
        __asm__ __volatile__("memw" ::: "memory");
        request->state = ACL_HANDOFF_DONE;
        return;
    }
    /*
     * - HCI 0x0442Even if HCI 0x0442 stop is rejected with status 0x0c, the Phase 2 callback may have re-registered
     *   CSBa CSB receive element that remains active.
     * - Stop it directly inside the controller to prevent contention.
     */
    if (ld_csb_rx_env != NULL) {
        (void)r_ld_csb_rx_stop();
    }
    if (ld_csb_rx_env != NULL) {
        request->pscan_rx_reject = UINT16_C(1) << 11;
        request->result = ESP_ERR_INVALID_STATE;
        __asm__ __volatile__("memw" ::: "memory");
        request->state = ACL_HANDOFF_DONE;
        return;
    }
    /*
     * - ACL reception and CSB reception share the receive-descriptor ring buffer and index
     *   ((index at ld_env+0x1AA = 0x3ffb96ba).
     * - Continuing wake-up reception would conflict with reception on the ACL link.
     */
    phase2_restore_hook();

    volatile uint16_t *const acl_control =
        (volatile uint16_t *)(ACL_CS_BASE + ACL_CS_STRIDE * link_id);
    for (uint8_t i = 0; i < ACL_CS_SNAPSHOT_WORDS; ++i) {
        request->cs_before[i] = acl_control[i];
    }
    acl_control[0] = (uint16_t)((acl_control[0] & ~UINT16_C(0x001f)) |
                       ACL_CS_FORMAT_PERIPHERAL);
    acl_control[1] = request->rx_bitoff;
    acl_control[2] = (uint16_t)(request->clock_delta & UINT32_C(0xffff));
    acl_control[3] = (uint16_t)((request->clock_delta >> 16) & UINT32_C(0x07ff));
    /*
     * - +0x08+0x08 stores LT_ADDR (bits 8-10), EDR (bit 11), whitening disabled (bit 12), and AFH (bit 14).
     * - The wake-up exchange is unwhitened, but the link becomes a normal ACL link after creation.
     * - Unless whitening is enabled, the received header fails its HEC check and the packet is discarded.
     */
    acl_control[4] = (uint16_t)((acl_control[4] & ~(UINT16_C(0x4700) |
                                  CS_LINKCNTL_WHITENING_DSB)) |
                       ((uint16_t)request->lt_addr << 8));

    acl_control[5] = (uint16_t)(request->target[5] |
                       ((uint16_t)request->target[4] << 8));
    acl_control[6] = (uint16_t)(request->target[3] |
                       ((uint16_t)request->target[2] << 8));
    acl_control[7] = (uint16_t)(request->target[1] |
                       ((uint16_t)request->target[0] << 8));
    acl_control[8] = (uint16_t)(request->bch34 & UINT64_C(0xffff));
    acl_control[9] = (uint16_t)((request->bch34 >> 16) & UINT64_C(0xffff));
    acl_control[10] = (uint16_t)((request->bch34 >> 32) & UINT64_C(0x3));
    /* r_ld_pscan_evt_handler(0x4003f2cf)Match the value r_ld_pscan_evt_handler (0x4003f2cf) writes when converting a page-scan control structure
     * ESP32into one for an ACL link where the ESP32 acts as a peripheral. */
    acl_control[11] |= CS_HOPCNTL_FH_EN;
    acl_control[12] &= (uint16_t)~UINT16_C(0x0008);
    acl_control[13] = 48;

    /*
     * 1. lm_page_scan_end_ind_handler(0x40052e40)Write in the same order as lm_page_scan_end_ind_handler (0x40052e40) prepares immediately before r_lc_start.
     * 2. r_ld_acl_startCalling r_ld_acl_start alone creates only the lower-level link, not lc_env or the link-control task state.
     */
    uint8_t *const lm_link_record =
        &lm_env[LM_LINK_RECORD_BASE + LM_LINK_RECORD_STRIDE * link_id];
    lc_bd_addr_t peer_addr;
    for (uint8_t i = 0; i < 6; ++i) {
        /* request->targetrequest->target is stored in display order (MSB first); the ROM path expects the
         * r_ld_util_fhs_unpkleast-significant-byte-first order produced by r_ld_util_fhs_unpk, so reverse it here. */
        peer_addr.addr[i] = request->target[5 - i];
        lm_link_record[i] = request->target[5 - i];
    }
    lm_link_record[LM_LINK_STATE_OFFSET] = LM_LINK_STATE_CONNECTED;
    lm_link_record[LM_LINK_ROLE_OFFSET] = ACL_ROLE_PERIPHERAL;
    /*
     * - LMLM link record +8 contains LT_ADDR, but the normal path for an ESP32 peripheral ACL link does not write it.
     * - Only three central-role paths use this field, so do not modify it here.
     */

    void **const rom_ip_funcs = *(void ***)PHASE2_ROM_IP_FUNCS_PTR_ADDR;
    if (rom_ip_funcs == NULL) {
        request->pscan_rx_reject = UINT16_C(1) << 12;
        request->result = ESP_ERR_INVALID_STATE;
        __asm__ __volatile__("memw" ::: "memory");
        request->state = ACL_HANDOFF_DONE;
        return;
    }
    const hci_bdaddr_register_fn_t hci_bdaddr_register =
        (hci_bdaddr_register_fn_t)rom_ip_funcs[IP_FUNCS_HCI_BDADDR_REGISTER_INDEX];
    const lc_start_fn_t lc_start =
        (lc_start_fn_t)rom_ip_funcs[IP_FUNCS_LC_START_INDEX];
    if (hci_bdaddr_register == NULL || lc_start == NULL) {
        request->pscan_rx_reject = UINT16_C(1) << 12;
        request->result = ESP_ERR_INVALID_STATE;
        __asm__ __volatile__("memw" ::: "memory");
        request->state = ACL_HANDOFF_DONE;
        return;
    }

    hci_bdaddr_register(link_id, lm_link_record);
    request->lc_start_called = true;
    /*
     * - class of deviceSet class of device to zero.
     * - SwitchThe Switch wake-up response contains no FHS.
     *   Therefore the Switch class of device is unavailable.
     * - This value is placed in lc_env+54-56 and only included in HCI Connection Request; it does not affect link creation.
     */
    lc_start(link_id, LC_START_REASON_PAGE_SCAN, peer_addr, 0, 0, 0,
             lm_env[LM_ENV_DEFAULT_LINK_POLICY]);

    /*
     * - r_lc_startAfter r_lc_start, set the same values ROM uses when creating an ACL link.
     *   Unless the Phase 2 reception settings are changed to ACL values, packet headers may decode but payloads will be discarded.
     * - +0x14+0x14 is the third BCH word; an ACL link uses only its lowest two bits.
     * - +0x5EBits 12 and 13 at +0x5E are set by all three ACL-creation paths, but page-scan initialization clears them, so set them explicitly here.
     */
    acl_control[10] = (uint16_t)((request->bch34 >> 32) & UINT64_C(0x3));
    acl_control[47] |= UINT16_C(0x3000);

    lmp_not_accepted_hook_install();
    packet_type_hook_install();
    /*
     * - Write the value saved before Phase 2 reception began to +0x60.
     * - +0x60+0x60 holds the last SEQN accepted by hardware.
     *   Receiving the Phase 2 response (SEQN=1) changes 0x0004 to 0x0405.
     * - If this value is not restored for ACL, the Switch's first LMP packet is mistaken for a retransmission and discarded.
     */
    if (g_phase2_capture.tail_first_taken) {
        request->cs60_before = acl_control[48];
        acl_control[48] = g_phase2_capture.tail_first[1];
        request->cs60_after = acl_control[48];
        request->cs60_restored = true;
    }

    uint8_t lc_start_flags = 0;
    lc_start_flags |= lc_env[link_id] != NULL ? LC_START_FLAG_LC_ENV : 0;
    lc_start_flags |= ld_acl_env[link_id] != NULL ? LC_START_FLAG_LD_ACL_ENV : 0;
    lc_start_flags |= llc_env[link_id] != NULL ? LC_START_FLAG_LLC_ENV : 0;
    lc_start_flags |= hci_env[16 + 7 * link_id] != 0 ? LC_START_FLAG_HCI_BDADDR : 0;
    request->lc_start_flags = lc_start_flags;
    request->lc_state_after = lc_state[link_id];
    for (uint8_t i = 0; i < ACL_CS_SNAPSHOT_WORDS; ++i) {
        request->cs_at_start[i] = acl_control[i];
    }
    if (ld_acl_env[link_id] != NULL) {
        const volatile uint8_t *const acl_env_snapshot = ld_acl_env[link_id];
        for (uint16_t i = 0; i < ACL_ENV_SNAPSHOT_BYTES; ++i) {
            request->acl_env_at_start[i] = acl_env_snapshot[i];
        }
    }
    if (lc_env[link_id] != NULL) {
        const volatile uint8_t *const lc_env_snapshot = lc_env[link_id];
        for (uint16_t i = 0; i < LC_ENV_SNAPSHOT_BYTES; ++i) {
            request->lc_env_at_start[i] = lc_env_snapshot[i];
        }
    }

    if ((lc_start_flags & (LC_START_FLAG_LC_ENV | LC_START_FLAG_LD_ACL_ENV)) ==
        (LC_START_FLAG_LC_ENV | LC_START_FLAG_LD_ACL_ENV)) {
        /*
         * 1. LMP_host_connection_reqSend LMP_host_connection_req.
         *    SwitchThe Switch does not transmit LMP until it receives this PDU.
         *    ESP32When the ESP32 sends it, the Switch sends the same PDU in response.
         *    lmp_host_con_req_handlerHCI Connection Request is delivered to the host only after lmp_host_con_req_handler processes it.
         * 2. Send internal message 0x0518 to the link-control task.
         *    Receiver lc_op_hl_cnx_ind_handler (0x400293fc) processes it only when the internal link state is 2 or 5.
         *    The state is 5 when the link is registered with normal controller processing.
         * 3. After transmission ROM changes the internal state to 20 while waiting for LMP_accepted.
         *    The follow-up task changes it back to 5 so the Switch's LMP_host_connection_req can be processed.
         * 4. Using the host GAP connection-complete event as the trigger can fail if that event is not delivered.
         *    Trigger when the link is registered with normal controller processing to avoid this problem.
         */
        {
            const ke_state_get_fn_t ke_state_get =
                (ke_state_get_fn_t)ROM_KE_STATE_GET_ADDR;
            const ke_msg_send_basic_fn_t send_basic =
                (ke_msg_send_basic_fn_t)ROM_KE_MSG_SEND_BASIC_ADDR;
            const uint16_t ke_task_id = (uint16_t)((link_id << 8) | 1u);
            request->host_connection_state_before = ke_state_get(ke_task_id);
            send_basic(LC_OP_HL_CNX_IND, ke_task_id, ke_task_id);
            request->host_connection_state_after = ke_state_get(ke_task_id);
            request->host_connection_request_posted = true;
        }
        /*
         * - LMP_setup_completeDo not transmit LMP_setup_complete here; doing so makes ROM change the internal link state to 1, preventing processing of
         *   Switchthe Switch's later LMP_host_connection_req.
         * - SwitchROM transmits this PDU once the host accepts the Switch's request.
         */
        request->result = ESP_OK;
        request->link_id = link_id;
        __asm__ __volatile__("memw" ::: "memory");
        request->state = ACL_HANDOFF_DONE;
        return;
    }
    /*
     * - ke_mallocIf ke_malloc fails, ROM calls assert_err and returns without changing state.
     * - lm_envOnly the lm_env record and hci_env registration have changed; restore their previous values before restoring the receive element.
     */
    lm_link_record[LM_LINK_STATE_OFFSET] = LM_LINK_STATE_PSCAN_RESERVED;
    lm_link_record[LM_LINK_ROLE_OFFSET] = 0;
    memset(lm_link_record, 0, 6);
    hci_env[16 + 7 * link_id] = 0;
    const uint32_t pscan_rx_start =
        (request->phase3_slot + PSCAN_RX_START_SLOTS) & ACL_CLOCK_MASK;
    const uint32_t pscan_rx_target =
        (request->phase3_slot + PSCAN_RX_TARGET_SLOTS) & ACL_CLOCK_MASK;
    const uint32_t pscan_rx_deadline =
        (request->phase3_slot + PSCAN_RX_DEADLINE_SLOTS) & ACL_CLOCK_MASK;
    *(uint32_t *)&pscan_env[0x08] = pscan_rx_start;
    *(uint32_t *)&pscan_env[0x0c] = pscan_rx_deadline;
    *(uint16_t *)&pscan_env[0x10] =
        (uint16_t)(UINT16_C(0x8840) | (rwip_priority[33] & 0x0f) |
                   ((request->clock_delta & 1) << 12));
    *(uint16_t *)&pscan_env[0x12] = PSCAN_RX_WINDOW_US;
    *(uint16_t *)&pscan_env[0x14] = request->rx_bitoff;
    pscan_env[0x16] = rwip_priority[32];
    *(uint32_t *)&pscan_env[0x30] = pscan_rx_target;
    for (uint8_t i = 0; i < 6; ++i) {
        pscan_env[0x34 + i] = request->target[5 - i];
    }
    memset(&pscan_env[0x3a], 0, 3);
    for (uint8_t i = 0; i < 5; ++i) {
        pscan_env[0x48 + i] = (uint8_t)(request->bch34 >> (8 * i));
    }
    pscan_env[0x50] = (uint8_t)(request->clock_delta & 1);

    // Enable frame-format-3 reception for 2500 us after Phase 3; only the successful-reception callback generates 0x0404.
    request->pscan_rx_active = true;
    request->pscan_rx_insert_now = r_ld_read_clock() & ACL_CLOCK_MASK;
    __asm__ __volatile__("memw" ::: "memory");
    const uint8_t insert_result = r_ea_elt_insert(pscan_env);
    if (insert_result == 0) {
        pscan_env[0x3f] = 1;
        __asm__ __volatile__("memw" ::: "memory");
    } else {
        request->pscan_rx_reject = UINT16_C(1) << 10;
        request->pscan_rx_active = false;
        phase2_restore_hook();
    }
    request->result = insert_result == 0 ? ESP_OK : ESP_ERR_INVALID_STATE;
    request->link_id = request->result == ESP_OK ? link_id : UINT8_MAX;
    __asm__ __volatile__("memw" ::: "memory");
    request->state = ACL_HANDOFF_DONE;
}

#define ACL_FOLLOWUP_FAST_MS 900
/*
 * - 900msAfter 900 ms, wait in 1 ms intervals and continue past the page-scan reconnection.
 * - Do not finish before the specified time, because this task must remove the two hooks it installed.
 */
#define ACL_FOLLOWUP_MS 8000

lc_version_req_t g_lc_version_req;
volatile bool g_con_req_dispatched;

/* Wait time between registering the link with normal controller processing and transmitting the ESP32 LMP_version_req.
 * SwitchSet it later than the expected arrival of the Switch's LMP_host_connection_req. */
#define LC_VERSION_REQ_DELAY_US 200000
/* Wait time between registering the link with normal controller processing and proactively notifying HCI Connection Request.
 * ROMSkip this path if ROM has already delivered the notification. */
#define CON_REQ_INJECT_DELAY_US 300000

static void acl_followup_task(void *task_arg)
{
    (void)task_arg;
    const uint8_t link_id =
        g_acl_handoff.link_id < 14 ? g_acl_handoff.link_id : 0;
    const int64_t followup_start_us = esp_timer_get_time();
    while (esp_timer_get_time() - followup_start_us <
           (int64_t)ACL_FOLLOWUP_MS * 1000) {
        /*
         * ROMIf ROM changed the internal link state to 20, restore it to 5 so the Switch's LMP_host_connection_req can be processed.
         */
        if (!g_lc_idle_state_dispatched && lc_state[link_id] == 20) {
            g_lc_idle_state_dispatched = true;
            g_lc_idle_state_request.done = false;
            g_lc_idle_state_request.link = link_id;
            __asm__ __volatile__("memw" ::: "memory");
            (void)btdm_dispatch_work_to_controller(lc_set_idle_state_controller,
                                                   &g_lc_idle_state_request, false);
        }
        /*
         * ESP32Sending LMP from the ESP32 changes the internal link state from 5 to another value.
         * LMP_host_connection_reqLMP_host_connection_req is processed only when the internal state is 5.
         * (`lmp_host_con_req_handler` 0x4002b402)It is handled by lmp_host_con_req_handler (0x4002b402); do not transmit until the Switch's request is processed.
         */
        if (!g_lc_version_req.requested &&
            esp_timer_get_time() - followup_start_us >= LC_VERSION_REQ_DELAY_US) {
            g_lc_version_req.link = link_id;
            __asm__ __volatile__("memw" ::: "memory");
            g_lc_version_req.requested = true;
            (void)btdm_dispatch_work_to_controller(lc_version_req_controller,
                                                   &g_lc_version_req, false);
        }
        /*
         * 1. SwitchNotify the host of HCI Connection Request after processing the Switch's LMP_host_connection_req and returning the internal state to 5.
         * 2. Without notification, the Switch finishes processing LMP_host_connection_req after 35 ms.
         *    Accept Connection Request from the host makes the controller send LMP_accepted, which the Switch is awaiting.
         * 3. ROMDo not send a duplicate notification if ROM already sent HCI Connection Request.
         *    If already sent, lmp_host_con_req_handler wrote 1 to lc_env+86 at 0x4002b461.
         *    Duplicate notification would make the host receive two incoming requests from the same Switch.
         */
        if (g_lc_version_req.requested && !g_lc_version_req.con_req_sent &&
            !g_con_req_dispatched &&
            (lc_env[link_id] == NULL ||
             ((const volatile uint8_t *)lc_env[link_id])[86] == 0) &&
            esp_timer_get_time() - followup_start_us >= CON_REQ_INJECT_DELAY_US) {
            g_con_req_dispatched = true;
            (void)btdm_dispatch_work_to_controller(lc_con_req_controller,
                                                   &g_lc_version_req, false);
        }
        if (esp_timer_get_time() - followup_start_us >=
            (int64_t)ACL_FOLLOWUP_FAST_MS * 1000) {
            vTaskDelay(1);
        }
    }

    lmp_not_accepted_hook_remove();
    packet_type_hook_remove();
    vTaskDelete(NULL);
}

esp_err_t switch1_acl_handoff_start(const uint8_t bd_addr[6], uint64_t bch34,
                                    uint8_t lt_addr, uint32_t clock_delta,
                                    uint32_t rx_clk, uint16_t rx_bitoff,
                                    uint32_t phase3_slot)
{
    if (bd_addr == NULL || lt_addr == 0 || lt_addr > 7 ||
        rx_bitoff >= 625 || phase3_slot > ACL_CLOCK_MASK ||
        g_acl_handoff.state == ACL_HANDOFF_PENDING) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(g_acl_handoff.target, bd_addr, sizeof(g_acl_handoff.target));
    g_acl_handoff.bch34 = bch34;
    g_acl_handoff.lt_addr = lt_addr;
    g_acl_handoff.clock_delta = clock_delta & UINT32_C(0x07ffffff);
    g_acl_handoff.rx_clk = rx_clk & UINT32_C(0x07ffffff);
    g_acl_handoff.phase3_slot = phase3_slot;
    g_acl_handoff.rx_bitoff = rx_bitoff;
    g_acl_handoff.link_id = 0xff;
    g_acl_handoff.result = ESP_ERR_TIMEOUT;
    g_acl_handoff.pscan_rx_active = false;
    g_acl_handoff.pscan_rx_dm1 = false;
    g_acl_handoff.pscan_rx_reported = false;
    g_acl_handoff.pscan_rx_reject = 0;
    g_acl_handoff.pscan_rx_status = 0;
    g_acl_handoff.pscan_rx_header = 0;
    g_acl_handoff.pscan_rx_pheader = 0;
    g_acl_handoff.pscan_rx_insert_now = 0;
    g_acl_handoff.state = ACL_HANDOFF_PENDING;
    memset((void *)&g_lc_version_req, 0, sizeof(g_lc_version_req));
    g_lmp_not_accepted_suppressed = 0;
    g_lmp_not_accepted_forwarded = 0;
    g_con_req_dispatched = false;
    g_lc_idle_state_dispatched = false;
    g_packet_type_suppressed = 0;
    __asm__ __volatile__("memw" ::: "memory");
    xTaskCreate(acl_followup_task, "aclfollowup", 2560, NULL, 5, NULL);

    /*
     * - falsefalse means fail immediately if the controller work queue is full.
     * - callbackThe callback does not wait or issue HCI commands; it only registers the frame-format-3 schedule element.
     */
    if (btdm_dispatch_work_to_controller(acl_handoff_controller,
                                         &g_acl_handoff, false) != 0) {
        g_acl_handoff.result = ESP_FAIL;
        __asm__ __volatile__("memw" ::: "memory");
        g_acl_handoff.state = ACL_HANDOFF_DONE;
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool switch1_lc_start_status(uint8_t *out_lc_state, uint8_t *out_flags)
{
    if (!g_acl_handoff.lc_start_called) {
        return false;
    }
    if (out_lc_state != NULL) {
        *out_lc_state = g_acl_handoff.lc_state_after;
    }
    if (out_flags != NULL) {
        *out_flags = g_acl_handoff.lc_start_flags;
    }
    return true;
}

/*
 * 1. r_lc_detach(0x4002037c)r_lc_detach (0x4002037c) accepts (link_id, reason), changes the internal link state to 25, and sends LMP_detach.
 * 2. After the state changes to 26 and then 27, it frees the link and notifies the host with HCI Disconnection Complete.
 */
#define IP_FUNCS_LC_DETACH_INDEX (UINT16_C(0x2cc) / sizeof(void *))
#define LC_DETACH_REASON_REMOTE_USER 19

typedef void (*lc_detach_fn_t)(uint8_t link_id, uint8_t reason);

static volatile bool s_acl_detach_requested;

static void acl_detach_controller(void *detach_arg)
{
    const uint8_t link_id = (uint8_t)(uintptr_t)detach_arg;
    void **const rom_ip_funcs = *(void ***)PHASE2_ROM_IP_FUNCS_PTR_ADDR;
    if (rom_ip_funcs == NULL || link_id >= ACL_LINK_COUNT || lc_env[link_id] == NULL) {
        return;
    }
    const lc_detach_fn_t lc_detach =
        (lc_detach_fn_t)rom_ip_funcs[IP_FUNCS_LC_DETACH_INDEX];
    if (lc_detach == NULL) {
        return;
    }
    lc_detach(link_id, LC_DETACH_REASON_REMOTE_USER);
    /*
     * - s_acl_detach_requestedWhile s_acl_detach_requested is true, permit only one request per cycle.
     */
    __asm__ __volatile__("memw" ::: "memory");
    s_acl_detach_requested = false;
}

uint8_t switch1_acl_lc_state(void)
{
    if (!g_acl_handoff.lc_start_called ||
        g_acl_handoff.link_id >= ACL_LINK_COUNT) {
        return 0xff;
    }
    return lc_state[g_acl_handoff.link_id];
}

/*
 * - r_lc_release(0x4001f8a8)r_lc_release (0x4001f8a8) releases the ESP32-side link without waiting for an LMP_detach exchange.
 * - SwitchUse if the Switch stops receiving on the link and disconnection cannot complete.
 * - When freed, ROM notifies the host with HCI Disconnection Complete and releases host-stack state as well.
 */
#define ROM_LC_RELEASE_ADDR UINT32_C(0x4001f8a8)

typedef void (*lc_release_fn_t)(uint16_t dest_id);

static void acl_release_controller(void *release_arg)
{
    const uint8_t link_id = (uint8_t)(uintptr_t)release_arg;
    if (link_id >= ACL_LINK_COUNT || lc_env[link_id] == NULL) {
        return;
    }
    const lc_release_fn_t lc_release = (lc_release_fn_t)ROM_LC_RELEASE_ADDR;
    lc_release((uint16_t)(((uint16_t)link_id << 8) | KE_TASK_LC));
}

esp_err_t switch1_acl_release(void)
{
    if (!g_acl_handoff.lc_start_called ||
        g_acl_handoff.link_id >= ACL_LINK_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t link_id = g_acl_handoff.link_id;
    if (lc_env[link_id] == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (btdm_dispatch_work_to_controller(
            acl_release_controller, (void *)(uintptr_t)link_id, false) != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/*
 * - If a disconnect request cannot be processed, change the internal link state to 5.
 * - r_lc_detachIf LMP processing is underway, r_lc_detach returns without changing the state.
 */
esp_err_t switch1_acl_force_idle(void)
{
    if (!g_acl_handoff.lc_start_called ||
        g_acl_handoff.link_id >= ACL_LINK_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (lc_env[g_acl_handoff.link_id] == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    g_lc_idle_state_request.done = false;
    g_lc_idle_state_request.link = g_acl_handoff.link_id;
    __asm__ __volatile__("memw" ::: "memory");
    if (btdm_dispatch_work_to_controller(lc_set_idle_state_controller,
                                         &g_lc_idle_state_request, false) != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t switch1_acl_detach(void)
{
    if (!g_acl_handoff.lc_start_called ||
        g_acl_handoff.link_id >= ACL_LINK_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_acl_detach_requested) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t link_id = g_acl_handoff.link_id;
    if (lc_env[link_id] == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /*
     * 1. `r_lc_detach`(0x4002037c)r_lc_detach (0x4002037c) changes the internal link state to 25 and sends LMP_detach.
     * 2. State becomes 26 on transmit completion, then 27 when r_lc_release (0x4001f8a8) runs.
     * 3. If internal state is 25-27, wait for disconnection to complete.
     *    Return success so the caller does not misinterpret it as a failed disconnect and reboot.
     */
    if (lc_state[link_id] >= 25 && lc_state[link_id] <= 27) {
        return ESP_OK;
    }
    s_acl_detach_requested = true;
    // btdm_dispatch_work_to_controllerbtdm_dispatch_work_to_controller returns 0 on success.
    if (btdm_dispatch_work_to_controller(acl_detach_controller,
                                         (void *)(uintptr_t)link_id, false) != 0) {
        s_acl_detach_requested = false;
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool switch1_acl_handoff_status(esp_err_t *out_result, uint8_t *out_link_id)
{
    if (g_acl_handoff.state != ACL_HANDOFF_DONE) {
        return false;
    }
    __asm__ __volatile__("memw" ::: "memory");
    if (out_result != NULL) {
        *out_result = g_acl_handoff.result;
    }
    if (out_link_id != NULL) {
        *out_link_id = g_acl_handoff.link_id;
    }
    if (!g_acl_handoff.pscan_rx_reported &&
        g_acl_handoff.pscan_rx_reject != 0) {
        g_acl_handoff.pscan_rx_reported = true;
        ESP_LOGI(TAG,
                 "Page-scan format 3 receive registration rejected: reason=%03x insert_clock=%08x phase3_clock=%08x",
                 g_acl_handoff.pscan_rx_reject,
                 (unsigned)g_acl_handoff.pscan_rx_insert_now,
                 (unsigned)g_acl_handoff.phase3_slot);
    } else if (g_acl_handoff.pscan_rx_header != 0 &&
               !g_acl_handoff.pscan_rx_reported) {
        g_acl_handoff.pscan_rx_reported = true;
        ESP_LOGI(TAG,
                 "Page-scan format 3 response received: status=%04x header=%03x "
                 "pheader=%04x DM1=%u",
                 g_acl_handoff.pscan_rx_status, g_acl_handoff.pscan_rx_header,
                 g_acl_handoff.pscan_rx_pheader, g_acl_handoff.pscan_rx_dm1);
    }
    return true;
}
