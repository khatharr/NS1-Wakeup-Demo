/*
 * Configure wake-up parity in the receiver control structure.
 * Determine successful reception by directly reading receive descriptors.
 * HCIWaiting for an HCI event would be too slow between reception of the response and Phase 3 transmission.
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

static const char *TAG = "wake_csb";

 /*
  * CSBCSB receiver control structure.
  * Corresponding ROM routine: r_ld_csb_rx_start (0x4003972c).
  * Has the same layout as the transmitter (0x3ffb22b6).
 * - +0x0A: BD_ADDR
 * - +0x10: BCH parity
 */
#define CSB_RX_CS_ADDR UINT32_C(0x3ffb231c)
/*
 * HCIHCI Set_Connectionless_Slave_Broadcast_Receive.
 * bluedroidNot defined in Bluedroid's hcidefs.h, but implemented by the ESP32 controller.
 * ROMROM lb_hci_command_handler_tab registers a handler for opcode 0x0442.
 */
#define HCI_SET_CLB_RECEIVE (0x0042 | HCI_GRP_LINK_CONTROL_CMDS)
/*
 * HCIHCI Set_Event_Mask_Page_2.
 * CSBCSB receive events (0x51) are disabled by default.
 * ROMROM r_hci_evt_mask_set (0x400189e4) copies the eight-byte page-2 mask to hci_env+8.
 * Do not alter the existing bit assignments of event mask page 2.
 * bluedroidBluedroid never issues this command, so enable bits 16-18 here.
 * - bit16: bit 16: synchronization-train reception
 * - bit17: CSBbit 17: CSB reception
 * - bit18: CSBbit 18: CSB reception timeout
 */
#define HCI_SET_EVT_MASK_PAGE_2 (0x0063 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_EVT_MASK_PAGE_2_CSB UINT8_C(0x07)
#define CSB_RX_PARAM_SIZE 34

static uint8_t s_csb_rx_target[6];
static uint64_t s_csb_rx_bch;
static uint8_t s_csb_rx_channel;
static volatile bool s_csb_rx_active;
// Control structure immediately after reception starts.
// Defer logging so that response detection is not delayed.
#define CSB_RX_START_CS_WORDS 24
static uint16_t s_csb_rx_start_cs[CSB_RX_START_CS_WORDS];
static bool s_csb_rx_start_cs_valid;
static uint8_t s_csb_rx_ltaddr_written;
static uint16_t s_csb_rx_ltaddr_readback;
static bool s_csb_rx_ltaddr_valid;

/*
 * Descriptor status alone cannot determine the number of consecutive receptions.
 * HCIHCI event 0x51 occurs once per reception; count them here.
 */
static volatile uint32_t s_csb_rx_event_count;

static volatile bool s_csb_rx_packet_pending;
static uint8_t s_csb_rx_addr[6];
static uint32_t s_csb_rx_clock;
static uint8_t s_csb_rx_status;
static uint8_t s_csb_rx_length;
static uint8_t s_csb_rx_data[32];


static void switch1_csb_rx_reapply(void);

static void csb_rx_write_bdaddr(const uint8_t address[6])
{
    volatile uint16_t *const base =
        (volatile uint16_t *)(CSB_RX_CS_ADDR + CS_BDADDR_OFFSET);
    base[0] = (uint16_t)(address[5] | ((uint16_t)address[4] << 8));
    base[1] = (uint16_t)(address[3] | ((uint16_t)address[2] << 8));
    base[2] = (uint16_t)(address[1] | ((uint16_t)address[0] << 8));
}

static void csb_rx_write_bch(uint64_t bch34)
{
    volatile uint16_t *const base =
        (volatile uint16_t *)(CSB_RX_CS_ADDR + CS_BCH_OFFSET);
    base[0] = (uint16_t)(bch34 & 0xFFFF);
    base[1] = (uint16_t)((bch34 >> 16) & 0xFFFF);
    /*
     * 3The third word uses different write formats for transmission and reception.
     * - Transmit (r_ld_csb_tx_en): write only the upper two bits of parity.
     * - Receive (r_ld_csb_rx_start): also write 0xFF40.
     * The meaning of the upper ten bits is unknown; preserve those ROM writes only when receiving.
     * Do not use the transmitter's write format here.
     */
    base[2] = (uint16_t)(CS_BCH_RX_FLAGS | ((bch34 >> 32) & 0x3));
}

/*
 * HCIHCI parameter fields:
 * - Enable(1)
 * - BD_ADDR(6)
 * - LT_ADDR(1)
 * - Interval(2)
 * - Clock_Offset(4)
 * - Next_CSB_Clock(4)
 * - Supervision_Timeout(2)
 * - Remote_Timing_Accuracy(1)
 * - Skip(1)
 * - Packet_Type(2)
 * - AFH_Channel_Map(10)
 */
static void csb_rx_build_params(uint8_t params[CSB_RX_PARAM_SIZE],
                                uint8_t enable, const uint8_t bd_addr[6],
                                uint16_t interval, uint32_t clock_offset,
                                uint32_t next_clock)
{
    uint8_t *p = params;
    UINT8_TO_STREAM(p, enable);
    for (int i = 0; i < 6; ++i) {
        UINT8_TO_STREAM(p, bd_addr[5 - i]);
    }
    UINT8_TO_STREAM(p, CSB_LT_ADDR);
    UINT16_TO_STREAM(p, interval);
    UINT32_TO_STREAM(p, clock_offset & UINT32_C(0x0fffffff));
    UINT32_TO_STREAM(p, next_clock & UINT32_C(0x0fffffff));
    UINT16_TO_STREAM(p, CSB_RX_SUPERVISION_TIMEOUT);
    UINT8_TO_STREAM(p, 250);  // Remote_Timing_Accuracy(ppm)
    UINT8_TO_STREAM(p, 0);    // Skip
    /*
     * LLID 3The fixed buffer for LLID 3 is only 20 bytes, so permit only DM1, which holds the 17-byte response.
     * Allowing longer packet types would overwrite adjacent fixed buffers.
     */
    UINT16_TO_STREAM(p, CSB_EDR_EXCLUSION_MASK | HCI_PKT_TYPES_MASK_DM1);
    for (int i = 0; i < 9; ++i) {
        UINT8_TO_STREAM(p, 0xFF);
    }
    UINT8_TO_STREAM(p, 0x7F);
}

esp_err_t switch1_csb_rx_start(const uint8_t bd_addr[6], uint64_t bch34,
                               uint16_t interval, uint32_t clock_offset,
                               uint32_t next_clock, uint8_t channel)
{
    /*
     * This API accepts only even interval values.
     * This path does not assign six different phases to one receive position.
     */
    if (bd_addr == NULL || interval < 2 || (interval & 1) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_csb_rx_active) {
        /*
         * If the previous Enable=0 stop is rejected, reception stays active and subsequent starts fail until reboot.
         * Retry stopping and, even if stopping fails, use Enable=1 to reset the start time.
         */
        const esp_err_t stop_err =
            switch1_csb_rx_stop(s_csb_rx_target, interval);
        ESP_LOGW(TAG, "CSB receive remained active before stop; stop result=%s",
                 esp_err_to_name(stop_err));
    }
    /*
     * Enable receive events before starting reception.
     * Without receive events enabled, the controller sends no event notifications.
     * descriptorReception could then be detected only by reading descriptors directly.
     */
    {
        uint8_t mask[8] = {0};
        mask[2] = HCI_EVT_MASK_PAGE_2_CSB;
        const esp_err_t mask_err =
            send_command_wait(HCI_SET_EVT_MASK_PAGE_2, mask, sizeof(mask));
        ESP_LOGI(TAG, "Page 2 event mask result=%s",
                 esp_err_to_name(mask_err));
    }
    uint8_t params[CSB_RX_PARAM_SIZE];
    csb_rx_build_params(params, 1, bd_addr, interval, clock_offset, next_clock);
    const esp_err_t err =
        send_command_wait(HCI_SET_CLB_RECEIVE, params, CSB_RX_PARAM_SIZE);
    if (err != ESP_OK) {
        return err;
    }
    s_csb_rx_bch = bch34;
    s_csb_rx_channel = channel;
    memcpy(s_csb_rx_target, bd_addr, sizeof(s_csb_rx_target));
    s_csb_rx_active = true;
    switch1_csb_rx_reapply();
    /*
     * Do not log here.
     * switch1_csb_rx_startThe Switch may respond immediately after switch1_csb_rx_start returns.
     * UART 4Printing four UART lines (about 20 ms) could delay detection and miss Phase 3 timing, so store only the 24 control-structure words.
     */
    {
        const volatile uint16_t *const cs =
            (const volatile uint16_t *)CSB_RX_CS_ADDR;
        for (int i = 0; i < CSB_RX_START_CS_WORDS; ++i) {
            s_csb_rx_start_cs[i] = cs[i];
        }
        s_csb_rx_start_cs_valid = true;
    }
    return ESP_OK;
}

/*
 * Reapply the settings to the receiver control structure.
 * ROMROM rebuilds settings for each event, so call repeatedly as on the transmitter side.
 * In particular, ROM sets bit 15 (hopping enabled) at +0x16; without reapplying, the receive frequency cycles through 79 channels.
 */
static void switch1_csb_rx_reapply(void)
{
    if (!s_csb_rx_active) {
        return;
    }
    /* ROMROM calculates and writes normal parity from BD_ADDR; replace it with wake-up BCH parity. */
    csb_rx_write_bdaddr(s_csb_rx_target);
    csb_rx_write_bch(s_csb_rx_bch);
    /*
     * SwitchThe Switch response header uses LT_ADDR 0, while the HCI command accepts only values 1-7.
     * control structureWrite 0 directly into the control structure.
     * LT_ADDRLT_ADDR occupies bits 10:8 at +0x08.
     * +0x00Do not change the lower five bits at +0x00, which represent the frame format.
     *
     * control structure +0x08Set bit 12 of receiver control structure +0x08 to disable whitening.
     * SwitchBecause the Switch transmits its response without whitening, set the initial whitening value to 0.
     * By default, the receiver derives the whitening seed from the piconet clock.
     * Without this change, the decoded header varies on each reception.
     */
    {
        volatile uint16_t *const link =
            (volatile uint16_t *)(CSB_RX_CS_ADDR + CS_LTADDR_OFFSET);
        *link = (uint16_t)((*link & ~CS_LTADDR_MASK) |
                           CS_LINKCNTL_WHITENING_DSB);
    }
    /*
     * As for transmission, disable frequency hopping and lock to the Switch's 2402 MHz channel.
     * Otherwise only the receive frequency will hop.
     */
    {
        volatile uint16_t *const hopcntl =
            (volatile uint16_t *)(CSB_RX_CS_ADDR + CS_HOPCNTL_OFFSET);
        uint16_t value = *hopcntl;
        value &= (uint16_t)~CS_HOPCNTL_FH_EN;
        value = (uint16_t)((value & (uint16_t)~CS_HOPCNTL_CHANNEL_MASK) |
                           (uint16_t)(hop_field(s_csb_rx_channel) << 8));
        *hopcntl = value;
    }
}

/*
 * control structureThe control structure contains the hardware-measured receive time.
 * - +0x52, +0x54: +0x52, +0x54: clock
 * - +0x56: +0x56: within-slot position (10 bits)
 * ld_acl_rx_sync(0x4002fbec)ld_acl_rx_sync (0x4002fbec) subtracts 68 us from +0x56 and normalizes to 0-624.
 */
#define CS_RXCLK_LO_OFFSET UINT32_C(0x52)
#define CS_RXCLK_HI_OFFSET UINT32_C(0x54)
#define CS_RXBITOFF_OFFSET UINT32_C(0x56)
// ld_acl_rx_sync(0x4002fc8c)Correction in ld_acl_rx_sync (0x4002fc8c) that subtracts 68 us from the receive-slot position.
#define CS_RXBITOFF_CORRECTION 68

bool switch1_csb_rx_read_sync(uint32_t *out_clk, uint16_t *out_bitoff,
                              uint16_t *out_raw)
{
    const volatile uint16_t *const lo =
        (const volatile uint16_t *)(CSB_RX_CS_ADDR + CS_RXCLK_LO_OFFSET);
    const volatile uint16_t *const hi =
        (const volatile uint16_t *)(CSB_RX_CS_ADDR + CS_RXCLK_HI_OFFSET);
    const volatile uint16_t *const off =
        (const volatile uint16_t *)(CSB_RX_CS_ADDR + CS_RXBITOFF_OFFSET);
    const uint16_t high = *hi;
    if (high >= 0x800) {
        return false;
    }
    uint32_t clk = ((uint32_t)high << 16) | *lo;
    const uint16_t raw = (uint16_t)(*off & 0x3FF);
    if (out_raw != NULL) {
        *out_raw = raw;
    }
    int32_t bitoff = (int32_t)raw - CS_RXBITOFF_CORRECTION;
    while (bitoff < 0) {
        bitoff += 625;
        clk = (clk - 1) & UINT32_C(0x07ffffff);
    }
    while (bitoff >= 625) {
        bitoff -= 625;
        clk = (clk + 1) & UINT32_C(0x07ffffff);
    }
    if (out_clk != NULL) {
        *out_clk = clk;
    }
    if (out_bitoff != NULL) {
        *out_bitoff = (uint16_t)bitoff;
    }
    return true;
}

void switch1_csb_rx_set_format(uint8_t format)
{
/*
 * control structure +0x00Bits 0-4 of control structure +0x00 specify the frame format.
 * - 2, 3: ACL
 * - 4: 4: page
 * - 5: 5: page scan
 * - 24: CSB24: CSB transmission and synchronization train
 * - 25: CSB25: CSB reception and synchronization scan
 *
 * CSBDuring CSB reception the hardware does not store received payload data.
 * ACLSwitching to an ACL format enters the code path that saves payloads.
 */
    volatile uint16_t *const cs = (volatile uint16_t *)CSB_RX_CS_ADDR;
    *cs = (uint16_t)((*cs & ~UINT16_C(0x001F)) | (format & 0x1F));
}

/*
 * Maximum wait for the stop operation to complete.
 * An active CSB receive env structure is not freed until its schedule-element processing finishes.
 * The receive interval is 22 slots (13.75 ms), so wait longer than one cycle.
 */
#define CSB_RX_STOP_WAIT_US 20000

esp_err_t switch1_csb_rx_restart(uint16_t interval, uint32_t next_clock)
{
    if (!s_csb_rx_active) {
        return ESP_ERR_INVALID_STATE;
    }
    /*
     * Send stop first to release ld_csb_rx_env (0x3ffb8278).
     * If this pointer is non-NULL, r_ld_csb_rx_start returns 12 (Command Disallowed).
     * If stop is rejected with 12 (Command Disallowed), the env pointer is already NULL; ignore that stop response and continue starting.
     */
    uint8_t params[CSB_RX_PARAM_SIZE];
    csb_rx_build_params(params, 0, s_csb_rx_target, interval < 2 ? 2 : interval,
                        0, 0);
    g_suppress_command_error_log = true;
    (void)send_command_wait(HCI_SET_CLB_RECEIVE, params, CSB_RX_PARAM_SIZE);
    g_suppress_command_error_log = false;
    /*
     * Stopping does not necessarily finish immediately.
     * r_ld_csb_rx_stop(0x40039bb8)When env+0x3E is 1 (running), r_ld_csb_rx_stop (0x40039bb8) writes 2 at 0x40039bf2 and returns.
     * envThe env is freed after processing its schedule element.
     * Starting before it is freed fails with 12 (Command Disallowed) at 0x40039738; wait until the pointer becomes NULL.
     * Abort this restart attempt if the pointer does not become NULL before the deadline.
     */
    {
        const void *volatile *const env =
            (const void *volatile *)CSB_RX_ENV_ADDR;
        int waited_us = 0;
        while (*env != NULL && waited_us < CSB_RX_STOP_WAIT_US) {
            esp_rom_delay_us(100);
            waited_us += 100;
        }
        if (*env != NULL) {
            return ESP_ERR_TIMEOUT;
        }
    }
    csb_rx_build_params(params, 1, s_csb_rx_target, interval, 0, next_clock);
    const esp_err_t err =
        send_command_wait(HCI_SET_CLB_RECEIVE, params, CSB_RX_PARAM_SIZE);
    if (err != ESP_OK) {
        /*
         * Stopping succeeded, so reception is no longer running.
         * Set s_csb_rx_active to false so the next start is not incorrectly rejected.
         */
        s_csb_rx_active = false;
        return err;
    }
    switch1_csb_rx_reapply();
    return ESP_OK;
}

esp_err_t switch1_csb_rx_stop(const uint8_t bd_addr[6], uint16_t interval)
{
    if (bd_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t params[CSB_RX_PARAM_SIZE];
    csb_rx_build_params(params, 0, bd_addr, interval < 2 ? 2 : interval, 0, 0);
    const esp_err_t err =
        send_command_wait(HCI_SET_CLB_RECEIVE, params, CSB_RX_PARAM_SIZE);
    if (err == ESP_OK) {
        s_csb_rx_active = false;
    }
    return err;
}

/*
 * Receive descriptor layout.
 * ld_csb_rx_frm_cbk(0x400393a4)ld_csb_rx_frm_cbk (0x400393a4) uses the lowest two bits of ld_env[0x1AA]
 * 4to select one of four receive descriptors.
 * Bit 15 of the first word indicates completion.
 * The low 15 bits of the first word address the next descriptor in shared memory.
 * 4The four descriptors are used in a ring.
 */
#define CSB_RX_DESC_ADDR UINT32_C(0x3ffb2382)
#define CSB_RX_DESC_WORDS 7
#define CSB_RX_DESC_COUNT 4
#define CSB_RX_DESC_DONE UINT16_C(0x8000)
// On synchronization failure, the least-significant bit of the second word is set.
#define CSB_RX_STAT_SYNC_ERR UINT16_C(0x0001)
/*
 * ld_csb_rx_frm_cbk(0x400393a4)Packet validity conditions applied to the second word at 0x400394fc in ld_csb_rx_frm_cbk (0x400393a4).
 * - bit0, bit1, bit2, bit6: Bits 0, 1, 2, and 6: if any is set, do not save the payload.
 * - bit0: bit 0: synchronization failure
 * - bit6: LT_ADDRbit 6: LT_ADDR mismatch
 */
#define CSB_RX_STAT_FAIL_MASK UINT16_C(0x0047)
/*
 * - 4Fourth word: payload header.
 *   Length is taken from bits 3-15 of descriptor[CSB_RX_DESC_PLHEADER].
 * - 6Sixth word: shared-memory location of the payload.
 *   ld_csb_rx_frm_cbkRead by ld_csb_rx_frm_cbk at 0x40039509-0x40039527.
 */
#define CSB_RX_DESC_PLHEADER 3
#define CSB_RX_DESC_PLBUF 5
/*
 * - 7The seventh word refers to a different buffer, laid out in 20-byte increments.
 * - 6The buffer pointed to by the sixth word uses 1028-byte increments and holds ACL data.
 * - LLIDHardware is believed to write LLID 3 LMP packets to the seventh-word buffer.
 * - ld_csb_rx_frm_cbkld_csb_rx_frm_cbk reads only the sixth word, so data passed on from ROM is zero.
 */
#define CSB_RX_DESC_LMBUF 6
#define CSB_RX_HIT_MAX 4
#define CSB_RX_HIT_BYTES 32
/*
 * Status is written as soon as header reconstruction completes; an immediate read can occur before the payload has finished arriving.
 * DM1A 17-byte DM1 takes about 300 us including 2/3 FEC, so read again after a delay.
 */
#define CSB_RX_LATE_US 700

struct csb_rx_hit {
    uint16_t desc[CSB_RX_DESC_WORDS];
    uint16_t length;
    uint8_t early[CSB_RX_HIT_BYTES];
    uint8_t late[CSB_RX_HIT_BYTES];
    uint8_t lm_early[CSB_RX_HIT_BYTES];
    uint8_t lm_late[CSB_RX_HIT_BYTES];
};

static void csb_rx_copy_payload(uint8_t *dst, uint16_t em_offset,
                                uint16_t length)
{
    memset(dst, 0, CSB_RX_HIT_BYTES);
    if (em_offset == 0 || (uint32_t)em_offset + length > EM_SIZE) {
        return;
    }
    const volatile uint8_t *const src =
        (const volatile uint8_t *)(EM_BASE + em_offset);
    const uint16_t copy = length > CSB_RX_HIT_BYTES ? CSB_RX_HIT_BYTES : length;
    for (uint16_t i = 0; i < copy; ++i) {
        dst[i] = src[i];
    }
}

static void csb_rx_log_hit(const struct csb_rx_hit *hit, unsigned index)
{
    ESP_LOGI(TAG,
             "  Received descriptor %u: %04x %04x %04x %04x %04x %04x %04x",
             index, hit->desc[0], hit->desc[1], hit->desc[2], hit->desc[3],
             hit->desc[4], hit->desc[5], hit->desc[6]);
    ESP_LOGI(TAG,
             "    header=%03x payload_header=%02x length=%u "
             "LLID=%u payload_shared_memory_offset=%04x",
             hit->desc[2] & 0x3FF, hit->desc[CSB_RX_DESC_PLHEADER] & 0xFF,
             hit->length, hit->desc[CSB_RX_DESC_PLHEADER] & 0x3,
             hit->desc[CSB_RX_DESC_PLBUF]);
    const uint16_t show = hit->length > CSB_RX_HIT_BYTES ? CSB_RX_HIT_BYTES
                                                         : hit->length;
    static const char *const labels[4] = {
        "ACL buffer (immediate)", "ACL buffer (delayed)",
        "LMP buffer (immediate)", "LMP buffer (delayed)"};
    const uint8_t *const sources[4] = {hit->early, hit->late, hit->lm_early,
                                       hit->lm_late};
    char text[CSB_RX_HIT_BYTES * 3 + 1];
    for (int s = 0; s < 4; ++s) {
        int used = 0;
        for (uint16_t i = 0; i < show; ++i) {
            used += snprintf(text + used, sizeof(text) - used, "%02x ",
                             sources[s][i]);
        }
        ESP_LOGI(TAG, "    %s: %s", labels[s], text);
    }
}

/*
 * Comparing only status words misses subsequent consecutive receptions with the same status.
 * descriptor[CSB_RX_DESC_LEVEL]descriptor[CSB_RX_DESC_LEVEL] changes on each reception, so combine it with status to detect changes.
 */
static uint16_t s_csb_rx_poll_last[CSB_RX_DESC_COUNT];
static uint16_t s_csb_rx_poll_last_level[CSB_RX_DESC_COUNT];
#define CSB_RX_DESC_LEVEL 4

/*
 * Collect the types and counts of statuses observed while monitoring.
 * CSB_RX_STAT_SYNC_ERRUse this to distinguish CSB_RX_STAT_SYNC_ERR receptions from CSB_RX_STAT_FAIL_MASK failures when the accepted count is zero.
 */
#define CSB_RX_POLL_KINDS 8
static uint16_t s_csb_rx_poll_seen[CSB_RX_POLL_KINDS];
static uint16_t s_csb_rx_poll_seen_count[CSB_RX_POLL_KINDS];
static uint8_t s_csb_rx_poll_kinds;

void switch1_csb_rx_poll_prime(void)
{
    const volatile uint16_t *const base =
        (const volatile uint16_t *)CSB_RX_DESC_ADDR;
    for (int d = 0; d < CSB_RX_DESC_COUNT; ++d) {
        s_csb_rx_poll_last[d] = base[d * CSB_RX_DESC_WORDS + 1];
        s_csb_rx_poll_last_level[d] =
            base[d * CSB_RX_DESC_WORDS + CSB_RX_DESC_LEVEL];
    }
    s_csb_rx_poll_kinds = 0;
}

static void csb_rx_poll_count(uint16_t stat)
{
    uint8_t i = 0;
    while (i < s_csb_rx_poll_kinds && s_csb_rx_poll_seen[i] != stat) {
        ++i;
    }
    if (i < s_csb_rx_poll_kinds) {
        if (s_csb_rx_poll_seen_count[i] < 0xFFFF) {
            s_csb_rx_poll_seen_count[i]++;
        }
        return;
    }
    if (s_csb_rx_poll_kinds >= CSB_RX_POLL_KINDS) {
        return;
    }
    s_csb_rx_poll_seen[s_csb_rx_poll_kinds] = stat;
    s_csb_rx_poll_seen_count[s_csb_rx_poll_kinds] = 1;
    s_csb_rx_poll_kinds++;
}

bool switch1_csb_rx_is_active(void)
{
    return s_csb_rx_active;
}

void switch1_csb_rx_poll_report(void)
{
    if (s_csb_rx_ltaddr_valid) {
        s_csb_rx_ltaddr_valid = false;
        ESP_LOGI(TAG, "CSB receive LT_ADDR set to %u (+0x08=%04x)",
                 s_csb_rx_ltaddr_written, (unsigned)s_csb_rx_ltaddr_readback);
    }
    if (s_csb_rx_start_cs_valid) {
        s_csb_rx_start_cs_valid = false;
        char text[8 * 5 + 1];
        for (int base = 0; base < CSB_RX_START_CS_WORDS; base += 8) {
            int used = 0;
            for (int i = 0; i < 8; ++i) {
                used += snprintf(text + used, sizeof(text) - used, "%04x ",
                                 s_csb_rx_start_cs[base + i]);
            }
            ESP_LOGI(TAG, "  Initial receive control structure +%02x: %s", base * 2,
                     text);
        }
    }
    for (uint8_t i = 0; i < s_csb_rx_poll_kinds; ++i) {
        ESP_LOGI(TAG, "  Status %04x detected %u times (sync=%s)",
                 s_csb_rx_poll_seen[i], s_csb_rx_poll_seen_count[i],
                 (s_csb_rx_poll_seen[i] & CSB_RX_STAT_SYNC_ERR) ? "failed"
                                                                : "succeeded");
    }
    if (s_csb_rx_poll_kinds == 0) {
        ESP_LOGI(TAG, "  No status changes");
    }
}

bool switch1_csb_rx_poll(uint16_t *out_status, uint16_t *out_header,
                         uint16_t *out_pheader)
{
    const volatile uint16_t *const base =
        (const volatile uint16_t *)CSB_RX_DESC_ADDR;
    for (int d = 0; d < CSB_RX_DESC_COUNT; ++d) {
        const uint16_t stat = base[d * CSB_RX_DESC_WORDS + 1];
        const uint16_t level = base[d * CSB_RX_DESC_WORDS + CSB_RX_DESC_LEVEL];
        if (stat == s_csb_rx_poll_last[d] &&
            level == s_csb_rx_poll_last_level[d]) {
            continue;
        }
        s_csb_rx_poll_last[d] = stat;
        s_csb_rx_poll_last_level[d] = level;
        csb_rx_poll_count(stat);
        if (stat == 0 || (stat & CSB_RX_STAT_FAIL_MASK) != 0) {
            continue;
        }
        if (out_status != NULL) {
            *out_status = stat;
        }
        if (out_header != NULL) {
            *out_header = base[d * CSB_RX_DESC_WORDS + 2];
        }
        if (out_pheader != NULL) {
            *out_pheader = base[d * CSB_RX_DESC_WORDS + CSB_RX_DESC_PLHEADER];
        }
        return true;
    }
    return false;
}

uint32_t switch1_csb_rx_watch(uint32_t duration_ms)
{
    const volatile uint16_t *const base =
        (const volatile uint16_t *)CSB_RX_DESC_ADDR;
    bool was_done[CSB_RX_DESC_COUNT];
    for (int d = 0; d < CSB_RX_DESC_COUNT; ++d) {
        was_done[d] = (base[d * CSB_RX_DESC_WORDS] & CSB_RX_DESC_DONE) != 0;
    }
    /*
     * Record the status without modifying it.
     * Waiting until the completion bit is set means ROM's interrupt handler processes the status word first, making it unavailable here.
     */
    uint16_t seen[8];
    uint16_t seen_count[8];
    uint8_t kinds = 0;
    uint32_t windows = 0;
    /*
     * statSave receive descriptors with nonzero stat that do not match CSB_RX_STAT_FAIL_MASK.
     * descriptorBy detecting changes per descriptor, this also captures later receptions with identical status words.
     * Initialize each descriptor status to 0 at monitoring start so descriptors completed before monitoring are captured as the first change.
     * Exclude status 0, which is the initial state of an unused descriptor.
     */
    uint16_t last_stat[CSB_RX_DESC_COUNT];
    for (int d = 0; d < CSB_RX_DESC_COUNT; ++d) {
        last_stat[d] = 0;
    }
    /*
     * 1Each entry is 144 bytes.
     * UARTThe UART task has only a 3072-byte stack; allocating four entries there would overflow it and cause a restart.
     * Place the array in static storage.
     * switch1_csb_rx_watchswitch1_csb_rx_watch is called only from the UART task, so sharing the array causes no contention.
     */
    static struct csb_rx_hit hit[CSB_RX_HIT_MAX];
    unsigned hits = 0;
    const int64_t until = esp_timer_get_time() + (int64_t)duration_ms * 1000;
    /*
     * ROMCapture the word at CSB_RX_CS_ADDR+CS_HOPCNTL_OFFSET immediately before reapplying settings, to detect ROM changes.
     */
    uint16_t hop_seen[4];
    uint16_t hop_count[4];
    uint8_t hop_kinds = 0;
    const volatile uint16_t *const hopcntl =
        (const volatile uint16_t *)(CSB_RX_CS_ADDR + CS_HOPCNTL_OFFSET);
    while (esp_timer_get_time() < until) {
        {
            const uint16_t hop = *hopcntl;
            uint8_t h = 0;
            while (h < hop_kinds && hop_seen[h] != hop) {
                ++h;
            }
            if (h < hop_kinds) {
                if (hop_count[h] < 0xFFFF) {
                    hop_count[h]++;
                }
            } else if (hop_kinds < 4) {
                hop_seen[hop_kinds] = hop;
                hop_count[hop_kinds] = 1;
                hop_kinds++;
            }
        }
        /*
         * ROM does not rebuild receiver settings here, so leave them unchanged.
         */
        for (int d = 0; d < CSB_RX_DESC_COUNT; ++d) {
            const uint16_t head = base[d * CSB_RX_DESC_WORDS];
            const uint16_t stat = base[d * CSB_RX_DESC_WORDS + 1];
            const bool done = (head & CSB_RX_DESC_DONE) != 0;
            if (done != was_done[d]) {
                was_done[d] = done;
                if (done) {
                    windows++;
                }
            }
            if (stat != last_stat[d]) {
                last_stat[d] = stat;
                if (stat != 0 && (stat & CSB_RX_STAT_FAIL_MASK) == 0 &&
                    hits < CSB_RX_HIT_MAX) {
                    struct csb_rx_hit *const h = &hit[hits];
                    for (int w = 0; w < CSB_RX_DESC_WORDS; ++w) {
                        h->desc[w] = base[d * CSB_RX_DESC_WORDS + w];
                    }
                    h->length = (uint16_t)(
                        (h->desc[CSB_RX_DESC_PLHEADER] >> 3) & 0x1FFF);
                    csb_rx_copy_payload(h->early, h->desc[CSB_RX_DESC_PLBUF],
                                        h->length);
                    csb_rx_copy_payload(h->lm_early, h->desc[CSB_RX_DESC_LMBUF],
                                        h->length);
                    const int64_t wait_until =
                        esp_timer_get_time() + CSB_RX_LATE_US;
                    while (esp_timer_get_time() < wait_until) {
                    }
                    csb_rx_copy_payload(h->late, h->desc[CSB_RX_DESC_PLBUF],
                                        h->length);
                    csb_rx_copy_payload(h->lm_late, h->desc[CSB_RX_DESC_LMBUF],
                                        h->length);
                    hits++;
                }
            }
            uint8_t i = 0;
            while (i < kinds && seen[i] != stat) {
                ++i;
            }
            if (i < kinds) {
                if (seen_count[i] < 0xFFFF) {
                    seen_count[i]++;
                }
                continue;
            }
            if (kinds >= 8) {
                continue;
            }
            seen[kinds] = stat;
            seen_count[kinds] = 1;
            kinds++;
        }
    }
    for (unsigned i = 0; i < hits; ++i) {
        csb_rx_log_hit(&hit[i], i);
    }
    for (int d = 0; d < CSB_RX_DESC_COUNT; ++d) {
        ESP_LOGI(TAG,
                 "  Descriptor %d at end of watch: %04x %04x %04x %04x %04x "
                 "%04x %04x", d, base[d * CSB_RX_DESC_WORDS],
                 base[d * CSB_RX_DESC_WORDS + 1], base[d * CSB_RX_DESC_WORDS + 2],
                 base[d * CSB_RX_DESC_WORDS + 3], base[d * CSB_RX_DESC_WORDS + 4],
                 base[d * CSB_RX_DESC_WORDS + 5], base[d * CSB_RX_DESC_WORDS + 6]);
    }
    ESP_LOGI(TAG, "  Saved packets=%u", hits);
    uint32_t synced = 0;
    for (uint8_t i = 0; i < kinds; ++i) {
        if (seen[i] != 0 && (seen[i] & CSB_RX_STAT_SYNC_ERR) == 0) {
            synced += seen_count[i];
        }
        ESP_LOGI(TAG, "  Status %04x detected %u times (sync=%s)", seen[i],
                 seen_count[i],
                 (seen[i] & CSB_RX_STAT_SYNC_ERR) ? "failed" : "succeeded");
    }
    for (uint8_t h = 0; h < hop_kinds; ++h) {
        ESP_LOGI(TAG, "  Hop control before reconfiguration %04x detected %u times",
                 hop_seen[h], hop_count[h]);
    }
    ESP_LOGI(TAG, "  Receive-completion notifications=%lu, status types=%u",
             (unsigned long)windows, kinds);
    return synced;
}

void switch1_csb_rx_write_ltaddr(uint8_t lt_addr)
{
    volatile uint16_t *const link =
        (volatile uint16_t *)(CSB_RX_CS_ADDR + CS_LTADDR_OFFSET);
    *link = (uint16_t)((*link & ~CS_LTADDR_MASK) |
                       (uint16_t)(((uint16_t)lt_addr << 8) & CS_LTADDR_MASK));
    s_csb_rx_ltaddr_written = lt_addr;
    s_csb_rx_ltaddr_readback = *link;
    s_csb_rx_ltaddr_valid = true;
}

esp_err_t switch1_csb_rx_dump(uint16_t offset, uint8_t count, uint16_t *out)
{
    if (out == NULL || count == 0 || count > 32) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint32_t base = CSB_RX_CS_ADDR + offset;
    if (base < EM_BASE || base + (uint32_t)count * 2 > EM_BASE + EM_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    const volatile uint16_t *const words = (const volatile uint16_t *)base;
    for (uint8_t i = 0; i < count; ++i) {
        out[i] = words[i];
    }
    return ESP_OK;
}


/*
 * CSBSave CSB reception HCI events (0x51).
 * bluedroidBluedroid does not handle this event; the event handler in bb_hci.c receives it and forwards it to bb_csb_rx_on_event.
 *
 * Event fields:
 * - BD_ADDR(6)
 * - LT_ADDR(1)
 * - CLK(4)
 * - Offset(4)
 * - Receive_Status(1)
 * - Fragment(1)
 * - Data_Length(1)
 * - Data(Data (variable length)
 */
void bb_csb_rx_on_event(BT_HDR *p_msg)
{
    if (p_msg != NULL && p_msg->len >= 2) {
        const uint8_t *const p = p_msg->data + p_msg->offset;
        if (p[0] == HCI_CSB_RECEIVE_EVT && p_msg->len >= 20) {
            for (int i = 0; i < 6; ++i) {
                s_csb_rx_addr[i] = p[2 + 5 - i];
            }
            s_csb_rx_clock = (uint32_t)p[9] | ((uint32_t)p[10] << 8) |
                             ((uint32_t)p[11] << 16) | ((uint32_t)p[12] << 24);
            s_csb_rx_status = p[17];
            const uint8_t length = p[19];
            s_csb_rx_length = length > 32 ? 32 : length;
            const uint16_t available = (uint16_t)(p_msg->len - 20);
            const uint8_t copy = s_csb_rx_length > available
                                     ? (uint8_t)available
                                     : s_csb_rx_length;
            memcpy(s_csb_rx_data, p + 20, copy);
            s_csb_rx_length = copy;
            s_csb_rx_packet_pending = true;
            // Even failed synchronization attempts are reported as events.
            if (s_csb_rx_status == 0) {
                s_csb_rx_event_count++;
            }
        }
    }
}

uint32_t switch1_csb_rx_event_count(void)
{
    return s_csb_rx_event_count;
}

void switch1_csb_rx_event_reset(void)
{
    s_csb_rx_event_count = 0;
}


bool switch1_csb_rx_take(uint8_t out_addr[6], uint32_t *out_clock,
                         uint8_t *out_status, uint8_t *out_data,
                         uint8_t *out_length)
{
    if (!s_csb_rx_packet_pending) {
        return false;
    }
    if (out_addr != NULL) {
        memcpy(out_addr, s_csb_rx_addr, sizeof(s_csb_rx_addr));
    }
    if (out_clock != NULL) {
        *out_clock = s_csb_rx_clock;
    }
    if (out_status != NULL) {
        *out_status = s_csb_rx_status;
    }
    if (out_length != NULL) {
        const uint8_t copy = s_csb_rx_length > 32 ? 32 : s_csb_rx_length;
        if (out_data != NULL) {
            memcpy(out_data, s_csb_rx_data, copy);
        }
        *out_length = copy;
    }
    s_csb_rx_packet_pending = false;
    return true;
}
