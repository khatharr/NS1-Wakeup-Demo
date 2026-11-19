#ifndef BB_ROM_H
#define BB_ROM_H

#include <stdbool.h>
#include <stdint.h>

#include "stack/bt_types.h"


/*
 * This is memory shared between baseband hardware and the CPU.
 * ROM symbols use the em_ prefix.
 *
 * - It is a 32 KB region starting at 0x3ffb0000.
 * - Below, this region is called shared memory.
 */
#define CSB_CS_ADDR UINT32_C(0x3ffb22b6)
#define CS_BDADDR_OFFSET UINT32_C(0x0A)
/*
 * LT_ADDR is stored in bits 10:8 of this word.
 * r_ld_csb_rx_start and r_ld_csb_tx_start write LT_ADDR into this word.
 */
#define CS_LTADDR_OFFSET UINT32_C(0x08)
#define CS_LTADDR_MASK UINT16_C(0x0700)
/*
 * Bit 12 of the same word disables whitening.
 * In the branch table of ld_acl_test_mode_update (0x40032050), only test cases without whitening set this bit and perform the main operation.
 * r_ld_csb_tx_en clears this bit, but r_ld_csb_rx_start does not alter it, so it can be set explicitly for reception.
 */
#define CS_LINKCNTL_WHITENING_DSB UINT16_C(0x1000)
#define CS_BCH_OFFSET UINT32_C(0x10)
/*
 * These upper bits of the third BCH word are written only by the receiver.
 * At 0x4003985a-0x4003986f in r_ld_csb_rx_start (0x4003972c), (upper two parity bits) | 0xFF40 is written.
 */
#define CS_BCH_RX_FLAGS UINT16_C(0xFF40)
#define CS_HOPCNTL_OFFSET UINT32_C(0x16)
#define CS_TXDESC_OFFSET UINT32_C(0x1C)
#define CS_HOPCNTL_FH_EN UINT16_C(0x8000)
#define CS_HOPCNTL_CHANNEL_MASK UINT16_C(0x7F00)
#define EM_BASE UINT32_C(0x3ffb0000)
#define EM_SIZE UINT16_C(0x8000)
#define CSB_DESC_SIZE UINT16_C(10)

/*
 * Native-clock sampling register of the Bluetooth core.
 * Based on r_ld_read_clock (0x4003c9e4).
 * Writing the register value with bit 31 set starts sampling; hardware clears bit 31 when done. The lower 28 bits are CLKN (units of 312.5 us).
 */
#define BT_CLKN_SAMPLE_REG UINT32_C(0x3ff7101c)
#define BT_CLKN_SAMPLE_REQ UINT32_C(0x80000000)
#define BT_CLKN_VALUE_MASK UINT32_C(0x0fffffff)
/* Sampling completes in a few hundred ns; limit the number of polling iterations. */
#define BT_CLKN_POLL_LIMIT 64

#define CSB_LT_ADDR UINT8_C(1)
#define CSB_SUPERVISION_TIMEOUT UINT16_C(0x0640)
/*
 * Separate the CSB receive supervision timeout from the transmit timeout.
 * ld_csb_rx_sched frees the receive schedule element once CSB_RX_SUP_TIMEOUT has elapsed.
 * The shared transmit value 0x0640 equals 1000 ms, shorter than a 1200 ms monitoring run; use a timeout corresponding to 2560 ms.
 */
#define CSB_RX_SUPERVISION_TIMEOUT UINT16_C(0x1000)
#define CSB_EDR_EXCLUSION_MASK                                               \
    (HCI_PKT_TYPES_MASK_NO_2_DH1 | HCI_PKT_TYPES_MASK_NO_3_DH1 |           \
     HCI_PKT_TYPES_MASK_NO_2_DH3 | HCI_PKT_TYPES_MASK_NO_3_DH3 |           \
     HCI_PKT_TYPES_MASK_NO_2_DH5 | HCI_PKT_TYPES_MASK_NO_3_DH5)
/*
 * Packet-type mask for the DH5 path with repeated wake-up codes in its payload.
 * Permit only DH5 initially; if HCI_SET_CLB rejects it, add DH3 and DH1 and retry once.
 */
#define CSB_PACKET_MASK_DH5 (CSB_EDR_EXCLUSION_MASK | HCI_PKT_TYPES_MASK_DH5)
#define CSB_PACKET_MASK_DH5_RETRY                                            \
    (CSB_PACKET_MASK_DH5 | HCI_PKT_TYPES_MASK_DH3 | HCI_PKT_TYPES_MASK_DH1)
/*
 * The DH3 path uses three slots, setting a minimum transmit interval of 3.75 ms.
 * The maximum payload-body length is 183 bytes.
 */
#define CSB_PACKET_MASK_DH3 (CSB_EDR_EXCLUSION_MASK | HCI_PKT_TYPES_MASK_DH3)
#define CSB_DH3_BODY_MAX UINT16_C(183)
/*
 * HCI_WRITE_CLB_DATA uses a one-byte parameter length, limiting the entire command to 255 bytes.
 * Subtracting three bytes for LT_ADDR, Fragment, and Data_Length leaves 252 bytes per fragment.
 */
#define CSB_FRAGMENT_MAX UINT16_C(252)


/*
 * HCI event that reports CSB reception.
 * Bluedroid does not handle it; the event handler reads the received data.
 */
#define HCI_CSB_RECEIVE_EVT UINT8_C(0x51)

extern uint8_t r_ld_pscan_stop(void);
extern uint8_t r_ld_csb_rx_stop(void);
extern uint32_t r_ld_read_clock(void);
extern uint8_t r_ea_elt_remove(void *element);
extern uint8_t r_ea_elt_insert(void *element);
typedef void (*controller_work_t)(void *arg);
extern int btdm_dispatch_work_to_controller(controller_work_t callback,
                                             void *arg, bool blocking);
#define ACL_CLOCK_MASK UINT32_C(0x07ffffff)

/*
 * Pointer to the CSB receive schedule element.
 * The ROM symbol is ld_csb_rx_env.
 *
 * - Located four bytes before the CSB transmit equivalent.
 * - r_ld_csb_rx_start (0x4003972c) stores its allocated element pointer at the location designated by CSB_RX_ENV_ADDR.
 * - If the existing CSB_RX_ENV_ADDR pointer is nonzero, return 12 (Command Disallowed).
 * - The pointer becomes zero not only on stop, but also on supervision timeout.
 * - Based on ld_csb_rx_sched (0x400391f0).
 * - CSB_RX_ENV_ADDR being zero does not necessarily mean reception was never started.
 */
#define CSB_RX_ENV_ADDR UINT32_C(0x3ffb8278)


/*
 * - Begin the normal controller path used just after Page Scan receives FHS, timed to wake-up Phase 3.
 * - Corresponding ROM routines: ld_pscan_frm_cbk and lm_page_scan_end_ind_handler.
 */
#define ACL_LINK_COUNT 7
#define ACL_CS_BASE UINT32_C(0x3ffb1dee)
#define ACL_CS_STRIDE UINT32_C(0x66)
#define ACL_CS_FORMAT_PERIPHERAL UINT16_C(3)
#define PSCAN_RX_START_SLOTS UINT32_C(1)
#define PSCAN_RX_TARGET_SLOTS UINT32_C(3)
#define PSCAN_RX_DEADLINE_SLOTS UINT32_C(5)
#define PSCAN_RX_WINDOW_US UINT16_C(2500)

extern uint8_t hci_env[];
/*
 * There is no need to replace name-request processing.
 * The name-request hook is unused because lc_con_req_controller now sets lc_env+0x56, which had caused disconnections.
 */
extern void *lc_env[];
extern uint8_t lc_state[];
extern void *ld_acl_env[];
/* Registering an ACL link from the wake-up response makes the ESP32 a Bluetooth peripheral on that ACL link. */
#define ACL_ROLE_PERIPHERAL UINT8_C(1)

typedef struct {
    uint8_t addr[6];
} lc_bd_addr_t;
/**
 * r_lc_start (0x4001ca28) allocates the structures needed for an ACL link with the ESP32 as Bluetooth peripheral.
 * It calls r_ld_acl_start at the end, so the caller must not invoke r_ld_acl_start first.
 * Calling it first populates ld_acl_env[link], causing r_lc_start's internal call to fail in internal link state 12.
 *
 * The function has seven arguments. BD_ADDR is passed as a six-byte value using a4 and a5; the last two arguments go on the stack.
 *
 * - link_id: a2; link index, must be below 7 or an assertion occurs
 * - reason: a3; 0 = central, 1 = page-scan success, 2 = Remote Name Request
 * - bd_addr: a4,a5; peer BD_ADDR in FHS order (least-significant byte first)
 * - packet_type: a6; not consulted when reason is 1
 * - allow_role_switch: a7; not consulted when reason is 1
 * - class_of_device: SP+0; low three bytes copied to lc_env+54-56
 * - link_policy: SP+4; default link-policy settings stored at lc_env+40
 *
 * When reason is 1, return with internal link state set to 5 and no LMP transmitted.
 * HCI Connection Complete is delivered when subsequent LMP processing calls r_lc_con_cmp.
 */
typedef void (*lc_start_fn_t)(uint8_t link_id, uint8_t reason,
                              lc_bd_addr_t bd_addr, uint16_t packet_type,
                              uint8_t allow_role_switch,
                              uint32_t class_of_device, uint32_t link_policy);
/**
 * r_hci_bt_acl_bdaddr_register (ROM 0x40018900) sets hci_env (0x3ffb9350) at 16+7*link to 1 and copies six BD_ADDR bytes to 17+7*link.
 * An assertion occurs if 16+7*link is already nonzero; always check it is zero before calling.
 */
typedef void (*hci_bdaddr_register_fn_t)(uint8_t link_id,
                                         const uint8_t *bd_addr);
/*
 * ip_funcs is the function-pointer table referenced by r_ip_funcs_p (0x3ffae70c).
 * ESP-IDF replaces +0x2b4 with r_lc_start_hack and initializes library-side arrays after calling the ROM implementation.
 * Direct invocation of the ROM address skips that initialization, so always call through the table.
 */
#define IP_FUNCS_HCI_BDADDR_REGISTER_INDEX (UINT16_C(0x174) / sizeof(void *))
#define IP_FUNCS_LC_START_INDEX (UINT16_C(0x2b4) / sizeof(void *))
/* Per-link records in lm_env; starting offset is 54 and each link uses nine bytes. */
#define LM_LINK_RECORD_BASE 54
#define LM_LINK_RECORD_STRIDE 9
#define LM_LINK_STATE_OFFSET 6
#define LM_LINK_ROLE_OFFSET 7
#define LM_LINK_STATE_PSCAN_RESERVED UINT8_C(2)
#define LM_LINK_STATE_CONNECTED UINT8_C(3)
/*
 * Storage for HCI Write Default Link Policy Settings.
 * ROM reads this location as a single byte with l8ui and passes it to r_lc_start.
 */
#define LM_ENV_DEFAULT_LINK_POLICY 42
#define LC_START_REASON_PAGE_SCAN UINT8_C(1)
/* The control structure is 102 bytes (51 words). */
#define ACL_CS_SNAPSHOT_WORDS 51
/* Bytes used to save the link structure; equal to the allocation size of r_ld_acl_start. */
#define ACL_ENV_SNAPSHOT_BYTES 252
/*
 * Number of bytes saved from lc_env.
 * Matches the region allocated by r_lc_start via r_ke_malloc(0x354, 0) at 0x4001ca74.
 * After an LMP is received, peer version and feature bits are stored here.
 */
#define LC_ENV_SNAPSHOT_BYTES 852
extern void *llc_env[];
extern uint8_t lm_env[];
extern uint8_t ld_env[];
extern uint8_t *volatile ld_pscan_env;
extern uint8_t *volatile ld_csb_rx_env;
extern uint8_t rwip_priority[];


enum {
    ACL_HANDOFF_IDLE,
    ACL_HANDOFF_PENDING,
    ACL_HANDOFF_DONE,
};

enum {
    ACL_PREPARE_IDLE,
    ACL_PREPARE_PENDING,
    ACL_PREPARE_DONE,
};

typedef struct {
    uint8_t target[6];
    uint64_t bch34;
    uint32_t clock_delta;
    uint32_t rx_clk;
    uint32_t phase3_slot;
    uint16_t rx_bitoff;
    uint8_t lt_addr;
    volatile uint8_t prepare_state;
    volatile uint8_t reserved_link;
    volatile esp_err_t prepare_result;
    volatile uint8_t state;
    volatile uint8_t link_id;
    volatile esp_err_t result;
    volatile bool pscan_rx_active;
    volatile bool pscan_rx_dm1;
    volatile bool pscan_rx_reported;
    volatile uint16_t pscan_rx_reject;
    volatile uint16_t pscan_rx_status;
    volatile uint16_t pscan_rx_header;
    volatile uint16_t pscan_rx_pheader;
    volatile uint32_t pscan_rx_insert_now;
    /*
     * - LC state after r_lc_start; output after ACL link creation finishes.
     * - Because r_lc_start has no return value, determine success by checking whether its structures were created.
     */
    volatile uint8_t lc_state_after;
    volatile uint8_t lc_start_flags;
    /*
     * - Whether internal message 0x518 requesting LMP_host_connection_req was sent to the link-control task.
     * - Internal link state at transmission; ROM processes it when the state is 2 or 5.
     */
    volatile bool host_connection_request_posted;
    volatile uint8_t host_connection_state_before;
    volatile uint8_t host_connection_state_after;
    volatile bool lc_start_called;
    /* - Whether control structure +0x60 was restored to its pre-Phase-2 value, and the +0x60 values before and after restoration. */
    volatile bool cs60_restored;
    volatile uint16_t cs60_before;
    volatile uint16_t cs60_after;
    /*
     * - ACL control structure captured immediately after creating the target ACL link.
     * - Compare later with the control structure to determine if hardware performed reception.
     */
    volatile uint16_t cs_at_start[ACL_CS_SNAPSHOT_WORDS];
    /*
     * - ACL control structure before this module modifies it.
     * - Save to determine whether page-scan initialization values or Phase 2 settings persist.
     */
    volatile uint16_t cs_before[ACL_CS_SNAPSHOT_WORDS];
    /*
     * - Save the link structure for the target ACL link.
     * - Region allocated by r_ld_acl_start via r_ke_malloc(252, 0) at 0x40034dfc.
     * - Compare fields that change on each reception or transmission later.
     */
    volatile uint8_t acl_env_at_start[ACL_ENV_SNAPSHOT_BYTES];
    volatile uint8_t lc_env_at_start[LC_ENV_SNAPSHOT_BYTES];
} acl_handoff_request_t;

#define LC_START_FLAG_LC_ENV UINT8_C(1 << 0)
#define LC_START_FLAG_LD_ACL_ENV UINT8_C(1 << 1)
#define LC_START_FLAG_LLC_ENV UINT8_C(1 << 2)
#define LC_START_FLAG_HCI_BDADDR UINT8_C(1 << 3)

/*
 * - Only on one CSB RX of the Switch wake-up response, capture the LLID 3 body and receive timestamp before the normal callback frees the descriptor.
 */
#define PHASE2_FM_PROG_PUSH_INDEX (UINT16_C(0x658) / sizeof(void *))
#define PHASE2_ROM_IP_FUNCS_PTR_ADDR UINT32_C(0x3ffae70c)
#define PHASE2_ROM_CSB_RX_SCHED_ADDR UINT32_C(0x400390d0)
#define PHASE2_CSB_CALLBACK_ADDR UINT32_C(0x400393a4)
#define PHASE2_PSCAN_CALLBACK_ADDR UINT32_C(0x4003ebe4)
#define PHASE2_CSB_ACTIVITY UINT8_C(13)
#define PHASE2_DIRECT_FORMAT UINT16_C(3)
#define PHASE2_LINKCNTL_EDR UINT16_C(0x0800)
#define PHASE2_LINKCNTL_AFH UINT16_C(0x4000)
#define PHASE2_CSB_CS_ADDR UINT32_C(0x3ffb231c)
#define PHASE2_DESC_ADDR UINT32_C(0x3ffb2382)
#define PHASE2_DESC_WORDS 7
#define PHASE2_CS_WORDS 16
#define PHASE2_DESC_DONE UINT16_C(0x8000)
#define PHASE2_STATUS_FAIL_MASK UINT16_C(0x0047)
#define PHASE2_HEADER UINT16_C(0x0298)
#define PHASE2_PHEADER UINT16_C(0x008b)
#define PHASE2_LENGTH 17
#define PHASE2_RXCLK_LO_WORD (UINT16_C(0x52) / 2)
#define PHASE2_RXCLK_HI_WORD (UINT16_C(0x54) / 2)
#define PHASE2_RXBITOFF_WORD (UINT16_C(0x56) / 2)
#define PHASE2_RXBITOFF_CORRECTION 68
/*
 * Pointer to the CSB receive schedule element (ld_csb_rx_env), and the ROM routine that frees it.
 * Used to perform the same cleanup as ROM from the replacement callback.
 */
#define PHASE2_ROM_CSB_RX_END_ADDR UINT32_C(0x40039098)
/* Supervision-timeout reference clock at schedule element +0x2C and state at +0x3E. */
#define PHASE2_ELT_SUP_BASE 0x2C
#define PHASE2_ELT_STATE 0x3E
#define PHASE2_ELT_STATE_STOPPING UINT8_C(2)

typedef void (*phase2_frame_cb_t)(uint32_t timestamp, uint32_t activity,
                                  uint32_t arg, uint32_t reason);
typedef void (*phase2_prog_push_t)(uint32_t timestamp, uint32_t activity,
                                   uint32_t mode, uint32_t arg,
                                   phase2_frame_cb_t callback);
typedef void (*phase2_csb_rx_sched_t)(uint8_t skipped);
typedef void (*phase2_csb_rx_end_t)(void);

enum {
    PHASE2_CAPTURE_IDLE,
    PHASE2_CAPTURE_STARTING,
    PHASE2_CAPTURE_WAITING,
    PHASE2_CAPTURE_DONE,
};

typedef struct {
    uint8_t target[6];
    uint64_t bch34;
    volatile uint8_t state;
    volatile bool active;
    volatile esp_err_t result;
    volatile uint8_t link;
    volatile uint8_t desc_index;
    volatile uint32_t programmed;
    volatile uint32_t push_count;
    volatile uint32_t last_activity;
    volatile uintptr_t last_callback;
    volatile uint16_t last_format;
    volatile uint32_t reasons[4];
    uint16_t desc[PHASE2_DESC_WORDS];
    uint16_t control[PHASE2_CS_WORDS];
    /*
     * - Capture the last four words of the control structure (+0x5E-+0x64) before Phase 2 reception begins and after the body is retrieved.
     * - Hardware updates this region on each reception; use it to inspect Phase 2 results.
     */
    uint16_t tail_first[4];
    uint16_t tail_last[4];
    volatile bool tail_first_taken;
    uint16_t payload_offset;
    uint8_t payload[PHASE2_LENGTH];
    uint32_t rx_clk;
    uint16_t rx_bitoff;
} phase2_capture_t;
extern void ld_pscan_assert_err_handle(void);

#define KE_TASK_LC UINT16_C(1)
#define IP_FUNCS_BUF_LMP_TX_ALLOC_INDEX (UINT16_C(0x004) / sizeof(void *))
#define IP_FUNCS_LC_SEND_LMP_INDEX (UINT16_C(0x208) / sizeof(void *))
#define IP_FUNCS_LC_SEND_PDU_VERS_REQ_INDEX (UINT16_C(0x2a0) / sizeof(void *))
#define IP_FUNCS_LC_START_LMP_TO_INDEX (UINT16_C(0x2b8) / sizeof(void *))
#define IP_FUNCS_LC_VERSION_INDEX (UINT16_C(0x43c) / sizeof(void *))
#define IP_FUNCS_LD_ACL_LMP_TX_INDEX (UINT16_C(0x540) / sizeof(void *))
#define ROM_KE_MSG_SEND_BASIC_ADDR UINT32_C(0x4001a26c)
#define ROM_KE_STATE_GET_ADDR UINT32_C(0x4001a7d8)
#define ROM_KE_STATE_SET_ADDR UINT32_C(0x4001a6fc)
/*
 * hci_accept_con_req_cmd_handler (0x40024ab4) processes HCI Accept Connection Request only when the internal link state is 12 or 6.
 * ROM's path for processing a Switch request also sets the internal state to 12 after notifying the host.
 */
#define LC_STATE_WAIT_ACCEPT 12
/* Waiting state of the target ACL link with ESP32 as Bluetooth peripheral; used when r_lc_start's second argument is 1. */
#define LC_STATE_PERIPHERAL_IDLE 5

typedef void (*ke_msg_send_basic_fn_t)(uint16_t id, uint16_t dest_id,
                                       uint16_t src_id);
typedef uint8_t (*ke_state_get_fn_t)(uint16_t id);

/**
  * Send an HCI Connection Request event to the host.
  *
  * - Generate the event in the same format as lmp_host_con_req_handler (0x4002b3d8) when handling a Switch request.
  * - Call r_ke_msg_alloc (id=0x803, dest=link index, src=4, length=10) and store BD_ADDR and Class of Device.
  * - Set type to 1 (ACL) and pass to r_hci_send_2_host; id and src determine the event type.
  *
  * - When the ESP32 sends LMP_host_connection_req as a Bluetooth peripheral, the controller reports only Connection Complete and not this event; host L2CAP then disconnects the target ACL link.
  * - Reporting Connection Request first makes the host allocate a control block for a normal incoming connection.
 */
#define ROM_KE_MSG_ALLOC_ADDR UINT32_C(0x4001a1e0)
#define KE_MSG_HCI_EVENT UINT16_C(0x803)
#define KE_SRC_HCI_CON_REQ UINT16_C(4)
#define IP_FUNCS_HCI_SEND_2_HOST_INDEX (UINT16_C(0x170) / sizeof(void *))
#define HCI_CON_REQ_PARAM_LEN 10

typedef void *(*ke_msg_alloc_fn_t)(uint16_t id, uint16_t dest_id,
                                   uint16_t src_id, uint16_t param_len);
typedef void (*hci_send_2_host_fn_t)(void *param);
typedef void (*ke_state_set_fn_t)(uint16_t id, uint8_t state);

typedef struct {
    volatile bool requested;
    volatile bool done;
    volatile bool con_req_sent;
    volatile uint8_t link;
    volatile uint16_t dest_id;
    volatile uint8_t ke_state_before;
    volatile uint8_t ke_state_after;
    volatile uint8_t lc_state_before;
    volatile uint8_t lc_state_after;
    volatile uint8_t lc_env_60;
    volatile uint8_t lc_env_850;
    volatile uint32_t fn_lc_version;
    volatile uint32_t fn_send_pdu_vers_req;
    volatile uint32_t fn_start_lmp_to;
    volatile uint32_t fn_send_lmp;
    volatile uint32_t fn_buf_alloc;
    volatile uint32_t fn_acl_lmp_tx;
} lc_version_req_t;


/**
  * Set the internal link state to 5.
  *
  * - r_lc_hl_connect (0x400209e8) changes the internal link state to 20 after sending LMP_host_connection_req, waiting for the Switch's LMP_accepted.
  * - The Switch does not send LMP_accepted; it sends LMP_host_connection_req after 27 ms.
  * - lmp_host_con_req_handler processes requests only in state 5, so it discards them in state 20 while awaiting LMP_accepted.
  *
  * - Immediately after sending LMP_host_connection_req, restore state 5 to allow processing the Switch's request.
  * - State 5 is the same state set by r_lc_start when creating the target ACL link.
 */
typedef struct {
    volatile bool done;
    volatile uint8_t link;
    volatile uint8_t state_before;
    volatile uint8_t state_after;
} lc_idle_state_request_t;
/*
 * If its second argument is 0, r_lc_start changes internal link state to 2, starts the LMP timeout, and sends 0x513, 0x514, and 0x516 in order to the same link's LC task.
 * If the second argument is 1, it exits with state 5; send those same three messages in order.
 * lc_op_vers_ind_handler (0x40028b20) receives 0x514 and invokes r_lc_version to send LMP_version_req when internal state is 1, 2, or 5.
 */
#define KE_MSG_LC_OP_FEAT_IND UINT16_C(0x513)
#define KE_MSG_LC_OP_VERS_IND UINT16_C(0x514)

#define LC_OP_HL_CNX_IND UINT16_C(0x0518)


#endif
