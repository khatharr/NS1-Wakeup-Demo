// Define internal APIs and shared state for direct access to controller ROM and shared memory; applications call through baseband.h.
#ifndef BB_INTERNAL_H
#define BB_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "bb_rom.h"

#include "stack/bt_types.h"

esp_err_t send_command_wait(uint16_t opcode, const uint8_t *params,
                            uint8_t length);

extern volatile uint8_t g_command_status;
extern uint8_t g_command_return[8];
extern volatile uint8_t g_command_return_length;

void bb_csb_rx_on_event(BT_HDR *p_msg);

extern volatile bool g_command_pending;
extern volatile bool g_command_timed_out;
// Set true to tolerate command failure and suppress errors on paths where logging would delay monitoring.
extern volatile bool g_suppress_command_error_log;

extern acl_handoff_request_t g_acl_handoff;
extern bool acl_link_context_clear(uint8_t link);

extern phase2_capture_t g_phase2_capture;

/*
 * - After registering the ACL link environment with normal ROM ACL processing, install two hooks to suppress unnecessary ESP32-originated LMP transmissions.
 * - Remove both hooks when the ACL follow-up task ends.
 * - Suppress LMP_not_accepted for opcode 0 and ESP32-initiated packet-type exchanges.
 */
void lmp_not_accepted_hook_install(void);
void lmp_not_accepted_hook_remove(void);
void packet_type_hook_install(void);
void packet_type_hook_remove(void);

void lc_con_req_controller(void *arg);
void lc_set_idle_state_controller(void *arg);
void lc_version_req_controller(void *arg);

bool phase2_install_hook(void);
void phase2_restore_hook(void);

extern volatile bool g_report_verbose;

extern lc_idle_state_request_t g_lc_idle_state_request;
extern volatile bool g_lc_idle_state_dispatched;
extern volatile uint16_t g_packet_type_suppressed;
extern volatile uint16_t g_lmp_not_accepted_suppressed;
extern volatile uint16_t g_lmp_not_accepted_forwarded;

extern lc_version_req_t g_lc_version_req;
extern volatile bool g_con_req_dispatched;

void csb_write_bdaddr(const uint8_t address[6]);
void csb_write_bch(uint64_t bch34);
esp_err_t csb_apply(void);
esp_err_t csb_apply_wait(void);
uint16_t hop_field(uint8_t channel);

#endif
