#ifndef WAKE_SESSION_H
#define WAKE_SESSION_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "baseband.h"
#include "wake_payload.h"

extern volatile bool g_wake_tx_active;

extern volatile bool g_wake_stop_requested;

extern uint32_t g_direct_clock_delta;
extern uint32_t g_direct_rx_clk;
extern uint16_t g_direct_rx_bitoff;
extern uint8_t g_direct_lt_addr;
extern volatile bool g_direct_clock_delta_valid;
extern volatile bool g_direct_sync_valid;

extern volatile bool g_direct_handoff_started;
extern volatile bool g_direct_handoff_disabled;
extern volatile bool g_direct_handoff_active;
extern volatile esp_err_t g_direct_handoff_start_err;
extern volatile int64_t g_direct_handoff_deadline_us;
extern volatile int64_t g_direct_handoff_quiet_until_us;

// bit15: Clock_Offset_Valid_Flag
extern uint16_t g_hci_clock_offset;

void wake_handoff_finished(void);

void wake_tx_task(void *argument);

void wake_finish_tx(const char *reason);

void wake_cmd_start(const uint8_t *payload, uint8_t length);

void wake_cmd_receive(const uint8_t *payload, uint8_t length);

#endif
