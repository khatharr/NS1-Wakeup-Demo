/**
 * @brief Bluetooth connection state shared between modules
 * @note Definitions are in swpro_link.c
 * @note Expose only variables referenced by the wake-up module and connection cycle
 * @note Mark state read or written by tasks on another core as volatile
 */
#ifndef LINK_STATE_H
#define LINK_STATE_H

#include <stdbool.h>
#include <stdint.h>

extern volatile bool g_hid_connected;

extern volatile bool g_acl_connected;

/**
 * @brief Flag that forbids automatic paging
 * @note Each paging attempt waits 3.2 seconds for a timeout and transmits paging radio signals throughout that interval.
 *       If paging is in progress, enabling CSB is rejected with status 0x0b; stop automatic paging before transmitting wake-up CSB.
 */
extern volatile bool g_reconnect_disabled;

/**
 * @brief Flag to temporarily suspend automatic paging
 * @note Set to true while wake-up CSB is being transmitted
 */
extern volatile bool g_reconnect_paused;

extern int64_t g_next_reconnect_us;

/**
 * @brief HIDDeadline for an HID connection request
 * @note 00 means no HID connection request is in progress
 */
extern int64_t g_connect_deadline_us;

extern volatile bool g_cycle_enabled;

/**
 * @brief 1Number of wake-up attempts in one connection cycle
 * @note Store in RTC memory to preserve it across restarts
 */
extern uint32_t g_cycle_wake_tries;

/**
 * @brief Number of wake-up attempts before switching to paging
 * @note HOMEWhen a connection cycle starts from HOME input, the Switch may not have finished entering sleep.
 *       Wake-up will then fail, so retry until the sleep transition completes.
 *       1Each attempt requires a 1.2-second restart and 2 seconds of wake-up processing; six attempts take approximately 20 seconds.
 */
#define CYCLE_WAKE_TRIES 3

/** @brief HCI Create ConnectionBit indicating that Clock_Offset is valid in HCI Create Connection */
#define HCI_CLOCK_OFFSET_VALID UINT16_C(0x8000)

/**
 * @brief HIDDeadline from starting an HID connection request until its result event arrives
 * @note BluedroidIf Bluedroid's HID state never leaves CONNECTING, subsequent requests are rejected as BUSY.
 *       After the deadline, request an HID disconnect to clear the internal state.
 * @note Set slightly longer than the 3.2-second paging timeout
 */
#define CONNECT_TIMEOUT_US UINT64_C(4000000)

void try_reconnect(void);

#endif
