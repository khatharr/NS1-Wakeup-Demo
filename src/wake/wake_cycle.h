#ifndef WAKE_CYCLE_H
#define WAKE_CYCLE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Earliest time the next connection cycle step may begin
 */
extern int64_t g_cycle_next_us;

/**
 * @brief Start or stop connection-cycle processing
 * @param enabled true starts, false stops
 */
void connect_cycle_set(bool enabled);

/**
 * @brief Advance connection-cycle processing by one stage
 * @param now Current time in microseconds
 *
 * Call once every 1 ms.
 */
void connect_cycle_step(int64_t now);

/**
 * @brief Perform a restart scheduled while connection processing is stopped
 * @param now Current time in microseconds
 *
 * Restart only after HID is disconnected and no other connection operation is active.
 * Used to begin the next attempt from the immediate post-boot state.
 */
void idle_restart_step(int64_t now);

/**
 * @brief Value retained across restarts and read by app_main at startup
 */
#define WAKE_RESTART_MARKER UINT32_C(0x57414B45)
#define CYCLE_RESTART_MARKER UINT32_C(0x43594331)
#define NS2_WAKE_RESTART_MARKER UINT32_C(0x4E533257)
/**
 * @brief Marker communicating the reason for restarting to app_main
 */
extern uint32_t g_restart_marker;

/**
 * @brief Flag indicating boot after a restart immediately following wake-up
 */
extern volatile bool g_page_right_after_boot;

/**
 * @brief Flag indicating a restart while connection attempts remain active
 */
extern volatile bool g_cycle_resume_after_boot;

/**
 * @brief Delay between HID disconnection and restart
 */
#define IDLE_RESTART_DELAY_US INT64_C(3000000)

#endif
