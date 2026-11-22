#ifndef SWPRO_HID_H
#define SWPRO_HID_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_event.h"

/**
 * @brief Length of an input report
 *
 * Always send report ID 0x30 as 48 bytes, regardless of mode.
 */
#define SWPRO_REPORT_SIZE 48

/**
 * @brief Default input-report interval in microseconds
 *
 * @note Increase the interval in sniff mode in response to GAP notifications
 */
#define SWPRO_REPORT_INTERVAL_DEFAULT_US UINT32_C(8000)

typedef struct {
    uint32_t buttons;
    uint32_t stick_left;
    uint32_t stick_right;
    int16_t accel[3];
    int16_t gyro[3];
} swpro_input_t;

/**
 * @brief Register the Bluetooth HID device
 *
 * @param handler Callback invoked by the esp_hid event loop
 *                Provided by swpro_link.c
 * @return ESP-IDF status code
 */
esp_err_t swpro_hid_register(esp_event_handler_t handler);

void swpro_hid_set_input(const swpro_input_t *input);

/**
 * @brief Override the A-button bit in outgoing reports without changing bridge input
 * @param enabled Whether the override is active
 * @param pressed A-button state while the override is active
 */
void swpro_hid_override_a(bool enabled, bool pressed);

uint32_t swpro_hid_input_frames(void);

esp_err_t swpro_hid_send_report(void);

/**
 * @brief Return the current input-report interval in microseconds
 *
 * @return Current interval
 * @note Update in response to GAP mode changes
 */
uint32_t swpro_hid_report_interval_us(void);

/**
 * @brief Configure the input-report interval
 *
 * @param interval_us New interval in microseconds
 */
void swpro_hid_set_report_interval_us(uint32_t interval_us);

void swpro_hid_reset_session(void);

/**
 * @brief Process an output report sent by the Switch
 *
 * @param report_id HID report ID
 * @param data Report body, excluding report ID
 * @param length Number of bytes in data
 * @note Report ID 0x01 has this data layout:
 * - data[0] is the counter
 * - data[1..8] is rumble data
 * - data[9] is the subcommand
 * - data[10..] contains the subcommand arguments
 */
void swpro_hid_handle_output(uint8_t report_id, const uint8_t *data,
                             uint16_t length);

/**
 * @brief Return whether the Switch has requested the stopped state
 *
 * @return true after receiving state 0 in subcommand 0x06, otherwise false
 * @note true while the Switch enters sleep; do not send input reports
 */
bool swpro_hid_host_stopped(void);

#endif
