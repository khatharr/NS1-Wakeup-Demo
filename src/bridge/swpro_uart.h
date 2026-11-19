/**
 * @brief Format of frames received from the input bridge over UART0
 * @note The fields appear in the following order:
 *       1. Header 0xAB 0xCD
 *       2. Command
 *       3. Payload length
 *       4. payload
 *       5. XOR checksum
 */
#ifndef SWPRO_UART_H
#define SWPRO_UART_H

#include <stdint.h>

#define CMD_INPUT_STATE 0x01
#define CMD_PAIR 0x02
#define CMD_CLEAR 0x03
#define CMD_REBOOT 0x04
/**
 * @brief Stop wake-up transmission and resume normal paging
 * @note No payload is used
 */
#define CMD_WAKE_STOP 0x09
/**
 * @brief Start wake-up transmission
 * @note The payload fields are:
 *       - [0] RF channel: 0 is 2402 MHz; 24 is 2426 MHz
 *       - [1] Transmit duration: 1 to 30 seconds
 *       - [2] Initial value for disabling whitening: 0 to 127; 0xFF leaves it enabled
 *       - [5] Position for switching on each burst: 0 to 31; 0xFF disables switching
 *       - [27] If nonzero, disable registration of an ACL link from the wake-up response
 */
#define CMD_WAKE_START 0x0B
/**
 * @brief Control CSB reception using the wake-up access code
 * @note The payload fields are:
 *       - Start if payload[0] is 1; stop if it is 0
 *       - payload[1] specifies the receive interval in slots
 * @note If started during Phase 1 transmission, detection of the Switch response is followed immediately by Phase 3 transmission and registration of the ACL link from the wake-up response.
 */
#define CMD_CSB_RX 0x12
/**
 * @brief Repeat connection attempts
 * @note Start if the one-byte payload is 1; stop if it is 0
 * @note Alternate between wake-up and normal reconnection until a connection succeeds
 */
#define CMD_CONNECT_CYCLE 0x13
/**
 * @brief Store only the Switch BD_ADDR
 * @note Set the target address so the ESP32 can initiate wake-up even after pairing information has been deleted.
 */
#define CMD_SET_HOST 0x14
/**
 * @brief Start one paging connection without performing wake-up
 * @note Reconnect to an awake Switch using the normal paging procedure
 */
#define CMD_PAGE_NOW 0x15
/**
 * @brief Stop connection attempts, wake-up transmission, and automatic connections, then disconnect the link
 * @note No payload is used
 */
#define CMD_DISCONNECT 0x16
/** @brief Begin a BLE scan to capture NS2 wake advertisements; no payload is used. */
#define CMD_BLE_SCAN 0x17
/** @brief Perform an NS2 wake using the saved advertisement; no payload is used. */
#define CMD_NS2_WAKE_UP 0x18


void apply_uart_frame(uint8_t command, const uint8_t *payload, uint8_t length);

void swpro_uart_task(void *argument);

#endif
