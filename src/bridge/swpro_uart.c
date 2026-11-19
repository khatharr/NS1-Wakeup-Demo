// Communicate with the input bridge over UART0. Receive frames contain, in order, header 0xAB 0xCD, command, length, payload, and XOR checksum.
#include "swpro_uart.h"

#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd.h"
#include "esp_hidd_api.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "link_state.h"
#include "swpro_hid.h"
#include "swpro_link.h"
#include "swpro_pairing.h"
#include "baseband.h"
#include "wakeup_code.h"
#include "wake_cycle.h"
#include "wake_session.h"
#include "ns2_wake.h"
#include "driver/uart.h"

#define TAG "swpro"

typedef enum {
    PARSE_MARKER_0,
    PARSE_MARKER_1,
    PARSE_COMMAND,
    PARSE_LENGTH,
    PARSE_PAYLOAD,
    PARSE_CHECKSUM,
} parse_state_t;

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_u24(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

#define INPUT_PAYLOAD_SIZE 21
#define UART_PAYLOAD_MAX 63

void apply_uart_frame(uint8_t command, const uint8_t *payload, uint8_t length)
{
    switch (command) {
    case CMD_INPUT_STATE:
        if (length != INPUT_PAYLOAD_SIZE) {
            return;
        }
        {
            swpro_input_t input;
            input.buttons = read_u24(&payload[0]);
            input.stick_left = read_u24(&payload[3]);
            input.stick_right = read_u24(&payload[6]);
            for (int i = 0; i < 3; ++i) {
                input.accel[i] = (int16_t)read_u16(&payload[9 + i * 2]);
                input.gyro[i] = (int16_t)read_u16(&payload[15 + i * 2]);
            }
            swpro_hid_set_input(&input);
        }
        break;
    case CMD_PAIR:
    case CMD_CLEAR:
        if (length == 0) {
            // Stop the connection cycle before deleting pairing information.
            g_restart_marker = 0;
            swpro_pairing_reset();
        }
        break;
    case CMD_CONNECT_CYCLE:
        if (length == 1) {
            connect_cycle_set(payload[0] != 0);
        }
        break;
    case CMD_WAKE_START:
        wake_cmd_start(payload, length);
        break;

    case CMD_CSB_RX:
        wake_cmd_receive(payload, length);
        break;

    case CMD_WAKE_STOP:
        if (length == 0) {
            g_wake_stop_requested = true;
            ESP_LOGI(TAG, "Wake-up transmission stop requested");
        }
        break;
    case CMD_REBOOT:
        if (length == 0) {
            ESP_LOGI(TAG, "Starting reboot");
            vTaskDelay(pdMS_TO_TICKS(50));
            esp_restart();
        }
        break;
    case CMD_SET_HOST:
        /*
         * Store only the Switch BD_ADDR in NVS, not the link key.
         * Even after deleting pairing information, the BD_ADDR identifies the target Switch for wake-up.
         * Pairing can be registered again when the Switch connects.
         * The payload bytes are in display order, with the most-significant byte first.
         */
        if (length == 6) {
            esp_bd_addr_t host_address;
            for (int i = 0; i < 6; ++i) {
                host_address[i] = payload[i];
            }
            swpro_host_store(host_address);
            ESP_LOGI(TAG, "Saved Switch BD_ADDR: " ESP_BD_ADDR_STR,
                     ESP_BD_ADDR_HEX(host_address));
        }
        break;
    case CMD_PAGE_NOW:
        // Reconnect to an awake Switch using the normal paging procedure.
        if (length == 0) {
            ESP_LOGI(TAG, "Starting one HCI page connection");
            g_reconnect_disabled = false;
            g_reconnect_paused = false;
            g_stay_disconnected = false;
            g_connect_failures = 0;
            g_next_reconnect_us = 0;
            g_connect_deadline_us = 0;
            try_reconnect();
        }
        break;
    case CMD_DISCONNECT:
        if (length == 0) {
            ESP_LOGI(TAG, "Starting GUI disconnect");
            ns2_wake_scan_stop();
            connect_cycle_set(false);
            g_wake_stop_requested = true;
            swpro_stop_auto_connect("GUI disconnect");
        }
        break;
    case CMD_BLE_SCAN:
        if (length == 0) {
            connect_cycle_set(false);
            g_wake_stop_requested = true;
            swpro_stop_auto_connect("NS2 BLE scan");
            const esp_err_t err = ns2_wake_scan_start();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start NS2 BLE scan: %s",
                         esp_err_to_name(err));
            }
        }
        break;
    case CMD_NS2_WAKE_UP:
        if (length == 0) {
            if (!ns2_wake_has_identity()) {
                ESP_LOGE(TAG, "Run BLE scan before NS2 wake-up");
                break;
            }
            ns2_wake_scan_stop();
            connect_cycle_set(false);
            g_wake_stop_requested = true;
            swpro_stop_auto_connect("NS2 wake-up");
            swpro_restart_after_disconnect(NS2_WAKE_RESTART_MARKER);
        }
        break;
    default:
        break;
    }
}

static void parse_uart_byte(uint8_t byte)
{
    static parse_state_t state;
    static uint8_t command;
    static uint8_t length;
    static uint8_t index;
    static uint8_t checksum;
    static uint8_t payload[UART_PAYLOAD_MAX];

    switch (state) {
    case PARSE_MARKER_0:
        if (byte == 0xAB) {
            state = PARSE_MARKER_1;
        }
        break;
    case PARSE_MARKER_1:
        if (byte == 0xCD) {
            state = PARSE_COMMAND;
        } else {
            state = byte == 0xAB ? PARSE_MARKER_1 : PARSE_MARKER_0;
        }
        break;
    case PARSE_COMMAND:
        command = byte;
        checksum = byte;
        state = PARSE_LENGTH;
        break;
    case PARSE_LENGTH:
        if (byte > UART_PAYLOAD_MAX) {
            state = PARSE_MARKER_0;
            break;
        }
        length = byte;
        index = 0;
        checksum ^= byte;
        state = length == 0 ? PARSE_CHECKSUM : PARSE_PAYLOAD;
        break;
    case PARSE_PAYLOAD:
        payload[index++] = byte;
        checksum ^= byte;
        if (index == length) {
            state = PARSE_CHECKSUM;
        }
        break;
    case PARSE_CHECKSUM:
        if (byte == checksum) {
            apply_uart_frame(command, payload, length);
        }
        state = PARSE_MARKER_0;
        break;
    default:
        state = PARSE_MARKER_0;
        break;
    }
}

void swpro_uart_task(void *argument)
{
    (void)argument;
    uint8_t buffer[128];
    while (true) {
        const int count = uart_read_bytes(UART_NUM_0, buffer, sizeof(buffer), pdMS_TO_TICKS(20));
        for (int i = 0; i < count; ++i) {
            parse_uart_byte(buffer[i]);
        }
    }
}
