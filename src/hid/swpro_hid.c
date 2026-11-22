#include "swpro_hid.h"

#include <string.h>

#include "esp_bt_device.h"
#include "esp_hidd.h"
#include "esp_hidd_api.h"
#include "esp_hid_common.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "swpro_spi.h"

#define TAG "swpro_hid"

#define REPORT_SIZE SWPRO_REPORT_SIZE

#define OUT_RUMBLE_AND_SUBCOMMAND 0x01
#define OUT_RUMBLE_ONLY 0x10

#define SUBCMD_SET_PAIRING 0x01
#define SUBCMD_GET_DEVICE_INFO 0x02
#define SUBCMD_SET_INPUT_MODE 0x03
#define SUBCMD_GET_TRIGGER_ELAPSED_TIME 0x04
#define SUBCMD_SET_HCI_STATE 0x06
#define SUBCMD_READ_SPI_FLASH 0x10
#define SUBCMD_SET_PLAYER_LIGHTS 0x30
#define SUBCMD_ENABLE_IMU 0x40

#define STICK_CENTER UINT32_C(0x00800800)
#define SWPRO_BUTTON_A (UINT32_C(1) << 3)

static const uint8_t s_report_descriptor[] = {
    0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x06, 0x01, 0xFF,
    0x85, 0x21, 0x09, 0x21, 0x75, 0x08, 0x95, 0x30, 0x81, 0x02,
    0x85, 0x30, 0x09, 0x30, 0x75, 0x08, 0x95, 0x30, 0x81, 0x02,
    0x85, 0x31, 0x09, 0x31, 0x75, 0x08, 0x96, 0x69, 0x01, 0x81, 0x02,
    0x85, 0x32, 0x09, 0x32, 0x75, 0x08, 0x96, 0x69, 0x01, 0x81, 0x02,
    0x85, 0x33, 0x09, 0x33, 0x75, 0x08, 0x96, 0x69, 0x01, 0x81, 0x02,
    0x85, 0x3F, 0x05, 0x09, 0x19, 0x01, 0x29, 0x10, 0x15, 0x00,
    0x25, 0x01, 0x75, 0x01, 0x95, 0x10, 0x81, 0x02, 0x05, 0x01,
    0x09, 0x39, 0x15, 0x00, 0x25, 0x07, 0x75, 0x04, 0x95, 0x01,
    0x81, 0x42, 0x05, 0x09, 0x75, 0x04, 0x95, 0x01, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x33, 0x09, 0x34,
    0x16, 0x00, 0x00, 0x27, 0xFF, 0xFF, 0x00, 0x00, 0x75, 0x10,
    0x95, 0x04, 0x81, 0x02, 0x06, 0x01, 0xFF,
    0x85, 0x01, 0x09, 0x01, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02,
    0x85, 0x10, 0x09, 0x10, 0x75, 0x08, 0x95, 0x09, 0x91, 0x02,
    0x85, 0x11, 0x09, 0x11, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02,
    0x85, 0x12, 0x09, 0x12, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02,
    0xC0,
};

_Static_assert(sizeof(s_report_descriptor) == 170,
               "HID report descriptor size changed");

static esp_hid_raw_report_map_t s_report_maps[1] = {
    {
        .data = s_report_descriptor,
        .len = sizeof(s_report_descriptor),
    },
};

static esp_hidd_dev_t *s_hid_dev;

static const esp_hid_device_config_t s_hid_config = {
    .vendor_id = 0x057E,
    .product_id = 0x2009,
    .version = 0x0100,
    .device_name = "Pro Controller",
    .manufacturer_name = "Nintendo",
    .serial_number = "000000",
    .report_maps = s_report_maps,
    .report_maps_len = 1,
};

static portMUX_TYPE s_input_lock = portMUX_INITIALIZER_UNLOCKED;
static swpro_input_t s_input = {
    .stick_left = STICK_CENTER,
    .stick_right = STICK_CENTER,
    .accel = {0, 0, 4096},
};
static uint32_t s_input_frames;
static bool s_override_a;
static bool s_override_a_pressed;

static volatile uint8_t s_reporting_mode = 0x30;
static volatile uint8_t s_imu_mode;
static volatile uint32_t s_report_interval_us = SWPRO_REPORT_INTERVAL_DEFAULT_US;
static uint8_t s_report_timer;
static volatile bool s_host_stopped;

esp_err_t swpro_hid_register(esp_event_handler_t handler)
{
    return esp_hidd_dev_init(&s_hid_config, ESP_HID_TRANSPORT_BT, handler,
                             &s_hid_dev);
}

void swpro_hid_set_input(const swpro_input_t *input)
{
    portENTER_CRITICAL(&s_input_lock);
    s_input = *input;
    s_input_frames++;
    portEXIT_CRITICAL(&s_input_lock);
}

void swpro_hid_override_a(bool enabled, bool pressed)
{
    portENTER_CRITICAL(&s_input_lock);
    s_override_a = enabled;
    s_override_a_pressed = pressed;
    portEXIT_CRITICAL(&s_input_lock);
}

uint32_t swpro_hid_input_frames(void)
{
    return s_input_frames;
}

uint32_t swpro_hid_report_interval_us(void)
{
    return s_report_interval_us;
}

void swpro_hid_set_report_interval_us(uint32_t interval_us)
{
    s_report_interval_us = interval_us;
}

bool swpro_hid_host_stopped(void)
{
    return s_host_stopped;
}

void swpro_hid_reset_session(void)
{
    s_host_stopped = false;
    s_reporting_mode = 0x30;
    s_imu_mode = 0;
    s_report_interval_us = SWPRO_REPORT_INTERVAL_DEFAULT_US;
}

static void store_u16(uint8_t *p, uint16_t value)
{
    p[0] = value & 0xFF;
    p[1] = value >> 8;
}

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void input_snapshot(swpro_input_t *out)
{
    portENTER_CRITICAL(&s_input_lock);
    *out = s_input;
    if (s_override_a) {
        out->buttons &= ~SWPRO_BUTTON_A;
        if (s_override_a_pressed) {
            out->buttons |= SWPRO_BUTTON_A;
        }
    }
    portEXIT_CRITICAL(&s_input_lock);
}

static void report_set_header(uint8_t report[REPORT_SIZE])
{
    report[0] = s_report_timer++;
    report[1] = 0x8E;
}

static void report_set_input(uint8_t report[REPORT_SIZE], const swpro_input_t *input)
{
    report[2] = input->buttons;
    report[3] = input->buttons >> 8;
    report[4] = input->buttons >> 16;
    report[5] = input->stick_left;
    report[6] = input->stick_left >> 8;
    report[7] = input->stick_left >> 16;
    report[8] = input->stick_right;
    report[9] = input->stick_right >> 8;
    report[10] = input->stick_right >> 16;
}

static void report_set_imu(uint8_t report[REPORT_SIZE], const swpro_input_t *input)
{
    if (s_imu_mode != 0x01) {
        return;
    }

    uint8_t group[12];
    store_u16(&group[0], (uint16_t)input->accel[0]);
    store_u16(&group[2], (uint16_t)input->accel[1]);
    store_u16(&group[4], (uint16_t)input->accel[2]);
    store_u16(&group[6], (uint16_t)input->gyro[0]);
    store_u16(&group[8], (uint16_t)input->gyro[1]);
    store_u16(&group[10], (uint16_t)input->gyro[2]);
    memcpy(&report[12], group, sizeof(group));
    memcpy(&report[24], group, sizeof(group));
    memcpy(&report[36], group, sizeof(group));
}

esp_err_t swpro_hid_send_report(void)
{
    swpro_input_t input;
    uint8_t report[REPORT_SIZE] = {0};

    input_snapshot(&input);
    report_set_header(report);
    report_set_input(report, &input);
    report_set_imu(report, &input);

    esp_err_t err = esp_bt_hid_device_send_report(
        ESP_HIDD_REPORT_TYPE_INTRDATA, 0x30, REPORT_SIZE, report);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send input report: %s", esp_err_to_name(err));
    }
    return err;
}

static void fill_pairing_reply(uint8_t report[REPORT_SIZE], uint8_t phase)
{
    static const uint8_t controller_string[24] = {
        0x00, 0x25, 0x08, 0x50, 0x72, 0x6F, 0x20, 0x43,
        0x6F, 0x6E, 0x74, 0x72, 0x6F, 0x6C, 0x6C, 0x65,
        0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68,
    };
    const uint8_t *local = esp_bt_dev_get_address();
    report[12] = 0x81;
    report[13] = SUBCMD_SET_PAIRING;
    report[14] = phase;

    if (phase == 1 && local != NULL) {
        for (size_t i = 0; i < ESP_BD_ADDR_LEN; ++i) {
            report[15 + i] = local[ESP_BD_ADDR_LEN - 1 - i];
        }
        memcpy(&report[21], controller_string, sizeof(controller_string));
    } else if (phase == 2) {
        memset(&report[15], 0, 16);
    }
}

static void fill_device_info(uint8_t report[REPORT_SIZE])
{
    const uint8_t *local = esp_bt_dev_get_address();
    report[14] = 0x03; // High byte of controller firmware version
    report[15] = 0x48; // Low byte of controller firmware version
    report[16] = 0x03; // High byte of controller type (Pro Controller)
    report[17] = 0x02; // Low byte of controller type
    if (local != NULL) {
        memcpy(&report[18], local, ESP_BD_ADDR_LEN);
    }
    report[24] = 0x00;
    report[25] = 0x02;
}

// Send 0x21 replies immediately after reception, without waiting for the normal 8 ms input interval.
static void handle_subcommand(uint8_t command, const uint8_t *args, uint16_t args_len)
{
    swpro_input_t input;
    uint8_t report[REPORT_SIZE] = {0};

    input_snapshot(&input);
    report_set_header(report);
    report_set_input(report, &input);
    report[12] = 0x80;
    report[13] = command;

    switch (command) {
    case SUBCMD_SET_PAIRING:
        if (args_len >= 1) {
            fill_pairing_reply(report, args[0]);
        }
        break;
    case SUBCMD_GET_DEVICE_INFO:
        report[12] = 0x82;
        fill_device_info(report);
        break;
    case SUBCMD_SET_INPUT_MODE:
        if (args_len >= 1) {
            s_reporting_mode = args[0];
            ESP_LOGI(TAG, "Input mode set: 0x%02x", args[0]);
        }
        break;
    case SUBCMD_GET_TRIGGER_ELAPSED_TIME:
        report[12] = 0x83;
        for (int i = 0; i < 14; i += 2) {
            store_u16(&report[14 + i], 100);
        }
        break;
    case SUBCMD_READ_SPI_FLASH: {
        if (args_len < 5) {
            break;
        }
        uint8_t length = args[4];
        if (length > REPORT_SIZE - 19) {
            length = REPORT_SIZE - 19;
        }
        const uint16_t address = read_u16(args);
        report[12] = 0x90;
        report[14] = address;
        report[15] = address >> 8;
        report[18] = length;
        switch_spi_read(address, length, &report[19]);
        break;
    }
    case SUBCMD_ENABLE_IMU:
        if (args_len >= 1) {
            s_imu_mode = args[0];
        }
        break;
    case SUBCMD_SET_PLAYER_LIGHTS:
        if (args_len >= 1) {
            ESP_LOGI(TAG, "Player LED setting received: 0x%02x",
                     args[0] & 0x0F);
        }
        break;
    case SUBCMD_SET_HCI_STATE:
        if (args_len >= 1) {
            ESP_LOGI(TAG, "Received HCI state: 0x%02x", args[0]);
            // State 0 is the disconnect command sent when the Switch enters sleep.
            // Stop input reports at this point.
            // Sending an input report within 8 ms after link disconnection makes Bluedroid HID reopen L2CAP and start paging the Switch during its sleep transition.
            // Since the HID connection flag is cleared only by the disconnect callback, set s_host_stopped first.
            if (args[0] == 0) {
                s_host_stopped = true;
            }
        }
        break;
    default:
        break;
    }

    esp_err_t err = esp_bt_hid_device_send_report(
        ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21, REPORT_SIZE, report);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send subcommand response: %s", esp_err_to_name(err));
    }
}

void swpro_hid_handle_output(uint8_t report_id, const uint8_t *data,
                             uint16_t length)
{
    if (report_id == OUT_RUMBLE_ONLY) {
        return;
    }
    if (report_id != OUT_RUMBLE_AND_SUBCOMMAND) {
        ESP_LOGW(TAG, "Unsupported output report: id=0x%02x len=%u", report_id, length);
        return;
    }
    if (data == NULL || length < 10) {
        ESP_LOGW(TAG, "Short output report received: id=0x%02x len=%u", report_id, length);
        return;
    }

    ESP_LOGI(TAG, "Subcommand received: 0x%02x", data[9]);
    handle_subcommand(data[9], &data[10], length - 10);
}
