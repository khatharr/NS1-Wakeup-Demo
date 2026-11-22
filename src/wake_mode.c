#include "wake_mode.h"
#include "boot_ctrl.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd.h"
#include "esp_hidd_api.h"
#include "esp_hid_common.h"
#include "esp_sdp_api.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "baseband.h"
#include "wake_payload.h"
#include "swpro_pairing.h"
#include "swpro_hid.h"
#include "swpro_link.h"
#include "swpro_uart.h"
#include "wake_cycle.h"
#include "link_state.h"
#include "wake_session.h"
#include "host_sec_state.h"
#include "wakeup_code.h"
#include "swpro_spi.h"
#include "ns2_wake.h"

#define TAG "swpro"

static bool wakeup_code_self_check(void)
{
    static const uint8_t addresses[2][6] = {
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0x00, 0x00, 0x00, 0x9E, 0x8B, 0x33},
    };
    static const uint64_t expected_parity[2] = {
        UINT64_C(0x2c7820e7e), UINT64_C(0x2331a3ae2),
    };
    static const uint64_t expected_wake_parity[2] = {
        UINT64_C(0x2387df181), UINT64_C(0x2cce5c51d),
    };
    static const uint8_t expected_wakeup_code[2][4] = {
        {0x81, 0xF1, 0x7D, 0x38},
        {0x1D, 0xC5, 0xE5, 0xCC},
    };

    for (size_t i = 0; i < 2; ++i) {
        uint8_t actual[4];
        switch1_wakeup_code(addresses[i], actual);
        if (switch1_parity34(addresses[i]) != expected_parity[i] ||
            switch1_wakeup_parity34(addresses[i]) != expected_wake_parity[i] ||
            memcmp(actual, expected_wakeup_code[i], sizeof(actual)) != 0) {
            return false;
        }
    }

    // The local BD_ADDR stored in Phase 3 is arranged least-significant byte first.
    static const uint8_t local[6] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB};
    static const uint8_t local_lsb_first[6] = {0xAB, 0x89, 0x67, 0x45, 0x23, 0x01};
    uint8_t actual_lsb_first[6];
    switch1_bd_addr_lsb_first(local, actual_lsb_first);
    if (memcmp(actual_lsb_first, local_lsb_first, 6) != 0) {
        return false;
    }
    // Verify the placement of length, FLOW, and LLID when constructing the payload header.
    if (switch1_txphdr(2, 0, 2) != 0x0012 ||
        switch1_txphdr(12, 0, 2) != 0x0062 ||
        switch1_txphdr(12, 0, 3) != 0x0063 ||
        switch1_txphdr(12, 1, 3) != 0x0067 ||
        switch1_txphdr(17, 0, 2) != 0x008A ||
        switch1_txphdr(17, 0, 3) != 0x008B) {
        return false;
    }
    return true;
}

static esp_err_t bluetooth_init(bool ns2_wake_only)
{
    esp_err_t err;
    esp_bt_controller_config_t config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&config)) != ESP_OK ||
        (err = esp_bt_controller_enable(ESP_BT_MODE_BTDM)) != ESP_OK ||
        (err = esp_bluedroid_init()) != ESP_OK ||
        (err = esp_bluedroid_enable()) != ESP_OK) {
        return err;
    }
    if ((err = esp_ble_gap_register_callback(ns2_wake_gap_callback)) != ESP_OK) {
        return err;
    }
    if (ns2_wake_only) {
        return ESP_OK;
    }

    // Set the link supervision timeout to 2 seconds.
    // With the default 20-second timeout, a Switch shutdown or sleep that stops responses takes 20 seconds to be recognized as a lost link.
    // Link 0 remains occupied, preventing the next wake-up transmission and paging request.
    // Reducing the timeout to 1 second would disconnect the link even during temporary radio interference.
    // The unit is a 0.625 ms slot.
    BTM_SetDefaultLinkSuperTout(LINK_SUPERVISION_TIMEOUT_SLOTS);

    // Set BR/EDR transmit power to ESP_PWR_LVL_P9 to provide enough range for the Switch to receive the wake-up signal.
    // The transmit-power field in the control structure is populated from rwip_rf+44, so this call also updates that structure.
    esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9);

    // Following HOJA's order, configure CoD and SSP before registering the HID device.
    esp_bt_cod_t cod = {
        .minor = 0x02,   // Gamepad
        .major = 0x05,   // Peripheral
        .service = 0x001, // Limited Discoverable
    };
    if ((err = esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD)) != ESP_OK) {
        return err;
    }

    esp_bt_io_cap_t capability = ESP_BT_IO_CAP_NONE;
    if ((err = esp_bt_gap_set_security_param(
             ESP_BT_SP_IOCAP_MODE, &capability, sizeof(capability))) != ESP_OK) {
        return err;
    }

    if ((err = esp_bt_gap_register_callback(swpro_gap_callback)) != ESP_OK) {
        return err;
    }

#if CONFIG_BT_SDP_COMMON_ENABLED
    if ((err = esp_sdp_register_callback(swpro_sdp_callback)) != ESP_OK) {
        return err;
    }
    if ((err = esp_sdp_init()) != ESP_OK) {
        return err;
    }
#endif

    if ((err = swpro_hid_register(swpro_hidd_event_callback)) != ESP_OK) {
        return err;
    }

    if ((err = esp_bt_gap_set_device_name("Pro Controller")) != ESP_OK) {
        return err;
    }
    return esp_bt_gap_set_page_timeout(WAKE_PAGE_TIMEOUT_SLOTS);
}

void wake_main(void)
{
    const bool ns2_wake_boot = g_restart_marker == NS2_WAKE_RESTART_MARKER;
    if (ns2_wake_boot) {
        g_restart_marker = 0;
    } else if (g_restart_marker == WAKE_RESTART_MARKER) {
        g_restart_marker = 0;
        g_page_right_after_boot = true;
    } else if (g_restart_marker == CYCLE_RESTART_MARKER) {
        g_restart_marker = 0;
        g_cycle_resume_after_boot = true;
    } else {
        // RTC memory has an undefined value at power-on. Treat any value other than the known marker in g_restart_marker as invalid.
        g_restart_marker = 0;
        g_cycle_wake_tries = 0;
    }
    // On a normal boot, start paging the saved Switch.
    // On a restart that continues the wake-up process, disable paging to avoid conflicting with CSB transmission.
    g_reconnect_disabled = g_page_right_after_boot || g_cycle_resume_after_boot;

    swpro_pairing_init();
    if (ns2_wake_init() != ESP_OK) {
        boot_return_wifi(WAKE_ERR_INIT);
    }
    g_pairing = !swpro_host_exists();
    if (g_pairing) {
        boot_return_wifi(WAKE_ERR_NO_HOST);
    }

    if (!wakeup_code_self_check()) {
        ESP_LOGE(TAG, "Switch 1 wake-up code startup self-check failed");
        boot_return_wifi(WAKE_ERR_INIT);
    }
    ESP_LOGI(TAG, "Switch 1 wake-up code startup self-check passed");
    if (ns2_wake_boot && !ns2_wake_prepare_controller_address()) {
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_restart();
    }

    if (!uart_is_driver_installed(UART_NUM_0)) {
        if (uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0) != ESP_OK) {
            boot_return_wifi(WAKE_ERR_INIT);
        }
    }
    if (bluetooth_init(ns2_wake_boot) != ESP_OK) {
        boot_return_wifi(WAKE_ERR_INIT);
    }
    if (ns2_wake_boot) {
        ns2_wake_advertise_and_restart();
    }

    // CSB reception monitoring uses recording arrays and log formatting.
    if (xTaskCreate(swpro_uart_task, "uart_input", 4608, NULL, 5, NULL) != pdPASS ||
        xTaskCreatePinnedToCore(swpro_protocol_task, "switch_reports", 4096, NULL, 5, NULL, 0) != pdPASS) {
        boot_return_wifi(WAKE_ERR_INIT);
    }
    // The Bluetooth controller runs on Core 1.
    // Pin burst-detection polling to Core 0 so it does not compete with controller processing on the same core.
    if (xTaskCreatePinnedToCore(wake_tx_task, "wake_tx", 2560, NULL, 6, NULL, 0) != pdPASS) {
        boot_return_wifi(WAKE_ERR_INIT);
    }
}

