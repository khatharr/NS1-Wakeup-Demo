#include <stdint.h>

#include "esp_attr.h"
#include "esp_system.h"
#include "esp_log.h"

#include "boot_ctrl.h"
#include "wake_mode.h"
#include "wifi_mode.h"
#include "wake_cycle.h"
#include "link_state.h"

#define BOOT_WIFI UINT32_C(0x57494649)
#define BOOT_WAKE UINT32_C(0x57414B45)
#define WAKE_BOOT_LIMIT 4

// Separate from the Bluetooth firmware's existing restart marker.
RTC_NOINIT_ATTR static uint32_t s_boot_mode;
RTC_NOINIT_ATTR static uint32_t s_wake_boots;
RTC_NOINIT_ATTR static uint32_t s_pending_error;

void boot_request_wake(void)
{
    // Begin a fresh Bluetooth wake cycle, rather than just paging the Switch.
    g_cycle_wake_tries = 0;
    g_restart_marker = CYCLE_RESTART_MARKER;
    s_wake_boots = 0;
    s_pending_error = WAKE_OK;
    s_boot_mode = BOOT_WAKE;
    esp_restart();
}

void boot_return_wifi(wake_result_t result)
{
    s_pending_error = result;
    s_boot_mode = BOOT_WIFI;
    s_wake_boots = 0;
    g_restart_marker = 0;
    g_cycle_wake_tries = 0;
    esp_restart();
}

void boot_report_pending_error(void)
{
    static const char *const reasons[] = {
        [WAKE_ERR_INIT] = "wake initialization failed",
        [WAKE_ERR_NO_HOST] = "no paired Switch",
        [WAKE_ERR_TIMEOUT] = "wake operation timed out",
        [WAKE_ERR_RETRY_LIMIT] = "wake recovery reboot limit reached",
        [WAKE_ERR_NO_CONNECTION] = "wake and paging attempts exhausted",
        [WAKE_ERR_INPUT] = "A-button sequence interrupted or failed",
        [WAKE_ERR_RESET] = "unexpected reset during wake",
    };
    // Stub for the future UDP logger. Successful operations remain silent.
    if (s_pending_error > WAKE_OK && s_pending_error <= WAKE_ERR_RESET) {
        ESP_LOGE("boot_ctrl", "%s", reasons[s_pending_error]);
    }
    s_pending_error = WAKE_OK;
}

void app_main(void)
{
    const esp_reset_reason_t reason = esp_reset_reason();

    // Only deliberate software resets may resume the wake state.
    if (reason == ESP_RST_SW && s_boot_mode == BOOT_WAKE) {
        if (s_wake_boots < WAKE_BOOT_LIMIT) {
            ++s_wake_boots;
            wake_main();
            return;
        }
        s_pending_error = WAKE_ERR_RETRY_LIMIT;
    } else if (reason != ESP_RST_SW || s_boot_mode != BOOT_WIFI) {
        // Crash/watchdog resets may leave a useful diagnostic marker,
        // but never resume Bluetooth. Cold boots discard retained state.
        s_pending_error = (s_boot_mode == BOOT_WAKE &&
            (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
             reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT))
            ? WAKE_ERR_RESET : WAKE_OK;
    }

    s_boot_mode = BOOT_WIFI;
    s_wake_boots = 0;
    g_restart_marker = 0;
    g_cycle_wake_tries = 0;
    wifi_main();
}
