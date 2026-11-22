#include <stdint.h>

#include "esp_attr.h"
#include "esp_system.h"

#include "boot_ctrl.h"
#include "wake_mode.h"
#include "wifi_mode.h"
#include "wake_cycle.h"
#include "link_state.h"

#define BOOT_WIFI UINT32_C(0x57494649)
#define BOOT_WAKE UINT32_C(0x57414B45)

// Separate from the Bluetooth firmware's existing restart marker.
RTC_NOINIT_ATTR static uint32_t s_boot_mode;

void boot_request_wake(void)
{
    // Begin a fresh Bluetooth wake cycle, rather than just paging the Switch.
    g_cycle_wake_tries = 0;
    g_restart_marker = CYCLE_RESTART_MARKER;
    s_boot_mode = BOOT_WAKE;
    esp_restart();
}

void app_main(void)
{
    // Only deliberate software resets may resume the wake state.
    if (esp_reset_reason() == ESP_RST_SW && s_boot_mode == BOOT_WAKE) {
        wake_main();
    } else {
        s_boot_mode = BOOT_WIFI;
        g_restart_marker = 0;
        g_cycle_wake_tries = 0;
        wifi_main();
    }
}
