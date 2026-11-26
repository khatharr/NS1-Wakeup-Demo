#ifndef WIFI_MODE_H
#define WIFI_MODE_H

#include "esp_err.h"

/**
 * Save the configured Wi-Fi station credentials to NVS.
 * Initializes all required subsystems; call once from app_main on a fresh boot.
 * Returns after saving or reporting an error, without starting Wi-Fi.
 */
esp_err_t wifi_inject_credentials(void);

/**
 * Initialize station-mode Wi-Fi using credentials saved in NVS.
 * Make up to three connection attempts, each with a 30-second association/DHCP
 * timeout and five seconds between attempts. Reuse the initialized network stack.
 * On success the station remains running; on failure the caller must reboot.
 */
esp_err_t wifi_connect(void);

/** Connect to Wi-Fi and listen for the UDP wake trigger. */
void wifi_main(void);

#endif
