#ifndef WIFI_MODE_H
#define WIFI_MODE_H

#include "esp_err.h"

/**
 * Save the configured Wi-Fi station credentials to NVS.
 * Requires an initialized Wi-Fi driver in station mode.
 */
esp_err_t wifi_inject_credentials(void);

/**
 * Initialize station-mode Wi-Fi using credentials saved in NVS.
 * Wait up to 30 seconds for a DHCP-assigned IPv4 address.
 * Call once per Wi-Fi boot; the caller handles failures and recovery.
 * The Wi-Fi driver and event handlers remain initialized on return.
 */
esp_err_t wifi_connect(void);

#endif
