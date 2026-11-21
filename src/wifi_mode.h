#ifndef WIFI_MODE_H
#define WIFI_MODE_H

#include "esp_err.h"

/**
 * Save the configured Wi-Fi station credentials to NVS.
 * Requires an initialized Wi-Fi driver in station mode.
 */
esp_err_t wifi_inject_credentials(void);

#endif
