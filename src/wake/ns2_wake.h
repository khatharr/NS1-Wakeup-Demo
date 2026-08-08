#ifndef NS2_WAKE_H
#define NS2_WAKE_H

#include <stdbool.h>

#include "esp_err.h"
#include "esp_gap_ble_api.h"

esp_err_t ns2_wake_init(void);
bool ns2_wake_has_identity(void);
esp_err_t ns2_wake_scan_start(void);
void ns2_wake_scan_stop(void);
void ns2_wake_step(bool radio_idle);
bool ns2_wake_prepare_controller_address(void);
void ns2_wake_advertise_and_restart(void) __attribute__((noreturn));
void ns2_wake_gap_callback(esp_gap_ble_cb_event_t event,
                           esp_ble_gap_cb_param_t *param);

#endif
