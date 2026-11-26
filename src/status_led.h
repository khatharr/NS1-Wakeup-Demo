#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdbool.h>

typedef enum {
    STATUS_LED_WIFI_WAIT,
    STATUS_LED_READY,
    STATUS_LED_WAKE,
} status_led_mode_t;

// Start the indicator once per boot. A retained wake failure is flashed once.
void status_led_start(status_led_mode_t mode, bool wake_failed);
// Update the indicator without blocking the calling task.
void status_led_set(status_led_mode_t mode);

#endif
