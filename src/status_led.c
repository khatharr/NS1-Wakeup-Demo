#include "status_led.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// FireBeetle 2 ESP32-E D9: active-high onboard LED, separate from charge LED.
#define STATUS_LED_GPIO GPIO_NUM_2
#define FLASH_MS 100
#define WIFI_BLINK_MS 500

static TaskHandle_t s_led_task;
static status_led_mode_t s_initial_mode;
static bool s_wake_failed;

static void flash(unsigned count)
{
    for (unsigned i = 0; i < count; ++i) {
        gpio_set_level(STATUS_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(FLASH_MS));
        gpio_set_level(STATUS_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(FLASH_MS));
    }
}

static void status_led_task(void *arg)
{
    (void)arg;
    status_led_mode_t mode = s_initial_mode;
    flash(2);
    if (s_wake_failed) {
        vTaskDelay(pdMS_TO_TICKS(300));
        flash(3);
    }

    uint32_t next;
    // Apply any state received during the initial flashes.
    if (xTaskNotifyWait(0, 0, &next, 0) == pdTRUE) {
        mode = (status_led_mode_t)next;
    }

    bool blink_on = true;
    for (;;) {
        const bool waiting = mode == STATUS_LED_WIFI_WAIT;
        gpio_set_level(STATUS_LED_GPIO,
                       mode == STATUS_LED_WAKE || (waiting && blink_on));

        // Steady states have no periodic wakeups. Notifications interrupt blinking.
        if (xTaskNotifyWait(0, 0, &next,
                            waiting ? pdMS_TO_TICKS(WIFI_BLINK_MS)
                                    : portMAX_DELAY) == pdTRUE) {
            mode = (status_led_mode_t)next;
            blink_on = true;
        } else {
            blink_on = !blink_on;
        }
    }
}

void status_led_start(status_led_mode_t mode, bool wake_failed)
{
    const gpio_config_t config = {
        .pin_bit_mask = UINT64_C(1) << STATUS_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&config) != ESP_OK) {
        ESP_LOGW("status_led", "Cannot configure D9");
        return;
    }
    gpio_set_level(STATUS_LED_GPIO, 0);
    s_initial_mode = mode;
    s_wake_failed = wake_failed;

    // Low priority on core 0; never block or preempt the wake protocol tasks.
    if (xTaskCreatePinnedToCore(status_led_task, "status_led", 2048, NULL,
                                1, &s_led_task, 0) != pdPASS) {
        s_led_task = NULL;
        ESP_LOGW("status_led", "Cannot start LED task");
    }
}

void status_led_set(status_led_mode_t mode)
{
    if (s_led_task != NULL) {
        xTaskNotify(s_led_task, (uint32_t)mode, eSetValueWithOverwrite);
    }
}
