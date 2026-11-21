#include "wifi_mode.h"

#include "esp_wifi.h"

esp_err_t wifi_inject_credentials(void)
{
    wifi_config_t config = {
        .sta = {
            .ssid = "placeholder",
            .password = "placeholder",
        },
    };

    esp_err_t err = esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    if (err != ESP_OK) {
        return err;
    }
    return esp_wifi_set_config(WIFI_IF_STA, &config);
}
