#include "swpro_pairing.h"

#include <stdlib.h>
#include <string.h>

#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#define TAG "swpro_pair"

static portMUX_TYPE s_host_lock = portMUX_INITIALIZER_UNLOCKED;
static esp_bd_addr_t s_host_addr;
static bool s_host_valid;
static nvs_handle_t s_nvs;

bool swpro_host_snapshot(esp_bd_addr_t out)
{
    portENTER_CRITICAL(&s_host_lock);
    const bool valid = s_host_valid;
    if (valid) {
        memcpy(out, s_host_addr, ESP_BD_ADDR_LEN);
    }
    portEXIT_CRITICAL(&s_host_lock);
    return valid;
}

bool swpro_host_exists(void)
{
    portENTER_CRITICAL(&s_host_lock);
    const bool valid = s_host_valid;
    portEXIT_CRITICAL(&s_host_lock);
    return valid;
}

void swpro_host_store(const esp_bd_addr_t address)
{
    bool changed;
    portENTER_CRITICAL(&s_host_lock);
    changed = !s_host_valid || memcmp(s_host_addr, address, ESP_BD_ADDR_LEN) != 0;
    memcpy(s_host_addr, address, ESP_BD_ADDR_LEN);
    s_host_valid = true;
    portEXIT_CRITICAL(&s_host_lock);

    if (!changed) {
        return;
    }
    esp_err_t err = nvs_set_blob(s_nvs, "host", address, ESP_BD_ADDR_LEN);
    if (err == ESP_OK) {
        err = nvs_commit(s_nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save Switch BD_ADDR: %s", esp_err_to_name(err));
    }
}

static void erase_host_record(void)
{
    portENTER_CRITICAL(&s_host_lock);
    memset(s_host_addr, 0, sizeof(s_host_addr));
    s_host_valid = false;
    portEXIT_CRITICAL(&s_host_lock);

    esp_err_t err = nvs_erase_key(s_nvs, "host");
    if (err == ESP_OK) {
        err = nvs_commit(s_nvs);
    }
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to erase Switch BD_ADDR: %s", esp_err_to_name(err));
    }
}

static void erase_bond_devices(void)
{
    int bond_count = esp_bt_gap_get_bond_device_num();
    if (bond_count <= 0) {
        return;
    }

    esp_bd_addr_t *bond_devices = calloc((size_t)bond_count, sizeof(*bond_devices));
    if (bond_devices == NULL) {
        ESP_LOGE(TAG, "Failed to allocate bond device list");
        return;
    }

    int listed_count = bond_count;
    if (esp_bt_gap_get_bond_device_list(&listed_count, bond_devices) == ESP_OK) {
        for (int i = 0; i < listed_count; ++i) {
            esp_bt_gap_remove_bond_device(bond_devices[i]);
        }
    }
    free(bond_devices);
}

void swpro_pairing_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(nvs_open("swpro", NVS_READWRITE, &s_nvs));

    esp_bd_addr_t host_address;
    size_t length = sizeof(host_address);
    if (nvs_get_blob(s_nvs, "host", host_address, &length) == ESP_OK &&
        length == sizeof(host_address)) {
        portENTER_CRITICAL(&s_host_lock);
        memcpy(s_host_addr, host_address, ESP_BD_ADDR_LEN);
        s_host_valid = true;
        portEXIT_CRITICAL(&s_host_lock);
    }
}

void swpro_pairing_reset(void)
{
    ESP_LOGI(TAG, "Erasing pairing information and rebooting");
    esp_bt_hid_device_virtual_cable_unplug();
    vTaskDelay(pdMS_TO_TICKS(300));
    erase_bond_devices();
    erase_host_record();
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}
