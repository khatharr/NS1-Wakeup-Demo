#include "wifi_mode.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

// Event bits shared by the Wi-Fi event loop and the waiting caller.
#define WIFI_IP_READY BIT0
#define WIFI_CONNECT_FAILED BIT1
// Maximum time to wait for association and DHCP combined.
#define WIFI_CONNECT_TIMEOUT_MS 30000

static EventGroupHandle_t s_wifi_events;
// Preserve the connection error before notifying the waiting task.
static esp_err_t s_connect_error = ESP_FAIL;

// Runs on the default ESP-IDF event loop, not in the waiting task.
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    (void)data;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        // The station has started; begin connecting with its saved configuration.
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            s_connect_error = err;
            xEventGroupSetBits(s_wifi_events, WIFI_CONNECT_FAILED);
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        // Invalidate IP readiness and notify the caller of failure.
        // Reconnection policy is deliberately left to the caller.
        xEventGroupClearBits(s_wifi_events, WIFI_IP_READY);
        s_connect_error = ESP_FAIL;
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECT_FAILED);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        // Association alone is not enough: wait for a usable IPv4 address.
        xEventGroupSetBits(s_wifi_events, WIFI_IP_READY);
    }
}

// Standalone provisioning entry point. Call once from app_main on a fresh boot.
// Not part of normal startup; calling this overwrites the stored configuration.
esp_err_t wifi_inject_credentials(void)
{
    // Development placeholders only. Do not commit real network credentials.
    wifi_config_t config = {
        .sta = {
            .ssid = "placeholder",
            .password = "placeholder",
        },
    };
    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err;
    const char *step;

    // Initialize NVS without erasing existing credentials or Bluetooth bonds.
    step = "nvs_flash_init";
    err = nvs_flash_init();
    if (err != ESP_OK) goto failed;

    // Set up networking and the event loop before initializing the driver.
    step = "esp_netif_init";
    err = esp_netif_init();
    if (err != ESP_OK) goto failed;

    step = "esp_event_loop_create_default";
    err = esp_event_loop_create_default();
    if (err != ESP_OK) goto failed;

    step = "esp_netif_create_default_wifi_sta";
    if (esp_netif_create_default_wifi_sta() == NULL) {
        err = ESP_ERR_NO_MEM;
        goto failed;
    }

    step = "esp_wifi_init";
    err = esp_wifi_init(&init_config);
    if (err != ESP_OK) goto failed;

    step = "esp_wifi_set_mode";
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) goto failed;

    // Save the station configuration directly to NVS via the Wi-Fi API.
    step = "esp_wifi_set_storage";
    err = esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    if (err != ESP_OK) goto failed;

    step = "esp_wifi_set_config";
    err = esp_wifi_set_config(WIFI_IF_STA, &config);
    if (err != ESP_OK) goto failed;

    // Do not start Wi-Fi, register connection handlers, or restart the ESP32.
    ESP_LOGI("wifi_mode", "Wi-Fi credentials saved to flash");
    return ESP_OK;

failed:
    // Return normally rather than aborting and repeatedly rebooting.
    ESP_LOGE("wifi_mode", "Credential provisioning failed at %s: %s",
             step, esp_err_to_name(err));
    return err;
}

// Initialize Wi-Fi once per boot and block until DHCP succeeds or an error occurs.
esp_err_t wifi_connect(void)
{
    // Do not initialize the station and register handlers a second time.
    if (s_wifi_events != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Open persistent storage for the Wi-Fi driver without erasing pairing data
    // or credentials if initialization fails.
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) return err;

    // Initialize TCP/IP networking and the default event dispatch loop.
    err = esp_netif_init();
    if (err != ESP_OK) return err;

    err = esp_event_loop_create_default();
    if (err != ESP_OK) return err;

    // Create the station network interface with the default DHCP client handlers.
    if (esp_netif_create_default_wifi_sta() == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Initialize the driver with ESP-IDF defaults, including NVS support.
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&config);
    if (err != ESP_OK) return err;

    // Join an existing access point rather than creating one.
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) return err;

    // Allocate the synchronization object before any events can notify us.
    s_wifi_events = xEventGroupCreate();
    if (s_wifi_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Register handlers before starting Wi-Fi so early events cannot be missed.
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                     wifi_event_handler, NULL);
    if (err != ESP_OK) return err;

    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                     wifi_event_handler, NULL);
    if (err != ESP_OK) return err;

    // Starting the station generates WIFI_EVENT_STA_START, which initiates
    // the connection in wifi_event_handler. DHCP follows association automatically.
    err = esp_wifi_start();
    if (err != ESP_OK) return err;

    // Sleep until either event bit is set, or the 30-second timeout expires.
    // Leave the bits set for subsequent connection-state handling.
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events, WIFI_IP_READY | WIFI_CONNECT_FAILED,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    // Prefer failure if both bits are present. Initialization and handlers remain
    // in place on return; the caller is responsible for recovery or reboot.
    if (bits & WIFI_CONNECT_FAILED) return s_connect_error;
    if (bits & WIFI_IP_READY) return ESP_OK;
    return ESP_ERR_TIMEOUT;
}
