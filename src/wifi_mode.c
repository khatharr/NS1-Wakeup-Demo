#include "wifi_mode.h"
#include "boot_ctrl.h"
#include "status_led.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include "lwip/sockets.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

// Temporary protocol settings; replace with the remote-control system's values.
#define WIFI_LISTEN_PORT 49152
static const char WIFI_WAKE_TRIGGER[] = "wake";

// Event bits shared by the Wi-Fi event loop and the waiting caller.
#define WIFI_IP_READY BIT0
#define WIFI_CONNECT_FAILED BIT1
#define WIFI_STA_STARTED BIT2
#define WIFI_STA_STOPPED BIT3
#define WIFI_ASSOCIATED BIT4
#define WIFI_CONNECT_TIMEOUT_MS 30000
#define WIFI_RETRY_DELAY_MS 5000
#define WIFI_CONNECT_ATTEMPTS 3
#define WIFI_STATION_EVENT_TIMEOUT_MS 5000
#define WIFI_RECV_TIMEOUT_MS 1000

static EventGroupHandle_t s_wifi_events;
static bool s_wifi_initialized;
static bool s_station_started;

static const char *disconnect_reason_name(unsigned reason)
{
    switch (reason) {
    case WIFI_REASON_NO_AP_FOUND: return "AP not found";
    case WIFI_REASON_AUTH_FAIL: return "authentication failed";
    case WIFI_REASON_ASSOC_FAIL: return "association failed";
    case WIFI_REASON_HANDSHAKE_TIMEOUT: return "handshake timeout";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return "4-way handshake timeout";
    case WIFI_REASON_BEACON_TIMEOUT: return "beacon timeout";
    case WIFI_REASON_AUTH_EXPIRE: return "authentication expired";
    case WIFI_REASON_ASSOC_EXPIRE: return "association expired";
    case WIFI_REASON_CONNECTION_FAIL: return "connection failed";
    case WIFI_REASON_ASSOC_LEAVE: return "association left";
    default: return "see ESP-IDF Wi-Fi reason codes";
    }
}

// Only signal events here; all retries and delays run in the Wi-Fi task.
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        xEventGroupSetBits(s_wifi_events, WIFI_STA_STARTED);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_STOP) {
        xEventGroupClearBits(s_wifi_events, WIFI_IP_READY | WIFI_ASSOCIATED);
        xEventGroupSetBits(s_wifi_events, WIFI_STA_STOPPED);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        xEventGroupSetBits(s_wifi_events, WIFI_ASSOCIATED);
        ESP_LOGI("wifi_mode", "Station associated; waiting for DHCP");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event = data;
        ESP_LOGW("wifi_mode", "Station disconnected: reason=%u (%s), RSSI=%d dBm",
                 (unsigned)event->reason, disconnect_reason_name(event->reason),
                 (int)event->rssi);
        status_led_set(STATUS_LED_WIFI_WAIT);
        xEventGroupClearBits(s_wifi_events, WIFI_IP_READY | WIFI_ASSOCIATED);
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECT_FAILED);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_events, WIFI_IP_READY);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
        ESP_LOGW("wifi_mode", "Station lost its IPv4 address");
        status_led_set(STATUS_LED_WIFI_WAIT);
        xEventGroupClearBits(s_wifi_events, WIFI_IP_READY);
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECT_FAILED);
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

// Initialize the network stack once per boot, before starting the station.
static esp_err_t wifi_init(void)
{
    if (s_wifi_initialized) return ESP_OK;

    const char *step = "nvs_flash_init";
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) goto failed;

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
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&config);
    if (err != ESP_OK) goto failed;

    step = "esp_wifi_set_mode";
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) goto failed;

    step = "xEventGroupCreate";
    s_wifi_events = xEventGroupCreate();
    if (s_wifi_events == NULL) {
        err = ESP_ERR_NO_MEM;
        goto failed;
    }

    step = "register Wi-Fi event handler";
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                     wifi_event_handler, NULL);
    if (err != ESP_OK) goto failed;

    step = "register IP event handler";
    err = esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                     wifi_event_handler, NULL);
    if (err != ESP_OK) goto failed;

    s_wifi_initialized = true;
    return ESP_OK;

failed:
    ESP_LOGE("wifi_mode", "Wi-Fi initialization failed at %s: %s (0x%x)",
             step, esp_err_to_name(err), (unsigned)err);
    return err;
}

static bool wifi_ip_ready(void)
{
    EventBits_t bits = xEventGroupGetBits(s_wifi_events);
    return (bits & WIFI_IP_READY) && !(bits & WIFI_CONNECT_FAILED);
}

// Stop the station, but retain the initialized driver, netif and handlers.
// Wait for the stop event before clearing bits for a new connection attempt.
static esp_err_t wifi_stop_station(void)
{
    if (!s_station_started) return ESP_OK;
    xEventGroupClearBits(s_wifi_events, WIFI_STA_STOPPED);
    esp_err_t err = esp_wifi_stop();
    if (err != ESP_OK) {
        ESP_LOGE("wifi_mode", "esp_wifi_stop failed: %s (0x%x)",
                 esp_err_to_name(err), (unsigned)err);
        return err;
    }
    EventBits_t bits = xEventGroupWaitBits(s_wifi_events, WIFI_STA_STOPPED,
        pdFALSE, pdTRUE, pdMS_TO_TICKS(WIFI_STATION_EVENT_TIMEOUT_MS));
    if (!(bits & WIFI_STA_STOPPED)) {
        ESP_LOGE("wifi_mode", "Timed out waiting for station stop event");
        return ESP_ERR_TIMEOUT;
    }
    s_station_started = false;
    return ESP_OK;
}

static esp_err_t wifi_connect_once(void)
{
    xEventGroupClearBits(s_wifi_events, WIFI_IP_READY | WIFI_CONNECT_FAILED |
                         WIFI_STA_STARTED | WIFI_STA_STOPPED | WIFI_ASSOCIATED);
    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE("wifi_mode", "esp_wifi_start failed: %s (0x%x)",
                 esp_err_to_name(err), (unsigned)err);
        return err;
    }
    s_station_started = true;
    EventBits_t bits = xEventGroupWaitBits(s_wifi_events, WIFI_STA_STARTED,
        pdFALSE, pdTRUE, pdMS_TO_TICKS(WIFI_STATION_EVENT_TIMEOUT_MS));
    if (!(bits & WIFI_STA_STARTED)) {
        ESP_LOGE("wifi_mode", "Timed out waiting for station start event");
        return ESP_ERR_TIMEOUT;
    }

    err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE("wifi_mode", "esp_wifi_connect failed: %s (0x%x)",
                 esp_err_to_name(err), (unsigned)err);
        return err;
    }

    bits = xEventGroupWaitBits(s_wifi_events, WIFI_IP_READY | WIFI_CONNECT_FAILED,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
    if (bits & WIFI_CONNECT_FAILED) return ESP_FAIL;
    if (bits & WIFI_IP_READY) return ESP_OK;
    ESP_LOGE("wifi_mode", "Connection timeout after %d ms (%s)",
             WIFI_CONNECT_TIMEOUT_MS,
             (bits & WIFI_ASSOCIATED) ? "waiting for DHCP" : "waiting for association");
    return ESP_ERR_TIMEOUT;
}

esp_err_t wifi_connect(void)
{
    esp_err_t err = wifi_init();
    if (err != ESP_OK) return err;

    status_led_set(STATUS_LED_WIFI_WAIT);
    for (unsigned attempt = 1; attempt <= WIFI_CONNECT_ATTEMPTS; ++attempt) {
        ESP_LOGI("wifi_mode", "Wi-Fi connection attempt %u/%u",
                 attempt, (unsigned)WIFI_CONNECT_ATTEMPTS);
        err = wifi_connect_once();
        if (err == ESP_OK && wifi_ip_ready()) {
            ESP_LOGI("wifi_mode", "Wi-Fi connected on attempt %u", attempt);
            return ESP_OK;
        }
        if (err == ESP_OK) err = ESP_FAIL;
        ESP_LOGE("wifi_mode", "Wi-Fi attempt %u/%u failed: %s (0x%x)",
                 attempt, (unsigned)WIFI_CONNECT_ATTEMPTS,
                 esp_err_to_name(err), (unsigned)err);

        esp_err_t stop_err = wifi_stop_station();
        if (stop_err != ESP_OK) return stop_err;

        if (attempt < WIFI_CONNECT_ATTEMPTS) {
            ESP_LOGW("wifi_mode", "Retrying Wi-Fi in %d ms", WIFI_RETRY_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY_MS));
        }
    }
    return err;
}

/* Listen on IPv4 UDP after DHCP succeeds. Only an exact payload match
 * requests a wake; all other datagrams are silently discarded. */
static esp_err_t wifi_listen(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE("wifi_mode", "UDP socket failed: errno %d", errno);
        return ESP_FAIL;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(WIFI_LISTEN_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE("wifi_mode", "UDP bind failed: errno %d", errno);
        close(sock);
        return ESP_FAIL;
    }

    // Poll readiness even when no UDP traffic arrives.
    const struct timeval timeout = {
        .tv_sec = WIFI_RECV_TIMEOUT_MS / 1000,
        .tv_usec = (WIFI_RECV_TIMEOUT_MS % 1000) * 1000,
    };
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        ESP_LOGE("wifi_mode", "UDP receive timeout setup failed: errno %d", errno);
        close(sock);
        return ESP_FAIL;
    }
    status_led_set(STATUS_LED_READY);

    // One extra byte beyond the trigger length rejects longer datagrams
    // even when recvfrom truncates them to the size of this buffer.
    char packet[sizeof(WIFI_WAKE_TRIGGER)];
    while (wifi_ip_ready()) {
        ssize_t received = recvfrom(sock, packet, sizeof(packet), 0, NULL, NULL);
        if (received < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            if (!wifi_ip_ready()) break;
            ESP_LOGE("wifi_mode", "UDP receive failed: errno %d", errno);
            close(sock);
            return ESP_FAIL;
        }

        if (!wifi_ip_ready()) break;
        if (received == sizeof(WIFI_WAKE_TRIGGER) - 1 &&
            memcmp(packet, WIFI_WAKE_TRIGGER, sizeof(WIFI_WAKE_TRIGGER) - 1) == 0) {
            close(sock);
            boot_request_wake();
        }
    }
    close(sock);
    ESP_LOGW("wifi_mode", "Wi-Fi lost; closing UDP listener");
    return ESP_ERR_INVALID_STATE;
}

void wifi_main(void)
{
    for (;;) {
        esp_err_t err = wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGE("wifi_mode", "Wi-Fi recovery failed: %s (0x%x); rebooting",
                     esp_err_to_name(err), (unsigned)err);
            // Also rate-limit reboots if initialization itself fails.
            vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY_MS));
            esp_restart();
        }

        boot_report_pending_error();
        err = wifi_listen();
        status_led_set(STATUS_LED_WIFI_WAIT);
        if (err != ESP_ERR_INVALID_STATE && wifi_ip_ready()) {
            // A local socket failure is not a station association failure.
            ESP_LOGE("wifi_mode", "UDP listener failed with Wi-Fi connected; rebooting");
            vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY_MS));
            esp_restart();
        }

        err = wifi_stop_station();
        if (err != ESP_OK) {
            ESP_LOGE("wifi_mode", "Cannot reset Wi-Fi station; rebooting");
            vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY_MS));
            esp_restart();
        }
        ESP_LOGW("wifi_mode", "Reconnecting Wi-Fi in %d ms", WIFI_RETRY_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY_MS));
    }
}
