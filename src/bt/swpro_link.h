#ifndef SWPRO_LINK_H
#define SWPRO_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_event.h"
#include "boot_ctrl.h"
#include "esp_gap_bt_api.h"
#include "esp_sdp_api.h"

/**
 * @brief No saved Switch address
 * @note Accept pairing in discoverable and connectable mode
 */
extern volatile bool g_pairing;

/**
 * @brief Automatic paging and connectable mode are stopped
 * @note Run the stop sequence only once; repeated stops on every disconnect notification cause Switch reconnections and ESP32 disconnect requests to repeat.
 */
extern volatile bool g_stay_disconnected;

/**
 * @brief Consecutive HID connection request failures
 * @note Restart and reinitialize the Bluetooth connection state after a threshold is reached
 */
extern uint32_t g_connect_failures;

/** @brief Number of slots in the link supervision timeout */
#define LINK_SUPERVISION_TIMEOUT_SLOTS 3200
/**
 * @brief Number of slots in a paging timeout
 * @note One slot is 0.625 ms; 0x1400 is 3.2 seconds.
 */
#define WAKE_PAGE_TIMEOUT_SLOTS UINT16_C(0x1400)
extern void BTM_SetDefaultLinkSuperTout(uint16_t timeout);

void swpro_gap_callback(esp_bt_gap_cb_event_t event,
                        esp_bt_gap_cb_param_t *param);
void swpro_hidd_event_callback(void *handler_args, esp_event_base_t base,
                               int32_t id, void *event_data);
#if CONFIG_BT_SDP_COMMON_ENABLED
void swpro_sdp_callback(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param);
#endif

/**
 * @brief Progress after registering the wake-up-created ACL link with the host Bluetooth stack until ACL disconnection and normal paging begin
 */
extern volatile bool g_wake_page_pending;
extern volatile int64_t g_wake_page_deadline_us;
extern volatile bool g_wake_hid_connect_pending;
extern volatile uint8_t g_wake_disconnect_tries;
extern bool g_wake_local_release_tried;

/**
 * @brief Scheduled restart time while HID connections and wake-up processing are stopped
 * @note 0 means no restart is scheduled
 */
extern int64_t g_idle_restart_due_us;

void swpro_protocol_task(void *argument);

/**
 * @brief Restart after waiting for ACL disconnection to finish
 * @param restart_marker Marker selecting the post-restart action
 * @note Avoid restarting with an active ACL link, since the Switch would retain that link
 */
void swpro_restart_after_disconnect(uint32_t restart_marker);

/** Stop Bluetooth, disconnect the ACL link, and return to Wi-Fi. */
void swpro_finish_wake(wake_result_t result) __attribute__((noreturn));

/**
 * @brief Stop automatic paging and connectable mode
 * @param reason Reason logged for stopping
 * @note Do not resume automatic connections until a connection attempt starts from HOME input
 */
void swpro_stop_auto_connect(const char *reason);

#endif
