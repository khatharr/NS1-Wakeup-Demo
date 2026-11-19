/*
 * If sm4 from the previous connection remains set, btm_sec_l2cap_access_req starts authentication without fetching the Switch's feature information.
 * Since this prevents an L2CAP connection response, set sm4 to the unknown state before starting wake-up.
 *
 * The fields involved are sec_state and sm4 in tBTM_SEC_DEV_REC.
 * Because the structure layout changes with sdkconfig settings, access members by name through internal header type definitions rather than calculating offsets manually.
 * The required include paths are configured in src/CMakeLists.txt.
 */
#include "host_sec_state.h"

#include <string.h>

#include "common/bt_target.h"
#include "stack/btm_api.h"
#include "btm_int.h"

#include "esp_log.h"

static const char *const TAG = "host_sec";

bool host_sec_state_read(const uint8_t bd_addr[6], host_sec_state_t *out)
{
    if (bd_addr == NULL || out == NULL) {
        return false;
    }
    tBTM_SEC_DEV_REC *const device_record = btm_find_dev((uint8_t *)bd_addr);
    if (device_record == NULL) {
        return false;
    }
    out->sec_state = device_record->sec_state;
    out->sm4 = device_record->sm4;
    out->sec_flags = device_record->sec_flags;
    out->security_required = device_record->security_required;
    out->link_key_type = device_record->link_key_type;
    out->is_originator = device_record->is_originator ? 1u : 0u;
    return true;
}

void host_sec_state_report(const uint8_t bd_addr[6], const char *log_label)
{
    host_sec_state_t state;
    if (!host_sec_state_read(bd_addr, &state)) {
        ESP_LOGI(TAG, "%s: device record is not registered",
                 log_label != NULL ? log_label : "");
        return;
    }
    ESP_LOGI(TAG,
             "%s: sec_state=%u sm4=0x%02x sec_flags=0x%04x "
             "security_required=0x%04x link_key_type=%u originator=%u",
             log_label != NULL ? log_label : "", state.sec_state, state.sm4,
             state.sec_flags, state.security_required, state.link_key_type,
             state.is_originator);
}

bool host_sec_state_reset_to_page_state(const uint8_t bd_addr[6])
{
    if (bd_addr == NULL) {
        return false;
    }
    tBTM_SEC_DEV_REC *const device_record = btm_find_dev((uint8_t *)bd_addr);
    if (device_record == NULL) {
        return false;
    }
    const uint8_t sec_state_before = device_record->sec_state;
    const uint8_t sm4_before = device_record->sm4;

    // Clear the state stuck waiting for the device name.
    // btm_sec_start_get_namebtm_sec_start_get_name has a timeout value of 0, so this state cannot clear until a device name is returned.
    // btm_sec_execute_procedurebtm_sec_execute_procedure exits without starting work unless sec_state is idle.
    if (device_record->sec_state != BTM_SEC_STATE_IDLE) {
        device_record->sec_state = BTM_SEC_STATE_IDLE;
    }

    // SwitchMark the Switch feature information as unknown.
    // sm4When sm4 is BTM_SM4_TRUE, btm_sec_l2cap_access_req requests authentication and encryption without waiting.
    // Resume processing after retrieving feature information, preserving the same order used during paging.
    device_record->sm4 = BTM_SM4_UNKNOWN;

    ESP_LOGI(TAG,
             "Set device record to post-page state: "
             "sec_state %u -> %u  "
             "sm4 0x%02x -> 0x%02x",
             sec_state_before, device_record->sec_state, sm4_before,
             device_record->sm4);
    return true;
}
