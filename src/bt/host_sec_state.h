#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t sec_state;
    uint8_t sm4;
    uint16_t sec_flags;
    uint16_t security_required;
    uint8_t link_key_type;
    uint8_t is_originator;
} host_sec_state_t;

/**
 * @brief SwitchRead connection-related fields from the Switch device record
 * @param bd_addr SwitchSwitch BD_ADDR
 * @param out Destination for the retrieved state
 * @return true if the device record exists, otherwise false
 */
bool host_sec_state_read(const uint8_t bd_addr[6], host_sec_state_t *out);

/**
 * @brief SwitchLog the connection state in the Switch device record
 * @param bd_addr SwitchSwitch BD_ADDR
 * @param log_label Label added to the log line; use an empty string if NULL
 */
void host_sec_state_report(const uint8_t bd_addr[6], const char *log_label);

/**
 * @brief Reset sec_state and sm4 in the device record to their post-paging state
 * @param bd_addr SwitchSwitch BD_ADDR
 * @return true if the device record was found and updated, otherwise false
 * @note Clear sec_state if it is stuck waiting for a device name, and mark the Switch feature information as unknown.
 *       The link key is not modified, so pairing information is preserved.
 */
bool host_sec_state_reset_to_page_state(const uint8_t bd_addr[6]);

#ifdef __cplusplus
}
#endif
