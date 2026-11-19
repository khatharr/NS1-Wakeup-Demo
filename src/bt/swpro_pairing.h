/**
 * @brief SwitchStorage locations for the Switch BD_ADDR and pairing information
 * @note Three storage locations:
 *       - NVSThe NVS key host
 *       - BlueDroidBlueDroid bond information
 *       - HID Virtual CableHID Virtual Cable information
 * @note On deletion, remove all three stores before restarting
 * @note If restarted without removing all three, the saved data will disagree on the next boot
 */
#ifndef SWPRO_PAIRING_H
#define SWPRO_PAIRING_H

#include <stdbool.h>

#include "esp_bt_defs.h"

/**
 * @brief NVSOpen NVS and load the saved Switch BD_ADDR
 */
void swpro_pairing_init(void);

/**
 * @brief Copy the saved Switch BD_ADDR into out
 * @param out BD_ADDROutput destination for the BD_ADDR
 * @return true if a value is stored, otherwise false
 */
bool swpro_host_snapshot(esp_bd_addr_t out);

bool swpro_host_exists(void);

/**
 * @brief SwitchSave the Switch BD_ADDR
 * @param address BD_ADDR to save
 * @note Do not write NVS if it matches the existing value
 */
void swpro_host_store(const esp_bd_addr_t address);

/**
 * @brief NVSDelete the NVS BD_ADDR, BlueDroid bond, and HID Virtual Cable information, then restart
 * @note Reconnection requires registering the controller again on the Switch
 * @note SwitchThe ESP32 cannot initiate wake-up until it is paired with the Switch again.
 */
void swpro_pairing_reset(void);

#endif
