/**
 * @brief SwitchのBD_ADDRとペアリング情報の保存先
 * @note 保存先は次の3か所
 *       - NVSのキーhost
 *       - BlueDroidのボンド情報
 *       - HID Virtual Cable情報
 * @note 削除時は3か所すべての情報を削除してから再起動する
 * @note 削除せずに再起動すると、次回起動時に保存内容が一致しない
 */
#ifndef SWPRO_PAIRING_H
#define SWPRO_PAIRING_H

#include <stdbool.h>

#include "esp_bt_defs.h"

/**
 * @brief NVSを開いて保存済みSwitchのBD_ADDRを読み込む
 */
void swpro_pairing_init(void);

/**
 * @brief 保存済みSwitchのBD_ADDRをoutへコピーする
 * @param out BD_ADDRの書き込み先
 * @return 保存値があればtrue、なければfalse
 */
bool swpro_host_snapshot(esp_bd_addr_t out);

bool swpro_host_exists(void);

/**
 * @brief SwitchのBD_ADDRを保存する
 * @param address 保存するBD_ADDR
 * @note 既存値と一致する場合はNVSへ書き込まない
 */
void swpro_host_store(const esp_bd_addr_t address);

/**
 * @brief NVSのBD_ADDR、BlueDroidのボンド情報、HID Virtual Cable情報を削除して再起動する
 * @note 再接続にはSwitch側の登録操作が必要である
 * @note Switchと再ペアリングするまで、ESP32側からSwitchのスリープ解除を開始できない。
 */
void swpro_pairing_reset(void);

#endif
