/**
 * @brief Bluetoothアドレスからアクセスコードとウェイクアップコードを算出する
 *
 * @note
 * - Switchのスリープ解除で使用するアクセスコードは、BR/EDR標準アクセスコードの下位32ビットを反転した値である。
 * - Bluetoothアドレスの末尾3バイト(LAP、24ビット)へBarker系列6ビットを付加した30ビットを(64,30)短縮BCH符号で符号化し、64ビットのPN系列を重ねる。
 * - 同じLAPであれば上位3バイトが異なっても結果は変わらない。
 */
#ifndef WAKEUP_CODE_H
#define WAKEUP_CODE_H

#include <stdint.h>

/**
 * @brief Bluetoothアドレスに対応するBR/EDRアクセスコードの64ビット部を返す
 * @param address Bluetoothアドレス
 * @return アクセスコードの64ビット部
 */
uint64_t switch1_sync_word(const uint8_t address[6]);

/**
 * @brief 64ビット値の下位34ビットであるBCHパリティ部を返す
 * @param address Bluetoothアドレス
 * @return BCHパリティ部
 * @note CSBのcontrol structureへ書き込むと、ハードウェアが72ビットのアクセスコードを組み立てる。
 */
uint64_t switch1_parity34(const uint8_t address[6]);

/**
 * @brief スリープ解除で使用するアクセスコードのパリティを返す
 * @param address Bluetoothアドレス
 * @return switch1_parity34()の下位32ビットを反転したパリティ
 */
uint64_t switch1_wakeup_parity34(const uint8_t address[6]);

/**
 * @brief フェーズ1のペイロードへ反復配置する4バイトを生成する
 * @param address Bluetoothアドレス
 * @param[out] out 生成した4バイトを下位バイトから格納するバッファ
 * @note 64ビット値の下位32ビットを反転した値であり、電波上ではウェイクアップ用アクセスコードの下位32ビットと同じ並びになる。
 */
void switch1_wakeup_code(const uint8_t address[6], uint8_t out[4]);

/**
 * @brief Bluetoothアドレスを送信バッファの下位バイト先頭の並びへ変換する
 * @param address esp_bd_addr_tの上位バイト先頭のアドレス
 * @param[out] out 変換後の6バイトを書き込むバッファ
 */
void switch1_bd_addr_lsb_first(const uint8_t address[6], uint8_t out[6]);

/**
 * @brief BR/EDRのペイロードヘッダーを生成する
 * @param length ペイロード長をバイト単位で指定する
 * @param flow FLOWフィールドへ格納するビット値
 * @param llid LLIDフィールドへ格納するビット値
 * @return 長さ、FLOW、LLIDを指定位置へ格納したペイロードヘッダー
 */
uint16_t switch1_txphdr(uint8_t length, uint8_t flow, uint8_t llid);

#endif
