#ifndef SWPRO_SPI_H
#define SWPRO_SPI_H

#include <stdint.h>

/**
 * @brief Switch ProコントローラーのSPIフラッシュ読み出しを模擬する
 *
 * @param address 読み出しを開始するSPIアドレス
 * @param length 読み出すバイト数
 * @param out 読み出し結果を書き込むバッファ
 * @note 未定義のアドレスと定義済み領域の終端を超えた範囲は0xFFで埋める
 * @note outがNULLまたはlengthが0の場合は書き込みを実行しない
 */
void switch_spi_read(uint16_t address, uint8_t length, uint8_t *out);

#endif
