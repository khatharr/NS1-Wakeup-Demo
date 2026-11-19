#ifndef SWPRO_SPI_H
#define SWPRO_SPI_H

#include <stdint.h>

/**
 * @brief Switch ProEmulate reading the Switch Pro Controller SPI flash
 *
 * @param address SPI address at which to begin reading
 * @param length Number of bytes to read
 * @param out Output buffer for the read result
 * @note Fill undefined addresses and ranges beyond the end of a defined region with 0xFF
 * @note outDo not write if out is NULL or length is 0
 */
void switch_spi_read(uint16_t address, uint8_t length, uint8_t *out);

#endif
