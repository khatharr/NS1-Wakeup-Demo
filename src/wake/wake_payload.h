/**
 * @brief Build wake-up transmit payloads and cancel whitening
 *
 * @note
 * - Hardware whitens transmitted bits using a seven-bit LFSR.
 * - The wake-up procedure uses unwhitened bits, so XOR the same whitening sequence before transmission.
 * - The sequence seed comes from the piconet clock and changes on each burst.
 */
#ifndef WAKE_PAYLOAD_H
#define WAKE_PAYLOAD_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Position in the whitening sequence corresponding to the first bit of the payload body
 * @note The packet header is 18 bits, followed by one of these payload headers:
 * - DM1: 8DM1: 8 bits
 * - DH3DH3 and DH5: 16 bits
 */
#define WHITEN_SKIP_DM1 26
#define WHITEN_SKIP_MULTISLOT 34

#define WHITEN_VARIANT_COUNT 32

/**
 * @brief Length of the Phase 3 payload body
 * @note Contains six bytes of the controller's BD_ADDR followed by six zero bytes
 */
#define WAKE_PHASE3_LENGTH 12

/**
 * @brief Calculate the whitening-sequence lookup index from the native clock
 * @param clkn Native clock
 * @param offset descriptorConstant correcting the difference between descriptor detection and the actual transmit slot
 * @return Index into the whitening-sequence lookup table
 * @note
 * - The seed depends on CLK[6:1]; CSB transmissions begin on even slots, yielding 32 possibilities.
 * - m = (CLKN >> 2) increases by 3 on each burst; the inverse of 3 modulo 32 is 11.
 * - Calculate the index as (11 * m + offset) mod 32.
 */
uint8_t wake_whiten_variant_index(uint32_t clkn, uint8_t offset);

/**
 * @brief XOR the sequence that cancels whitening into a payload body
 * @param[in,out] body Body from after the payload header to before the CRC
 * @param length Body length in bytes
 * @param seed 7Initial value of the seven-bit LFSR
 * @param skip Number of bits to skip to the beginning of the body
 * @note Bits within each byte are ordered least-significant bit first
 */
void wake_whiten_cancel(uint8_t *body, size_t length, uint8_t seed, int skip);

/**
 * @brief Build the Phase 1 payload body
 * @param target Target Switch Bluetooth address
 * @param[out] body Buffer receiving the constructed body
 * @param length Body length in bytes
 * @note
 * - Repeat the wake-up code rotated right 10 bits as four little-endian bytes.
 * - lengthIf length is at its maximum, zero the last three bytes.
 */
void wake_build_phase1(const uint8_t target[6], uint8_t *body,
                        uint16_t length);

/**
 * @brief Generate Phase 1 body variants for each whitening seed
 * @param target Target Switch Bluetooth address
 * @param[out] variants [WHITEN_VARIANT_COUNT][length]Buffer of at least [WHITEN_VARIANT_COUNT][length] for the generated data
 * @param stride variantsDistance between variant entries, in bytes
 * @param length Length of each body, in bytes
 */
void wake_build_phase1_variants(const uint8_t target[6], uint8_t *variants,
                                 uint16_t stride, uint16_t length);

/**
 * @brief Build Phase 3 bodies for all 32 whitening sequences and an unprocessed variant
 * @param addr Own Bluetooth address
 * @param[out] variants Array to receive 32 whitening-cancelled bodies
 * @param[out] plain Unprocessed body used when transmitter whitening is disabled
 * @note The body consists of six bytes of addr, least-significant byte first, and six zero bytes
 */
void wake_build_phase3(const uint8_t addr[6],
                        uint8_t variants[][WAKE_PHASE3_LENGTH],
                        uint8_t plain[WAKE_PHASE3_LENGTH]);

#endif
