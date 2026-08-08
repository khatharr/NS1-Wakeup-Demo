#include "wakeup_code.h"

/*
 * Bluetooth Core SpecificationのBaseband Specificationで定義されている値。
 * - PNは64ビットの疑似乱数列である。
 * - 0x37CD0EB67は原始2元(63,30)BCH符号の生成多項式である。
 * - GENERATORには、この多項式へ(D+1)を乗じた値を使用する。
 */
#define PN UINT64_C(0x83848D96BBCC54FC)
#define GENERATOR UINT64_C(0x585713DA9)

// GF(2)上で多項式除算を行い、剰余を返す。加減算にはXORを使用する。
static uint64_t gf2_remainder(uint64_t value, uint64_t divisor, int top_bit)
{
    for (int bit = top_bit; bit >= 34; --bit) {
        if (value & (UINT64_C(1) << bit)) {
            value ^= divisor << (bit - 34);
        }
    }
    return value;
}

uint64_t switch1_sync_word(const uint8_t address[6])
{
    /*
     * - 使用するのは表記上の末尾3バイト(LAP)だけである。
     * - LAPの最上位ビットで選択した6ビットのBarker系列を付加し、30ビットにする。
     * - 符号化前にPN系列の上位30ビットを重ねる。
     * - 34ビット左へシフトし、生成多項式で割った剰余をパリティにする。
     * - 64ビットのPN系列を重ねて同期語を得る。
     */
    const uint32_t lap = ((uint32_t)address[3] << 16) |
                         ((uint32_t)address[4] << 8) | address[5];

    const uint32_t information =
        ((lap & UINT32_C(0x800000)) ? UINT32_C(0x13000000)
                                    : UINT32_C(0x2C000000)) | lap;

    const uint32_t encoded = information ^ (uint32_t)(PN >> 34);

    const uint64_t shifted = (uint64_t)encoded << 34;
    const uint64_t codeword =
        shifted ^ gf2_remainder(shifted, GENERATOR, 63);

    return codeword ^ PN;
}

uint64_t switch1_parity34(const uint8_t address[6])
{
    return switch1_sync_word(address) & UINT64_C(0x3FFFFFFFF);
}

uint64_t switch1_wakeup_parity34(const uint8_t address[6])
{
    return switch1_parity34(address) ^ UINT64_C(0xFFFFFFFF);
}

void switch1_wakeup_code(const uint8_t address[6], uint8_t out[4])
{
    const uint32_t value = (uint32_t)~switch1_sync_word(address);
    for (unsigned i = 0; i < 4; ++i) {
        out[i] = (uint8_t)(value >> (8 * i));
    }
}

void switch1_bd_addr_lsb_first(const uint8_t address[6], uint8_t out[6])
{
    for (unsigned i = 0; i < 6; ++i) {
        out[i] = address[5 - i];
    }
}

uint16_t switch1_txphdr(uint8_t length, uint8_t flow, uint8_t llid)
{
    return (uint16_t)(((uint16_t)length << 3) | ((uint16_t)(flow & 1u) << 2) |
                      (uint16_t)(llid & 3u));
}
