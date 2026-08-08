#include "wake_payload.h"

#include <string.h>

#include "baseband.h"
#include "wakeup_code.h"

/*
 * 白色化列の初期値はピコネットクロックのCLK[6:1]で決まり、送信間隔3.75ms(6スロット)ではバーストごとに次の列へ進む。
 *
 * - 要素はバーストごとの白色化列の順に並ぶ。
 * - 周期32で全32種類を順に使用する。
 * - この順序を変更してはならない。
 */
static const uint8_t k_whitening_seed_cycle[WHITEN_VARIANT_COUNT] = {
    0x69, 0x01, 0x31, 0x7D, 0x05, 0x5B, 0x6B, 0x03,
    0x5F, 0x37, 0x07, 0x59, 0x21, 0x6D, 0x5D, 0x35,
    0x7B, 0x13, 0x23, 0x6F, 0x17, 0x49, 0x79, 0x11,
    0x4D, 0x25, 0x15, 0x4B, 0x33, 0x7F, 0x4F, 0x27,
};

uint8_t wake_whiten_variant_index(uint32_t clkn, uint8_t offset)
{
    const uint32_t m = (clkn >> 2) & 0x1F;
    return (uint8_t)((11u * m + offset) & 0x1F);
}

/*
 * 白色化用7ビットLFSRを1段進める。
 * - 出力 = reg & 1
 * - fb = (reg ^ (reg >> 4)) & 1
 * - reg = (reg >> 1) | (fb << 6)
 */
static uint8_t advance_whitening_lfsr(uint8_t reg, uint8_t *out_bit)
{
    if (out_bit != NULL) {
        *out_bit = reg & 1;
    }
    const uint8_t fb = (reg ^ (reg >> 4)) & 1;
    return (uint8_t)((reg >> 1) | (fb << 6));
}

void wake_whiten_cancel(uint8_t *body, size_t length, uint8_t seed, int skip)
{
    uint8_t reg = seed & 0x7F;
    for (int i = 0; i < skip; ++i) {
        reg = advance_whitening_lfsr(reg, NULL);
    }
    for (size_t j = 0; j < length * 8; ++j) {
        uint8_t bit;
        reg = advance_whitening_lfsr(reg, &bit);
        if (bit) {
            body[j / 8] ^= (uint8_t)(1u << (j % 8));
        }
    }
}

void wake_build_phase1(const uint8_t target[6], uint8_t *body, uint16_t length)
{
    uint8_t wakeup_code[4];
    switch1_wakeup_code(target, wakeup_code);
    const uint32_t wakeup_code_value = (uint32_t)wakeup_code[0] |
                                       ((uint32_t)wakeup_code[1] << 8) |
                                       ((uint32_t)wakeup_code[2] << 16) |
                                       ((uint32_t)wakeup_code[3] << 24);
    const uint32_t rotated_wakeup_code =
        (wakeup_code_value >> 10) | (wakeup_code_value << (32 - 10));
    uint8_t repeated_bytes[4];
    repeated_bytes[0] = (uint8_t)(rotated_wakeup_code & 0xFF);
    repeated_bytes[1] = (uint8_t)((rotated_wakeup_code >> 8) & 0xFF);
    repeated_bytes[2] = (uint8_t)((rotated_wakeup_code >> 16) & 0xFF);
    repeated_bytes[3] = (uint8_t)((rotated_wakeup_code >> 24) & 0xFF);
    for (uint16_t i = 0; i < length; ++i) {
        body[i] = repeated_bytes[i % 4];
    }
    if (length == SWITCH1_CSB_DATA_MAX) {
        body[336] = 0x00;
        body[337] = 0x00;
        body[338] = 0x00;
    }
}

void wake_build_phase1_variants(const uint8_t target[6], uint8_t *variants,
                                 uint16_t stride, uint16_t length)
{
    for (uint8_t i = 0; i < WHITEN_VARIANT_COUNT; ++i) {
        uint8_t *const body = variants + (size_t)i * stride;
        wake_build_phase1(target, body, length);
        wake_whiten_cancel(body, length, k_whitening_seed_cycle[i],
                           WHITEN_SKIP_MULTISLOT);
    }
}

void wake_build_phase3(const uint8_t addr[6],
                        uint8_t variants[][WAKE_PHASE3_LENGTH],
                        uint8_t plain[WAKE_PHASE3_LENGTH])
{
    /*
     * ウェイクアップ用アクセスコードの往復では白色化を使用しないため、フェーズ3もフェーズ1と同様に相殺処理を施し、電波上の本体を白色化前のビット列にする。
     *
     * - DM1ではペイロードヘッダーが8ビット形式である。
     * - 読み飛ばすビット数はDH3やDH5と異なる。
     */
    for (uint8_t i = 0; i < WHITEN_VARIANT_COUNT; ++i) {
        uint8_t *const body = variants[i];
        memset(body, 0, WAKE_PHASE3_LENGTH);
        switch1_bd_addr_lsb_first(addr, body);
        wake_whiten_cancel(body, WAKE_PHASE3_LENGTH, k_whitening_seed_cycle[i],
                           WHITEN_SKIP_DM1);
    }
    memset(plain, 0, WAKE_PHASE3_LENGTH);
    switch1_bd_addr_lsb_first(addr, plain);
}
