/**
 * @brief ウェイクアップ送信用ペイロードを組み立て、白色化を相殺する
 *
 * @note
 * - ハードウェアは送信ビット列を7ビットのLFSRで白色化する。
 * - ウェイクアップ手順では白色化していないビット列を使用するため、送信前に同じ白色化列をXORする。
 * - 白色化列の初期値はピコネットクロックで決まり、バーストごとに変化する。
 */
#ifndef WAKE_PAYLOAD_H
#define WAKE_PAYLOAD_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief 白色化列でペイロード本体の先頭ビットに対応する位置を示す
 * @note パケットヘッダーは18ビットで、その後ろに次のペイロードヘッダーが続く
 * - DM1: 8ビット
 * - DH3とDH5: 16ビット
 */
#define WHITEN_SKIP_DM1 26
#define WHITEN_SKIP_MULTISLOT 34

#define WHITEN_VARIANT_COUNT 32

/**
 * @brief フェーズ3のペイロード本体の長さ
 * @note 自身のBD_ADDR 6バイトとゼロ6バイトで構成する
 */
#define WAKE_PHASE3_LENGTH 12

/**
 * @brief ネイティブクロックから白色化列の巡回表の添字を算出する
 * @param clkn ネイティブクロック
 * @param offset descriptor検出時刻と実際の送信スロットとの差を補正する定数
 * @return 白色化列の巡回表の添字
 * @note
 * - 初期値はCLK[6:1]で決まり、CSB送信は偶数スロットで始まるため32通りである。
 * - バーストごとにm = (CLKN >> 2)は3増加し、3のmod 32における逆元は11である。
 * - 添字は(11 * m + offset) mod 32で算出する。
 */
uint8_t wake_whiten_variant_index(uint32_t clkn, uint8_t offset);

/**
 * @brief ペイロード本体へ白色化を相殺するビット列をXORする
 * @param[in,out] body ペイロードヘッダーの後ろからCRCの手前までの本体
 * @param length 本体の長さをバイト単位で指定する
 * @param seed 7ビットLFSRの初期値
 * @param skip 本体の先頭ビットまで読み飛ばすビット数
 * @note ビットは各バイト内で下位ビットから並ぶ
 */
void wake_whiten_cancel(uint8_t *body, size_t length, uint8_t seed, int skip);

/**
 * @brief フェーズ1のペイロード本体を組み立てる
 * @param target 対象SwitchのBluetoothアドレス
 * @param[out] body 組み立てた本体を書き込むバッファ
 * @param length 本体の長さをバイト単位で指定する
 * @note
 * - ウェイクアップコードを右へ10ビット回した値をリトルエンディアンの4バイトとして反復配置する。
 * - lengthが最大長の場合は末尾3バイトを0にする。
 */
void wake_build_phase1(const uint8_t target[6], uint8_t *body,
                        uint16_t length);

/**
 * @brief フェーズ1の本体を白色化列の初期値ごとに生成する
 * @param target 対象SwitchのBluetoothアドレス
 * @param[out] variants [WHITEN_VARIANT_COUNT][length]以上の領域へ生成結果を書き込む
 * @param stride variantsの各要素間の距離をバイト単位で指定する
 * @param length 各本体の長さをバイト単位で指定する
 */
void wake_build_phase1_variants(const uint8_t target[6], uint8_t *variants,
                                 uint16_t stride, uint16_t length);

/**
 * @brief フェーズ3のペイロード本体を白色化列32通りに対応する内容と未処理の内容で生成する
 * @param addr 自身のBluetoothアドレス
 * @param[out] variants 白色化を相殺した32通りの本体を書き込む配列
 * @param[out] plain 送信側の白色化を無効にするときに使用する未処理の本体
 * @note 内容はaddrの下位バイト先頭6バイトとゼロ6バイトである
 */
void wake_build_phase3(const uint8_t addr[6],
                        uint8_t variants[][WAKE_PHASE3_LENGTH],
                        uint8_t plain[WAKE_PHASE3_LENGTH]);

#endif
