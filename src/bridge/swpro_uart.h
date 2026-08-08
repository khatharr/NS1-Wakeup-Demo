/**
 * @brief 入力ブリッジからUART0で受信するフレームの構成
 * @note フィールドの順序は次のとおり
 *       1. ヘッダー0xAB 0xCD
 *       2. コマンド
 *       3. payload長
 *       4. payload
 *       5. XORチェックサム
 */
#ifndef SWPRO_UART_H
#define SWPRO_UART_H

#include <stdint.h>

#define CMD_INPUT_STATE 0x01
#define CMD_PAIR 0x02
#define CMD_CLEAR 0x03
#define CMD_REBOOT 0x04
/**
 * @brief ウェイクアップ処理の送信を停止し、通常のページ接続を再開する
 * @note payloadは使用しない
 */
#define CMD_WAKE_STOP 0x09
/**
 * @brief ウェイクアップ処理の送信を開始する
 * @note payloadの項目は次のとおり
 *       - [0] RFチャンネル。0は2402MHz、24は2426MHz
 *       - [1] 送信時間。1~30秒
 *       - [2] 白色化を無効にするときの初期値。0~127。0xFFは無効化しない
 *       - [5] バーストごとの切り替えを開始する位置。0~31。0xFFは切り替えない
 *       - [27] 0以外なら、ウェイクアップ応答からACLリンクを登録する処理を無効にする
 */
#define CMD_WAKE_START 0x0B
/**
 * @brief ウェイクアップ処理用アクセスコードでCSB受信を制御する
 * @note payloadの項目は次のとおり
 *       - payload[0]が1なら開始し、0なら停止する
 *       - payload[1]は受信間隔をスロット数で指定する
 * @note フェーズ1の送信中に開始すると、Switchの応答検出後にフェーズ3の送信とウェイクアップ応答からACLリンクを登録する処理まで連続して実行する。
 */
#define CMD_CSB_RX 0x12
/**
 * @brief 接続試行を繰り返す
 * @note 1バイトpayloadの値が1なら開始し、0なら停止する
 * @note ウェイクアップ処理と通常の再接続を交互に実行し、接続が成立するまで継続する
 */
#define CMD_CONNECT_CYCLE 0x13
/**
 * @brief SwitchのBD_ADDRだけを保存する
 * @note ペアリング情報を削除したあとでもESP32からウェイクアップ処理を開始できるように、接続先アドレスを設定する。
 */
#define CMD_SET_HOST 0x14
/**
 * @brief ウェイクアップ処理を実行せず、ページ接続を1回だけ開始する
 * @note 起動中のSwitchへ通常のページング手順で再接続する
 */
#define CMD_PAGE_NOW 0x15
/**
 * @brief 接続試行、ウェイクアップ送信、自動接続を停止してリンクを切断する
 * @note payloadは使用しない
 */
#define CMD_DISCONNECT 0x16
/** @brief NS2ウェイク広告を取得するBLEスキャンを開始する。payloadは使用しない。 */
#define CMD_BLE_SCAN 0x17
/** @brief 保存済み広告を使用してNS2ウェイクを実行する。payloadは使用しない。 */
#define CMD_NS2_WAKE_UP 0x18


void apply_uart_frame(uint8_t command, const uint8_t *payload, uint8_t length);

void swpro_uart_task(void *argument);

#endif
