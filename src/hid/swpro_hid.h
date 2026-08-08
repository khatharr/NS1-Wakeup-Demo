#ifndef SWPRO_HID_H
#define SWPRO_HID_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_event.h"

/**
 * @brief 入力レポートの長さ
 *
 * モードに関わらず、レポートID 0x30を48バイトで送信する。
 */
#define SWPRO_REPORT_SIZE 48

/**
 * @brief 入力レポートの既定送信間隔をマイクロ秒で示す
 *
 * @note GAPの通知に応じて、sniffモードでは送信間隔を延長する
 */
#define SWPRO_REPORT_INTERVAL_DEFAULT_US UINT32_C(8000)

typedef struct {
    uint32_t buttons;
    uint32_t stick_left;
    uint32_t stick_right;
    int16_t accel[3];
    int16_t gyro[3];
} swpro_input_t;

/**
 * @brief Bluetooth HIDデバイスを登録する
 *
 * @param handler esp_hidのイベントループから呼び出されるコールバック
 *                swpro_link.cが提供する
 * @return ESP-IDFのステータスコード
 */
esp_err_t swpro_hid_register(esp_event_handler_t handler);

void swpro_hid_set_input(const swpro_input_t *input);

uint32_t swpro_hid_input_frames(void);

void swpro_hid_send_report(void);

/**
 * @brief 現在の入力レポート送信間隔をマイクロ秒で返す
 *
 * @return 現在の送信間隔
 * @note GAPのモード変更に応じて値を更新する
 */
uint32_t swpro_hid_report_interval_us(void);

/**
 * @brief 入力レポート送信間隔を設定する
 *
 * @param interval_us マイクロ秒単位の新しい送信間隔
 */
void swpro_hid_set_report_interval_us(uint32_t interval_us);

void swpro_hid_reset_session(void);

/**
 * @brief Switchが送信した出力レポートを処理する
 *
 * @param report_id HIDレポートID
 * @param data レポートIDを除いたレポート本体
 * @param length dataのバイト数
 * @note レポートID 0x01のdataは次の配置とする
 * - data[0]はカウンタ
 * - data[1..8]は振動データ
 * - data[9]はサブコマンド
 * - data[10..]はサブコマンドの引数
 */
void swpro_hid_handle_output(uint8_t report_id, const uint8_t *data,
                             uint16_t length);

/**
 * @brief Switchが停止状態を要求したかどうかを返す
 *
 * @return サブコマンド0x06で状態0を受信した後はtrue、状態0を受信していない場合はfalse
 * @note Switchのスリープ移行中はtrueとなり、入力レポートを送信しない
 */
bool swpro_hid_host_stopped(void);

#endif
