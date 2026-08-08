#ifndef BASEBAND_H
#define BASEBAND_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * @brief CSBで送出できるペイロード本体の最大長
 * @note ウェイクアップコードを反復したDH5(339バイト)を1本収める
 */
#define SWITCH1_CSB_DATA_MAX 339

/**
 * @brief 白色化処理を適用していないペイロード本体をCSBで送出する
 * @param bd_addr 接続相手のBD_ADDR
 * @param bch34 ウェイクアップ用のBCHパリティ
 * @param data 白色化前のペイロード本体
 * @param length dataの長さ
 * @param interval 送信間隔をスロット単位で指定する
 * @param channel チャンネルを指定する
 * @note channelは次の値を使用する。
 * - 0: 2402MHz
 * - 24: 2426MHz
 * ヘッダーとペイロードヘッダーはControllerが生成する。
 */
esp_err_t switch1_csb_raw_start(const uint8_t bd_addr[6], uint64_t bch34,
                                const uint8_t *data, uint16_t length,
                                uint16_t interval, uint8_t channel);

/**
 * @brief ROMが生成したdescriptorへウェイクアップ用のparityとチャンネルを更新する
 * @note 更新しない場合、アクセスコードが通常値になる。
 * 各バーストの送信前に呼び出す。
 */
esp_err_t switch1_csb_wake_refresh(void);

/**
 * @brief CSB送信の停止完了を待つ
 * @note 待機時間の上限はHCI層と同じ8秒である。
 */
esp_err_t switch1_csb_wake_stop_wait(void);

uint32_t switch1_csb_wake_events(void);

/**
 * @brief 送信descriptorの位置を返す
 * @return descriptorが未生成の場合は0を返す
 */
uint16_t switch1_csb_descriptor_offset(void);

/**
 * @brief 現在のdescriptorが指すペイロード領域の位置を返す
 * @return 位置を取得できない場合は0を返す
 * @note バースト間はdescriptorが存在しないため、送信中に取得した位置を保存して使用する。
 */
uint16_t switch1_csb_payload_offset(void);

/**
 * @brief 指定位置の送信ペイロードへ直接書き込む
 * @param payload_offset ペイロード領域の位置
 * @param data 書き込むデータ
 * @param length dataの長さ
 * @note descriptorが存在しないバースト間にも、保存済みの位置へ書き込める。
 */
esp_err_t switch1_csb_write_payload_at(uint16_t payload_offset,
                                       const uint8_t *data, uint16_t length);

esp_err_t switch1_csb_read_descriptor(uint16_t *out_header,
                                      uint16_t *out_pheader);

esp_err_t switch1_csb_write_descriptor(uint16_t header, uint16_t pheader);

/**
 * @brief 送信位置をスロット境界からのずれとして指定する
 * @param offset_us 0us~1023usの位置
 * @note control structure +0x02とスケジュール要素+0x14の両方へ書き込む。
 * 送信時刻は625us刻みだが、両方へ書くことで1us単位の位置を指定できる。
 */
esp_err_t switch1_csb_tx_set_bit_offset(uint16_t offset_us);

/**
 * @brief 送信スケジュール要素のスケジュール上の送信時間を更新する
 * @param duration_us スケジューラへ登録する送信時間をus単位で指定する
 * @note ESP-IDFが登録する9255usのままでは、CSB受信とのスケジュール競合を避けるため送信周期が15msになる。
 * 送信イベントの継続時間は1875usに設定する。
 */
esp_err_t switch1_csb_tx_set_duration(uint16_t duration_us);

/**
 * @brief 送信の白色化を有効または無効にする
 * @param disable trueで無効にする
 * @note control structure +0x08のbit12を操作する。
 * 無効時は次の処理を行う。
 * - ペイロード本体とペイロードヘッダーを白色化しない。
 * - ペイロードへ白色化前の内容を書く。
 */
void switch1_csb_tx_set_whitening_disable(bool disable);

/**
 * @brief 送信のフレームフォーマットを書き換える
 * @param format control structure +0x00のbit0-4へ指定するフレームフォーマット
 * @param[out] out_before NULLでなければ、書き換え前のワードを返す
 * @return 成功時はESP_OKを返す
 * @note 既定の形式は24(CSB送信)である。
 */
esp_err_t switch1_csb_tx_set_frame_format(uint8_t format, uint16_t *out_before);

bool switch1_csb_tx_read_schedule(uint32_t *out_next_slot,
                                  uint16_t *out_interval);

/**
 * @brief 待機中の送信イベントを指定位置へ再登録する
 * @param target_slot 送信スロット
 * @param bit_offset_us スロット内の位置をus単位で指定する
 * @param minimum_ahead_slots 現在時刻から確保する最小先行スロット数
 * @param[out] out_scheduled_slot 実際に登録したスロットを返す
 * @note 指定位置が近すぎる場合は現在の送信周期を加算し、同じ位相へ移動する。
 */
esp_err_t switch1_csb_tx_reschedule(uint32_t target_slot,
                                    uint16_t bit_offset_us,
                                    uint16_t minimum_ahead_slots,
                                    uint32_t *out_scheduled_slot);

/**
 * @brief 送信周期を書き換える
 * @param expect 現在値として期待する周期
 * @param interval 新しい周期をスロット単位で指定する
 * @return expectが現在値と一致しない場合はESP_ERR_INVALID_STATEを返す
 * @note 分解能は1スロット(625us)である。
 * 1回だけ別の値を設定すると、その送信だけ間隔が変わる。
 * 以降の送信時刻にも同じ変更量が反映され、フェーズ3の送信時刻を調整できる。
 */
esp_err_t switch1_csb_tx_set_interval(uint16_t expect, uint16_t interval);

/**
 * @brief CSB受信を開始する
 * @param bd_addr 接続相手のBD_ADDR
 * @param bch34 ウェイクアップ用のBCHパリティ
 * @param interval 受信を試みる周期をスロット単位で指定する
 * 偶数に限る
 * @param clock_offset 受信側へ設定するクロックオフセット
 * @param next_clock 次に受信を試みるクロック
 * @param channel 受信チャンネル
 * @return HCIコマンドの結果を返す
 * @note HCIのSet_Connectionless_Slave_Broadcast_Receive(0x0442)を送信したあと、受信側のcontrol structureへウェイクアップ用のparityを書き込む。
 */
esp_err_t switch1_csb_rx_start(const uint8_t bd_addr[6], uint64_t bch34,
                               uint16_t interval, uint32_t clock_offset,
                               uint32_t next_clock, uint8_t channel);

/**
 * @brief 監視タイムアウトで解放された受信スケジュール要素を再登録する
 * @param interval 受信を試みる周期をスロット単位で指定する
 * @param next_clock 次に受信を試みるクロック
 * @return HCIコマンドの結果を返す
 * @note 要素解放後もld_csb_rx_envが残るため、開始要求だけではr_ld_csb_rx_startが12(Command Disallowed)を返す。
 * 開始要求の前に停止要求を送信する。
 */
esp_err_t switch1_csb_rx_restart(uint16_t interval, uint32_t next_clock);

esp_err_t switch1_csb_rx_stop(const uint8_t bd_addr[6], uint16_t interval);

/**
 * @brief CSB受信が開始状態かを返す
 * @return 受信が動作中、または停止要求が拒否されて動作が継続中の場合はtrueを返す
 */
bool switch1_csb_rx_is_active(void);

/**
 * @brief 直近に受信したCSBパケットを取得する
 * @param[out] out_addr 受信元のBD_ADDRを返す
 * @param[out] out_clock 受信クロックを返す
 * @param[out] out_status 受信ステータスを返す
 * @param[out] out_data 受信データを返す
 * @param[out] out_length 受信データ長を返す
 * @return 受信データがない場合はfalseを返す
 * @note 取得後は保持中のパケットを解放する。
 */
bool switch1_csb_rx_take(uint8_t out_addr[6], uint32_t *out_clock,
                         uint8_t *out_status, uint8_t *out_data,
                         uint8_t *out_length);

/**
 * @brief ハードウェアが測った直近の受信時刻を読み出す
 * @param[out] out_clk 受信クロックをスロット単位で返す
 * @param[out] out_bitoff スロット内の位置をus単位で返す
 * 範囲は0us~624usである
 * @param[out] out_raw 補正前の値を返す
 * @return 上位ワードが範囲外の場合はfalseを返す
 */
bool switch1_csb_rx_read_sync(uint32_t *out_clk, uint16_t *out_bitoff,
                              uint16_t *out_raw);

/**
 * @brief 受信側control structureのLT_ADDRを書き換える
 * @param lt_addr +0x08のbit10:8へ書くLT_ADDR
 * @note Switchの応答はLT_ADDRが0だが、HCIコマンドは1~7しか受け付けない。
 */
void switch1_csb_rx_write_ltaddr(uint8_t lt_addr);

/**
 * @brief 受信のフレームフォーマットを書き換える
 * @param format control structure +0x00のbit0-4へ指定するフレームフォーマット
 * @note 既定は25である。
 * ACLの形式では次の値を使用する。
 * - 2または3: ハードウェアが受信ペイロードをバッファへ保存する。
 */
void switch1_csb_rx_set_format(uint8_t format);

esp_err_t switch1_csb_rx_dump(uint16_t offset, uint8_t count, uint16_t *out);

/**
 * @brief 成立した受信の件数を返す
 * @return 受信件数を返す
 * @note HCIイベント0x51のうちReceive_Statusが0のイベントを数える。
 * 受信descriptorのステータスは連続する成功で同じ値になるため、値の変化では受信を数えられない。
 */
uint32_t switch1_csb_rx_event_count(void);

void switch1_csb_rx_event_reset(void);

/**
 * @brief 受信descriptorを1回調べ、失敗フラグのない受信を返す
 * @param[out] out_status 受信descriptorのステータスを返す
 * @param[out] out_header 受信ヘッダーを返す
 * @param[out] out_pheader 受信ペイロードヘッダーを返す
 * @return 受信descriptorのステータス更新があり、ステータスが0でなく失敗フラグもない場合はtrueを返す
 * @note 待機処理を行わないため送信中にも呼び出せる。
 * フェーズ3の送信時刻をSwitchの応答時刻に合わせるために使用する。
 */
bool switch1_csb_rx_poll(uint16_t *out_status, uint16_t *out_header,
                         uint16_t *out_pheader);

/**
 * @brief 4個の受信descriptorのステータスとレベルを基準値として保存し、ステータス集計をクリアする
 * @note 監視開始前に存在していた受信を検出対象にしないために呼び出す。
 */
void switch1_csb_rx_poll_prime(void);

/**
 * @brief switch1_csb_rx_pollが集計したステータスの種類と回数をログへ出力する
 * @note 受信0件のとき、同期が成立していないのか、同期後に検査で除外されたのかを切り分ける。
 */
void switch1_csb_rx_poll_report(void);

/**
 * @brief 受信descriptorを指定時間だけ監視し、変化したdescriptorをログへ出力する
 * @param duration_ms 監視時間をms単位で指定する
 * @return 変化を検出した回数を返す
 * @note ld_csb_rx_frm_cbkが見る4個のdescriptorを直接読み出すため、HCIイベントがなくても受信を確認できる。
 */
uint32_t switch1_csb_rx_watch(uint32_t duration_ms);

/**
 * @brief 送信と受信のスケジュール要素を格納する
 * @note 送信と受信は同じ無線イベントのスケジューラを共有する。
 * - r_ea_elt_insertは要素+0x16の優先度を比較し、既存の要素が新規要素以上であれば新規登録を拒否する。
 * - 受信の優先度は10から始まり、失敗するたびに1ずつ増加する。
 * - 送信の優先度はフレーム完了のたびに10へ更新される。
 * - 受信が11以上になると送信を登録できない。
 */
typedef struct {
    /** スケジュール要素そのものの位置。0の場合は要素が解放され、CSBが停止している */
    uint32_t tx_element;
    uint32_t rx_element;
    /** +0x12にスケジューラへ登録する送信時間をus単位で格納 */
    uint16_t tx_duration_us;
    uint16_t rx_duration_us;
    /** +0x16の優先度を格納する */
    uint8_t tx_priority;
    uint8_t rx_priority;
    /** 受信の+0x36にある受信間隔をスロット単位で格納する。送信周期は+0x30にあるが、受信の+0x30はクロックオフセットである。 */
    uint16_t rx_interval_slots;
    /**
     * 受信の+0x3E状態を格納する。
     * - 0: 停止
     * - 1: 動作中
     * - 2: 停止要求の保留中
     */
    uint8_t rx_state;
    /**
     * 受信の+0x2Cにある監視タイムアウトの基準クロックと、+0x3Aにある監視タイムアウトを格納する。
     * ld_csb_rx_sched(0x400391f0)が経過時間と比較し、超過時に要素を解放する。
     */
    uint32_t rx_sup_base;
    uint16_t rx_sup_timeout_slots;
} switch1_csb_sched_t;

/**
 * @brief 送信と受信のスケジュール要素を読み出す
 * @param[out] out 読み出したスケジュール要素を返す
 * @note 読み出しだけを行い、スケジュール要素へ書き込まない。
 */
void switch1_csb_sched_read(switch1_csb_sched_t *out);

/**
 * @brief CSB受信callbackがdescriptorを解放する前に、17バイトの応答を取得する準備を行う
 * @param bd_addr 接続相手のBD_ADDR
 * @param bch34 ウェイクアップ用のBCHパリティ
 * @return 準備処理の結果を返す
 * @note 成功するとSwitchとESP32の相対クロック差と、次のACLで使用するLT_ADDRを取得できる。
 */
esp_err_t switch1_phase2_capture_start(const uint8_t bd_addr[6], uint64_t bch34);
bool switch1_phase2_capture_take(esp_err_t *out_result, uint8_t *out_lt_addr,
                                 uint32_t *out_clock_delta,
                                 uint32_t *out_rx_clk,
                                 uint16_t *out_rx_bitoff);
void switch1_phase2_capture_cancel(void);

/**
 * @brief 取得した本文と受信側control structureをUARTへ出力する
 * @note switch1_phase2_capture_takeから分離している。
 * 115200bpsの1行の出力に約10msかかる。
 * 取得からフェーズ3までの時間に遅延として加わるため、フェーズ3を送り終えたあとに呼び出す。
 */
void switch1_phase2_capture_report(void);

/**
 * @brief Switchの応答直後にフレームフォーマット3のACL受信イベントを登録する
 * @param bd_addr 接続相手のBD_ADDR
 * @param bch34 ウェイクアップ用のBCHパリティ
 * @param lt_addr ACLで使用するLT_ADDR
 * @param clock_delta SwitchとESP32の相対クロック差
 * @param rx_clk Switchの応答を受信したクロック
 * @param rx_bitoff Switchの応答を受信したスロット内位置をus単位で指定する
 * @param phase3_slot フェーズ3を送信したスロット
 * @return ACL受信イベントの登録結果を返す
 * @note 受信成功時のcallbackが、ページング接続と同じACLリンク処理を開始する。
 */
esp_err_t switch1_acl_handoff_start(const uint8_t bd_addr[6], uint64_t bch34,
                                    uint8_t lt_addr, uint32_t clock_delta,
                                    uint32_t rx_clk, uint16_t rx_bitoff,
                                    uint32_t phase3_slot);
bool switch1_acl_handoff_status(esp_err_t *out_result, uint8_t *out_link_id);

/**
 * @brief r_lc_startの呼び出し有無と、呼び出し後の状態を返す
 * @param[out] out_lc_state r_lc_start後のlc_state[link_id]
 * @note ESP32がBluetooth周辺側となるACLリンクでは、r_lc_start完了後にlc_state[link_id]がLC_STATE_PERIPHERAL_IDLE(5)になる
 * @param[out] out_flags 次の状態をbitで返す
 * - bit0: lc_env
 * - bit1: ld_acl_env
 * - bit2: llc_env
 * - bit3: hci_envへのBD_ADDR登録済み
 * @return 状態を取得できた場合はtrueを返す
 */
bool switch1_lc_start_status(uint8_t *out_lc_state, uint8_t *out_flags);

/**
 * @brief ウェイクアップで作ったACLリンクをコントローラー側から切断する
 * @return 切断要求の結果を返す
 * @note Switchの画面表示後に通常のページで再接続するために使用する。
 * link 0をACLリンクが使用している間はページがACL Connection Already Existsで拒否される。
 * 理由コードは0x13(Remote User Terminated)である。
 */
esp_err_t switch1_acl_detach(void);

/**
 * @brief ウェイクアップで作成したリンクの内部状態を返す
 * @return リンクがない場合は0xffを返す
 */
uint8_t switch1_acl_lc_state(void);

/**
 * @brief 切断要求を受理しない状態からlc_state[link_id]をLC_STATE_PERIPHERAL_IDLE(5)へ変更する
 * @return 遷移処理の結果を返す
 * @note LC_STATE_PERIPHERAL_IDLE(5)では切断要求を受理できる。
 */
esp_err_t switch1_acl_force_idle(void);

/**
 * @brief 切断完了を待たずにESP32側のリンクを解放する
 * @return 解放処理の結果を返す
 */
esp_err_t switch1_acl_release(void);

/**
 * @brief BluetoothコアのレジスタからネイティブクロックCLKNを読み出す
 * @param[out] out_clkn 312.5us単位の28ビット値を返す
 * @return 読み出しに成功した場合はtrue
 * サンプル完了を待てなかった場合はfalse。
 * @note HCI通信を必要としないため、各バーストの処理中に呼び出せる。
 */
bool switch1_csb_read_clkn(uint32_t *out_clkn);

/**
 * @brief 次のHCI Create Connectionを1回だけ許可し、Clock_Offsetを渡す
 * @param bd_addr 接続相手のBD_ADDR
 * @param offset 適用するClock_Offset
 * 0の場合は既定値を維持する。
 */
void switch1_hci_clock_offset_set(const uint8_t bd_addr[6], uint16_t offset);
void switch1_hci_page_cancel(void);

/**
 * @brief 状態の詳細出力を有効または無効にする
 * @param enabled trueで有効にする
 * @note 既定値は有効である。
 * 接続サイクル中は無効にする。詳細出力がリンク成立から再接続までの処理を遅延させるためである。
 */
void switch1_set_report_verbose(bool enabled);

#endif
