/*
 * 受信側のcontrol structureにはウェイクアップ用のパリティを設定する。
 * 受信成立の判定は受信descriptorを直接読み出して行う。
 * HCIイベントを待つと、応答受信からフェーズ3の送信までの時間に処理が間に合わない。
 */

#include "baseband.h"

#include "bb_internal.h"
#include "bb_rom.h"

#include <stdbool.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "common/bt_target.h"
#include "hci/hci_layer.h"
#include "stack/bt_types.h"
#include "stack/hcidefs.h"
#include "stack/hcimsgs.h"

static const char *TAG = "wake_csb";

 /*
  * CSB受信のcontrol structure。
  * 対応するROM関数はr_ld_csb_rx_start(0x4003972c)。
  * 送信側(0x3ffb22b6)と同じ配置である。
 * - +0x0A: BD_ADDR
 * - +0x10: BCH parity
 */
#define CSB_RX_CS_ADDR UINT32_C(0x3ffb231c)
/*
 * HCIのSet_Connectionless_Slave_Broadcast_Receive。
 * bluedroidのhcidefs.hには定義がないが、ESP32のControllerは実装している。
 * ROMのlb_hci_command_handler_tabにopcode 0x0442のハンドラが登録されている。
 */
#define HCI_SET_CLB_RECEIVE (0x0042 | HCI_GRP_LINK_CONTROL_CMDS)
/*
 * HCIのSet_Event_Mask_Page_2。
 * CSB受信イベント(0x51)は既定では無効になっている。
 * ROMのr_hci_evt_mask_set(0x400189e4)はページ2のマスクをhci_env+8へ8バイトコピーする。
 * イベントマスクページ2の既存ビット配置を変更しない。
 * bluedroidはこのコマンドを発行しないため、bit16~18をここで有効にする。
 * - bit16: 同期列の受信
 * - bit17: CSB受信
 * - bit18: CSB受信のタイムアウト
 */
#define HCI_SET_EVT_MASK_PAGE_2 (0x0063 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_EVT_MASK_PAGE_2_CSB UINT8_C(0x07)
#define CSB_RX_PARAM_SIZE 34

static uint8_t s_csb_rx_target[6];
static uint64_t s_csb_rx_bch;
static uint8_t s_csb_rx_channel;
static volatile bool s_csb_rx_active;
// 受信開始直後のcontrol structure。
// 応答の検出を遅らせないよう、ログ出力は後で行う。
#define CSB_RX_START_CS_WORDS 24
static uint16_t s_csb_rx_start_cs[CSB_RX_START_CS_WORDS];
static bool s_csb_rx_start_cs_valid;
static uint8_t s_csb_rx_ltaddr_written;
static uint16_t s_csb_rx_ltaddr_readback;
static bool s_csb_rx_ltaddr_valid;

/*
 * 受信descriptorのステータスだけでは連続受信の件数を判定できない。
 * HCIイベント0x51は受信1件ごとに発生するため、件数はここで数える。
 */
static volatile uint32_t s_csb_rx_event_count;

static volatile bool s_csb_rx_packet_pending;
static uint8_t s_csb_rx_addr[6];
static uint32_t s_csb_rx_clock;
static uint8_t s_csb_rx_status;
static uint8_t s_csb_rx_length;
static uint8_t s_csb_rx_data[32];


static void switch1_csb_rx_reapply(void);

static void csb_rx_write_bdaddr(const uint8_t address[6])
{
    volatile uint16_t *const base =
        (volatile uint16_t *)(CSB_RX_CS_ADDR + CS_BDADDR_OFFSET);
    base[0] = (uint16_t)(address[5] | ((uint16_t)address[4] << 8));
    base[1] = (uint16_t)(address[3] | ((uint16_t)address[2] << 8));
    base[2] = (uint16_t)(address[1] | ((uint16_t)address[0] << 8));
}

static void csb_rx_write_bch(uint64_t bch34)
{
    volatile uint16_t *const base =
        (volatile uint16_t *)(CSB_RX_CS_ADDR + CS_BCH_OFFSET);
    base[0] = (uint16_t)(bch34 & 0xFFFF);
    base[1] = (uint16_t)((bch34 >> 16) & 0xFFFF);
    /*
     * 3番目のワードは送信と受信で書き込み形式が異なる。
     * - 送信(r_ld_csb_tx_en): parityの上位2ビットだけを書き込む。
     * - 受信(r_ld_csb_rx_start): 0xFF40も同時に書き込む。
     * 上位10ビットの用途は特定できていないが、ROMが受信時だけ書き込む上位10ビットを保持する。
     * 送信側と同じ形式で書く実装にはしない。
     */
    base[2] = (uint16_t)(CS_BCH_RX_FLAGS | ((bch34 >> 32) & 0x3));
}

/*
 * HCIパラメータのフィールド:
 * - Enable(1)
 * - BD_ADDR(6)
 * - LT_ADDR(1)
 * - Interval(2)
 * - Clock_Offset(4)
 * - Next_CSB_Clock(4)
 * - Supervision_Timeout(2)
 * - Remote_Timing_Accuracy(1)
 * - Skip(1)
 * - Packet_Type(2)
 * - AFH_Channel_Map(10)
 */
static void csb_rx_build_params(uint8_t params[CSB_RX_PARAM_SIZE],
                                uint8_t enable, const uint8_t bd_addr[6],
                                uint16_t interval, uint32_t clock_offset,
                                uint32_t next_clock)
{
    uint8_t *p = params;
    UINT8_TO_STREAM(p, enable);
    for (int i = 0; i < 6; ++i) {
        UINT8_TO_STREAM(p, bd_addr[5 - i]);
    }
    UINT8_TO_STREAM(p, CSB_LT_ADDR);
    UINT16_TO_STREAM(p, interval);
    UINT32_TO_STREAM(p, clock_offset & UINT32_C(0x0fffffff));
    UINT32_TO_STREAM(p, next_clock & UINT32_C(0x0fffffff));
    UINT16_TO_STREAM(p, CSB_RX_SUPERVISION_TIMEOUT);
    UINT8_TO_STREAM(p, 250);  // Remote_Timing_Accuracy(ppm)
    UINT8_TO_STREAM(p, 0);    // Skip
    /*
     * LLID 3用の固定バッファは20バイトであるため、17バイトの応答を収められるDM1だけを許可する。
     * より長いpacket typeを許可すると隣接する固定バッファを上書きする。
     */
    UINT16_TO_STREAM(p, CSB_EDR_EXCLUSION_MASK | HCI_PKT_TYPES_MASK_DM1);
    for (int i = 0; i < 9; ++i) {
        UINT8_TO_STREAM(p, 0xFF);
    }
    UINT8_TO_STREAM(p, 0x7F);
}

esp_err_t switch1_csb_rx_start(const uint8_t bd_addr[6], uint64_t bch34,
                               uint16_t interval, uint32_t clock_offset,
                               uint32_t next_clock, uint8_t channel)
{
    /*
     * このAPIで指定できる周期は偶数に限られる。
     * この経路では1つの受信位置に6通りの位相を割り当てない。
     */
    if (bd_addr == NULL || interval < 2 || (interval & 1) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_csb_rx_active) {
        /*
         * 前回のEnable=0停止要求が拒否されると受信状態が継続し、その後の開始が再起動まで失敗し続ける。
         * 停止を再試行し、停止が成立しなくてもEnable=1で開始時刻を再設定する。
         */
        const esp_err_t stop_err =
            switch1_csb_rx_stop(s_csb_rx_target, interval);
        ESP_LOGW(TAG, "CSB receive remained active before stop; stop result=%s",
                 esp_err_to_name(stop_err));
    }
    /*
     * 受信イベントを有効にしてから受信を開始する。
     * 受信イベントが無効な状態ではControllerがイベントを通知しない。
     * descriptorの直接読み出し以外では受信を確認できない。
     */
    {
        uint8_t mask[8] = {0};
        mask[2] = HCI_EVT_MASK_PAGE_2_CSB;
        const esp_err_t mask_err =
            send_command_wait(HCI_SET_EVT_MASK_PAGE_2, mask, sizeof(mask));
        ESP_LOGI(TAG, "Page 2 event mask result=%s",
                 esp_err_to_name(mask_err));
    }
    uint8_t params[CSB_RX_PARAM_SIZE];
    csb_rx_build_params(params, 1, bd_addr, interval, clock_offset, next_clock);
    const esp_err_t err =
        send_command_wait(HCI_SET_CLB_RECEIVE, params, CSB_RX_PARAM_SIZE);
    if (err != ESP_OK) {
        return err;
    }
    s_csb_rx_bch = bch34;
    s_csb_rx_channel = channel;
    memcpy(s_csb_rx_target, bd_addr, sizeof(s_csb_rx_target));
    s_csb_rx_active = true;
    switch1_csb_rx_reapply();
    /*
     * ここではログを出力しない。
     * switch1_csb_rx_startから戻った直後にSwitchの応答を受信する可能性がある。
     * UART 4行分(約20ms)の出力で検出が遅れると、フェーズ3の送信タイミングを逃すため、control structureの24ワードだけ保存する。
     */
    {
        const volatile uint16_t *const cs =
            (const volatile uint16_t *)CSB_RX_CS_ADDR;
        for (int i = 0; i < CSB_RX_START_CS_WORDS; ++i) {
            s_csb_rx_start_cs[i] = cs[i];
        }
        s_csb_rx_start_cs_valid = true;
    }
    return ESP_OK;
}

/*
 * 受信側control structureへ設定を再適用する。
 * ROMがイベントごとに設定を再構成するため、送信側と同様に繰り返し呼び出す必要がある。
 * 特に+0x16ではROMがbit15(ホッピング有効)を1に設定するため、再適用しないと受信周波数が79チャンネルを巡回する。
 */
static void switch1_csb_rx_reapply(void)
{
    if (!s_csb_rx_active) {
        return;
    }
    /* ROMはBD_ADDRから通常のparityを計算して書き込むため、ウェイクアップ用BCH parityへ置き換える。 */
    csb_rx_write_bdaddr(s_csb_rx_target);
    csb_rx_write_bch(s_csb_rx_bch);
    /*
     * Switchの応答ヘッダーのLT_ADDRは0だが、HCIコマンドは1~7だけを受け付ける。
     * control structureへ0を直接設定する。
     * LT_ADDRは+0x08のbit10:8にある。
     * +0x00の下位5ビットはフレームフォーマットなので変更しない。
     *
     * control structure +0x08のbit12を立てて白色化を無効にする。
     * Switchの応答は白色化なしで送信されるため、初期値0を設定する。
     * 受信側は既定でピコネットクロックから白色化初期値を生成する。
     * 変更しないとヘッダーの復号結果が受信ごとに変化する。
     */
    {
        volatile uint16_t *const link =
            (volatile uint16_t *)(CSB_RX_CS_ADDR + CS_LTADDR_OFFSET);
        *link = (uint16_t)((*link & ~CS_LTADDR_MASK) |
                           CS_LINKCNTL_WHITENING_DSB);
    }
    /*
     * 送信側と同じく周波数ホッピングを無効にし、Switchが送信する2402MHzへ固定する。
     * 設定しない場合、受信周波数だけがホッピングする。
     */
    {
        volatile uint16_t *const hopcntl =
            (volatile uint16_t *)(CSB_RX_CS_ADDR + CS_HOPCNTL_OFFSET);
        uint16_t value = *hopcntl;
        value &= (uint16_t)~CS_HOPCNTL_FH_EN;
        value = (uint16_t)((value & (uint16_t)~CS_HOPCNTL_CHANNEL_MASK) |
                           (uint16_t)(hop_field(s_csb_rx_channel) << 8));
        *hopcntl = value;
    }
}

/*
 * control structureには、ハードウェアが測定した受信時刻が格納される。
 * - +0x52, +0x54: クロック
 * - +0x56: スロット内位置(10ビット)
 * ld_acl_rx_sync(0x4002fbec)は+0x56から68usを減算して0~624へ正規化する。
 */
#define CS_RXCLK_LO_OFFSET UINT32_C(0x52)
#define CS_RXCLK_HI_OFFSET UINT32_C(0x54)
#define CS_RXBITOFF_OFFSET UINT32_C(0x56)
// ld_acl_rx_sync(0x4002fc8c)が受信スロット位置から68usを減算する補正。
#define CS_RXBITOFF_CORRECTION 68

bool switch1_csb_rx_read_sync(uint32_t *out_clk, uint16_t *out_bitoff,
                              uint16_t *out_raw)
{
    const volatile uint16_t *const lo =
        (const volatile uint16_t *)(CSB_RX_CS_ADDR + CS_RXCLK_LO_OFFSET);
    const volatile uint16_t *const hi =
        (const volatile uint16_t *)(CSB_RX_CS_ADDR + CS_RXCLK_HI_OFFSET);
    const volatile uint16_t *const off =
        (const volatile uint16_t *)(CSB_RX_CS_ADDR + CS_RXBITOFF_OFFSET);
    const uint16_t high = *hi;
    if (high >= 0x800) {
        return false;
    }
    uint32_t clk = ((uint32_t)high << 16) | *lo;
    const uint16_t raw = (uint16_t)(*off & 0x3FF);
    if (out_raw != NULL) {
        *out_raw = raw;
    }
    int32_t bitoff = (int32_t)raw - CS_RXBITOFF_CORRECTION;
    while (bitoff < 0) {
        bitoff += 625;
        clk = (clk - 1) & UINT32_C(0x07ffffff);
    }
    while (bitoff >= 625) {
        bitoff -= 625;
        clk = (clk + 1) & UINT32_C(0x07ffffff);
    }
    if (out_clk != NULL) {
        *out_clk = clk;
    }
    if (out_bitoff != NULL) {
        *out_bitoff = (uint16_t)bitoff;
    }
    return true;
}

void switch1_csb_rx_set_format(uint8_t format)
{
/*
 * control structure +0x00のbit0-4はフレームフォーマットである。
 * - 2, 3: ACL
 * - 4: ページ
 * - 5: ページスキャン
 * - 24: CSB送信と同期トレイン
 * - 25: CSB受信と同期スキャン
 *
 * CSB受信ではハードウェアが受信ペイロードを保存しない。
 * ACLの形式へ変更すると、ペイロードを保存する処理経路に入る。
 */
    volatile uint16_t *const cs = (volatile uint16_t *)CSB_RX_CS_ADDR;
    *cs = (uint16_t)((*cs & ~UINT16_C(0x001F)) | (format & 0x1F));
}

/*
 * 停止処理の完了を待つ上限時間。
 * 動作中のCSB受信env構造体は、スケジュール要素の処理が終わるまで解放されない。
 * 受信周期は22スロット(13.75ms)であるため、1周期を超える時間を設定する。
 */
#define CSB_RX_STOP_WAIT_US 20000

esp_err_t switch1_csb_rx_restart(uint16_t interval, uint32_t next_clock)
{
    if (!s_csb_rx_active) {
        return ESP_ERR_INVALID_STATE;
    }
    /*
     * 先に停止を送信してld_csb_rx_env(0x3ffb8278)を解放する。
     * このポインタがNULLでない場合、r_ld_csb_rx_startは12(Command Disallowed)を返す。
     * 停止が12(Command Disallowed)で拒否されるのはenvポインタがすでにNULLの場合であるため、その停止応答を無視して開始処理へ進む。
     */
    uint8_t params[CSB_RX_PARAM_SIZE];
    csb_rx_build_params(params, 0, s_csb_rx_target, interval < 2 ? 2 : interval,
                        0, 0);
    g_suppress_command_error_log = true;
    (void)send_command_wait(HCI_SET_CLB_RECEIVE, params, CSB_RX_PARAM_SIZE);
    g_suppress_command_error_log = false;
    /*
     * 停止処理は直ちに完了するとは限らない。
     * r_ld_csb_rx_stop(0x40039bb8)はenv+0x3Eが1(動作中)の場合、0x40039bf2で2を書き込み、処理を終了する。
     * envの解放はスケジュール要素の処理後に行われる。
     * 解放前に開始を送信すると0x40039738の判定で12(Command Disallowed)になるため、ポインタがNULLになるまで待つ。
     * 上限時間内にNULLにならない場合は、この回の再開を中止する。
     */
    {
        const void *volatile *const env =
            (const void *volatile *)CSB_RX_ENV_ADDR;
        int waited_us = 0;
        while (*env != NULL && waited_us < CSB_RX_STOP_WAIT_US) {
            esp_rom_delay_us(100);
            waited_us += 100;
        }
        if (*env != NULL) {
            return ESP_ERR_TIMEOUT;
        }
    }
    csb_rx_build_params(params, 1, s_csb_rx_target, interval, 0, next_clock);
    const esp_err_t err =
        send_command_wait(HCI_SET_CLB_RECEIVE, params, CSB_RX_PARAM_SIZE);
    if (err != ESP_OK) {
        /*
         * 停止は成立しているため、受信は動作していない。
         * 次の開始処理を誤って拒否しないよう、s_csb_rx_activeをfalseに設定する。
         */
        s_csb_rx_active = false;
        return err;
    }
    switch1_csb_rx_reapply();
    return ESP_OK;
}

esp_err_t switch1_csb_rx_stop(const uint8_t bd_addr[6], uint16_t interval)
{
    if (bd_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t params[CSB_RX_PARAM_SIZE];
    csb_rx_build_params(params, 0, bd_addr, interval < 2 ? 2 : interval, 0, 0);
    const esp_err_t err =
        send_command_wait(HCI_SET_CLB_RECEIVE, params, CSB_RX_PARAM_SIZE);
    if (err == ESP_OK) {
        s_csb_rx_active = false;
    }
    return err;
}

/*
 * 受信descriptorの配置。
 * ld_csb_rx_frm_cbk(0x400393a4)は、ld_env[0x1AA]の下位2ビットを使用して
 * 4個の受信descriptorから1個を選択する。
 * 先頭ワードのbit15で受信完了を判定する。
 * 先頭ワードの下位15ビットは次のdescriptorの共有メモリ位置である。
 * 4個のdescriptorを循環して使用する。
 */
#define CSB_RX_DESC_ADDR UINT32_C(0x3ffb2382)
#define CSB_RX_DESC_WORDS 7
#define CSB_RX_DESC_COUNT 4
#define CSB_RX_DESC_DONE UINT16_C(0x8000)
// 同期に失敗した場合、2番目のワードの最下位ビットが1に設定される
#define CSB_RX_STAT_SYNC_ERR UINT16_C(0x0001)
/*
 * ld_csb_rx_frm_cbk(0x400393a4)の0x400394fcが2番目のワードに適用するパケット有効条件。
 * - bit0, bit1, bit2, bit6: いずれかが1の場合はペイロードを保存しない。
 * - bit0: 同期失敗
 * - bit6: LT_ADDR不一致
 */
#define CSB_RX_STAT_FAIL_MASK UINT16_C(0x0047)
/*
 * - 4番目のワード: ペイロードヘッダー。
 *   長さはdescriptor[CSB_RX_DESC_PLHEADER]のビット3~15から得る。
 * - 6番目のワード: ペイロードの共有メモリ位置。
 *   ld_csb_rx_frm_cbkの0x40039509~0x40039527が読み出すフィールドである。
 */
#define CSB_RX_DESC_PLHEADER 3
#define CSB_RX_DESC_PLBUF 5
/*
 * - 7番目のワードは別のバッファで、20バイト間隔に配置される。
 * - 6番目のワードが指すバッファは1028バイト間隔で配置され、ACLデータ用である。
 * - LLIDが3のLMPパケットは、ハードウェアが7番目のバッファへ書き込むと考えられる。
 * - ld_csb_rx_frm_cbkは6番目のワードだけを読むため、ROMから渡されるデータは0になる。
 */
#define CSB_RX_DESC_LMBUF 6
#define CSB_RX_HIT_MAX 4
#define CSB_RX_HIT_BYTES 32
/*
 * ヘッダーの復元が終わった時点でステータスが書き込まれるため、直後に読み出すとペイロードの受信が完了していない。
 * DM1の17バイトは2/3誤り訂正符号を含めて約300usを要するため、遅延後に再度読み出す。
 */
#define CSB_RX_LATE_US 700

struct csb_rx_hit {
    uint16_t desc[CSB_RX_DESC_WORDS];
    uint16_t length;
    uint8_t early[CSB_RX_HIT_BYTES];
    uint8_t late[CSB_RX_HIT_BYTES];
    uint8_t lm_early[CSB_RX_HIT_BYTES];
    uint8_t lm_late[CSB_RX_HIT_BYTES];
};

static void csb_rx_copy_payload(uint8_t *dst, uint16_t em_offset,
                                uint16_t length)
{
    memset(dst, 0, CSB_RX_HIT_BYTES);
    if (em_offset == 0 || (uint32_t)em_offset + length > EM_SIZE) {
        return;
    }
    const volatile uint8_t *const src =
        (const volatile uint8_t *)(EM_BASE + em_offset);
    const uint16_t copy = length > CSB_RX_HIT_BYTES ? CSB_RX_HIT_BYTES : length;
    for (uint16_t i = 0; i < copy; ++i) {
        dst[i] = src[i];
    }
}

static void csb_rx_log_hit(const struct csb_rx_hit *hit, unsigned index)
{
    ESP_LOGI(TAG,
             "  Received descriptor %u: %04x %04x %04x %04x %04x %04x %04x",
             index, hit->desc[0], hit->desc[1], hit->desc[2], hit->desc[3],
             hit->desc[4], hit->desc[5], hit->desc[6]);
    ESP_LOGI(TAG,
             "    header=%03x payload_header=%02x length=%u "
             "LLID=%u payload_shared_memory_offset=%04x",
             hit->desc[2] & 0x3FF, hit->desc[CSB_RX_DESC_PLHEADER] & 0xFF,
             hit->length, hit->desc[CSB_RX_DESC_PLHEADER] & 0x3,
             hit->desc[CSB_RX_DESC_PLBUF]);
    const uint16_t show = hit->length > CSB_RX_HIT_BYTES ? CSB_RX_HIT_BYTES
                                                         : hit->length;
    static const char *const labels[4] = {
        "ACL buffer (immediate)", "ACL buffer (delayed)",
        "LMP buffer (immediate)", "LMP buffer (delayed)"};
    const uint8_t *const sources[4] = {hit->early, hit->late, hit->lm_early,
                                       hit->lm_late};
    char text[CSB_RX_HIT_BYTES * 3 + 1];
    for (int s = 0; s < 4; ++s) {
        int used = 0;
        for (uint16_t i = 0; i < show; ++i) {
            used += snprintf(text + used, sizeof(text) - used, "%02x ",
                             sources[s][i]);
        }
        ESP_LOGI(TAG, "    %s: %s", labels[s], text);
    }
}

/*
 * ステータスワードだけを比較すると、同じステータスワードを持つ連続受信で2件目以降を検出できない。
 * descriptor[CSB_RX_DESC_LEVEL]は受信ごとに変化するため、ステータスワードと組み合わせて変化を判定する。
 */
static uint16_t s_csb_rx_poll_last[CSB_RX_DESC_COUNT];
static uint16_t s_csb_rx_poll_last_level[CSB_RX_DESC_COUNT];
#define CSB_RX_DESC_LEVEL 4

/*
 * 監視中に現れたステータスの種類と回数を収集する。
 * CSB_RX_STAT_SYNC_ERRが1の受信とCSB_RX_STAT_FAIL_MASKに該当する受信を、受信0件の場合に切り分けるために使用する。
 */
#define CSB_RX_POLL_KINDS 8
static uint16_t s_csb_rx_poll_seen[CSB_RX_POLL_KINDS];
static uint16_t s_csb_rx_poll_seen_count[CSB_RX_POLL_KINDS];
static uint8_t s_csb_rx_poll_kinds;

void switch1_csb_rx_poll_prime(void)
{
    const volatile uint16_t *const base =
        (const volatile uint16_t *)CSB_RX_DESC_ADDR;
    for (int d = 0; d < CSB_RX_DESC_COUNT; ++d) {
        s_csb_rx_poll_last[d] = base[d * CSB_RX_DESC_WORDS + 1];
        s_csb_rx_poll_last_level[d] =
            base[d * CSB_RX_DESC_WORDS + CSB_RX_DESC_LEVEL];
    }
    s_csb_rx_poll_kinds = 0;
}

static void csb_rx_poll_count(uint16_t stat)
{
    uint8_t i = 0;
    while (i < s_csb_rx_poll_kinds && s_csb_rx_poll_seen[i] != stat) {
        ++i;
    }
    if (i < s_csb_rx_poll_kinds) {
        if (s_csb_rx_poll_seen_count[i] < 0xFFFF) {
            s_csb_rx_poll_seen_count[i]++;
        }
        return;
    }
    if (s_csb_rx_poll_kinds >= CSB_RX_POLL_KINDS) {
        return;
    }
    s_csb_rx_poll_seen[s_csb_rx_poll_kinds] = stat;
    s_csb_rx_poll_seen_count[s_csb_rx_poll_kinds] = 1;
    s_csb_rx_poll_kinds++;
}

bool switch1_csb_rx_is_active(void)
{
    return s_csb_rx_active;
}

void switch1_csb_rx_poll_report(void)
{
    if (s_csb_rx_ltaddr_valid) {
        s_csb_rx_ltaddr_valid = false;
        ESP_LOGI(TAG, "CSB receive LT_ADDR set to %u (+0x08=%04x)",
                 s_csb_rx_ltaddr_written, (unsigned)s_csb_rx_ltaddr_readback);
    }
    if (s_csb_rx_start_cs_valid) {
        s_csb_rx_start_cs_valid = false;
        char text[8 * 5 + 1];
        for (int base = 0; base < CSB_RX_START_CS_WORDS; base += 8) {
            int used = 0;
            for (int i = 0; i < 8; ++i) {
                used += snprintf(text + used, sizeof(text) - used, "%04x ",
                                 s_csb_rx_start_cs[base + i]);
            }
            ESP_LOGI(TAG, "  Initial receive control structure +%02x: %s", base * 2,
                     text);
        }
    }
    for (uint8_t i = 0; i < s_csb_rx_poll_kinds; ++i) {
        ESP_LOGI(TAG, "  Status %04x detected %u times (sync=%s)",
                 s_csb_rx_poll_seen[i], s_csb_rx_poll_seen_count[i],
                 (s_csb_rx_poll_seen[i] & CSB_RX_STAT_SYNC_ERR) ? "failed"
                                                                : "succeeded");
    }
    if (s_csb_rx_poll_kinds == 0) {
        ESP_LOGI(TAG, "  No status changes");
    }
}

bool switch1_csb_rx_poll(uint16_t *out_status, uint16_t *out_header,
                         uint16_t *out_pheader)
{
    const volatile uint16_t *const base =
        (const volatile uint16_t *)CSB_RX_DESC_ADDR;
    for (int d = 0; d < CSB_RX_DESC_COUNT; ++d) {
        const uint16_t stat = base[d * CSB_RX_DESC_WORDS + 1];
        const uint16_t level = base[d * CSB_RX_DESC_WORDS + CSB_RX_DESC_LEVEL];
        if (stat == s_csb_rx_poll_last[d] &&
            level == s_csb_rx_poll_last_level[d]) {
            continue;
        }
        s_csb_rx_poll_last[d] = stat;
        s_csb_rx_poll_last_level[d] = level;
        csb_rx_poll_count(stat);
        if (stat == 0 || (stat & CSB_RX_STAT_FAIL_MASK) != 0) {
            continue;
        }
        if (out_status != NULL) {
            *out_status = stat;
        }
        if (out_header != NULL) {
            *out_header = base[d * CSB_RX_DESC_WORDS + 2];
        }
        if (out_pheader != NULL) {
            *out_pheader = base[d * CSB_RX_DESC_WORDS + CSB_RX_DESC_PLHEADER];
        }
        return true;
    }
    return false;
}

uint32_t switch1_csb_rx_watch(uint32_t duration_ms)
{
    const volatile uint16_t *const base =
        (const volatile uint16_t *)CSB_RX_DESC_ADDR;
    bool was_done[CSB_RX_DESC_COUNT];
    for (int d = 0; d < CSB_RX_DESC_COUNT; ++d) {
        was_done[d] = (base[d * CSB_RX_DESC_WORDS] & CSB_RX_DESC_DONE) != 0;
    }
    /*
     * ステータスを変更せず記録する。
     * 完了ビットが1になるまで待ってから読み出すと、ROMの割り込み処理が先にステータスワードを処理するため取得できない。
     */
    uint16_t seen[8];
    uint16_t seen_count[8];
    uint8_t kinds = 0;
    uint32_t windows = 0;
    /*
     * statが0ではなくCSB_RX_STAT_FAIL_MASKに該当しない受信descriptorを保存する。
     * descriptorごとの変化を検出するため、同じステータスワードの受信が続いても2件目以降を取得できる。
     * 監視開始時の各descriptor statusを0で初期化し、監視開始前に成立したdescriptorも最初の変化として取得する。
     * ステータスが0であれば未使用descriptorの初期状態であるため対象から除外する。
     */
    uint16_t last_stat[CSB_RX_DESC_COUNT];
    for (int d = 0; d < CSB_RX_DESC_COUNT; ++d) {
        last_stat[d] = 0;
    }
    /*
     * 1件144バイトである。
     * UARTタスクのスタックは3072バイトしかないため、4件分をスタック領域に確保するとオーバーフローして再起動する。
     * 配列を静的領域へ配置する。
     * switch1_csb_rx_watchはUARTタスクからだけ呼び出されるため、共有しても競合しない。
     */
    static struct csb_rx_hit hit[CSB_RX_HIT_MAX];
    unsigned hits = 0;
    const int64_t until = esp_timer_get_time() + (int64_t)duration_ms * 1000;
    /*
     * ROMによる設定変更を検知するため、CSB_RX_CS_ADDR+CS_HOPCNTL_OFFSETのワードを再設定直前に収集する。
     */
    uint16_t hop_seen[4];
    uint16_t hop_count[4];
    uint8_t hop_kinds = 0;
    const volatile uint16_t *const hopcntl =
        (const volatile uint16_t *)(CSB_RX_CS_ADDR + CS_HOPCNTL_OFFSET);
    while (esp_timer_get_time() < until) {
        {
            const uint16_t hop = *hopcntl;
            uint8_t h = 0;
            while (h < hop_kinds && hop_seen[h] != hop) {
                ++h;
            }
            if (h < hop_kinds) {
                if (hop_count[h] < 0xFFFF) {
                    hop_count[h]++;
                }
            } else if (hop_kinds < 4) {
                hop_seen[hop_kinds] = hop;
                hop_count[hop_kinds] = 1;
                hop_kinds++;
            }
        }
        /*
         * 受信側ではROMが設定を再構成しないため、ここでは変更しない。
         */
        for (int d = 0; d < CSB_RX_DESC_COUNT; ++d) {
            const uint16_t head = base[d * CSB_RX_DESC_WORDS];
            const uint16_t stat = base[d * CSB_RX_DESC_WORDS + 1];
            const bool done = (head & CSB_RX_DESC_DONE) != 0;
            if (done != was_done[d]) {
                was_done[d] = done;
                if (done) {
                    windows++;
                }
            }
            if (stat != last_stat[d]) {
                last_stat[d] = stat;
                if (stat != 0 && (stat & CSB_RX_STAT_FAIL_MASK) == 0 &&
                    hits < CSB_RX_HIT_MAX) {
                    struct csb_rx_hit *const h = &hit[hits];
                    for (int w = 0; w < CSB_RX_DESC_WORDS; ++w) {
                        h->desc[w] = base[d * CSB_RX_DESC_WORDS + w];
                    }
                    h->length = (uint16_t)(
                        (h->desc[CSB_RX_DESC_PLHEADER] >> 3) & 0x1FFF);
                    csb_rx_copy_payload(h->early, h->desc[CSB_RX_DESC_PLBUF],
                                        h->length);
                    csb_rx_copy_payload(h->lm_early, h->desc[CSB_RX_DESC_LMBUF],
                                        h->length);
                    const int64_t wait_until =
                        esp_timer_get_time() + CSB_RX_LATE_US;
                    while (esp_timer_get_time() < wait_until) {
                    }
                    csb_rx_copy_payload(h->late, h->desc[CSB_RX_DESC_PLBUF],
                                        h->length);
                    csb_rx_copy_payload(h->lm_late, h->desc[CSB_RX_DESC_LMBUF],
                                        h->length);
                    hits++;
                }
            }
            uint8_t i = 0;
            while (i < kinds && seen[i] != stat) {
                ++i;
            }
            if (i < kinds) {
                if (seen_count[i] < 0xFFFF) {
                    seen_count[i]++;
                }
                continue;
            }
            if (kinds >= 8) {
                continue;
            }
            seen[kinds] = stat;
            seen_count[kinds] = 1;
            kinds++;
        }
    }
    for (unsigned i = 0; i < hits; ++i) {
        csb_rx_log_hit(&hit[i], i);
    }
    for (int d = 0; d < CSB_RX_DESC_COUNT; ++d) {
        ESP_LOGI(TAG,
                 "  Descriptor %d at end of watch: %04x %04x %04x %04x %04x "
                 "%04x %04x", d, base[d * CSB_RX_DESC_WORDS],
                 base[d * CSB_RX_DESC_WORDS + 1], base[d * CSB_RX_DESC_WORDS + 2],
                 base[d * CSB_RX_DESC_WORDS + 3], base[d * CSB_RX_DESC_WORDS + 4],
                 base[d * CSB_RX_DESC_WORDS + 5], base[d * CSB_RX_DESC_WORDS + 6]);
    }
    ESP_LOGI(TAG, "  Saved packets=%u", hits);
    uint32_t synced = 0;
    for (uint8_t i = 0; i < kinds; ++i) {
        if (seen[i] != 0 && (seen[i] & CSB_RX_STAT_SYNC_ERR) == 0) {
            synced += seen_count[i];
        }
        ESP_LOGI(TAG, "  Status %04x detected %u times (sync=%s)", seen[i],
                 seen_count[i],
                 (seen[i] & CSB_RX_STAT_SYNC_ERR) ? "failed" : "succeeded");
    }
    for (uint8_t h = 0; h < hop_kinds; ++h) {
        ESP_LOGI(TAG, "  Hop control before reconfiguration %04x detected %u times",
                 hop_seen[h], hop_count[h]);
    }
    ESP_LOGI(TAG, "  Receive-completion notifications=%lu, status types=%u",
             (unsigned long)windows, kinds);
    return synced;
}

void switch1_csb_rx_write_ltaddr(uint8_t lt_addr)
{
    volatile uint16_t *const link =
        (volatile uint16_t *)(CSB_RX_CS_ADDR + CS_LTADDR_OFFSET);
    *link = (uint16_t)((*link & ~CS_LTADDR_MASK) |
                       (uint16_t)(((uint16_t)lt_addr << 8) & CS_LTADDR_MASK));
    s_csb_rx_ltaddr_written = lt_addr;
    s_csb_rx_ltaddr_readback = *link;
    s_csb_rx_ltaddr_valid = true;
}

esp_err_t switch1_csb_rx_dump(uint16_t offset, uint8_t count, uint16_t *out)
{
    if (out == NULL || count == 0 || count > 32) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint32_t base = CSB_RX_CS_ADDR + offset;
    if (base < EM_BASE || base + (uint32_t)count * 2 > EM_BASE + EM_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    const volatile uint16_t *const words = (const volatile uint16_t *)base;
    for (uint8_t i = 0; i < count; ++i) {
        out[i] = words[i];
    }
    return ESP_OK;
}


/*
 * CSB受信のHCIイベント(0x51)を保存する。
 * bluedroidはこのイベントを処理しないため、bb_hci.cのイベントハンドラーが受信し、bb_csb_rx_on_eventへ渡す。
 *
 * イベントのフィールド:
 * - BD_ADDR(6)
 * - LT_ADDR(1)
 * - CLK(4)
 * - Offset(4)
 * - Receive_Status(1)
 * - Fragment(1)
 * - Data_Length(1)
 * - Data(可変長)
 */
void bb_csb_rx_on_event(BT_HDR *p_msg)
{
    if (p_msg != NULL && p_msg->len >= 2) {
        const uint8_t *const p = p_msg->data + p_msg->offset;
        if (p[0] == HCI_CSB_RECEIVE_EVT && p_msg->len >= 20) {
            for (int i = 0; i < 6; ++i) {
                s_csb_rx_addr[i] = p[2 + 5 - i];
            }
            s_csb_rx_clock = (uint32_t)p[9] | ((uint32_t)p[10] << 8) |
                             ((uint32_t)p[11] << 16) | ((uint32_t)p[12] << 24);
            s_csb_rx_status = p[17];
            const uint8_t length = p[19];
            s_csb_rx_length = length > 32 ? 32 : length;
            const uint16_t available = (uint16_t)(p_msg->len - 20);
            const uint8_t copy = s_csb_rx_length > available
                                     ? (uint8_t)available
                                     : s_csb_rx_length;
            memcpy(s_csb_rx_data, p + 20, copy);
            s_csb_rx_length = copy;
            s_csb_rx_packet_pending = true;
            // 同期に失敗した受信もイベントとして通知される
            if (s_csb_rx_status == 0) {
                s_csb_rx_event_count++;
            }
        }
    }
}

uint32_t switch1_csb_rx_event_count(void)
{
    return s_csb_rx_event_count;
}

void switch1_csb_rx_event_reset(void)
{
    s_csb_rx_event_count = 0;
}


bool switch1_csb_rx_take(uint8_t out_addr[6], uint32_t *out_clock,
                         uint8_t *out_status, uint8_t *out_data,
                         uint8_t *out_length)
{
    if (!s_csb_rx_packet_pending) {
        return false;
    }
    if (out_addr != NULL) {
        memcpy(out_addr, s_csb_rx_addr, sizeof(s_csb_rx_addr));
    }
    if (out_clock != NULL) {
        *out_clock = s_csb_rx_clock;
    }
    if (out_status != NULL) {
        *out_status = s_csb_rx_status;
    }
    if (out_length != NULL) {
        const uint8_t copy = s_csb_rx_length > 32 ? 32 : s_csb_rx_length;
        if (out_data != NULL) {
            memcpy(out_data, s_csb_rx_data, copy);
        }
        *out_length = copy;
    }
    s_csb_rx_packet_pending = false;
    return true;
}
