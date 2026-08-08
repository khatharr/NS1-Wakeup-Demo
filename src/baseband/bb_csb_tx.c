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


static bool s_lt_reserved;
static volatile uint32_t s_csb_events;
static uint8_t s_channel_index;
static uint16_t s_csb_packet_type;
static uint16_t s_csb_interval;
static uint16_t s_csb_header;
static uint16_t s_csb_pheader;
static uint16_t s_csb_data_length;
static uint8_t s_csb_staging[SWITCH1_CSB_DATA_MAX];
static uint8_t s_target_addr[6];
static uint64_t s_target_bch;
static bool s_csb_raw_mode;
static volatile bool s_cs_override_active;
static uint16_t s_cs_override_offset;
static uint16_t s_cs_override_mask;
static uint16_t s_cs_override_value;
static uint16_t s_cs_override_base;
static uint8_t s_csb_raw_channel;
static uint32_t s_csb_native_clock;

static volatile bool s_csb_running;

static esp_err_t write_clb_fragment(uint8_t fragment, const uint8_t *data,
                                    uint8_t length)
{
    // 長さフィールドは1バイトなので、3+lengthが255を超えると桁あふれが発生し、コマンド長が不正になる。
    // 呼び出し側から不正な長さを受けた場合はここで拒否する。
    if (length > CSB_FRAGMENT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t buf[3 + CSB_FRAGMENT_MAX];
    uint8_t *p = buf;
    UINT8_TO_STREAM(p, CSB_LT_ADDR);
    UINT8_TO_STREAM(p, fragment);
    UINT8_TO_STREAM(p, length);
    memcpy(p, data, length);
    return send_command_wait(HCI_WRITE_CLB_DATA, buf, (uint8_t)(3 + length));
}

static esp_err_t write_clb_data_raw(const uint8_t *data, uint16_t length)
{
    if (length <= CSB_FRAGMENT_MAX) {
        return write_clb_fragment(HCI_CLB_FRAGMENT_SINGLE, data,
                                  (uint8_t)length);
    }
    const esp_err_t err = write_clb_fragment(HCI_CLB_FRAGMENT_START, data,
                                             (uint8_t)CSB_FRAGMENT_MAX);
    if (err != ESP_OK) {
        return err;
    }
    const uint16_t remaining = (uint16_t)(length - CSB_FRAGMENT_MAX);
    return write_clb_fragment(HCI_CLB_FRAGMENT_END,
                              data + CSB_FRAGMENT_MAX, (uint8_t)remaining);
}

void csb_write_bdaddr(const uint8_t address[6])
{
    volatile uint16_t *const base =
        (volatile uint16_t *)(CSB_CS_ADDR + CS_BDADDR_OFFSET);
    base[0] = (uint16_t)(address[5] | ((uint16_t)address[4] << 8));
    base[1] = (uint16_t)(address[3] | ((uint16_t)address[2] << 8));
    base[2] = (uint16_t)(address[1] | ((uint16_t)address[0] << 8));
}

void csb_write_bch(uint64_t bch34)
{
    volatile uint16_t *const base =
        (volatile uint16_t *)(CSB_CS_ADDR + CS_BCH_OFFSET);
    base[0] = (uint16_t)(bch34 & 0xFFFF);
    base[1] = (uint16_t)((bch34 >> 16) & 0xFFFF);
    base[2] = (uint16_t)((bch34 >> 32) & 0x3);
}

uint16_t hop_field(uint8_t channel)
{
    uint16_t value = (uint16_t)(channel >> 1);
    if (channel & 1) {
        value = (uint16_t)(value + 40);
    }
    return value;
}

static void csb_set_channel(uint8_t channel)
{
    volatile uint16_t *const hopcntl =
        (volatile uint16_t *)(CSB_CS_ADDR + CS_HOPCNTL_OFFSET);
    uint16_t value = *hopcntl;
    value &= (uint16_t)~CS_HOPCNTL_FH_EN;
    value = (uint16_t)((value & (uint16_t)~CS_HOPCNTL_CHANNEL_MASK) |
                       (uint16_t)(hop_field(channel) << 8));
    *hopcntl = value;
}

/*
 * - ROMがイベントごとに再生成するdescriptorへ、この段階で設定したdescriptorフィールドを書き込む。
 * - HEC、CRC、誤り訂正符号、白色化は後段のベースバンドハードウェアが生成する。
 */
esp_err_t csb_apply(void)
{
    if (s_csb_raw_mode) {
        /*
         * - ヘッダーとペイロードヘッダーはControllerが生成したフィールド値を使用する。
         * - 共有メモリのペイロード領域へは書き込まない。
         * - 再指定するのはBD_ADDR、パリティ、チャンネルの3項目だけなので、descriptorを監視する必要はない。
         */
        csb_write_bdaddr(s_target_addr);
        csb_write_bch(s_target_bch);
        csb_set_channel(s_csb_raw_channel);
        if (s_cs_override_active) {
            volatile uint16_t *const word =
                (volatile uint16_t *)(CSB_CS_ADDR + s_cs_override_offset);
            *word = (uint16_t)((s_cs_override_base & ~s_cs_override_mask) |
                               (s_cs_override_value & s_cs_override_mask));
        }
        return ESP_OK;
    }

    const uint16_t descriptor_offset =
        *(volatile uint16_t *)(CSB_CS_ADDR + CS_TXDESC_OFFSET);
    if (descriptor_offset == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    if (descriptor_offset > EM_SIZE - CSB_DESC_SIZE) {
        return ESP_ERR_INVALID_STATE;
    }

    volatile uint16_t *const descriptor =
        (volatile uint16_t *)(EM_BASE + descriptor_offset);
    descriptor[1] = s_csb_header;
    descriptor[2] = s_csb_pheader;

    const uint16_t payload_offset = descriptor[3];
    if (payload_offset == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    if (payload_offset > EM_SIZE - s_csb_data_length) {
        return ESP_ERR_INVALID_STATE;
    }

    volatile uint8_t *const payload =
        (volatile uint8_t *)(EM_BASE + payload_offset);
    for (size_t i = 0; i < s_csb_data_length; ++i) {
        payload[i] = s_csb_staging[i];
    }
    csb_write_bdaddr(s_target_addr);
    csb_write_bch(s_target_bch);
    return ESP_OK;
}

esp_err_t csb_apply_wait(void)
{
    for (unsigned i = 0; i < 10; ++i) {
        const esp_err_t err = csb_apply();
        if (err != ESP_ERR_NOT_FOUND) {
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_TIMEOUT;
}

/*
 * 1. ROMのr_ld_read_clockと同じく、bit31を立ててサンプルを要求する。
 * 2. bit31が0になるまで待機する。
 * 3. 下位28ビットを読み出す。
 *
 * ROMはネイティブクロック値を625us単位に丸めて返す。
 * ここでは白色化の初期値がCLK[6:1]で決まるため、丸めずにCLKNを返す。
 */
bool switch1_csb_read_clkn(uint32_t *out_clkn)
{
    volatile uint32_t *const reg = (volatile uint32_t *)BT_CLKN_SAMPLE_REG;
    __asm__ __volatile__("memw" ::: "memory");
    *reg = (*reg & ~BT_CLKN_SAMPLE_REQ) | BT_CLKN_SAMPLE_REQ;
    __asm__ __volatile__("memw" ::: "memory");
    for (unsigned i = 0; i < BT_CLKN_POLL_LIMIT; ++i) {
        const uint32_t value = *reg;
        __asm__ __volatile__("memw" ::: "memory");
        if ((value & BT_CLKN_SAMPLE_REQ) == 0) {
            if (out_clkn != NULL) {
                *out_clkn = value & BT_CLKN_VALUE_MASK;
            }
            return true;
        }
    }
    return false;
}

uint16_t switch1_csb_payload_offset(void)
{
    const uint16_t descriptor_offset =
        *(volatile uint16_t *)(CSB_CS_ADDR + CS_TXDESC_OFFSET);
    if (descriptor_offset == 0 || descriptor_offset > EM_SIZE - CSB_DESC_SIZE) {
        return 0;
    }
    return *(volatile uint16_t *)(EM_BASE + descriptor_offset + 6);
}

esp_err_t switch1_csb_read_payload_at(uint16_t payload_offset, uint8_t *out,
                                      uint16_t length)
{
    if (out == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload_offset == 0 || payload_offset > EM_SIZE - length) {
        return ESP_ERR_NOT_FOUND;
    }
    const volatile uint8_t *const payload =
        (const volatile uint8_t *)(EM_BASE + payload_offset);
    for (uint16_t i = 0; i < length; ++i) {
        out[i] = payload[i];
    }
    return ESP_OK;
}

esp_err_t switch1_csb_write_payload_at(uint16_t payload_offset,
                                       const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0 || length > s_csb_data_length) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_csb_running || !s_csb_raw_mode) {
        return ESP_ERR_INVALID_STATE;
    }
    if (payload_offset == 0 || payload_offset > EM_SIZE - length) {
        return ESP_ERR_NOT_FOUND;
    }
    volatile uint8_t *const payload =
        (volatile uint8_t *)(EM_BASE + payload_offset);
    for (uint16_t i = 0; i < length; ++i) {
        payload[i] = data[i];
    }
    return ESP_OK;
}

/*
 * - ld_csb_tx_env(0x3ffb827c)が先頭アドレスを保持する。
 * - ld_csb_tx_sched(0x40039c1c)はバーストごとに要素+0x08(次の送信時刻)へ要素+0x30(周期)を加算して再登録する。
 * - クロックから再計算しないため、周期を1回変更すると、その後の送信時刻は変更分だけずれた状態で維持される。
 * - 時刻の単位は625usのスロットである。
 */
#define CSB_TX_ENV_ADDR UINT32_C(0x3ffb827c)
#define CSB_TX_EVT_TIME 0x08
#define CSB_TX_EVT_INTERVAL 0x30
#define CSB_TX_EVT_DISABLED 0x3C
#define CSB_TX_EVT_STATE 0x44
#define CSB_TX_EVT_PENDING UINT8_C(1)
#define CSB_TX_EVT_ACTIVE UINT8_C(2)

// スケジュール要素の優先度。r_ea_elt_insert(0x400153b2)はこのフィールドを比較し、既存要素の優先度が新規要素以上の場合は、新規要素の登録を拒否する。
#define CSB_EVT_PRIORITY 0x16
#define CSB_EVT_DURATION 0x12
/*
 * 受信要素だけに存在するフィールド。
 * 送信要素とは配置が異なる。
 *
 * - +0x36: 受信間隔(スロット)。送信周期は+0x30だが、受信要素の+0x30はクロックオフセットを格納する。
 * - +0x3E: 状態。0は停止、1は動作中、2は停止要求の保留中。
 */
#define CSB_RX_EVT_INTERVAL 0x36
#define CSB_RX_EVT_STATE 0x3E
#define CSB_RX_EVT_SUP_BASE 0x2C
#define CSB_RX_EVT_SUP_TIMEOUT 0x3A

/*
 * - 要素はControllerのヒープ、つまりDRAM内に4バイト境界で確保される。
 * - ld_csb_rx_end(0x40039098)が要素を解放した直後は0になる。
 * - 別用途へ再割り当てされる可能性があるため、読み出し後にポインタを再確認し、ポインタが変化していれば読み出し結果を破棄する。
 */
static bool csb_sched_element_valid(uint32_t element)
{
    return element >= UINT32_C(0x3ff80000) &&
           element < UINT32_C(0x40000000) && (element & 3) == 0;
}

void switch1_csb_sched_read(switch1_csb_sched_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    const uint32_t tx = *(volatile uint32_t *)CSB_TX_ENV_ADDR;
    const uint32_t rx = *(volatile uint32_t *)CSB_RX_ENV_ADDR;
    out->tx_element = csb_sched_element_valid(tx) ? tx : 0;
    out->rx_element = csb_sched_element_valid(rx) ? rx : 0;
    if (out->tx_element != 0) {
        out->tx_duration_us = *(volatile uint16_t *)(tx + CSB_EVT_DURATION);
        out->tx_priority = *(volatile uint8_t *)(tx + CSB_EVT_PRIORITY);
        /*
         * 1. 読み出し中に要素が解放されると、別用途へ再割り当てされた領域を読み出すことになる。
         * 2. 読み出し後にポインタを再確認する。
         * 3. ポインタが変化していれば、読み出したスケジュール情報を破棄する。
         */
        if (*(volatile uint32_t *)CSB_TX_ENV_ADDR != tx) {
            out->tx_element = 0;
            out->tx_duration_us = 0;
            out->tx_priority = 0;
        }
    }
    if (out->rx_element != 0) {
        out->rx_duration_us = *(volatile uint16_t *)(rx + CSB_EVT_DURATION);
        out->rx_priority = *(volatile uint8_t *)(rx + CSB_EVT_PRIORITY);
        out->rx_interval_slots =
            *(volatile uint16_t *)(rx + CSB_RX_EVT_INTERVAL);
        out->rx_state = *(volatile uint8_t *)(rx + CSB_RX_EVT_STATE);
        out->rx_sup_base = *(volatile uint32_t *)(rx + CSB_RX_EVT_SUP_BASE);
        out->rx_sup_timeout_slots =
            *(volatile uint16_t *)(rx + CSB_RX_EVT_SUP_TIMEOUT);
        if (*(volatile uint32_t *)CSB_RX_ENV_ADDR != rx) {
            out->rx_element = 0;
            out->rx_duration_us = 0;
            out->rx_priority = 0;
            out->rx_interval_slots = 0;
            out->rx_state = 0;
            out->rx_sup_base = 0;
            out->rx_sup_timeout_slots = 0;
        }
    }
}

static volatile uint8_t *csb_tx_event(void)
{
    const uint32_t evt = *(const volatile uint32_t *)CSB_TX_ENV_ADDR;
    // ControllerのヒープはDRAMにあるため、アドレス範囲を検証してから参照する。
    if (evt < UINT32_C(0x3ff80000) || evt >= UINT32_C(0x40000000) ||
        (evt & 3) != 0) {
        return NULL;
    }
    return (volatile uint8_t *)evt;
}

/*
 * - スケジュール要素+0x12は、スケジュール上の送信時間をus単位で設定するフィールドである。
 * - ea_conflict_check(0x40014e9c)は区間の重なりだけを判定する。
 * - ESP-IDFのr_ld_csb_tx_en_wrapperはDH3に対して9255usを設定するが、無線バーストの送信時間は1875usである。
 * - +0x12の設定値を変更せずにCSB受信を有効にすると、受信時刻を中心とする18スロット内に送信イベントを登録できず、送信間隔は3.75msから15msへ延長される。
 * - +0x12にはCSB送信を有効にするときだけ設定値が書き込まれるため、1回の書き込み後も設定値が保持される。
 */
#define CSB_TX_EVT_DURATION 0x12

/*
 * - control structure +0x02は、スロット境界からの位置をus単位で指定する。
 * - スロット内オフセット値は0~1023の範囲で、ACLとCSB受信ではハードウェアが測定値を格納する。
 * - CSB送信ではスロット内オフセット値が0から変更されないため、送信位置は常にスロット境界と一致する。
 * - 無線イベントのスケジュール要素+0x14は競合判定だけに使用される。
 * - control structureだけを変更すると判定位置と実際の送信位置がずれるため、同じスロット内オフセット値を両方へ書き込む。
 */
#define CS_BITOFF_OFFSET UINT32_C(0x02)
#define CSB_TX_EVT_BITOFF 0x14

esp_err_t switch1_csb_tx_set_bit_offset(uint16_t offset_us)
{
    if (offset_us > 1023) {
        return ESP_ERR_INVALID_ARG;
    }
    *(volatile uint16_t *)(CSB_CS_ADDR + CS_BITOFF_OFFSET) = offset_us;
    volatile uint8_t *const evt = csb_tx_event();
    if (evt != NULL) {
        *(volatile uint16_t *)(evt + CSB_TX_EVT_BITOFF) = offset_us;
    }
    return ESP_OK;
}

esp_err_t switch1_csb_tx_set_duration(uint16_t duration_us)
{
    volatile uint8_t *const evt = csb_tx_event();
    if (evt == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    volatile uint16_t *const field =
        (volatile uint16_t *)(evt + CSB_TX_EVT_DURATION);
    const uint16_t before = *field;
    *field = duration_us;
    const uint16_t after = *field;
    /*
     * - 書き込み結果をログへ記録する。
     * - スケジュール上の送信時間の書き込みに失敗すると、CSB受信を有効にした直後から送信間隔は3.75msから15msへ延長される。
     */
    ESP_LOGI(TAG, "Scheduled TX time (us): %u -> %u (requested=%u)", before, after,
             duration_us);
    return after == duration_us ? ESP_OK : ESP_FAIL;
}

/*
 * +0x08のうち、白色化を無効にした送信でROMが設定するフィールド。
 *
 * - bit8-10: LT_ADDR
 * - bit11: EDR(速度)
 * - bit12: 白色化の無効化
 * - bit14: AFHの有効化
 */
#define CS_LINKCNTL_EDR UINT16_C(0x0800)
#define CS_LINKCNTL_AFH_EN UINT16_C(0x4000)

static uint16_t s_csb_tx_linkcntl_saved;
static bool s_csb_tx_linkcntl_saved_valid;

void switch1_csb_tx_set_whitening_disable(bool disable)
{
    /*
     * - 白色化を無効にして送信するときは、r_ld_strain_start(0x40040a8c)と同じビット構成にする。
     * - 同期トレインは仕様上、白色化を使用しないパケットである。
     * - LT_ADDRを0にし、AFHとEDRを無効にして、bit12を設定する。
     * - bit12だけを設定して他のフィールドを維持すると、送信パケット長が316.8usから960.5usへ延長され、アクセスコードを正しく生成できない。
     */
    volatile uint16_t *const link =
        (volatile uint16_t *)(CSB_CS_ADDR + CS_LTADDR_OFFSET);
    if (disable) {
        if (!s_csb_tx_linkcntl_saved_valid) {
            s_csb_tx_linkcntl_saved = *link;
            s_csb_tx_linkcntl_saved_valid = true;
        }
        uint16_t value = *link;
        value &= (uint16_t)~(CS_LTADDR_MASK | CS_LINKCNTL_EDR |
                             CS_LINKCNTL_AFH_EN);
        value |= CS_LINKCNTL_WHITENING_DSB;
        *link = value;
    } else if (s_csb_tx_linkcntl_saved_valid) {
        *link = s_csb_tx_linkcntl_saved;
        s_csb_tx_linkcntl_saved_valid = false;
    }
}

/*
 * - control structure +0x00のbit0-4にフレームフォーマットが格納される。
 * - ヘッダーのFLOW、ARQN、SEQNはROMではなくベースバンドのハードウェアが生成する。
 * - CSB送信のフレームフォーマット24では仕様どおり3ビットとも0になり、descriptorへ0x0298を書き込んでも無線上のヘッダー値は0x018になった。
 * - ACLのフレームフォーマットではリンクのARQ状態から生成される。
 */
esp_err_t switch1_csb_tx_set_frame_format(uint8_t format, uint16_t *out_before)
{
    if (format > 31) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_csb_running || !s_csb_raw_mode) {
        return ESP_ERR_INVALID_STATE;
    }
    volatile uint16_t *const word = (volatile uint16_t *)CSB_CS_ADDR;
    const uint16_t before = *word;
    if (out_before != NULL) {
        *out_before = before;
    }
    *word = (uint16_t)((before & ~UINT16_C(0x001f)) | format);
    return ESP_OK;
}

bool switch1_csb_tx_read_schedule(uint32_t *out_next_slot,
                                  uint16_t *out_interval)
{
    volatile uint8_t *const evt = csb_tx_event();
    if (evt == NULL) {
        return false;
    }
    if (out_next_slot != NULL) {
        *out_next_slot = *(const volatile uint32_t *)(evt + CSB_TX_EVT_TIME);
    }
    if (out_interval != NULL) {
        *out_interval =
            *(const volatile uint16_t *)(evt + CSB_TX_EVT_INTERVAL);
    }
    return true;
}

typedef struct {
    uint32_t target_slot;
    uint32_t scheduled_slot;
    uint16_t bit_offset_us;
    uint16_t minimum_ahead_slots;
    volatile esp_err_t result;
    volatile uint8_t state;
    volatile bool cancel_requested;
} csb_tx_reschedule_request_t;

enum {
    CSB_TX_RESCHEDULE_IDLE,
    CSB_TX_RESCHEDULE_PENDING,
    CSB_TX_RESCHEDULE_RUNNING,
    CSB_TX_RESCHEDULE_DONE,
};

static csb_tx_reschedule_request_t s_csb_tx_reschedule;

static void csb_tx_reschedule_finish(csb_tx_reschedule_request_t *request,
                                     esp_err_t result)
{
    request->result = result;
    __asm__ __volatile__("memw" ::: "memory");
    request->state = CSB_TX_RESCHEDULE_DONE;
}

static void csb_tx_reschedule_controller(void *arg)
{
    csb_tx_reschedule_request_t *const request = arg;
    request->state = CSB_TX_RESCHEDULE_RUNNING;
    __asm__ __volatile__("memw" ::: "memory");
    if (request->cancel_requested) {
        csb_tx_reschedule_finish(request, ESP_ERR_TIMEOUT);
        return;
    }
    volatile uint8_t *const evt = csb_tx_event();
    if (evt == NULL || evt[CSB_TX_EVT_DISABLED] != 0) {
        csb_tx_reschedule_finish(request, ESP_ERR_INVALID_STATE);
        return;
    }
    const uint8_t event_state = evt[CSB_TX_EVT_STATE];
    if (event_state == CSB_TX_EVT_ACTIVE) {
        // 送信中のスケジュール要素は、フレーム完了コールバックが次の周期を登録するまで待機する
        csb_tx_reschedule_finish(request, ESP_ERR_NOT_FINISHED);
        return;
    }
    if (event_state != CSB_TX_EVT_PENDING) {
        csb_tx_reschedule_finish(request, ESP_ERR_INVALID_STATE);
        return;
    }

    const uint16_t interval =
        *(const volatile uint16_t *)(evt + CSB_TX_EVT_INTERVAL);
    if (interval == 0) {
        csb_tx_reschedule_finish(request, ESP_ERR_INVALID_STATE);
        return;
    }

    const uint32_t now = r_ld_read_clock() & ACL_CLOCK_MASK;
    uint32_t target = request->target_slot & ACL_CLOCK_MASK;
    const uint32_t raw_delta = (target - now) & ACL_CLOCK_MASK;
    const int32_t ahead =
        (raw_delta & UINT32_C(0x04000000)) != 0
            ? (int32_t)(raw_delta | ~ACL_CLOCK_MASK)
            : (int32_t)raw_delta;
    if (ahead < request->minimum_ahead_slots) {
        const uint32_t periods =
            ((uint32_t)(request->minimum_ahead_slots - ahead) + interval - 1u) /
            interval;
        target = (target + periods * interval) & ACL_CLOCK_MASK;
    }

    const uint32_t old_slot =
        *(const volatile uint32_t *)(evt + CSB_TX_EVT_TIME);
    const uint16_t old_event_bitoff =
        *(const volatile uint16_t *)(evt + CSB_TX_EVT_BITOFF);
    const uint16_t old_cs_bitoff =
        *(const volatile uint16_t *)(CSB_CS_ADDR + CS_BITOFF_OFFSET);
    if (r_ea_elt_remove((void *)evt) != 0) {
        csb_tx_reschedule_finish(request, ESP_ERR_INVALID_STATE);
        return;
    }

    *(volatile uint32_t *)(evt + CSB_TX_EVT_TIME) = target;
    *(volatile uint16_t *)(evt + CSB_TX_EVT_BITOFF) = request->bit_offset_us;
    *(volatile uint16_t *)(CSB_CS_ADDR + CS_BITOFF_OFFSET) =
        request->bit_offset_us;
    __asm__ __volatile__("memw" ::: "memory");
    if (r_ea_elt_insert((void *)evt) != 0) {
        *(volatile uint32_t *)(evt + CSB_TX_EVT_TIME) = old_slot;
        *(volatile uint16_t *)(evt + CSB_TX_EVT_BITOFF) = old_event_bitoff;
        *(volatile uint16_t *)(CSB_CS_ADDR + CS_BITOFF_OFFSET) = old_cs_bitoff;
        __asm__ __volatile__("memw" ::: "memory");
        if (r_ea_elt_insert((void *)evt) != 0) {
            evt[CSB_TX_EVT_STATE] = 0;
            csb_tx_reschedule_finish(request, ESP_FAIL);
            return;
        }
        evt[CSB_TX_EVT_STATE] = CSB_TX_EVT_PENDING;
        csb_tx_reschedule_finish(request, ESP_ERR_INVALID_STATE);
        return;
    }

    evt[CSB_TX_EVT_STATE] = CSB_TX_EVT_PENDING;
    request->scheduled_slot =
        *(const volatile uint32_t *)(evt + CSB_TX_EVT_TIME) & ACL_CLOCK_MASK;
    csb_tx_reschedule_finish(request, ESP_OK);
}

esp_err_t switch1_csb_tx_reschedule(uint32_t target_slot,
                                    uint16_t bit_offset_us,
                                    uint16_t minimum_ahead_slots,
                                    uint32_t *out_scheduled_slot)
{
    if (bit_offset_us > 1023 || out_scheduled_slot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_csb_tx_reschedule.state == CSB_TX_RESCHEDULE_PENDING ||
        s_csb_tx_reschedule.state == CSB_TX_RESCHEDULE_RUNNING) {
        return ESP_ERR_INVALID_STATE;
    }
    const int64_t until = esp_timer_get_time() + 10000;
    esp_err_t result;
    do {
        s_csb_tx_reschedule.target_slot = target_slot;
        s_csb_tx_reschedule.scheduled_slot = UINT32_MAX;
        s_csb_tx_reschedule.bit_offset_us = bit_offset_us;
        s_csb_tx_reschedule.minimum_ahead_slots = minimum_ahead_slots;
        s_csb_tx_reschedule.result = ESP_ERR_TIMEOUT;
        s_csb_tx_reschedule.cancel_requested = false;
        s_csb_tx_reschedule.state = CSB_TX_RESCHEDULE_PENDING;
        __asm__ __volatile__("memw" ::: "memory");
        if (btdm_dispatch_work_to_controller(csb_tx_reschedule_controller,
                                             &s_csb_tx_reschedule, true) != 0) {
            s_csb_tx_reschedule.state = CSB_TX_RESCHEDULE_IDLE;
            return ESP_FAIL;
        }

        while (s_csb_tx_reschedule.state != CSB_TX_RESCHEDULE_DONE &&
               esp_timer_get_time() < until) {
        }
        if (s_csb_tx_reschedule.state != CSB_TX_RESCHEDULE_DONE) {
            s_csb_tx_reschedule.cancel_requested = true;
            __asm__ __volatile__("memw" ::: "memory");
            // 処理開始済みの場合は結果が確定するまで待機し、変更完了後にタイムアウトを返さない
            while (s_csb_tx_reschedule.state == CSB_TX_RESCHEDULE_RUNNING) {
            }
            if (s_csb_tx_reschedule.state != CSB_TX_RESCHEDULE_DONE) {
                return ESP_ERR_TIMEOUT;
            }
        }
        __asm__ __volatile__("memw" ::: "memory");
        result = s_csb_tx_reschedule.result;
        if (result == ESP_OK) {
            *out_scheduled_slot = s_csb_tx_reschedule.scheduled_slot;
            s_csb_tx_reschedule.state = CSB_TX_RESCHEDULE_IDLE;
            return ESP_OK;
        }
        s_csb_tx_reschedule.state = CSB_TX_RESCHEDULE_IDLE;
        if (result != ESP_ERR_NOT_FINISHED || esp_timer_get_time() >= until) {
            return result;
        }
        esp_rom_delay_us(100);
    } while (esp_timer_get_time() < until);

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t switch1_csb_tx_set_interval(uint16_t expect, uint16_t interval)
{
    volatile uint8_t *const evt = csb_tx_event();
    if (evt == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    volatile uint16_t *const field =
        (volatile uint16_t *)(evt + CSB_TX_EVT_INTERVAL);
    /*
     * - 書き込み前にintervalフィールドの現在値を検証し、対象フィールドが想定した位置にあることを確認する。
     * - 期待するinterval値と異なる場合は、要素の配置が異なるか、別の周期で動作している。
     */
    if (*field != expect) {
        return ESP_ERR_INVALID_STATE;
    }
    *field = interval;
    return ESP_OK;
}

/*
 * - descriptorの先頭2ワードにはヘッダーとペイロードヘッダーが格納される。
 * - csb_apply()が通常のペイロード送信経路で書き込む位置と同じである。
 */
static volatile uint16_t *csb_descriptor(void)
{
    const uint16_t descriptor_offset =
        *(volatile uint16_t *)(CSB_CS_ADDR + CS_TXDESC_OFFSET);
    if (descriptor_offset == 0 || descriptor_offset > EM_SIZE - CSB_DESC_SIZE) {
        return NULL;
    }
    return (volatile uint16_t *)(EM_BASE + descriptor_offset);
}

esp_err_t switch1_csb_read_descriptor(uint16_t *out_header,
                                      uint16_t *out_pheader)
{
    volatile uint16_t *const descriptor = csb_descriptor();
    if (descriptor == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    if (out_header != NULL) {
        *out_header = descriptor[1];
    }
    if (out_pheader != NULL) {
        *out_pheader = descriptor[2];
    }
    return ESP_OK;
}

esp_err_t switch1_csb_write_descriptor(uint16_t header, uint16_t pheader)
{
    if (!s_csb_running || !s_csb_raw_mode) {
        return ESP_ERR_INVALID_STATE;
    }
    volatile uint16_t *const descriptor = csb_descriptor();
    if (descriptor == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    descriptor[1] = header;
    descriptor[2] = pheader;
    /*
     * - ペイロードヘッダーのLLIDが3(LMPの区分)の場合は、データ位置をdescriptor +0x08にも書き込む。
     * - ld_acl_tx(0x4003000d)は+0x08を先に読み出し、0の場合だけ+0x06を読み出す。
     * - ACLのLMP送信は+0x08へ、データ送信は+0x06へデータ位置を書き分ける。
     * - CSB送信は+0x06しか設定しないため、LLIDだけを3にするとLMP用ペイロードの参照位置(+0x08)が0のままになる。
     */
    if ((pheader & 0x3) == 3) {
        descriptor[4] = descriptor[3];
    } else {
        descriptor[4] = 0;
    }
    return ESP_OK;
}

uint16_t switch1_csb_descriptor_offset(void)
{
    const uint16_t descriptor_offset =
        *(volatile uint16_t *)(CSB_CS_ADDR + CS_TXDESC_OFFSET);
    if (descriptor_offset > EM_SIZE - CSB_DESC_SIZE) {
        return 0;
    }
    return descriptor_offset;
}

esp_err_t switch1_csb_raw_start(const uint8_t bd_addr[6], uint64_t bch34,
                                const uint8_t *data, uint16_t length,
                                uint16_t interval, uint8_t channel)
{
    if (bd_addr == NULL || data == NULL || length == 0 ||
        length > SWITCH1_CSB_DATA_MAX || interval < 2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_csb_running || g_command_pending) {
        return ESP_ERR_INVALID_STATE;
    }

    memcpy(s_target_addr, bd_addr, sizeof(s_target_addr));
    s_target_bch = bch34;
    memcpy(s_csb_staging, data, length);
    s_csb_data_length = length;
    s_csb_interval = interval;
    s_csb_raw_mode = true;
    s_csb_raw_channel = channel;
    s_cs_override_active = false;
    s_csb_events = 0;
    s_channel_index = 0;

    uint8_t params[16];
    uint8_t *p = params;
    esp_err_t err;
    if (!s_lt_reserved) {
        UINT8_TO_STREAM(p, CSB_LT_ADDR);
        err = send_command_wait(HCI_SET_RESERVED_LT_ADDR, params, 1);
        if (err != ESP_OK) {
            /*
             * - status 0x0bは、そのLT_ADDRがすでに使用されていることを示す。
             * - 既存のLT_ADDR予約が残っている場合は正常な状態として扱い、予約済みの状態で処理を継続する。
             * - それ以外のstatusは受信したエラーとして返す。
             */
            if (g_command_status != HCI_ERR_CONNECTION_EXISTS) {
                return err;
            }
            ESP_LOGW(TAG, "LT_ADDR is already reserved; reusing the existing reservation");
        }
        s_lt_reserved = true;
    }
    /*
     * 1. CSBを有効化する前にデータを書き込まない。
     * 2. Controllerは長さをその時点で有効なパケット種別に対して検査する。
     * 3. DH5を許可するHCI_SET_CLBより前に339バイトを渡すと、status 0x12で拒否される。
     */

    // ROMが最初のdescriptorを生成する間は通常のBCH parityビット列を使用する
    csb_write_bdaddr(s_target_addr);
    csb_write_bch((s_target_bch & ~UINT64_C(0xffffffff)) |
                  (~s_target_bch & UINT64_C(0xffffffff)));

    bool retried = false;
    // 183バイト以下の場合はDH3で送信する
    const bool use_dh3 = s_csb_data_length <= CSB_DH3_BODY_MAX;
    s_csb_packet_type = use_dh3 ? CSB_PACKET_MASK_DH3 : CSB_PACKET_MASK_DH5;
    p = params;
    UINT8_TO_STREAM(p, HCI_CLB_ENABLE);
    UINT8_TO_STREAM(p, CSB_LT_ADDR);
    UINT8_TO_STREAM(p, 0);
    UINT16_TO_STREAM(p, s_csb_packet_type);
    UINT16_TO_STREAM(p, interval);
    UINT16_TO_STREAM(p, interval);
    UINT16_TO_STREAM(p, CSB_SUPERVISION_TIMEOUT);
    err = send_command_wait(HCI_SET_CLB, params, 11);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HCI_SET_CLB rejected the dedicated packet mask; retrying with DH3/DH1 included");
        s_csb_packet_type = CSB_PACKET_MASK_DH5_RETRY;
        p = params;
        UINT8_TO_STREAM(p, HCI_CLB_ENABLE);
        UINT8_TO_STREAM(p, CSB_LT_ADDR);
        UINT8_TO_STREAM(p, 0);
        UINT16_TO_STREAM(p, s_csb_packet_type);
        UINT16_TO_STREAM(p, interval);
        UINT16_TO_STREAM(p, interval);
        UINT16_TO_STREAM(p, CSB_SUPERVISION_TIMEOUT);
        err = send_command_wait(HCI_SET_CLB, params, 11);
        if (err != ESP_OK) {
            return err;
        }
        retried = true;
    }


    // HCI_SET_CLBの戻り値はstatus、LT_ADDR、採用された送信間隔の順
    const uint16_t accepted_interval =
        (g_command_return_length >= 4)
            ? (uint16_t)(g_command_return[2] | ((uint16_t)g_command_return[3] << 8))
            : 0;

    /*
     * 白色化の初期値はpiconet clockのCLK[6:1]で決まる。
     * 送信開始時のクロックを読み出して保存すると、バーストごとに白色化を打ち消す開始位置を計算できる。
     */
    uint32_t native_clock = 0;
    {
        uint8_t clock_params[3];
        uint8_t *cp = clock_params;
        /*
         * - Connection_Handle。ローカル読み出しでは使用しない。
         * - Which_Clock = 0はローカルクロックを指定する。
         */
        UINT16_TO_STREAM(cp, 0);
        UINT8_TO_STREAM(cp, 0);
        if (send_command_wait(HCI_READ_CLOCK, clock_params, 3) == ESP_OK &&
            g_command_return_length >= 7) {
            native_clock = (uint32_t)g_command_return[3] |
                           ((uint32_t)g_command_return[4] << 8) |
                           ((uint32_t)g_command_return[5] << 16) |
                           ((uint32_t)g_command_return[6] << 24);
        }
    }
    s_csb_native_clock = native_clock;

    s_csb_running = true;
    err = write_clb_data_raw(s_csb_staging, s_csb_data_length);
    if (err != ESP_OK && s_csb_data_length > CSB_FRAGMENT_MAX) {
        /*
         * - Controllerが分割送信を受け付けない場合に備える。
         * - 1つのfragmentで送信できる252バイトへ短縮して1回だけ再試行する。
         * - 252は4バイトのウェイクアップコード63個分なので、反復は途切れず、本体の開始ビット位置も変わらない。
         */
        ESP_LOGW(TAG, "%u-byte transmission was rejected; retrying with %u bytes",
                 s_csb_data_length, CSB_FRAGMENT_MAX);
        s_csb_data_length = CSB_FRAGMENT_MAX;
        err = write_clb_data_raw(s_csb_staging, s_csb_data_length);
    }
    if (err != ESP_OK) {
        switch1_csb_wake_stop_wait();
        return err;
    }
    csb_set_channel(channel);
    const esp_err_t after_err = csb_apply_wait();
    if (after_err != ESP_OK) {
        switch1_csb_wake_stop_wait();
        return after_err;
    }
    s_csb_events++;

    /*
     * - スケジュール上の送信時間を無線バーストの送信時間へ修正する。
     * - CSB受信と同時に動作させたときの送信間引きを防ぐ。
     * - DH5は5スロット、DH3は3スロットの送信イベント継続時間を設定する。
     */
    switch1_csb_tx_set_duration(
        (uint16_t)((s_csb_data_length > CSB_DH3_BODY_MAX ? 5 : 3) * 625));

    uint16_t descriptor_header = 0;
    uint16_t descriptor_pheader = 0;
    uint16_t descriptor_payload_offset = 0;
    const uint16_t descriptor_offset =
        *(volatile uint16_t *)(CSB_CS_ADDR + CS_TXDESC_OFFSET);
    if (descriptor_offset != 0 && descriptor_offset <= EM_SIZE - CSB_DESC_SIZE) {
        volatile uint16_t *const descriptor =
            (volatile uint16_t *)(EM_BASE + descriptor_offset);
        descriptor_header = descriptor[1];
        descriptor_pheader = descriptor[2];
        descriptor_payload_offset = descriptor[3];
    }
    ESP_LOGI(TAG,
             "CSB TX started: packet_type=%s interval=%u accepted_interval=%u "
             "channel=%u payload_length=%u native_clock=%08x packet_mask=%04x%s "
             "descriptor_header=%04x descriptor_pheader=%04x payload_offset=%04x",
             use_dh3 ? "DH3" : "DH5",
             interval, accepted_interval, channel, s_csb_data_length,
             (unsigned)s_csb_native_clock,
             s_csb_packet_type, retried ? "(retry)" : "",
             descriptor_header, descriptor_pheader, descriptor_payload_offset);
    return ESP_OK;
}

esp_err_t switch1_csb_wake_refresh(void)
{
    if (!s_csb_running) {
        return ESP_ERR_INVALID_STATE;
    }
    return csb_apply();
}

esp_err_t switch1_csb_wake_stop_wait(void)
{
    if (g_command_timed_out) {
        return ESP_ERR_TIMEOUT;
    }
    if (!s_csb_running) {
        return g_command_pending ? ESP_ERR_INVALID_STATE : ESP_OK;
    }

    uint8_t params[16];
    uint8_t *p = params;
    UINT8_TO_STREAM(p, HCI_CLB_DISABLE);
    UINT8_TO_STREAM(p, CSB_LT_ADDR);
    UINT8_TO_STREAM(p, 0);
    UINT16_TO_STREAM(p, s_csb_packet_type);
    UINT16_TO_STREAM(p, s_csb_interval);
    UINT16_TO_STREAM(p, s_csb_interval);
    UINT16_TO_STREAM(p, CSB_SUPERVISION_TIMEOUT);

    const esp_err_t err = send_command_wait(HCI_SET_CLB, params, 11);
    if (err == ESP_OK) {
        s_csb_running = false;
        s_csb_raw_mode = false;
        // Command Completeを受信した後、ROMが実行中のフレームを完了するまで待機する
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return err;
}

uint32_t switch1_csb_wake_events(void)
{
    return s_csb_events;
}
