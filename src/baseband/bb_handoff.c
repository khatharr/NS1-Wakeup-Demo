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

static const char *TAG = "wake_link";


void lmp_not_accepted_hook_install(void);
void lmp_not_accepted_hook_remove(void);
void packet_type_hook_install(void);
void packet_type_hook_remove(void);

acl_handoff_request_t g_acl_handoff;
bool acl_link_context_clear(uint8_t link_id);

bool acl_link_context_clear(uint8_t link_id)
{
    const uint8_t *const hci_record = &hci_env[16 + 7 * link_id];
    return hci_record[0] == 0 && lc_state[link_id] == 0 &&
           lc_env[link_id] == NULL && ld_acl_env[link_id] == NULL &&
           llc_env[link_id] == NULL;
}

static void acl_handoff_controller(void *handoff_arg)
{
    acl_handoff_request_t *const request = handoff_arg;
    const uint8_t link_id = request->reserved_link;
    uint8_t *const pscan_env = ld_pscan_env;

    for (uint8_t i = 0; i < ACL_LINK_COUNT; ++i) {
        const uint8_t *const hci_record = &hci_env[16 + 7 * i];
        if (hci_record[0] == 0) {
            continue;
        }
        /*
         * - hci_env[17 + 7*i]には、r_hci_bt_acl_bdaddr_registerが下位バイト先頭の順序でBD_ADDRを6バイト複製する。
         * - request->targetは表示順で保持しているため、比較時にバイト順を反転する。
         */
        bool same_peer = true;
        for (uint8_t j = 0; j < 6; ++j) {
            if (hci_record[1 + j] != request->target[5 - j]) {
                same_peer = false;
                break;
            }
        }
        if (same_peer) {
            request->result = ESP_ERR_INVALID_STATE;
            __asm__ __volatile__("memw" ::: "memory");
            request->state = ACL_HANDOFF_DONE;
            return;
        }
    }

    // フェーズ2の取得時にはスケジュール要素だけを削除しているため、フレームフォーマット3の受信を再登録する
    uint16_t reject_flags = 0;
    reject_flags |= request->prepare_state != ACL_PREPARE_DONE ? UINT16_C(1) << 0 : 0;
    reject_flags |= request->prepare_result != ESP_OK ? UINT16_C(1) << 1 : 0;
    reject_flags |= link_id >= ACL_LINK_COUNT ? UINT16_C(1) << 2 : 0;
    reject_flags |= pscan_env == NULL ? UINT16_C(1) << 3 : 0;
    if (pscan_env != NULL) {
        reject_flags |= pscan_env[0x3d] != link_id ? UINT16_C(1) << 4 : 0;
        reject_flags |= pscan_env[0x3e] != 0 ? UINT16_C(1) << 5 : 0;
        reject_flags |= pscan_env[0x3f] != 0 ? UINT16_C(1) << 6 : 0;
    }
    if (link_id < ACL_LINK_COUNT) {
        reject_flags |= lm_env[60 + 9 * link_id] != 2 ? UINT16_C(1) << 7 : 0;
        reject_flags |= !acl_link_context_clear(link_id) ? UINT16_C(1) << 8 : 0;
    }
    request->pscan_rx_reject = reject_flags;
    if (reject_flags != 0) {
        request->result = ESP_ERR_INVALID_STATE;
        __asm__ __volatile__("memw" ::: "memory");
        request->state = ACL_HANDOFF_DONE;
        return;
    }
    /*
     * - HCI 0x0442の停止がstatus 0x0cで拒否されても、フェーズ2のcallbackが再登録した
     *   CSB受信要素が残存する。
     * - コントローラー内部で直接停止して競合を防ぐ。
     */
    if (ld_csb_rx_env != NULL) {
        (void)r_ld_csb_rx_stop();
    }
    if (ld_csb_rx_env != NULL) {
        request->pscan_rx_reject = UINT16_C(1) << 11;
        request->result = ESP_ERR_INVALID_STATE;
        __asm__ __volatile__("memw" ::: "memory");
        request->state = ACL_HANDOFF_DONE;
        return;
    }
    /*
     * - 受信descriptorのリングバッファと添字はACL受信とCSB受信で共有される
     *   (添字はld_env+0x1AA = 0x3ffb96ba)。
     * - ウェイクアップ用の受信を継続するとACLリンクの受信と競合する。
     */
    phase2_restore_hook();

    volatile uint16_t *const acl_control =
        (volatile uint16_t *)(ACL_CS_BASE + ACL_CS_STRIDE * link_id);
    for (uint8_t i = 0; i < ACL_CS_SNAPSHOT_WORDS; ++i) {
        request->cs_before[i] = acl_control[i];
    }
    acl_control[0] = (uint16_t)((acl_control[0] & ~UINT16_C(0x001f)) |
                       ACL_CS_FORMAT_PERIPHERAL);
    acl_control[1] = request->rx_bitoff;
    acl_control[2] = (uint16_t)(request->clock_delta & UINT32_C(0xffff));
    acl_control[3] = (uint16_t)((request->clock_delta >> 16) & UINT32_C(0x07ff));
    /*
     * - +0x08にはLT_ADDR(bit8~10)、EDR(bit11)、白色化の無効化(bit12)、AFH(bit14)が割り当てられている。
     * - ウェイクアップの往復は白色化なしだが、ACLリンク生成後は通常のACLリンクである。
     * - 白色化を有効に設定しないと受信したヘッダーのHECが成立せず、パケットが破棄される。
     */
    acl_control[4] = (uint16_t)((acl_control[4] & ~(UINT16_C(0x4700) |
                                  CS_LINKCNTL_WHITENING_DSB)) |
                       ((uint16_t)request->lt_addr << 8));

    acl_control[5] = (uint16_t)(request->target[5] |
                       ((uint16_t)request->target[4] << 8));
    acl_control[6] = (uint16_t)(request->target[3] |
                       ((uint16_t)request->target[2] << 8));
    acl_control[7] = (uint16_t)(request->target[1] |
                       ((uint16_t)request->target[0] << 8));
    acl_control[8] = (uint16_t)(request->bch34 & UINT64_C(0xffff));
    acl_control[9] = (uint16_t)((request->bch34 >> 16) & UINT64_C(0xffff));
    acl_control[10] = (uint16_t)((request->bch34 >> 32) & UINT64_C(0x3));
    /* r_ld_pscan_evt_handler(0x4003f2cf)がページスキャン用のcontrol structureを
     * ESP32が周辺側となるACLリンク用へ変換するときに書き込む値と一致させる。 */
    acl_control[11] |= CS_HOPCNTL_FH_EN;
    acl_control[12] &= (uint16_t)~UINT16_C(0x0008);
    acl_control[13] = 48;

    /*
     * 1. lm_page_scan_end_ind_handler(0x40052e40)がr_lc_start直前に行う準備と同じ順序で書き込む。
     * 2. r_ld_acl_startだけを呼ぶと、下位リンクだけが生成され、lc_envとリンク制御タスクの内部状態が生成されない。
     */
    uint8_t *const lm_link_record =
        &lm_env[LM_LINK_RECORD_BASE + LM_LINK_RECORD_STRIDE * link_id];
    lc_bd_addr_t peer_addr;
    for (uint8_t i = 0; i < 6; ++i) {
        /* request->targetは表示順(先頭がMSB)で保持する。ROMの経路へ渡す値は
         * r_ld_util_fhs_unpkが展開した下位バイト先頭の順序なので、この処理で反転する。 */
        peer_addr.addr[i] = request->target[5 - i];
        lm_link_record[i] = request->target[5 - i];
    }
    lm_link_record[LM_LINK_STATE_OFFSET] = LM_LINK_STATE_CONNECTED;
    lm_link_record[LM_LINK_ROLE_OFFSET] = ACL_ROLE_PERIPHERAL;
    /*
     * - LMリンク記録の+8はLT_ADDRだが、ESP32がBluetooth周辺側となるACLリンクの通常経路では書き込まれない。
     * - 中央側の3経路だけが使用するフィールドなので、この処理では変更しない。
     */

    void **const rom_ip_funcs = *(void ***)PHASE2_ROM_IP_FUNCS_PTR_ADDR;
    if (rom_ip_funcs == NULL) {
        request->pscan_rx_reject = UINT16_C(1) << 12;
        request->result = ESP_ERR_INVALID_STATE;
        __asm__ __volatile__("memw" ::: "memory");
        request->state = ACL_HANDOFF_DONE;
        return;
    }
    const hci_bdaddr_register_fn_t hci_bdaddr_register =
        (hci_bdaddr_register_fn_t)rom_ip_funcs[IP_FUNCS_HCI_BDADDR_REGISTER_INDEX];
    const lc_start_fn_t lc_start =
        (lc_start_fn_t)rom_ip_funcs[IP_FUNCS_LC_START_INDEX];
    if (hci_bdaddr_register == NULL || lc_start == NULL) {
        request->pscan_rx_reject = UINT16_C(1) << 12;
        request->result = ESP_ERR_INVALID_STATE;
        __asm__ __volatile__("memw" ::: "memory");
        request->state = ACL_HANDOFF_DONE;
        return;
    }

    hci_bdaddr_register(link_id, lm_link_record);
    request->lc_start_called = true;
    /*
     * - class of deviceは0に設定する。
     * - Switchのウェイクアップ応答にはFHSが含まれない。
     *   そのためSwitchのclass of deviceを取得できない。
     * - この値はlc_env+54~56へ入り、HCI Connection Requestイベントに含まれるだけで、リンク生成条件には影響しない。
     */
    lc_start(link_id, LC_START_REASON_PAGE_SCAN, peer_addr, 0, 0, 0,
             lm_env[LM_ENV_DEFAULT_LINK_POLICY]);

    /*
     * - r_lc_startの後に、ROMがACLリンクを生成するときと同じ値へ設定する。
     *   フェーズ2の受信向け設定をACL用の値へ変更しないと、パケットヘッダーを復号できてもペイロードで破棄される。
     * - +0x14はBCHの3番目のワードで、ACLリンクでは下位2ビットだけを使用する。
     * - +0x5Eのbit12とbit13はACLリンクを生成する3経路すべてが設定するフィールドだが、ページスキャンの初期化で0になるため、この処理で明示的に設定する。
     */
    acl_control[10] = (uint16_t)((request->bch34 >> 32) & UINT64_C(0x3));
    acl_control[47] |= UINT16_C(0x3000);

    lmp_not_accepted_hook_install();
    packet_type_hook_install();
    /*
     * - フェーズ2の受信開始前に保存した値を+0x60へ書き込む。
     * - +0x60はハードウェアが最後に受理したSEQNを保持するフィールドである。
     *   フェーズ2の応答(SEQN=1)を受けると0x0004が0x0405へ変わる。
     * - この値を復元せずACLリンクへ移ると、Switchの最初のLMPが再送とみなされて破棄される。
     */
    if (g_phase2_capture.tail_first_taken) {
        request->cs60_before = acl_control[48];
        acl_control[48] = g_phase2_capture.tail_first[1];
        request->cs60_after = acl_control[48];
        request->cs60_restored = true;
    }

    uint8_t lc_start_flags = 0;
    lc_start_flags |= lc_env[link_id] != NULL ? LC_START_FLAG_LC_ENV : 0;
    lc_start_flags |= ld_acl_env[link_id] != NULL ? LC_START_FLAG_LD_ACL_ENV : 0;
    lc_start_flags |= llc_env[link_id] != NULL ? LC_START_FLAG_LLC_ENV : 0;
    lc_start_flags |= hci_env[16 + 7 * link_id] != 0 ? LC_START_FLAG_HCI_BDADDR : 0;
    request->lc_start_flags = lc_start_flags;
    request->lc_state_after = lc_state[link_id];
    for (uint8_t i = 0; i < ACL_CS_SNAPSHOT_WORDS; ++i) {
        request->cs_at_start[i] = acl_control[i];
    }
    if (ld_acl_env[link_id] != NULL) {
        const volatile uint8_t *const acl_env_snapshot = ld_acl_env[link_id];
        for (uint16_t i = 0; i < ACL_ENV_SNAPSHOT_BYTES; ++i) {
            request->acl_env_at_start[i] = acl_env_snapshot[i];
        }
    }
    if (lc_env[link_id] != NULL) {
        const volatile uint8_t *const lc_env_snapshot = lc_env[link_id];
        for (uint16_t i = 0; i < LC_ENV_SNAPSHOT_BYTES; ++i) {
            request->lc_env_at_start[i] = lc_env_snapshot[i];
        }
    }

    if ((lc_start_flags & (LC_START_FLAG_LC_ENV | LC_START_FLAG_LD_ACL_ENV)) ==
        (LC_START_FLAG_LC_ENV | LC_START_FLAG_LD_ACL_ENV)) {
        /*
         * 1. LMP_host_connection_reqを送信する。
         *    SwitchはこのPDUを受信するまでLMPを送信しない。
         *    ESP32から送信するとSwitchも同じPDUを返す。
         *    lmp_host_con_req_handlerが処理して初めてHCI Connection Requestがホストへ通知される。
         * 2. リンク制御タスクへ内部メッセージ0x0518を送信する。
         *    受信側のlc_op_hl_cnx_ind_handler(0x400293fc)は、リンク処理の内部状態が2または5の場合だけ処理する。
         *    リンク情報を通常のController処理へ登録した時点では5である。
         * 3. 送信後、ROMはLMP_acceptedを待つために内部状態を20へ変更する。
         *    後続タスクはSwitchのLMP_host_connection_reqを処理できるように内部状態を5へ変更する。
         * 4. ホスト側のGAP接続完了イベントを契機にするとイベントが通知されない場合がある。
         *    リンク情報を通常のController処理へ登録した時点で通知すればこの問題を回避できる。
         */
        {
            const ke_state_get_fn_t ke_state_get =
                (ke_state_get_fn_t)ROM_KE_STATE_GET_ADDR;
            const ke_msg_send_basic_fn_t send_basic =
                (ke_msg_send_basic_fn_t)ROM_KE_MSG_SEND_BASIC_ADDR;
            const uint16_t ke_task_id = (uint16_t)((link_id << 8) | 1u);
            request->host_connection_state_before = ke_state_get(ke_task_id);
            send_basic(LC_OP_HL_CNX_IND, ke_task_id, ke_task_id);
            request->host_connection_state_after = ke_state_get(ke_task_id);
            request->host_connection_request_posted = true;
        }
        /*
         * - LMP_setup_completeはこの処理では送信しない。送信するとROMがリンク処理の内部状態を1へ変更し、その後に届く
         *   SwitchのLMP_host_connection_reqが処理されない。
         * - Switchの要求をホストが受理した時点で、ROMがこのPDUを送信する。
         */
        request->result = ESP_OK;
        request->link_id = link_id;
        __asm__ __volatile__("memw" ::: "memory");
        request->state = ACL_HANDOFF_DONE;
        return;
    }
    /*
     * - ke_mallocが失敗した場合、ROM側はassert_errを呼んだあと状態を変更せずに呼び出し元へ制御を返す。
     * - lm_envの記録とhci_envの登録だけが変更された状態で維持されるため、受信要素を復元する前に、これらを変更前の値へ設定する。
     */
    lm_link_record[LM_LINK_STATE_OFFSET] = LM_LINK_STATE_PSCAN_RESERVED;
    lm_link_record[LM_LINK_ROLE_OFFSET] = 0;
    memset(lm_link_record, 0, 6);
    hci_env[16 + 7 * link_id] = 0;
    const uint32_t pscan_rx_start =
        (request->phase3_slot + PSCAN_RX_START_SLOTS) & ACL_CLOCK_MASK;
    const uint32_t pscan_rx_target =
        (request->phase3_slot + PSCAN_RX_TARGET_SLOTS) & ACL_CLOCK_MASK;
    const uint32_t pscan_rx_deadline =
        (request->phase3_slot + PSCAN_RX_DEADLINE_SLOTS) & ACL_CLOCK_MASK;
    *(uint32_t *)&pscan_env[0x08] = pscan_rx_start;
    *(uint32_t *)&pscan_env[0x0c] = pscan_rx_deadline;
    *(uint16_t *)&pscan_env[0x10] =
        (uint16_t)(UINT16_C(0x8840) | (rwip_priority[33] & 0x0f) |
                   ((request->clock_delta & 1) << 12));
    *(uint16_t *)&pscan_env[0x12] = PSCAN_RX_WINDOW_US;
    *(uint16_t *)&pscan_env[0x14] = request->rx_bitoff;
    pscan_env[0x16] = rwip_priority[32];
    *(uint32_t *)&pscan_env[0x30] = pscan_rx_target;
    for (uint8_t i = 0; i < 6; ++i) {
        pscan_env[0x34 + i] = request->target[5 - i];
    }
    memset(&pscan_env[0x3a], 0, 3);
    for (uint8_t i = 0; i < 5; ++i) {
        pscan_env[0x48 + i] = (uint8_t)(request->bch34 >> (8 * i));
    }
    pscan_env[0x50] = (uint8_t)(request->clock_delta & 1);

    // フェーズ3の送信後2500usだけフレームフォーマット3の受信を有効にし、受信成功時のcallbackだけが0x0404を生成する
    request->pscan_rx_active = true;
    request->pscan_rx_insert_now = r_ld_read_clock() & ACL_CLOCK_MASK;
    __asm__ __volatile__("memw" ::: "memory");
    const uint8_t insert_result = r_ea_elt_insert(pscan_env);
    if (insert_result == 0) {
        pscan_env[0x3f] = 1;
        __asm__ __volatile__("memw" ::: "memory");
    } else {
        request->pscan_rx_reject = UINT16_C(1) << 10;
        request->pscan_rx_active = false;
        phase2_restore_hook();
    }
    request->result = insert_result == 0 ? ESP_OK : ESP_ERR_INVALID_STATE;
    request->link_id = request->result == ESP_OK ? link_id : UINT8_MAX;
    __asm__ __volatile__("memw" ::: "memory");
    request->state = ACL_HANDOFF_DONE;
}

#define ACL_FOLLOWUP_FAST_MS 900
/*
 * - 900ms経過後は1ms間隔で待機し、ページスキャンによる再接続より後まで実行する。
 * - このタスクが登録した2つのフックを解除するため、指定時間まで終了しない。
 */
#define ACL_FOLLOWUP_MS 8000

lc_version_req_t g_lc_version_req;
volatile bool g_con_req_dispatched;

/* リンク情報を通常のController処理へ登録してからESP32のLMP_version_reqを送信するまでの待機時間。
 * SwitchのLMP_host_connection_reqが到着する時刻より後に設定する。 */
#define LC_VERSION_REQ_DELAY_US 200000
/* リンク情報を通常のController処理へ登録してからHCI Connection Requestを先行通知するまでの待機時間。
 * ROMが通知済みの場合はこの経路を実行しない。 */
#define CON_REQ_INJECT_DELAY_US 300000

static void acl_followup_task(void *task_arg)
{
    (void)task_arg;
    const uint8_t link_id =
        g_acl_handoff.link_id < 14 ? g_acl_handoff.link_id : 0;
    const int64_t followup_start_us = esp_timer_get_time();
    while (esp_timer_get_time() - followup_start_us <
           (int64_t)ACL_FOLLOWUP_MS * 1000) {
        /*
         * ROMがリンク処理の内部状態を20へ変更した場合、SwitchのLMP_host_connection_reqを処理できるように内部状態を5へ変更する。
         */
        if (!g_lc_idle_state_dispatched && lc_state[link_id] == 20) {
            g_lc_idle_state_dispatched = true;
            g_lc_idle_state_request.done = false;
            g_lc_idle_state_request.link = link_id;
            __asm__ __volatile__("memw" ::: "memory");
            (void)btdm_dispatch_work_to_controller(lc_set_idle_state_controller,
                                                   &g_lc_idle_state_request, false);
        }
        /*
         * ESP32からLMPを送信するとリンク処理の内部状態が5から別の値へ変化する。
         * LMP_host_connection_reqは内部状態が5の場合にのみ処理される。
         * (`lmp_host_con_req_handler` 0x4002b402)が処理するため、Switchの要求を処理するまで送信しない。
         */
        if (!g_lc_version_req.requested &&
            esp_timer_get_time() - followup_start_us >= LC_VERSION_REQ_DELAY_US) {
            g_lc_version_req.link = link_id;
            __asm__ __volatile__("memw" ::: "memory");
            g_lc_version_req.requested = true;
            (void)btdm_dispatch_work_to_controller(lc_version_req_controller,
                                                   &g_lc_version_req, false);
        }
        /*
         * 1. SwitchのLMP_host_connection_reqを処理してリンク処理の内部状態が5になった後に、ホストへHCI Connection Requestを通知する。
         * 2. 通知しない場合、Switchは35msでLMP_host_connection_reqの処理を終了する。
         *    ホストがAccept Connection Requestを返すとControllerがLMP_acceptedを送信するため、Switchはその応答を待機している。
         * 3. ROMがHCI Connection Requestを通知済みの場合は重複通知しない。
         *    通知済みの場合、`lmp_host_con_req_handler`が0x4002b461でlc_env+86へ1を書き込む。
         *    重複通知すると、ホストが同じSwitchからの着信を2回受信する。
         */
        if (g_lc_version_req.requested && !g_lc_version_req.con_req_sent &&
            !g_con_req_dispatched &&
            (lc_env[link_id] == NULL ||
             ((const volatile uint8_t *)lc_env[link_id])[86] == 0) &&
            esp_timer_get_time() - followup_start_us >= CON_REQ_INJECT_DELAY_US) {
            g_con_req_dispatched = true;
            (void)btdm_dispatch_work_to_controller(lc_con_req_controller,
                                                   &g_lc_version_req, false);
        }
        if (esp_timer_get_time() - followup_start_us >=
            (int64_t)ACL_FOLLOWUP_FAST_MS * 1000) {
            vTaskDelay(1);
        }
    }

    lmp_not_accepted_hook_remove();
    packet_type_hook_remove();
    vTaskDelete(NULL);
}

esp_err_t switch1_acl_handoff_start(const uint8_t bd_addr[6], uint64_t bch34,
                                    uint8_t lt_addr, uint32_t clock_delta,
                                    uint32_t rx_clk, uint16_t rx_bitoff,
                                    uint32_t phase3_slot)
{
    if (bd_addr == NULL || lt_addr == 0 || lt_addr > 7 ||
        rx_bitoff >= 625 || phase3_slot > ACL_CLOCK_MASK ||
        g_acl_handoff.state == ACL_HANDOFF_PENDING) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(g_acl_handoff.target, bd_addr, sizeof(g_acl_handoff.target));
    g_acl_handoff.bch34 = bch34;
    g_acl_handoff.lt_addr = lt_addr;
    g_acl_handoff.clock_delta = clock_delta & UINT32_C(0x07ffffff);
    g_acl_handoff.rx_clk = rx_clk & UINT32_C(0x07ffffff);
    g_acl_handoff.phase3_slot = phase3_slot;
    g_acl_handoff.rx_bitoff = rx_bitoff;
    g_acl_handoff.link_id = 0xff;
    g_acl_handoff.result = ESP_ERR_TIMEOUT;
    g_acl_handoff.pscan_rx_active = false;
    g_acl_handoff.pscan_rx_dm1 = false;
    g_acl_handoff.pscan_rx_reported = false;
    g_acl_handoff.pscan_rx_reject = 0;
    g_acl_handoff.pscan_rx_status = 0;
    g_acl_handoff.pscan_rx_header = 0;
    g_acl_handoff.pscan_rx_pheader = 0;
    g_acl_handoff.pscan_rx_insert_now = 0;
    g_acl_handoff.state = ACL_HANDOFF_PENDING;
    memset((void *)&g_lc_version_req, 0, sizeof(g_lc_version_req));
    g_lmp_not_accepted_suppressed = 0;
    g_lmp_not_accepted_forwarded = 0;
    g_con_req_dispatched = false;
    g_lc_idle_state_dispatched = false;
    g_packet_type_suppressed = 0;
    __asm__ __volatile__("memw" ::: "memory");
    xTaskCreate(acl_followup_task, "aclfollowup", 2560, NULL, 5, NULL);

    /*
     * - falseはControllerのキューが満杯の場合に直ちに失敗を返す指定である。
     * - callbackは待機やHCIコマンドを実行せず、フレームフォーマット3のスケジュール要素だけを登録する。
     */
    if (btdm_dispatch_work_to_controller(acl_handoff_controller,
                                         &g_acl_handoff, false) != 0) {
        g_acl_handoff.result = ESP_FAIL;
        __asm__ __volatile__("memw" ::: "memory");
        g_acl_handoff.state = ACL_HANDOFF_DONE;
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool switch1_lc_start_status(uint8_t *out_lc_state, uint8_t *out_flags)
{
    if (!g_acl_handoff.lc_start_called) {
        return false;
    }
    if (out_lc_state != NULL) {
        *out_lc_state = g_acl_handoff.lc_state_after;
    }
    if (out_flags != NULL) {
        *out_flags = g_acl_handoff.lc_start_flags;
    }
    return true;
}

/*
 * 1. r_lc_detach(0x4002037c)は(link_id, reason)を受け取り、リンク処理の内部状態を25へ変更してLMP_detachを送信する。
 * 2. 内部状態が26、27の順に変化した後にリンクを解放し、ホストへHCI Disconnection Completeを通知する。
 */
#define IP_FUNCS_LC_DETACH_INDEX (UINT16_C(0x2cc) / sizeof(void *))
#define LC_DETACH_REASON_REMOTE_USER 19

typedef void (*lc_detach_fn_t)(uint8_t link_id, uint8_t reason);

static volatile bool s_acl_detach_requested;

static void acl_detach_controller(void *detach_arg)
{
    const uint8_t link_id = (uint8_t)(uintptr_t)detach_arg;
    void **const rom_ip_funcs = *(void ***)PHASE2_ROM_IP_FUNCS_PTR_ADDR;
    if (rom_ip_funcs == NULL || link_id >= ACL_LINK_COUNT || lc_env[link_id] == NULL) {
        return;
    }
    const lc_detach_fn_t lc_detach =
        (lc_detach_fn_t)rom_ip_funcs[IP_FUNCS_LC_DETACH_INDEX];
    if (lc_detach == NULL) {
        return;
    }
    lc_detach(link_id, LC_DETACH_REASON_REMOTE_USER);
    /*
     * - s_acl_detach_requestedがtrueに設定されている間は1サイクルにつき1回しか要求できない。
     */
    __asm__ __volatile__("memw" ::: "memory");
    s_acl_detach_requested = false;
}

uint8_t switch1_acl_lc_state(void)
{
    if (!g_acl_handoff.lc_start_called ||
        g_acl_handoff.link_id >= ACL_LINK_COUNT) {
        return 0xff;
    }
    return lc_state[g_acl_handoff.link_id];
}

/*
 * - r_lc_release(0x4001f8a8)は、LMP_detachの往復を待たずにESP32側のリンクを解放する。
 * - Switchがそのリンクの受信を停止して切断が完了しない場合に使用する。
 * - 解放するとROMがホストへHCI Disconnection Completeを通知し、ホスト側スタックの解放処理も実行される。
 */
#define ROM_LC_RELEASE_ADDR UINT32_C(0x4001f8a8)

typedef void (*lc_release_fn_t)(uint16_t dest_id);

static void acl_release_controller(void *release_arg)
{
    const uint8_t link_id = (uint8_t)(uintptr_t)release_arg;
    if (link_id >= ACL_LINK_COUNT || lc_env[link_id] == NULL) {
        return;
    }
    const lc_release_fn_t lc_release = (lc_release_fn_t)ROM_LC_RELEASE_ADDR;
    lc_release((uint16_t)(((uint16_t)link_id << 8) | KE_TASK_LC));
}

esp_err_t switch1_acl_release(void)
{
    if (!g_acl_handoff.lc_start_called ||
        g_acl_handoff.link_id >= ACL_LINK_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t link_id = g_acl_handoff.link_id;
    if (lc_env[link_id] == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (btdm_dispatch_work_to_controller(
            acl_release_controller, (void *)(uintptr_t)link_id, false) != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/*
 * - 切断要求を処理できない場合は、リンク処理の内部状態を5へ変更する。
 * - r_lc_detachはLMP処理中の場合、状態を変更せずに呼び出し元へ制御を返す。
 */
esp_err_t switch1_acl_force_idle(void)
{
    if (!g_acl_handoff.lc_start_called ||
        g_acl_handoff.link_id >= ACL_LINK_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (lc_env[g_acl_handoff.link_id] == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    g_lc_idle_state_request.done = false;
    g_lc_idle_state_request.link = g_acl_handoff.link_id;
    __asm__ __volatile__("memw" ::: "memory");
    if (btdm_dispatch_work_to_controller(lc_set_idle_state_controller,
                                         &g_lc_idle_state_request, false) != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t switch1_acl_detach(void)
{
    if (!g_acl_handoff.lc_start_called ||
        g_acl_handoff.link_id >= ACL_LINK_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_acl_detach_requested) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t link_id = g_acl_handoff.link_id;
    if (lc_env[link_id] == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /*
     * 1. `r_lc_detach`(0x4002037c)はリンク処理の内部状態を25へ変更してLMP_detachを送信する。
     * 2. 送信完了時に内部状態が26へ変わり、`r_lc_release`(0x4001f8a8)の実行時に27へ変わる。
     * 3. 内部状態が25~27の場合は切断処理の完了を待機する。
     *    呼び出し側が切断失敗と判断して再起動しないよう、成功を返す。
     */
    if (lc_state[link_id] >= 25 && lc_state[link_id] <= 27) {
        return ESP_OK;
    }
    s_acl_detach_requested = true;
    // btdm_dispatch_work_to_controllerは成功時に0を返す。
    if (btdm_dispatch_work_to_controller(acl_detach_controller,
                                         (void *)(uintptr_t)link_id, false) != 0) {
        s_acl_detach_requested = false;
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool switch1_acl_handoff_status(esp_err_t *out_result, uint8_t *out_link_id)
{
    if (g_acl_handoff.state != ACL_HANDOFF_DONE) {
        return false;
    }
    __asm__ __volatile__("memw" ::: "memory");
    if (out_result != NULL) {
        *out_result = g_acl_handoff.result;
    }
    if (out_link_id != NULL) {
        *out_link_id = g_acl_handoff.link_id;
    }
    if (!g_acl_handoff.pscan_rx_reported &&
        g_acl_handoff.pscan_rx_reject != 0) {
        g_acl_handoff.pscan_rx_reported = true;
        ESP_LOGI(TAG,
                 "Page-scan format 3 receive registration rejected: reason=%03x insert_clock=%08x phase3_clock=%08x",
                 g_acl_handoff.pscan_rx_reject,
                 (unsigned)g_acl_handoff.pscan_rx_insert_now,
                 (unsigned)g_acl_handoff.phase3_slot);
    } else if (g_acl_handoff.pscan_rx_header != 0 &&
               !g_acl_handoff.pscan_rx_reported) {
        g_acl_handoff.pscan_rx_reported = true;
        ESP_LOGI(TAG,
                 "Page-scan format 3 response received: status=%04x header=%03x "
                 "pheader=%04x DM1=%u",
                 g_acl_handoff.pscan_rx_status, g_acl_handoff.pscan_rx_header,
                 g_acl_handoff.pscan_rx_pheader, g_acl_handoff.pscan_rx_dm1);
    }
    return true;
}
