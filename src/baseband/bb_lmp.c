/*
 * このリンクはROMの通常の接続手順を経由しないため、一部のLMPメッセージが送信されない。
 * 必要な内部メッセージとLMP処理をリンクコントローラーへ直接要求する。
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


void lc_con_req_controller(void *arg)
{
    lc_version_req_t *const request = (lc_version_req_t *)arg;
    void **const ip_funcs = *(void ***)PHASE2_ROM_IP_FUNCS_PTR_ADDR;
    const uint8_t link = request->link;
    const uint16_t lc_task_id = (uint16_t)(((uint16_t)link << 8) | KE_TASK_LC);
    if (ip_funcs == NULL || lc_env[link] == NULL) {
        return;
    }
    const ke_msg_alloc_fn_t ke_msg_alloc =
        (ke_msg_alloc_fn_t)ROM_KE_MSG_ALLOC_ADDR;
    const hci_send_2_host_fn_t hci_send_2_host =
        (hci_send_2_host_fn_t)ip_funcs[IP_FUNCS_HCI_SEND_2_HOST_INDEX];
    uint8_t *const event_param = (uint8_t *)ke_msg_alloc(
        KE_MSG_HCI_EVENT, link, KE_SRC_HCI_CON_REQ, HCI_CON_REQ_PARAM_LEN);
    if (event_param == NULL || hci_send_2_host == NULL) {
        return;
    }
    volatile uint8_t *const lc_env_ptr = lc_env[link];
    lc_env_ptr[850] = LC_STATE_PERIPHERAL_IDLE;
    for (uint8_t i = 0; i < 6; ++i) {
        event_param[i] = lc_env_ptr[130 + i];
    }
    event_param[6] = lc_env_ptr[54];
    event_param[7] = lc_env_ptr[55];
    event_param[8] = lc_env_ptr[56];
    event_param[9] = 1;
    __asm__ __volatile__("memw" ::: "memory");
    hci_send_2_host(event_param);
    /*
     * lc_env+0x56は、ホストがこのリンクを管理対象として登録したかを示すフラグである。
     * HCI Connection Requestイベントをこの関数で生成するため、ROMの登録処理を経由せず、このフィールドには初期値の0が保持される。
     *
     * - 値が0であれば、名前取得完了時にr_lc_rem_name_cont(0x4001f290)がこのフィールドを読み出し、HCI Connection Requestがホストへ登録されていない一時リンクと判定してr_lc_detachを呼び出す。
     * - 名前要求の2ms後にリンクが解放される。
     */
    lc_env_ptr[86] = 1;
    const ke_state_set_fn_t ke_state_set =
        (ke_state_set_fn_t)ROM_KE_STATE_SET_ADDR;
    ke_state_set(lc_task_id, LC_STATE_WAIT_ACCEPT);
    request->con_req_sent = true;
}


lc_idle_state_request_t g_lc_idle_state_request;
volatile bool g_lc_idle_state_dispatched;


void lc_set_idle_state_controller(void *arg)
{
    lc_idle_state_request_t *const request = (lc_idle_state_request_t *)arg;
    const uint16_t lc_task_id =
        (uint16_t)(((uint16_t)request->link << 8) | KE_TASK_LC);
    const ke_state_get_fn_t ke_state_get =
        (ke_state_get_fn_t)ROM_KE_STATE_GET_ADDR;
    const ke_state_set_fn_t ke_state_set =
        (ke_state_set_fn_t)ROM_KE_STATE_SET_ADDR;
    request->state_before = ke_state_get(lc_task_id);
    ke_state_set(lc_task_id, LC_STATE_PERIPHERAL_IDLE);
    request->state_after = ke_state_get(lc_task_id);
    __asm__ __volatile__("memw" ::: "memory");
    request->done = true;
}

void lc_version_req_controller(void *arg)
{
    lc_version_req_t *const request = (lc_version_req_t *)arg;
    void **const ip_funcs = *(void ***)PHASE2_ROM_IP_FUNCS_PTR_ADDR;
    const uint8_t link = request->link;
    const uint16_t lc_task_id = (uint16_t)(((uint16_t)link << 8) | KE_TASK_LC);
    request->dest_id = lc_task_id;
    if (ip_funcs != NULL) {
        request->fn_buf_alloc =
            (uint32_t)ip_funcs[IP_FUNCS_BUF_LMP_TX_ALLOC_INDEX];
        request->fn_send_lmp = (uint32_t)ip_funcs[IP_FUNCS_LC_SEND_LMP_INDEX];
        request->fn_send_pdu_vers_req =
            (uint32_t)ip_funcs[IP_FUNCS_LC_SEND_PDU_VERS_REQ_INDEX];
        request->fn_start_lmp_to =
            (uint32_t)ip_funcs[IP_FUNCS_LC_START_LMP_TO_INDEX];
        request->fn_lc_version =
            (uint32_t)ip_funcs[IP_FUNCS_LC_VERSION_INDEX];
        request->fn_acl_lmp_tx =
            (uint32_t)ip_funcs[IP_FUNCS_LD_ACL_LMP_TX_INDEX];
    }
    const ke_state_get_fn_t ke_state_get =
        (ke_state_get_fn_t)ROM_KE_STATE_GET_ADDR;
    request->ke_state_before = ke_state_get(lc_task_id);
    request->lc_state_before = lc_state[link];
    if (lc_env[link] != NULL) {
        const volatile uint8_t *const lc_env_ptr = lc_env[link];
        request->lc_env_60 = lc_env_ptr[60];
    }
    const ke_msg_send_basic_fn_t ke_msg_send_basic =
        (ke_msg_send_basic_fn_t)ROM_KE_MSG_SEND_BASIC_ADDR;
    /*
     * 0x516(timing accuracy)は送信しない。
     * lc_op_timacc_ind_handler(0x400292e4)は処理末尾で0x518を同じリンクのLCタスクへ無条件に送信するため、ここからLMP_host_connection_reqが送信される。
     * ESP32がACLリンクのBluetooth周辺側としてこの要求を送信すると、ホストへHCI Connection Requestを通知せず、Connection Completeだけを通知するため、L2CAPがリンクを切断する。
     */
    ke_msg_send_basic(KE_MSG_LC_OP_FEAT_IND, lc_task_id, lc_task_id);
    ke_msg_send_basic(KE_MSG_LC_OP_VERS_IND, lc_task_id, lc_task_id);
    request->ke_state_after = ke_state_get(lc_task_id);
    request->lc_state_after = lc_state[link];
    if (lc_env[link] != NULL) {
        const volatile uint8_t *const lc_env_ptr = lc_env[link];
        request->lc_env_850 = lc_env_ptr[850];
    }
    __asm__ __volatile__("memw" ::: "memory");
    request->done = true;
}

/*
 * Switchから受信する00 04 00 66に対し、ROMはLMP_not_accepted(opcode 0、理由0x19)を送信する。
 * この応答だけを抑制する。
 *
 * lc_lmp_rx_ind_handler(0x40027d50)は解釈できなかったPDUについてr_ip_funcs_p+0x214を呼び出す。
 * 引数は(リンク番号、拒否対象opcode、理由、TID)である。
 * この関数ポインターをフックし、opcode 0であればLMP_not_acceptedの送信処理を抑制する。
 */
#define IP_FUNCS_LC_SEND_PDU_NOT_ACCEPTED_INDEX (UINT16_C(0x214) / sizeof(void *))

/*
 * ROMは接続完了の直後にパケット種別の交換を開始する。
 * r_lc_con_cmpが0x517を同じリンクのLCタスクへ送り、lc_op_pt_ind_handlerがr_lc_packet_typeを呼び出す。
 *
 * ESP32から開始するパケット種別の変更がSwitchのLMP_packet_type_table_reqと重なると、Transaction Collisionにより拒否される。
 * パケット種別の変更が完了しなければリンク処理の内部状態が1にならず、ホストが送信するRead Remote Extended Featuresがstatus 0x02で失敗し、リンクが切断される。
 */
#define IP_FUNCS_LC_PACKET_TYPE_INDEX (UINT16_C(0x45c) / sizeof(void *))
typedef void (*lc_packet_type_fn_t)(uint32_t link);

static lc_packet_type_fn_t s_packet_type_original;
volatile uint16_t g_packet_type_suppressed;

static void IRAM_ATTR lc_packet_type_hook(uint32_t link)
{
    (void)link;
    g_packet_type_suppressed++;
}

void packet_type_hook_install(void)
{
    void **const ip_funcs = *(void ***)PHASE2_ROM_IP_FUNCS_PTR_ADDR;
    if (ip_funcs == NULL || s_packet_type_original != NULL) {
        return;
    }
    s_packet_type_original =
        (lc_packet_type_fn_t)ip_funcs[IP_FUNCS_LC_PACKET_TYPE_INDEX];
    ip_funcs[IP_FUNCS_LC_PACKET_TYPE_INDEX] = (void *)lc_packet_type_hook;
    __asm__ __volatile__("memw" ::: "memory");
}

void packet_type_hook_remove(void)
{
    void **const ip_funcs = *(void ***)PHASE2_ROM_IP_FUNCS_PTR_ADDR;
    if (ip_funcs == NULL || s_packet_type_original == NULL) {
        return;
    }
    if (ip_funcs[IP_FUNCS_LC_PACKET_TYPE_INDEX] ==
        (void *)lc_packet_type_hook) {
        ip_funcs[IP_FUNCS_LC_PACKET_TYPE_INDEX] =
            (void *)s_packet_type_original;
    }
    __asm__ __volatile__("memw" ::: "memory");
    s_packet_type_original = NULL;
}

typedef void (*lc_send_pdu_not_accepted_fn_t)(uint32_t link, uint32_t opcode,
                                              uint32_t reason, uint32_t tid);

static lc_send_pdu_not_accepted_fn_t s_not_accepted_original;
volatile uint16_t g_lmp_not_accepted_suppressed;
volatile uint16_t g_lmp_not_accepted_forwarded;

static void IRAM_ATTR lc_send_pdu_not_accepted_hook(uint32_t link,
                                                    uint32_t opcode,
                                                    uint32_t reason,
                                                    uint32_t tid)
{
    /*
     * opcode 0はSwitchから受信する00 04 00 66に対する拒否である。
     * opcode 0だけを抑制し、他のopcodeへの拒否はROMの処理へ渡す。
     */
    if ((opcode & 0xff) == 0) {
        g_lmp_not_accepted_suppressed++;
        return;
    }
    g_lmp_not_accepted_forwarded++;
    if (s_not_accepted_original != NULL) {
        s_not_accepted_original(link, opcode, reason, tid);
    }
}

void lmp_not_accepted_hook_install(void)
{
    void **const ip_funcs = *(void ***)PHASE2_ROM_IP_FUNCS_PTR_ADDR;
    if (ip_funcs == NULL || s_not_accepted_original != NULL) {
        return;
    }
    s_not_accepted_original =
        (lc_send_pdu_not_accepted_fn_t)
            ip_funcs[IP_FUNCS_LC_SEND_PDU_NOT_ACCEPTED_INDEX];
    ip_funcs[IP_FUNCS_LC_SEND_PDU_NOT_ACCEPTED_INDEX] =
        (void *)lc_send_pdu_not_accepted_hook;
    __asm__ __volatile__("memw" ::: "memory");
}

void lmp_not_accepted_hook_remove(void)
{
    void **const ip_funcs = *(void ***)PHASE2_ROM_IP_FUNCS_PTR_ADDR;
    if (ip_funcs == NULL || s_not_accepted_original == NULL) {
        return;
    }
    if (ip_funcs[IP_FUNCS_LC_SEND_PDU_NOT_ACCEPTED_INDEX] ==
        (void *)lc_send_pdu_not_accepted_hook) {
        ip_funcs[IP_FUNCS_LC_SEND_PDU_NOT_ACCEPTED_INDEX] =
            (void *)s_not_accepted_original;
    }
    __asm__ __volatile__("memw" ::: "memory");
    s_not_accepted_original = NULL;
}
