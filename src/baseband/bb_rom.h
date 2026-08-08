#ifndef BB_ROM_H
#define BB_ROM_H

#include <stdbool.h>
#include <stdint.h>

#include "stack/bt_types.h"


/*
 * ベースバンドハードウェアとCPUが共有するメモリである。
 * ROMシンボルではem_を接頭辞として使用する。
 *
 * - 0x3ffb0000から32KBの領域である。
 * - 以下ではこの領域を共有メモリと呼ぶ。
 */
#define CSB_CS_ADDR UINT32_C(0x3ffb22b6)
#define CS_BDADDR_OFFSET UINT32_C(0x0A)
/*
 * LT_ADDRはこのワードのbit10:8に格納される。
 * r_ld_csb_rx_startとr_ld_csb_tx_startがこのワードへLT_ADDRを書き込む。
 */
#define CS_LTADDR_OFFSET UINT32_C(0x08)
#define CS_LTADDR_MASK UINT16_C(0x0700)
/*
 * 同じワードのbit12は白色化を無効にする。
 * ld_acl_test_mode_update(0x40032050)の分岐表では、白色化を使用しない試験項目だけがこのビットを設定して本体処理を実行する。
 * r_ld_csb_tx_enはこのビットをクリアするが、r_ld_csb_rx_startは変更しないため、受信側ではこのビットを明示的に設定できる。
 */
#define CS_LINKCNTL_WHITENING_DSB UINT16_C(0x1000)
#define CS_BCH_OFFSET UINT32_C(0x10)
/*
 * BCHの3番目のワードへ受信側だけが書き込む上位ビットである。
 * r_ld_csb_rx_start(0x4003972c)の0x4003985a~0x4003986fでは、(parity上位2ビット) | 0xFF40を書き込む。
 */
#define CS_BCH_RX_FLAGS UINT16_C(0xFF40)
#define CS_HOPCNTL_OFFSET UINT32_C(0x16)
#define CS_TXDESC_OFFSET UINT32_C(0x1C)
#define CS_HOPCNTL_FH_EN UINT16_C(0x8000)
#define CS_HOPCNTL_CHANNEL_MASK UINT16_C(0x7F00)
#define EM_BASE UINT32_C(0x3ffb0000)
#define EM_SIZE UINT16_C(0x8000)
#define CSB_DESC_SIZE UINT16_C(10)

/*
 * Bluetoothコアのネイティブクロックのサンプルレジスタである。
 * 根拠はr_ld_read_clock(0x4003c9e4)である。
 * bit31を1にしたレジスタ値を同じレジスタへ書き込むとサンプルを開始し、完了時にハードウェアがbit31を0にする。下位28ビットがCLKN(312.5us単位)である。
 */
#define BT_CLKN_SAMPLE_REG UINT32_C(0x3ff7101c)
#define BT_CLKN_SAMPLE_REQ UINT32_C(0x80000000)
#define BT_CLKN_VALUE_MASK UINT32_C(0x0fffffff)
/* サンプルは数百nsで完了する。完了待ちの反復回数に上限を設ける。 */
#define BT_CLKN_POLL_LIMIT 64

#define CSB_LT_ADDR UINT8_C(1)
#define CSB_SUPERVISION_TIMEOUT UINT16_C(0x0640)
/*
 * CSB受信の監視タイムアウトは送信側と分離する。
 * CSB_RX_SUP_TIMEOUTの設定値に相当する時間が経過するとld_csb_rx_schedが受信のスケジュール要素を解放する。
 * 送信側と共用する0x0640は1000msで1回の監視(1200ms)より短いため、2560msに相当する監視タイムアウト値を使用する。
 */
#define CSB_RX_SUPERVISION_TIMEOUT UINT16_C(0x1000)
#define CSB_EDR_EXCLUSION_MASK                                               \
    (HCI_PKT_TYPES_MASK_NO_2_DH1 | HCI_PKT_TYPES_MASK_NO_3_DH1 |           \
     HCI_PKT_TYPES_MASK_NO_2_DH3 | HCI_PKT_TYPES_MASK_NO_3_DH3 |           \
     HCI_PKT_TYPES_MASK_NO_2_DH5 | HCI_PKT_TYPES_MASK_NO_3_DH5)
/*
 * ウェイクアップコードを反復したペイロードのDH5経路で使用するパケット種別マスクである。
 * DH5だけを許可し、HCI_SET_CLBが拒否されれば、DH3とDH1を追加して1回だけ再試行する。
 */
#define CSB_PACKET_MASK_DH5 (CSB_EDR_EXCLUSION_MASK | HCI_PKT_TYPES_MASK_DH5)
#define CSB_PACKET_MASK_DH5_RETRY                                            \
    (CSB_PACKET_MASK_DH5 | HCI_PKT_TYPES_MASK_DH3 | HCI_PKT_TYPES_MASK_DH1)
/*
 * DH3経路では3スロットのため送信間隔の下限が3.75msになる。
 * 本文の最大長は183バイトである。
 */
#define CSB_PACKET_MASK_DH3 (CSB_EDR_EXCLUSION_MASK | HCI_PKT_TYPES_MASK_DH3)
#define CSB_DH3_BODY_MAX UINT16_C(183)
/*
 * HCI_WRITE_CLB_DATAのパラメータ長は1バイトで、コマンド全体の上限は255バイトである。
 * LT_ADDR、Fragment、Data_Lengthの3バイトを除いた252バイトが、1回のfragmentで送信できる最大長になる。
 */
#define CSB_FRAGMENT_MAX UINT16_C(252)


/*
 * CSB受信を通知するHCIイベントである。
 * bluedroidはこのイベントを処理しないため、イベントハンドラーで受信データを読み出す。
 */
#define HCI_CSB_RECEIVE_EVT UINT8_C(0x51)

extern uint8_t r_ld_pscan_stop(void);
extern uint8_t r_ld_csb_rx_stop(void);
extern uint32_t r_ld_read_clock(void);
extern uint8_t r_ea_elt_remove(void *element);
extern uint8_t r_ea_elt_insert(void *element);
typedef void (*controller_work_t)(void *arg);
extern int btdm_dispatch_work_to_controller(controller_work_t callback,
                                             void *arg, bool blocking);
#define ACL_CLOCK_MASK UINT32_C(0x07ffffff)

/*
 * CSB受信のスケジュール要素へのポインタである。
 * ROMシンボル名はld_csb_rx_envである。
 *
 * - CSB送信側の4バイト手前に位置する。
 * - r_ld_csb_rx_start(0x4003972c)は、確保した要素のポインタをCSB_RX_ENV_ADDRが指す領域へ格納する。
 * - CSB_RX_ENV_ADDRの既存ポインタ値が0以外であれば12(Command Disallowed)を返す。
 * - このポインタは停止時だけでなく、監視タイムアウトの満了時にも0になる。
 * - 根拠はld_csb_rx_sched(0x400391f0)である。
 * - CSB_RX_ENV_ADDRが0でも受信を開始していないとは限らない。
 */
#define CSB_RX_ENV_ADDR UINT32_C(0x3ffb8278)


/*
 * - Page ScanでFHSを受信した直後にControllerが実行する通常経路を、ウェイクアップのフェーズ3に合わせて開始する。
 * - 対応するROM関数: ld_pscan_frm_cbk、lm_page_scan_end_ind_handler
 */
#define ACL_LINK_COUNT 7
#define ACL_CS_BASE UINT32_C(0x3ffb1dee)
#define ACL_CS_STRIDE UINT32_C(0x66)
#define ACL_CS_FORMAT_PERIPHERAL UINT16_C(3)
#define PSCAN_RX_START_SLOTS UINT32_C(1)
#define PSCAN_RX_TARGET_SLOTS UINT32_C(3)
#define PSCAN_RX_DEADLINE_SLOTS UINT32_C(5)
#define PSCAN_RX_WINDOW_US UINT16_C(2500)

extern uint8_t hci_env[];
/*
 * 名前要求の処理を置換する必要はない。
 * 切断原因だったlc_env+0x56を設定する処理をlc_con_req_controllerへ追加したため、名前要求のフックは使用しない。
 */
extern void *lc_env[];
extern uint8_t lc_state[];
extern void *ld_acl_env[];
/* ウェイクアップ応答からACLリンクを登録する処理では、ESP32がBluetooth周辺側となるACLリンクを生成する。 */
#define ACL_ROLE_PERIPHERAL UINT8_C(1)

typedef struct {
    uint8_t addr[6];
} lc_bd_addr_t;
/**
 * r_lc_start(0x4001ca28)はESP32がBluetooth周辺側となるACLリンクに必要な構造体を生成する。
 * 関数末尾でr_ld_acl_startを呼び出すため、呼び出し元から先にr_ld_acl_startを呼び出してはならない。
 * 先に呼び出すとld_acl_env[link]が設定され、r_lc_start内部の呼び出しがリンク処理の内部状態12で失敗する。
 *
 * 引数は7個である。BD_ADDRは6バイトの値渡しでa4とa5を使用するため、後ろの2個はスタックに配置される。
 *
 * - link_id: a2。リンク番号。7未満でなければassertが発生する
 * - reason: a3。0=中央側、1=ページスキャン成立、2=Remote Name Request
 * - bd_addr: a4,a5。接続先のBD_ADDR。FHSと同じ下位バイト先頭の並び
 * - packet_type: a6。reasonが1であれば参照されない
 * - allow_role_switch: a7。reasonが1であれば参照されない
 * - class_of_device: SP+0。下位3バイトをlc_env+54~56へ格納する
 * - link_policy: SP+4。既定のリンクポリシー設定をlc_env+40へ格納する
 *
 * reasonが1であれば、LMPを送信せずリンク処理の内部状態を5へ変更して復帰する。
 * HCI Connection Completeは、後続するLMPを処理してr_lc_con_cmpが呼び出された時点で通知される。
 */
typedef void (*lc_start_fn_t)(uint8_t link_id, uint8_t reason,
                              lc_bd_addr_t bd_addr, uint16_t packet_type,
                              uint8_t allow_role_switch,
                              uint32_t class_of_device, uint32_t link_policy);
/**
 * r_hci_bt_acl_bdaddr_register(ROM 0x40018900)は、hci_env(0x3ffb9350)の16+7*linkを1に設定し、17+7*linkへBD_ADDRを6バイト複製する。
 * 16 + 7*linkが既に0以外であればassertが発生するため、呼び出し前に必ず0を確認する。
 */
typedef void (*hci_bdaddr_register_fn_t)(uint8_t link_id,
                                         const uint8_t *bd_addr);
/*
 * ip_funcsはr_ip_funcs_p(0x3ffae70c)が指す関数ポインタ表である。
 * ESP-IDFは+0x2b4をr_lc_start_hackへ置き換え、ROM本体の呼び出し後にライブラリ側の配列を初期化する。
 * ROMアドレスを直接呼び出すとこの初期化を実行しないため、必ず表を介して呼び出す。
 */
#define IP_FUNCS_HCI_BDADDR_REGISTER_INDEX (UINT16_C(0x174) / sizeof(void *))
#define IP_FUNCS_LC_START_INDEX (UINT16_C(0x2b4) / sizeof(void *))
/* lm_envにあるリンクごとの記録。先頭オフセットは54で、1リンクあたり9バイトである。 */
#define LM_LINK_RECORD_BASE 54
#define LM_LINK_RECORD_STRIDE 9
#define LM_LINK_STATE_OFFSET 6
#define LM_LINK_ROLE_OFFSET 7
#define LM_LINK_STATE_PSCAN_RESERVED UINT8_C(2)
#define LM_LINK_STATE_CONNECTED UINT8_C(3)
/*
 * HCI Write Default Link Policy Settingsの保存先である。
 * ROMの呼び出し元はこの位置をl8uiで1バイトだけ読み出してr_lc_startへ渡す。
 */
#define LM_ENV_DEFAULT_LINK_POLICY 42
#define LC_START_REASON_PAGE_SCAN UINT8_C(1)
/* control structureは102バイト(51ワード)。 */
#define ACL_CS_SNAPSHOT_WORDS 51
/* リンク構造体を保存するバイト数。r_ld_acl_startが確保するサイズと同じである。 */
#define ACL_ENV_SNAPSHOT_BYTES 252
/*
 * lc_envを保存するバイト数である。
 * r_lc_startがr_ke_malloc(0x354, 0)で確保する領域(0x4001ca74)と同じである。
 * 受信したLMPを処理すると、接続先のバージョンと機能ビットがこの領域へ格納される。
 */
#define LC_ENV_SNAPSHOT_BYTES 852
extern void *llc_env[];
extern uint8_t lm_env[];
extern uint8_t ld_env[];
extern uint8_t *volatile ld_pscan_env;
extern uint8_t *volatile ld_csb_rx_env;
extern uint8_t rwip_priority[];


enum {
    ACL_HANDOFF_IDLE,
    ACL_HANDOFF_PENDING,
    ACL_HANDOFF_DONE,
};

enum {
    ACL_PREPARE_IDLE,
    ACL_PREPARE_PENDING,
    ACL_PREPARE_DONE,
};

typedef struct {
    uint8_t target[6];
    uint64_t bch34;
    uint32_t clock_delta;
    uint32_t rx_clk;
    uint32_t phase3_slot;
    uint16_t rx_bitoff;
    uint8_t lt_addr;
    volatile uint8_t prepare_state;
    volatile uint8_t reserved_link;
    volatile esp_err_t prepare_result;
    volatile uint8_t state;
    volatile uint8_t link_id;
    volatile esp_err_t result;
    volatile bool pscan_rx_active;
    volatile bool pscan_rx_dm1;
    volatile bool pscan_rx_reported;
    volatile uint16_t pscan_rx_reject;
    volatile uint16_t pscan_rx_status;
    volatile uint16_t pscan_rx_header;
    volatile uint16_t pscan_rx_pheader;
    volatile uint32_t pscan_rx_insert_now;
    /*
     * - r_lc_start()呼び出し後のLC状態。ACLリンク生成完了後に出力する。
     * - r_lc_startは戻り値を持たないため、生成された構造体の有無でr_lc_startの実行結果を判定する。
     */
    volatile uint8_t lc_state_after;
    volatile uint8_t lc_start_flags;
    /*
     * - LMP_host_connection_reqを要求する内部メッセージ0x518をリンク制御タスクへ送信したかどうか。
     * - 送信時点のリンク処理の内部状態。2または5であればROMが処理する。
     */
    volatile bool host_connection_request_posted;
    volatile uint8_t host_connection_state_before;
    volatile uint8_t host_connection_state_after;
    volatile bool lc_start_called;
    /* - control structure +0x60をフェーズ2の受信前のフィールド値へ復元したかどうかと、復元前後の+0x60フィールド値。 */
    volatile bool cs60_restored;
    volatile uint16_t cs60_before;
    volatile uint16_t cs60_after;
    /*
     * - 対象ACLリンク生成直後に保存したACLのcontrol structure。
     * - 後で読み出したcontrol structureと比較し、ハードウェアが受信処理を実行したか判定する。
     */
    volatile uint16_t cs_at_start[ACL_CS_SNAPSHOT_WORDS];
    /*
     * - このモジュールが変更する前のACL control structure。
     * - ページスキャンの初期化値とフェーズ2の設定値のどちらが保持されているかを判定するために保存する。
     */
    volatile uint16_t cs_before[ACL_CS_SNAPSHOT_WORDS];
    /*
     * - 対象ACLリンクのリンク構造体を保存する。
     * - r_ld_acl_startがr_ke_malloc(252, 0)で確保する領域(0x40034dfc)である。
     * - 受信または送信ごとに更新されるフィールドを後で比較する。
     */
    volatile uint8_t acl_env_at_start[ACL_ENV_SNAPSHOT_BYTES];
    volatile uint8_t lc_env_at_start[LC_ENV_SNAPSHOT_BYTES];
} acl_handoff_request_t;

#define LC_START_FLAG_LC_ENV UINT8_C(1 << 0)
#define LC_START_FLAG_LD_ACL_ENV UINT8_C(1 << 1)
#define LC_START_FLAG_LLC_ENV UINT8_C(1 << 2)
#define LC_START_FLAG_HCI_BDADDR UINT8_C(1 << 3)

/*
 * - CSB RXがSwitchのウェイクアップ応答を受信する1回に限り、通常callbackがdescriptorを解放する前にLLID 3の本文と受信時刻を保存する。
 */
#define PHASE2_FM_PROG_PUSH_INDEX (UINT16_C(0x658) / sizeof(void *))
#define PHASE2_ROM_IP_FUNCS_PTR_ADDR UINT32_C(0x3ffae70c)
#define PHASE2_ROM_CSB_RX_SCHED_ADDR UINT32_C(0x400390d0)
#define PHASE2_CSB_CALLBACK_ADDR UINT32_C(0x400393a4)
#define PHASE2_PSCAN_CALLBACK_ADDR UINT32_C(0x4003ebe4)
#define PHASE2_CSB_ACTIVITY UINT8_C(13)
#define PHASE2_DIRECT_FORMAT UINT16_C(3)
#define PHASE2_LINKCNTL_EDR UINT16_C(0x0800)
#define PHASE2_LINKCNTL_AFH UINT16_C(0x4000)
#define PHASE2_CSB_CS_ADDR UINT32_C(0x3ffb231c)
#define PHASE2_DESC_ADDR UINT32_C(0x3ffb2382)
#define PHASE2_DESC_WORDS 7
#define PHASE2_CS_WORDS 16
#define PHASE2_DESC_DONE UINT16_C(0x8000)
#define PHASE2_STATUS_FAIL_MASK UINT16_C(0x0047)
#define PHASE2_HEADER UINT16_C(0x0298)
#define PHASE2_PHEADER UINT16_C(0x008b)
#define PHASE2_LENGTH 17
#define PHASE2_RXCLK_LO_WORD (UINT16_C(0x52) / 2)
#define PHASE2_RXCLK_HI_WORD (UINT16_C(0x54) / 2)
#define PHASE2_RXBITOFF_WORD (UINT16_C(0x56) / 2)
#define PHASE2_RXBITOFF_CORRECTION 68
/*
 * CSB受信のスケジュール要素へのポインタ(ld_csb_rx_env)と、要素を解放するROM関数である。
 * フックで置き換えたcallbackからROMと同じ終了処理を実行するために使用する。
 */
#define PHASE2_ROM_CSB_RX_END_ADDR UINT32_C(0x40039098)
/* スケジュール要素+0x2Cの監視タイムアウト基準クロックと、+0x3Eの状態。 */
#define PHASE2_ELT_SUP_BASE 0x2C
#define PHASE2_ELT_STATE 0x3E
#define PHASE2_ELT_STATE_STOPPING UINT8_C(2)

typedef void (*phase2_frame_cb_t)(uint32_t timestamp, uint32_t activity,
                                  uint32_t arg, uint32_t reason);
typedef void (*phase2_prog_push_t)(uint32_t timestamp, uint32_t activity,
                                   uint32_t mode, uint32_t arg,
                                   phase2_frame_cb_t callback);
typedef void (*phase2_csb_rx_sched_t)(uint8_t skipped);
typedef void (*phase2_csb_rx_end_t)(void);

enum {
    PHASE2_CAPTURE_IDLE,
    PHASE2_CAPTURE_STARTING,
    PHASE2_CAPTURE_WAITING,
    PHASE2_CAPTURE_DONE,
};

typedef struct {
    uint8_t target[6];
    uint64_t bch34;
    volatile uint8_t state;
    volatile bool active;
    volatile esp_err_t result;
    volatile uint8_t link;
    volatile uint8_t desc_index;
    volatile uint32_t programmed;
    volatile uint32_t push_count;
    volatile uint32_t last_activity;
    volatile uintptr_t last_callback;
    volatile uint16_t last_format;
    volatile uint32_t reasons[4];
    uint16_t desc[PHASE2_DESC_WORDS];
    uint16_t control[PHASE2_CS_WORDS];
    /*
     * - control structureの末尾4ワード(+0x5E~+0x64)を、フェーズ2の受信開始前と本文取得後の2時点で保存する。
     * - ハードウェアが受信ごとに更新する領域であり、フェーズ2の受信結果を確認するために使用する。
     */
    uint16_t tail_first[4];
    uint16_t tail_last[4];
    volatile bool tail_first_taken;
    uint16_t payload_offset;
    uint8_t payload[PHASE2_LENGTH];
    uint32_t rx_clk;
    uint16_t rx_bitoff;
} phase2_capture_t;
extern void ld_pscan_assert_err_handle(void);

#define KE_TASK_LC UINT16_C(1)
#define IP_FUNCS_BUF_LMP_TX_ALLOC_INDEX (UINT16_C(0x004) / sizeof(void *))
#define IP_FUNCS_LC_SEND_LMP_INDEX (UINT16_C(0x208) / sizeof(void *))
#define IP_FUNCS_LC_SEND_PDU_VERS_REQ_INDEX (UINT16_C(0x2a0) / sizeof(void *))
#define IP_FUNCS_LC_START_LMP_TO_INDEX (UINT16_C(0x2b8) / sizeof(void *))
#define IP_FUNCS_LC_VERSION_INDEX (UINT16_C(0x43c) / sizeof(void *))
#define IP_FUNCS_LD_ACL_LMP_TX_INDEX (UINT16_C(0x540) / sizeof(void *))
#define ROM_KE_MSG_SEND_BASIC_ADDR UINT32_C(0x4001a26c)
#define ROM_KE_STATE_GET_ADDR UINT32_C(0x4001a7d8)
#define ROM_KE_STATE_SET_ADDR UINT32_C(0x4001a6fc)
/*
 * hci_accept_con_req_cmd_handler(0x40024ab4)はリンク処理の内部状態が12または6であればHCI Accept Connection Requestを処理する。
 * Switchからの要求を処理するROM経路も、イベントをホストへ通知した後に内部状態を12へ変更する。
 */
#define LC_STATE_WAIT_ACCEPT 12
/* ESP32がBluetooth周辺側となる対象ACLリンクの待機状態。r_lc_startの第2引数が1の場合に設定される。 */
#define LC_STATE_PERIPHERAL_IDLE 5

typedef void (*ke_msg_send_basic_fn_t)(uint16_t id, uint16_t dest_id,
                                       uint16_t src_id);
typedef uint8_t (*ke_state_get_fn_t)(uint16_t id);

/**
  * ホストへHCI Connection Requestイベントを通知する
  *
  * - lmp_host_con_req_handler(0x4002b3d8)がSwitchからの要求を処理するときと同じ形式でイベントを生成する
  * - r_ke_msg_alloc(id=0x803、dest=リンク番号、src=4、長さ10)を呼び出し、BD_ADDRとClass of Deviceを格納する
  * - 種別に1(ACL)を設定してr_hci_send_2_hostへ渡す。イベント種別はidとsrcの組み合わせで決まる
  *
  * - ESP32がBluetooth周辺側となる対象ACLリンクからLMP_host_connection_reqを送信すると、コントローラーがこのイベントを通知せず、Connection Completeだけを通知するため、ホストのL2CAPが対象ACLリンクを切断する
  * - Connection Requestを先に通知すると、ホストは通常の着信として制御ブロックを生成する
 */
#define ROM_KE_MSG_ALLOC_ADDR UINT32_C(0x4001a1e0)
#define KE_MSG_HCI_EVENT UINT16_C(0x803)
#define KE_SRC_HCI_CON_REQ UINT16_C(4)
#define IP_FUNCS_HCI_SEND_2_HOST_INDEX (UINT16_C(0x170) / sizeof(void *))
#define HCI_CON_REQ_PARAM_LEN 10

typedef void *(*ke_msg_alloc_fn_t)(uint16_t id, uint16_t dest_id,
                                   uint16_t src_id, uint16_t param_len);
typedef void (*hci_send_2_host_fn_t)(void *param);
typedef void (*ke_state_set_fn_t)(uint16_t id, uint8_t state);

typedef struct {
    volatile bool requested;
    volatile bool done;
    volatile bool con_req_sent;
    volatile uint8_t link;
    volatile uint16_t dest_id;
    volatile uint8_t ke_state_before;
    volatile uint8_t ke_state_after;
    volatile uint8_t lc_state_before;
    volatile uint8_t lc_state_after;
    volatile uint8_t lc_env_60;
    volatile uint8_t lc_env_850;
    volatile uint32_t fn_lc_version;
    volatile uint32_t fn_send_pdu_vers_req;
    volatile uint32_t fn_start_lmp_to;
    volatile uint32_t fn_send_lmp;
    volatile uint32_t fn_buf_alloc;
    volatile uint32_t fn_acl_lmp_tx;
} lc_version_req_t;


/**
  * リンク処理の内部状態を5へ変更する
  *
  * - r_lc_hl_connect(0x400209e8)はLMP_host_connection_reqを送信した後、リンク処理の内部状態を20へ変更してSwitchからのLMP_acceptedを待つ
  * - SwitchはLMP_acceptedを送信せず、LMP_host_connection_reqを27ms後に送信する
  * - lmp_host_con_req_handlerは内部状態が5の場合だけ要求を処理するため、LMP_acceptedの待機中を示す20では要求を破棄する
  *
  * - LMP_host_connection_reqの送信直後に内部状態を5へ変更し、Switchからの要求を処理できるようにする
  * - リンク処理の内部状態が5の場合は、r_lc_startが対象ACLリンクの生成時に設定する状態と同じである
 */
typedef struct {
    volatile bool done;
    volatile uint8_t link;
    volatile uint8_t state_before;
    volatile uint8_t state_after;
} lc_idle_state_request_t;
/*
 * r_lc_startは第2引数が0であれば、リンク処理の内部状態を2へ変更してLMPタイムアウトを開始し、0x513、0x514、0x516をこの順で同じリンクのLCタスクへ送信する。
 * 第2引数が1であれば内部状態が5のまま処理を終了するため、同じ3メッセージを同じ順序で送信する。
 * 0x514を受信したlc_op_vers_ind_handler(0x40028b20)は、内部状態が1、2、5のいずれかであればr_lc_versionへ進み、LMP_version_reqを送信する。
 */
#define KE_MSG_LC_OP_FEAT_IND UINT16_C(0x513)
#define KE_MSG_LC_OP_VERS_IND UINT16_C(0x514)

#define LC_OP_HL_CNX_IND UINT16_C(0x0518)


#endif
