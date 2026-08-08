#include "wake_session.h"

#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd_api.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "link_state.h"
#include "swpro_pairing.h"
#include "host_sec_state.h"
#include "wakeup_code.h"
#include "wake_internal.h"

#define TAG "wake"

void wake_try_receive(const uint8_t target[6], uint16_t interval)
{
    // 送信スケジュール要素の解放が完了する前にCSB受信を開始すると、ld_csb_tx.cのassertで停止する。
    // 送信停止後80ms待機して受信を開始する。
    vTaskDelay(pdMS_TO_TICKS(80));
    uint32_t clkn = 0;
    switch1_csb_read_clkn(&clkn);
    // 応答を受信する時刻は、最後のバーストのクロックからのオフセットで指定する。
    // 単位はスロット(625us)で、ld_csb_rx_schedが扱うクロックと合わせる。
    //
    // Switchの応答はESP32のバースト開始から2248us後に発生する。
    // 実際の位相はIntervalで割った余りで決まるので、0~Interval-1の各位相を検索すれば全位相を網羅できる。
    const uint32_t clk = clkn >> 1;
    uint32_t next = (g_wake_last_clkn >> 1) + g_wake_rx_offset;
    while ((int32_t)(next - clk) < 4) {
        next += interval;
    }
    const esp_err_t err = switch1_csb_rx_start(
        target, switch1_wakeup_parity34(target), interval, 0,
        next & UINT32_C(0x0fffffff), g_wake_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to start CSB receive: %s", esp_err_to_name(err));
        return;
    }
    // 受信を有効にしたあと、送信スケジュール要素に設定する送信時間を更新する。
    // 開始時に設定した送信時間が保持されていない場合、送信間隔は3.75msから15msへ延びる。
    switch1_csb_tx_set_duration(
        (uint16_t)((g_wake_length > 183 ? 5 : 3) * 625 -
                   CSB_TX_DECLARED_MARGIN_US));
    ESP_LOGI(TAG,
             "CSB receive started: offset=%u slots last burst=%08x(slot %08x) "
             "scheduled receive=%08x current=%08x(slots)",
             (unsigned)g_wake_rx_offset, (unsigned)g_wake_last_clkn,
             (unsigned)(g_wake_last_clkn >> 1), (unsigned)next,
             (unsigned)clk);
    // 応答バーストは318.75ms周期で発生する。1周期では取りこぼすため2周期待機する。
    const int64_t until = esp_timer_get_time() + WAKE_RX_WATCH_US;
    uint32_t received = 0;
    while (esp_timer_get_time() < until) {
        uint8_t addr[6];
        uint32_t clock = 0;
        uint8_t status = 0;
        uint8_t data[32];
        uint8_t length = 0;
        if (switch1_csb_rx_take(addr, &clock, &status, data, &length)) {
            received++;
            char text[32 * 3 + 1];
            int used = 0;
            for (uint8_t i = 0; i < length && used < (int)sizeof(text) - 3;
                 ++i) {
                used += snprintf(text + used, sizeof(text) - used, "%02x ",
                                 data[i]);
            }
            ESP_LOGI(TAG,
                     "Switch response received: clock=%08x status=%02x length=%u payload %s",
                     (unsigned)clock, status, length, text);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (received == 0) {
        ESP_LOGI(TAG, "CSB receive: no event detected in %dms (offset=%u)",
                 (int)(WAKE_RX_WATCH_US / 1000),
                 (unsigned)g_wake_rx_offset);
    }
    switch1_csb_rx_stop(target, interval);
}

// CSB受信を停止し、最後のバーストのクロックからoffsetスロット後の位置で再開始する。
//
// - Enable=1だけを指定する開始要求は、CSB受信が動作している場合に限り成功する。
// - スケジュール要素は監視タイムアウトの満了で解放されるため、検索の2回目以降はstatus=0x0cで拒否されていた。
//   停止を挟む手順にすると、どちらの状態からでも再開始できる。
static esp_err_t wake_restart_rx(uint16_t interval, uint8_t offset)
{
    uint32_t clkn = 0;
    if (!switch1_csb_read_clkn(&clkn)) {
        return ESP_FAIL;
    }
    const uint32_t clk = clkn >> 1;
    uint32_t next = (g_wake_last_clkn >> 1) + offset;
    while ((int32_t)(next - clk) < 4) {
        next += interval;
    }
    return switch1_csb_rx_restart(interval, next & UINT32_C(0x0fffffff));
}

// CSB受信を再開始する間隔。HCIを2往復するため、連続して実行しない。
#define RX_RESTART_INTERVAL_US 20000

static void take_phase2_capture(void)
{
    esp_err_t result = ESP_ERR_TIMEOUT;
    uint8_t lt_addr = 0;
    uint32_t clock_delta = 0;
    uint32_t rx_clk = 0;
    uint16_t rx_bitoff = 0;
    if (!switch1_phase2_capture_take(&result, &lt_addr, &clock_delta, &rx_clk,
                                     &rx_bitoff)) {
        return;
    }
    if (result != ESP_OK || lt_addr == 0 || lt_addr > 7) {
        ESP_LOGW(TAG, "Unable to register phase 2 payload with the controller: %s LT_ADDR=%u",
                 esp_err_to_name(result), lt_addr);
        return;
    }

    g_direct_lt_addr = lt_addr;
    g_direct_clock_delta = clock_delta;
    g_direct_rx_clk = rx_clk;
    g_direct_rx_bitoff = rx_bitoff;
    g_hci_clock_offset =
        (uint16_t)(HCI_CLOCK_OFFSET_VALID |
                   ((clock_delta >> 1) & UINT32_C(0x7fff)));
    g_direct_clock_delta_valid = true;
    __asm__ __volatile__("memw" ::: "memory");
    g_direct_sync_valid = true;
}

// 1. Switchの応答を待つ。
// 2. フェーズ3を送信し、送信を停止してページ処理へ移る。
// 3. 応答の検出は受信descriptorを直接読み出す。
//    HCIイベントの往復を待つと、応答からフェーズ3までを634us以内に実行できない。
//    フェーズ3への切り替えは送信タスクが行うため、送信要求フラグを設定して送信完了を待つ。
static uint32_t wake_roundtrip_run(const uint8_t target[6], uint16_t interval,
                                  uint8_t trigger, uint8_t bursts,
                                  uint32_t timeout_ms, bool page_after,
                                  uint8_t base_offset, uint8_t positions,
                                  uint32_t dwell_ms)
{
    // 監視開始時点でdescriptorに入っている受信は、CSB受信の開始処理とその記録の出力の間に成立した古い値の可能性がある。
    // これを契機にするとフェーズ3の送信時刻が応答受信時刻から大きく遅れるため、現在の値を取り込み、以降の新しい受信だけを契機にする。
    switch1_csb_rx_poll_prime();
    switch1_csb_rx_event_reset();
    g_wake_phase3_limit = (bursts != 0) ? bursts : 1;
    g_wake_phase3_done_us = 0;
    g_wake_phase3_pending = false;
    g_wake_phase3_schedule_slot = UINT32_MAX;
    g_wake_phase3_actual_slot = 0;
    g_direct_sync_valid = false;
    g_wake_rx_stop_err = ESP_ERR_NOT_FOUND;

    uint32_t hits = 0;
    uint16_t first_status = 0;
    uint16_t first_header = 0;
    uint16_t first_pheader = 0;
    int64_t first_us = 0;
    int64_t detected_us = 0;
    const int64_t started = esp_timer_get_time();
    const int64_t until = started + (int64_t)timeout_ms * 1000;
    // - Switchの応答がESP32の送信時刻に対してどの位相で発生するかは、送信のたびに変化する。
    // - 受信開始位置を1つに固定すると、成立する試行と成立しない試行が発生する。
    // - 1回の試行の中で受信位置を順に変更して探索する。
    const uint8_t position_count = (positions != 0) ? positions : 1;
    uint8_t used_offset = base_offset;
    uint8_t tried = 0;
    uint8_t move_failures = 0;
    bool direct_capture_complete = false;
    int64_t yielded_us = started;
    // - 送信と受信のスケジュール要素を監視中ずっと読み続ける。
    // - 受信の優先度は低下させる経路がなく、スケジュールに失敗するたびに増加する。
    //   優先度が11以上に上昇すると送信を登録できないため、値が11以上か確認する。
    switch1_csb_sched_t sched = {0};
    switch1_csb_sched_t rx_live = {0};
    uint8_t rx_priority_max = 0;
    uint8_t tx_priority_max = 0;
    uint32_t sched_samples = 0;
    uint32_t rx_freed_samples = 0;
    // CSB受信を再開始する。HCIを2往復するため、間隔を置いて実行する。
    int64_t rx_restart_next_us = 0;
    uint32_t rx_restart_calls = 0;
    uint32_t rx_restart_ok = 0;
    for (uint8_t p = 0; p < position_count; ++p) {
        if (p != 0) {
            used_offset = (uint8_t)(base_offset + p);
            const esp_err_t move_err = wake_restart_rx(interval, used_offset);
            if (move_err != ESP_OK) {
                // 受信開始位置を変更できない場合は現在の位置で受信を継続し、位置探索だけを終了する。
                ESP_LOGW(TAG, "Wake-up round trip: unable to move the receive position to %u: %s",
                         used_offset, esp_err_to_name(move_err));
                move_failures++;
                used_offset = (uint8_t)(used_offset - 1);
            }
            switch1_csb_rx_poll_prime();
        }
        tried++;
        const int64_t dwell_until =
            esp_timer_get_time() + (int64_t)dwell_ms * 1000;
        // - descriptorのステータスは、descriptorが次に再利用されるまで保持される。
        //   受信周期が6スロットであれば4件で一巡するため、15ms保持される。
        // - 200usごとに確認すれば取りこぼさない。
        // - 10msごとにCPUの実行権を他のタスクへ移し、他のタスクの実行を妨げないようにする。
        while (esp_timer_get_time() < dwell_until &&
               esp_timer_get_time() < until) {
            switch1_csb_sched_read(&sched);
            sched_samples++;
            if (sched.rx_element == 0) {
                rx_freed_samples++;
                // 受信のスケジュール要素は監視タイムアウトの満了で解放される(ld_csb_rx_sched 0x400391f0)。
                // 解放された状態では受信イベントを1件も取得できない。
                // 解放された状態では受信イベントを取得できないため、ここでCSB受信を再開始する。
                const int64_t now_us = esp_timer_get_time();
                if (now_us >= rx_restart_next_us) {
                    rx_restart_next_us = now_us + RX_RESTART_INTERVAL_US;
                    const esp_err_t restart_err =
                        wake_restart_rx(interval, used_offset);
                    rx_restart_calls++;
                    if (restart_err == ESP_OK) {
                        rx_restart_ok++;
                        switch1_csb_rx_poll_prime();
                    }
                }
            } else {
                rx_live = sched;
            }
            if (sched.rx_priority > rx_priority_max) {
                rx_priority_max = sched.rx_priority;
            }
            if (sched.tx_priority > tx_priority_max) {
                tx_priority_max = sched.tx_priority;
            }
            take_phase2_capture();
            const bool direct_response =
                g_direct_clock_delta_valid && g_direct_sync_valid &&
                g_direct_lt_addr >= 1 && g_direct_lt_addr <= 7;
            const bool direct_trigger = direct_response && trigger <= 1;
            // - descriptorのステータスは受信状態の記録だけに使用する。
            // - フラグはHCIイベントの件数に基づいて設定する。
            //   連続して成立した受信はどれも同じステータスを書き込むので、値の変化では数えられない。
            uint16_t status = 0;
            uint16_t header = 0;
            uint16_t pheader = 0;
            if (switch1_csb_rx_poll(&status, &header, &pheader)) {
                if (first_status == 0) {
                    first_status = status;
                    first_header = header;
                    first_pheader = pheader;
                }
            }
            // フラグはステータスが検査条件を満たす値へ変わった時点で設定する。
            // 応答バースト列の先頭では6801から6800への変化が必ず発生する。
            // HCIイベントのclockフィールドは受信予定時刻で到着時刻ではないため、位置決めには使えない。
            if ((status != 0 || direct_trigger) &&
                !g_wake_phase3_pending) {
                if (first_us == 0) {
                    first_us = esp_timer_get_time();
                }
                if (direct_trigger ||
                    (g_direct_clock_delta_valid &&
                     switch1_csb_rx_event_count() + 1 >= trigger)) {
                    // - CSB受信が有効な状態では送信間隔を変更できない。
                    //   受信イベントが送信候補時刻を6スロット分予約するため、送信周期を6の倍数以外に設定するとスケジュール登録に失敗し、送信が停止する。
                    // - 応答を受信した時点でCSB受信を停止する。
                    // - ハードウェアが測定した受信時刻は、CSB受信の停止処理を実行する前に読み出す。
                    uint32_t rx_clk = g_direct_rx_clk;
                    uint16_t rx_bitoff = g_direct_rx_bitoff;
                    uint16_t rx_raw = g_direct_rx_bitoff;
                    const bool have_sync =
                        direct_response || switch1_csb_rx_read_sync(
                                               &rx_clk, &rx_bitoff, &rx_raw);
                    if (have_sync && g_direct_clock_delta_valid &&
                        !g_direct_sync_valid &&
                        g_direct_lt_addr >= 1 && g_direct_lt_addr <= 7) {
                        g_direct_rx_clk = rx_clk;
                        g_direct_rx_bitoff = rx_bitoff;
                        __asm__ __volatile__("memw" ::: "memory");
                        g_direct_sync_valid = true;
                    }
                    uint32_t tx_slot = 0;
                    uint16_t tx_interval_now = 0;
                    switch1_csb_tx_read_schedule(&tx_slot, &tx_interval_now);
                    const int64_t before = esp_timer_get_time();
                    // 停止要求がstatus=0x0Cで拒否されると、CSB受信が有効な状態でフェーズ3へ遷移する。
                    // ここで再度停止するとHCIの往復に17ms掛かり、フェーズ3の送信時刻が応答バーストの時刻からずれるため、停止結果だけを記録して処理を続ける。
                    // CSB受信の停止処理は次の試行のswitch1_csb_rx_startが実行する。
                    g_wake_rx_stop_err = switch1_csb_rx_stop(target,
                                                                 interval);
                    detected_us = esp_timer_get_time();
                    // - 応答の時刻を取得できたら、その634us後へフェーズ3を設定する。
                    // - スロット単位の位置は送信の周期で、スロット内の位置はcontrol structure +0x02で決める。
                    // - 応答は6スロットごとに続くので、次の送信の4スロット後以降になるまで6の倍数を加算する。
                    uint32_t target_slot = 0;
                    uint16_t target_bitoff = 0;
                    const bool phase3_target_available =
                        have_sync && tx_interval_now == g_wake_interval;
                    if (phase3_target_available) {
                        // 実測値に基づいて補正を行う
                        const uint32_t total = rx_bitoff +
                                               WAKE_PHASE3_TARGET_US -
                                               WAKE_PHASE3_BIAS_US;
                        target_slot = rx_clk + total / 625u +
                                      g_wake_phase3_target_slot_adjustment;
                        target_bitoff = (uint16_t)(total % 625u);
                        g_wake_phase3_target_slot = target_slot;
                        g_wake_phase3_bitoff = target_bitoff;
                    }
                    g_wake_rx_detect_us = detected_us;
                    // フラグを設定する前に、フェーズ3の本体を共有メモリへ書き込む。
                    // 書き込み内容は次のバーストから送信に反映されるため、送信タスク側で書き込むと反映が1バースト遅れる。
                    if (g_wake_phase3_raw &&
                        switch1_csb_write_payload_at(
                            g_wake_payload_offset,
                            g_wake_phase3_plain,
                            WAKE_PHASE3_LENGTH) == ESP_OK) {
                        g_wake_phase3_body_writes++;
                    }
                    __asm__ __volatile__("memw" ::: "memory");
                    g_wake_phase3_pending = true;
                    __asm__ __volatile__("memw" ::: "memory");
                    // フラグを設定してから本体を再書き込みする。
                    // 上の書き込みとフラグ設定の間に送信タスクがWAIT_GAP状態を実行すると、フラグ設定がまだ反映されていないためフェーズ1の本体で上書きされる。
                    // フラグ設定後に本体を再書き込みすることで、どちらの実行順序でも正しい内容を共有メモリへ保持できる。
                    if (g_wake_phase3_raw &&
                        switch1_csb_write_payload_at(
                            g_wake_payload_offset,
                            g_wake_phase3_plain,
                            WAKE_PHASE3_LENGTH) == ESP_OK) {
                        g_wake_phase3_body_writes++;
                    }
                    direct_capture_complete =
                        direct_trigger && g_wake_phase3_rounds == 1;
                    // 出力はフラグ設定後に行う。
                    // 115200bpsのUARTは1行に10msほど掛かり、フラグを設定する前に出力するとその時間がすべて応答からフェーズ3までの時間に加算される。
                    // 送信タスクは別のcoreで動作しているため、ここでログを出力してもフェーズ3の送信タスクは待機しない。
                    ESP_LOGI(TAG,
                             "  Response detected; stopping receive (%lldus) "
                             "sync=%s clk=%08x in-slot=%uus(raw %u) "
                             "next transmission=%08x interval=%u",
                             (long long)(detected_us - before),
                              have_sync ? "valid" : "invalid",
                             (unsigned)rx_clk, rx_bitoff, rx_raw,
                             (unsigned)tx_slot, tx_interval_now);
                    if (phase3_target_available) {
                        ESP_LOGI(TAG,
                                  "  Phase 3 target time: slot %08x offset %uus",
                                 (unsigned)target_slot, target_bitoff);
                    }
                }
            }
            hits = switch1_csb_rx_event_count();
            if (direct_response && hits == 0) {
                hits = 1;
            }
            // ESP32が本文と受信時刻を取得したら、その値でフェーズ3を送信するまで受信位置を変更しない。
            if (direct_capture_complete || g_wake_phase3_done_us != 0) {
                break;
            }
            const int64_t now = esp_timer_get_time();
            if (now - yielded_us >= 10000) {
                yielded_us = now;
                vTaskDelay(1);
            } else {
                esp_rom_delay_us(200);
            }
        }
        if (direct_capture_complete || g_wake_phase3_done_us != 0 ||
            esp_timer_get_time() >= until) {
            break;
        }
    }
    // フラグを設定したあとは、フェーズ3の送信が完了するまで待つ。
    // 送信時刻は送信タスクが決めるので、この関数は待機だけを行う。
    // 送信完了まで400msの猶予を設ける。送信イベントは受信と競合してスケジューラにより省略されることがあり、次のバーストまで数十msかかる場合がある。
    while (g_wake_phase3_pending && esp_timer_get_time() < until + 400000) {
        const int64_t done = g_wake_phase3_done_us;
        if (done != 0 && esp_timer_get_time() >= done) {
            break;
        }
        vTaskDelay(1);
    }
    // ROMコールバックへ復帰する前にCSB受信を停止する。
    // フェーズ2の本文取得はページスキャンが確保したlink 0の受信コンテキストを使用するため、そのリンクの完了descriptorは解放されない。
    // 完了descriptorを解放せずにROM callbackへ復帰すると、activityが13でないことを検出してassertで停止する。
    if (switch1_csb_rx_is_active()) {
        switch1_csb_rx_stop(target, interval);
    }
    switch1_phase2_capture_cancel();
    take_phase2_capture();
    bool direct_path = false;
    bool direct_result_ready = false;
    esp_err_t direct_result = ESP_ERR_TIMEOUT;
    uint8_t direct_link = 0xff;
    const int64_t sent_us = g_wake_phase3_done_us;

    // フェーズ3の送信後は、ログ出力より先にページ処理を開始する。
    // ログをここより前に出力すると、115200bpsのUARTが1行あたり9msほど掛かるため、数行で100ms以上遅延する。
    // ページ処理を開始したあとに記録を出力する。
    int64_t t_page = 0;
    int64_t t_rx_stopped = 0;
    int64_t t_tx_stopped = 0;
    if (page_after && sent_us != 0) {
        const int64_t t0 = esp_timer_get_time();
        // 受信は応答を検出した時点で停止している。ここで再度停止するとHCIの往復に17ms追加で必要になる。
        if (!g_wake_phase3_pending) {
            switch1_csb_rx_stop(target, interval);
        }
        t_rx_stopped = esp_timer_get_time();
        if (g_wake_tx_active) {
            g_wake_stop_requested = true;
        }
        const int64_t stop_until = esp_timer_get_time() + 1000000;
        while (g_wake_tx_active && esp_timer_get_time() < stop_until) {
            esp_rom_delay_us(100);
        }
        t_tx_stopped = esp_timer_get_time();
        __asm__ __volatile__("memw" ::: "memory");
        direct_path =
            g_direct_handoff_started && g_direct_handoff_start_err == ESP_OK;
        if (direct_path) {
            direct_result_ready =
                switch1_acl_handoff_status(&direct_result, &direct_link);
            if (direct_result_ready && direct_result != ESP_OK) {
                direct_path = false;
                g_direct_handoff_active = false;
                g_direct_handoff_deadline_us = 0;
            }
        }
        // ACLの直接接続開始が成立していればページ処理を開始しない。
        // ページ処理はdeadlineが満了したときに使用する代替経路である。
        if (!direct_path) {
            const int64_t page_not_before = sent_us + 20000;
            while (esp_timer_get_time() < page_not_before) {
                esp_rom_delay_us(100);
            }
            try_reconnect();
            t_page = esp_timer_get_time();
        }
        (void)t0;
    }

    switch1_phase2_capture_report();
    if (g_direct_handoff_started) {
        if (g_direct_handoff_start_err != ESP_OK) {
            ESP_LOGW(TAG, "Unable to queue ACL link registration: %s",
                     esp_err_to_name(g_direct_handoff_start_err));
        } else if (!direct_result_ready) {
            ESP_LOGI(TAG, "ACL link registration: waiting for Controller processing");
        } else {
            ESP_LOGI(TAG, "ACL link registration: %s link=%u LT_ADDR=%u",
                     esp_err_to_name(direct_result), direct_link,
                     g_direct_lt_addr);
        }
        uint8_t lc_state_after = 0;
        uint8_t lc_flags = 0;
        if (switch1_lc_start_status(&lc_state_after, &lc_flags)) {
            ESP_LOGI(TAG,
                     "  r_lc_start: link state=%u(completion state is 5 when argument 2 is 1) "
                     "lc_env=%u ld_acl_env=%u llc_env=%u BD_ADDR registered=%u",
                     lc_state_after, (lc_flags & 1u) ? 1u : 0u,
                     (lc_flags & 2u) ? 1u : 0u, (lc_flags & 4u) ? 1u : 0u,
                     (lc_flags & 8u) ? 1u : 0u);
        } else {
            ESP_LOGI(TAG, "  r_lc_start: not called");
        }
    }
    ESP_LOGI(TAG, "  Phase 3 position search: %u/%u rounds", g_wake_phase3_rounds_done,
             g_wake_phase3_rounds);

    ESP_LOGI(TAG,
             "Wake-up round trip: responses=%lu phase3_pending=%s "
             "first response=%lldms header=%03x payload_header=%02x "
             "status=%04x receive_position=%u(attempts=%u move_failures=%u)",
             (unsigned long)hits,
              g_wake_phase3_pending ? "yes" : "no",
             (long long)((first_us != 0 ? first_us - started : 0) / 1000),
             first_header & 0x3FF, first_pheader & 0xFF, first_status,
             used_offset, tried, move_failures);
    ESP_LOGI(TAG,
             "  Schedule elements: RX max_priority=%u state=%u interval=%u slots "
             "RX reservation=%uus freed=%lu%%(%lu samples)/"
             "TX max_priority=%u TX reservation=%uus/RX stop %s",
             rx_priority_max, rx_live.rx_state, rx_live.rx_interval_slots,
             rx_live.rx_duration_us,
             sched_samples != 0
                 ? (unsigned long)(rx_freed_samples * 100u / sched_samples)
                 : 0ul,
             (unsigned long)sched_samples, tx_priority_max,
             sched.tx_duration_us, esp_err_to_name(g_wake_rx_stop_err));
    ESP_LOGI(TAG, "  CSB receive restarts: %lu calls, %lu succeeded",
             (unsigned long)rx_restart_calls, (unsigned long)rx_restart_ok);
    switch1_csb_rx_poll_report();
    if (sent_us != 0) {
        ESP_LOGI(TAG,
                 "  Phase 3 transmission: interval=%u slots bursts=%lu "
                 "(response-time offset changes by %d slots per burst)",
                 g_wake_shift_report,
                 (unsigned long)g_wake_phase3_bursts,
                 (int)g_wake_shift_report - (int)g_wake_interval);
        ESP_LOGI(TAG,
                 "  Phase 3 position: target=%08x scheduled=%08x actual=%08x "
                 "(%ld periods late at %u-slot interval)",
                 (unsigned)g_wake_phase3_target_slot,
                 (unsigned)g_wake_phase3_schedule_slot,
                 (unsigned)g_wake_phase3_actual_slot,
                 g_wake_interval,
                 g_wake_phase3_schedule_slot == UINT32_MAX
                     ? -1L
                     : (long)(((g_wake_phase3_actual_slot -
                                g_wake_phase3_schedule_slot) &
                               UINT32_C(0x07ffffff)) /
                              g_wake_interval));
        ESP_LOGI(TAG,
                 "  Phase 3 details: flag-to-completion=%lldus "
                 "bursts=%lu header=%04x pheader=%04x payload_writes=%lu "
                 "offset=%04x",
                 (long long)(sent_us - detected_us),
                 (long long)(sent_us - started),
                 (unsigned long)g_wake_phase3_bursts,
                 g_wake_phase3_header, g_wake_phase3_pheader,
                 (unsigned long)g_wake_phase3_body_writes,
                 g_wake_payload_offset);
    }

    if (t_page != 0) {
        ESP_LOGI(TAG,
                 "Wake-up round trip: started page processing %lldus after Phase 3 "
                 "(RX stop=%lldus TX stop=%lldus request=%lldus "
                 "Clock_Offset=%04x)",
                 (long long)(t_page - sent_us),
                 (long long)(t_rx_stopped - sent_us),
                 (long long)(t_tx_stopped - t_rx_stopped),
                 (long long)(t_page - t_tx_stopped), g_hci_clock_offset);
        return hits;
    }
    if (direct_path) {
        return hits;
    }
    switch1_csb_rx_stop(target, interval);
    if (!page_after) {
        return hits;
    }
    // フェーズ3の送信完了時点で送信タスクは停止する。
    // 送信タスクが動作中の場合だけ停止要求を設定する。
    if (g_wake_tx_active) {
        g_wake_stop_requested = true;
    }
    // 送信タスクが停止するまで待つ。
    // g_wake_tx_activeは停止完了の確認直後にfalseに変わるため、その後の記録出力を待機する必要はない。
    const int64_t stop_until = esp_timer_get_time() + 3000000;
    while (g_wake_tx_active && esp_timer_get_time() < stop_until) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (g_wake_tx_active) {
        ESP_LOGW(TAG, "Wake-up round trip: transmission task did not stop");
        return hits;
    }
    // 次の試行が残っている場合は、ここでページ処理を開始しない。
    // ページ処理が進行中だとLT_ADDRの予約とCSBの有効化がstatus 0x0bで失敗し、次の試行が受信失敗になる。
    // 規定回数の試行を完了した後、CYCLE_PAGE_WAITがページ処理を開始する。
    if (g_cycle_enabled && g_cycle_wake_tries < CYCLE_WAKE_TRIES) {
        // 定期の再接続処理もページ処理を開始しないようにする。
        // CMD_CSB_RXのページ指定により再接続禁止が解除されるため、ここで再設定しないと数百ms後にページ処理が開始される。
        g_reconnect_disabled = true;
        g_reconnect_paused = true;
        ESP_LOGI(TAG,
                 "Wake-up round trip: Phase 3 transmission failed; ending processing "
                 "(page processing deferred because another attempt remains)");
        return hits;
    }
    ESP_LOGI(TAG, "Wake-up round trip: Phase 3 transmission failed; starting page processing");
    try_reconnect();
    return hits;
}

// UARTのCMD_CSB_RXを処理する。
//
// 1. CSB受信を開始する。
// 2. Switchの応答を検出したらフェーズ3の送信とACL直接接続開始へ移る。
// 3. 応答の検出からフェーズ3の送信までは送信タスクが行うため、受信設定と受信状態を監視する。
void wake_cmd_receive(const uint8_t *payload, uint8_t length)
{
    if (length < 1 || length > 23) {
        return;
    }
    g_hci_clock_offset = 0;
    g_direct_clock_delta = 0;
    g_direct_lt_addr = 0;
    g_direct_clock_delta_valid = false;
    g_direct_sync_valid = false;
    esp_bd_addr_t host;
    if (!swpro_host_snapshot(host)) {
        ESP_LOGW(TAG, "CSB receive: peer BD_ADDR is not configured");
        return;
    }
    const uint16_t interval =
        (length >= 2 && payload[1] >= 2) ? payload[1] : 6;
    uint16_t before[8];
    uint16_t after[8];
    switch1_csb_rx_dump(0, 8, before);
    esp_err_t err;
    if (payload[0] != 0) {
        uint32_t clkn = 0;
        const bool clkn_ok = switch1_csb_read_clkn(&clkn);
        if (length >= 8 && payload[7] != 0 && g_wake_started) {
            const esp_err_t capture_err = switch1_phase2_capture_start(
                host, switch1_wakeup_parity34(host));
            ESP_LOGI(TAG, "Phase 2 payload capture preparation: %s",
                     esp_err_to_name(capture_err));
        }
        if (length >= 22 && clkn_ok) {
            const uint32_t remote_slot =
                (uint32_t)payload[18] |
                ((uint32_t)payload[19] << 8) |
                ((uint32_t)payload[20] << 16) |
                ((uint32_t)payload[21] << 24);
            const uint32_t local_slot = clkn >> 1;
            const uint32_t d =
                (remote_slot - local_slot) & UINT32_C(0x07ffffff);
            g_direct_clock_delta = d;
            g_direct_clock_delta_valid = true;
            g_hci_clock_offset =
                (uint16_t)(HCI_CLOCK_OFFSET_VALID |
                           ((d >> 1) & UINT32_C(0x7fff)));
        }
        if (length >= 23 && payload[22] >= 1 && payload[22] <= 7) {
            g_direct_lt_addr = payload[22];
        }
        // 開始時刻はスロット単位で渡す。
        // ld_csb_rx_schedがIntervalの剰余から受信位相を算出するためである。
        // ウェイクアップ送信中は最後のバーストのクロックからのオフセットで指定する。
        // 応答は送信バーストに追随するため、この指定がないと受信位置を一致させられない。
        const uint32_t clk = clkn >> 1;
        uint32_t next = (clk + 16u) & UINT32_C(0x0fffffff);
        uint32_t offset = 0;
        if (length >= 3 && g_wake_started) {
            next = (g_wake_last_clkn >> 1) + payload[2];
            while ((int32_t)(next - clk) < 4) {
                next += interval;
            }
            // payload[3]が0でなければ、白色化の初期値が0になるようにピコネットクロックを移動する。
            // 初期値はスロット単位のクロックの下位6ビットなので、64の倍数へ合わせる。
            // 位相はClock_Offsetを含めて決まるため、受信の時刻は変化しない。
            if (length >= 4 && payload[3] != 0) {
                // payload[6]は補正値である。オフセット計算が一致しない場合に備えて、0~63の値を探索できるようにする。
                const uint32_t correction =
                    (length >= 7) ? (payload[6] & 0x3Fu) : 0u;
                offset = (64u - (next & 0x3Fu) + correction) & 0x3Fu;
                next += offset;
            }
            next &= UINT32_C(0x0fffffff);
            ESP_LOGI(TAG,
                     "CSB receive start: offset=%u slots "
                     "last burst=%08x(slot %08x) "
                     "scheduled=%08x current=%08x correction=%u "
                     "whitening seed=%u(all in slots)",
                     payload[2], (unsigned)g_wake_last_clkn,
                     (unsigned)(g_wake_last_clkn >> 1),
                     (unsigned)next, (unsigned)clk, (unsigned)offset,
                     (unsigned)(next & 0x3Fu));
        }
        err = switch1_csb_rx_start(host, switch1_wakeup_parity34(host),
                                   interval, offset, next, 0);
        if (err == ESP_OK) {
            // CSB受信のスケジュール要素を開始直後に読み出す。
            // ld_csb_rx_sched(0x400391f0)は経過が監視タイムアウトに達すると要素を解放するため、基準クロックが分かればいつ解放されるかを計算できる。
            switch1_csb_sched_t started_sched;
            switch1_csb_sched_read(&started_sched);
            uint32_t clkn_now = 0;
            (void)switch1_csb_read_clkn(&clkn_now);
            ESP_LOGI(TAG,
                     "CSB receive: schedule_element=%08x state=%u interval=%u slots "
                     "priority=%u RX reservation=%uus supervision_base=%08x "
                     "supervision_timeout=%u slots current=%08x(slots)",
                     (unsigned)started_sched.rx_element,
                     started_sched.rx_state,
                     started_sched.rx_interval_slots,
                     started_sched.rx_priority,
                     started_sched.rx_duration_us,
                     (unsigned)started_sched.rx_sup_base,
                     started_sched.rx_sup_timeout_slots,
                     (unsigned)(clkn_now >> 1));
        }
        if (err != ESP_OK) {
            // 受信を開始できなければwake_roundtrip_runは実行されないため、フェーズ2の本文取得をここでキャンセルする。
            // キャプチャ状態を解除しないと、次の試行のswitch1_phase2_capture_startがESP_ERR_INVALID_STATEで拒否され、再起動まで取得できない。
            switch1_phase2_capture_cancel();
        }
        // 受信を有効にしたあと、送信スケジュール要素に設定する送信時間を実際の送信時間へ更新する。
        // 開始時に設定した送信時間が保持されないと、送信間隔は3.75msから15msへ延びる。
        if (err == ESP_OK) {
            switch1_csb_tx_set_duration(
                (uint16_t)((g_wake_length > 183 ? 5 : 3) * 625 -
                           CSB_TX_DECLARED_MARGIN_US));
        }
        // payload[4]でLT_ADDRを指定する。省略時は0を指定する。
        // 0はSwitchの応答に合わせた値だが放送用の番号でもあるため、受信側が不正として扱っていないかを比較できるようにする。
        if (err == ESP_OK && length >= 5) {
            switch1_csb_rx_write_ltaddr(payload[4]);
        }
        // payload[3]が2~28であれば、CSB受信のフレームフォーマットとして扱う。
        // ACLのフレームフォーマットにすると受信ペイロードが保存される経路に入る。
        if (err == ESP_OK && length >= 4 && payload[3] >= 2) {
            switch1_csb_rx_set_format(payload[3]);
            ESP_LOGI(TAG, "CSB receive frame format set to %u",
                     payload[3]);
        }
    } else {
        err = switch1_csb_rx_stop(host, interval);
    }
    // control structureのスナップショットはここで取得するが、出力は監視を終えてから行う。
    // 受信を有効にした直後にSwitchの応答を受信することがあり、UARTの3行ぶん(25msほど)が応答の検出を遅らせて、フェーズ3が応答バーストの時刻からずれる。
    switch1_csb_rx_dump(0, 8, after);

    if (payload[0] != 0 && err == ESP_OK) {
        // 受信の有無は受信descriptorを直接読み出す。
        // HCIイベントが生成されない場合も検出でき、UART経由の読み出しと異なり受信状態を取りこぼさない。
        // 送信中にも実行できるよう、待機せずにポーリングする。
        // payload[5]は監視する時間で10ms単位。省略時は600ms。
        const uint32_t watch_ms =
            (length >= 6 && payload[5] != 0)
                ? (uint32_t)payload[5] * 10u
                : 600u;
        // payload[7]が0でなければ、応答をその件数だけ検出した時点でフェーズ3へ移行し、送信を停止してページ処理を開始する。
        // 0であれば受信状態だけを記録し、送信設定は変更しない。
        // payload[8]はフェーズ3のバースト数、payload[9]はページ処理の有無、payload[10]は受信位置の箇所数、payload[11]は1箇所を保持する時間で10ms単位である。
        const uint8_t trigger = (length >= 8) ? payload[7] : 0;
        if (trigger != 0 && g_wake_started) {
            const uint8_t bursts =
                (length >= 9 && payload[8] != 0) ? payload[8] : 1;
            const bool page_after =
                (length >= 10) ? payload[9] != 0 : true;
            g_reconnect_disabled = !page_after;
            const uint8_t positions =
                (length >= 11 && payload[10] != 0) ? payload[10] : 1;
            const uint32_t dwell_ms =
                (length >= 12 && payload[11] != 0)
                    ? (uint32_t)payload[11] * 10u
                    : watch_ms;
            // payload[12]はフェーズ3を送信している間の送信間隔で、スロット単位である。
            // 送信の周期6と互いに素な7にすると、応答時刻との差が1バーストごとに1スロットずつ変化する。
            g_wake_phase3_step =
                (length >= 13 && payload[12] != 0) ? payload[12] : 7;
            // payload[13]は位置の検索を繰り返す回数
            g_wake_phase3_rounds =
                (length >= 14 && payload[13] != 0) ? payload[13] : 1;
            g_wake_phase3_rounds_done = 0;
            // payload[14]はスロット境界からのオフセットで、4us単位
            g_wake_phase3_bitoff =
                (length >= 15) ? (uint16_t)payload[14] * 4u : 0u;
            // payload[15]は応答の時刻に対するスロット単位の補正
            g_wake_phase3_target_slot_adjustment =
                (length >= 16) ? payload[15] : 0;
            // payload[16]はフェーズ1を停止してからフェーズ3を送信するまでのスロット数
            g_wake_phase3_gap =
                (length >= 17 && payload[16] != 0) ? payload[16] : 4;
            // payload[17]は応答を受信してからフェーズ3へ移るまでフェーズ1を最大速度で送信し続ける時間で、10ms単位である。
            g_wake_phase3_hold_ms =
                (length >= 18) ? (uint16_t)payload[17] * 10u : 0u;
            wake_roundtrip_run(host, interval, trigger, bursts, watch_ms,
                              page_after, payload[2], positions,
                              dwell_ms);
        } else {
            const uint32_t changes = switch1_csb_rx_watch(watch_ms);
            ESP_LOGI(TAG, "  Synchronization events detected during %lums: %lu",
                     (unsigned long)watch_ms, (unsigned long)changes);
        }
    }
    ESP_LOGI(TAG, "CSB receive %s: %s interval=%u slots",
             payload[0] != 0 ? "started" : "stopped", esp_err_to_name(err),
             interval);
    char text[16 * 5 + 1];
    int used = 0;
    for (int i = 0; i < 8; ++i) {
        used += snprintf(text + used, sizeof(text) - used, "%04x ",
                         before[i]);
    }
    ESP_LOGI(TAG, "  RX control structure before: %s", text);
    used = 0;
    for (int i = 0; i < 8; ++i) {
        used += snprintf(text + used, sizeof(text) - used, "%04x ",
                         after[i]);
    }
    ESP_LOGI(TAG, "  RX control structure after: %s", text);
}
