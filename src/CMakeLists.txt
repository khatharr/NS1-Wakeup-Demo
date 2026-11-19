idf_component_register(
    SRCS "main.c"
         "bt/swpro_link.c" "bt/swpro_pairing.c" "bt/host_sec_state.c"
         "hid/swpro_hid.c" "hid/swpro_spi.c"
         "bridge/swpro_uart.c"
         "wake/wake_session.c" "wake/wake_tx.c" "wake/wake_roundtrip.c"
         "wake/wake_payload.c" "wake/wakeup_code.c" "wake/wake_cycle.c"
         "wake/ns2_wake.c"
         "baseband/bb_hci.c" "baseband/bb_csb_tx.c" "baseband/bb_csb_rx.c"
         "baseband/bb_stage2.c"
         "baseband/bb_handoff.c" "baseband/bb_lmp.c"
    INCLUDE_DIRS "." "bt" "hid" "bridge" "wake" "baseband"
    REQUIRES bt esp_hid nvs_flash esp_timer esp_driver_uart
)

# CSBAdd Bluedroid internal headers to the include path so CSB HCI commands can be constructed directly.
set(BLUEDROID_DIR "$ENV{IDF_PATH}/components/bt/host/bluedroid")
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${BLUEDROID_DIR}/common/include"
    "${BLUEDROID_DIR}/stack/include"
    # host_sec_state.cNeeded for host_sec_state.c to access tBTM_SEC_DEV_REC fields by name.
    # The structure layout depends on sdkconfig, so do not use fixed offsets.
    "${BLUEDROID_DIR}/stack/btm/include"
    "${BLUEDROID_DIR}/osi/include"
    "${BLUEDROID_DIR}/hci/include"
    "${BLUEDROID_DIR}/device/include"
    "${BLUEDROID_DIR}/api/include"
    "$ENV{IDF_PATH}/components/bt/common/include"
    "$ENV{IDF_PATH}/components/bt/common/osi/include"
    "$ENV{IDF_PATH}/components/bt/common/api/include"
)
