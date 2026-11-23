#ifndef BOOT_CTRL_H
#define BOOT_CTRL_H

/** Select wake mode, initialize a fresh wake cycle, and reboot. */
void boot_request_wake(void);

typedef enum {
    WAKE_OK = 0,
    WAKE_ERR_INIT,
    WAKE_ERR_NO_HOST,
    WAKE_ERR_TIMEOUT,
    WAKE_ERR_RETRY_LIMIT,
    WAKE_ERR_NO_CONNECTION,
    WAKE_ERR_INPUT,
    WAKE_ERR_RESET,
} wake_result_t;

/** Return to Wi-Fi with a retained error code; never returns. */
void boot_return_wifi(wake_result_t result) __attribute__((noreturn));

/** Report a retained error once Wi-Fi is available. */
void boot_report_pending_error(void);

#endif
