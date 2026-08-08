#ifndef SERIAL_BRIDGE_H
#define SERIAL_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <windows.h>

typedef struct {
    bool connected;
    bool connection_known;
    int64_t updated_ms;
    char target[32];
    bool target_known;
} serial_status_t;

typedef struct {
    char buffer[256];
    size_t length;
} serial_reader_t;

typedef void (*serial_log_fn)(void *context, const char *format, ...);

HANDLE serial_open(const char *port);
void serial_close(HANDLE handle);
bool serial_send_frame(HANDLE handle, uint8_t command, const uint8_t *payload,
                       uint8_t length);
void serial_drain(HANDLE handle, serial_reader_t *reader, serial_status_t *status,
                  serial_log_fn log, void *context);
bool serial_status_connected(const serial_status_t *status);
bool serial_status_parser_self_check(void);

#endif
