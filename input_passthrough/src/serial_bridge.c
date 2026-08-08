#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0A00
#endif
#define WIN32_LEAN_AND_MEAN
#include "serial_bridge.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MARKER_0 0xAB
#define FRAME_MARKER_1 0xCD
#define FRAME_OVERHEAD 5
#define STATUS_STALE_MS 3000

HANDLE serial_open(const char *port)
{
    wchar_t path[64];
    size_t length = strlen(port);
    size_t i;
    HANDLE handle;
    DCB dcb;
    COMMTIMEOUTS timeouts;

    if(length == 0 || length + 5 > (sizeof(path) / sizeof(path[0])))
        return INVALID_HANDLE_VALUE;
    path[0] = L'\\'; path[1] = L'\\'; path[2] = L'.'; path[3] = L'\\';
    for(i = 0; i < length; i++) path[4 + i] = (wchar_t)(unsigned char)port[i];
    path[4 + length] = L'\0';

    handle = CreateFileW(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                         OPEN_EXISTING, 0, NULL);
    if(handle == INVALID_HANDLE_VALUE) return INVALID_HANDLE_VALUE;
    EscapeCommFunction(handle, CLRDTR);
    EscapeCommFunction(handle, CLRRTS);

    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if(!GetCommState(handle, &dcb))
    {
        CloseHandle(handle);
        return INVALID_HANDLE_VALUE;
    }
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fErrorChar = FALSE;
    dcb.fNull = FALSE;
    dcb.fAbortOnError = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    if(!SetCommState(handle, &dcb))
    {
        CloseHandle(handle);
        return INVALID_HANDLE_VALUE;
    }
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 200;
    if(!SetCommTimeouts(handle, &timeouts))
    {
        CloseHandle(handle);
        return INVALID_HANDLE_VALUE;
    }
    PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return handle;
}

void serial_close(HANDLE handle)
{
    CloseHandle(handle);
}

bool serial_send_frame(HANDLE handle, uint8_t command, const uint8_t *payload,
                       uint8_t length)
{
    uint8_t frame[FRAME_OVERHEAD + 255];
    uint8_t checksum = (uint8_t)(command ^ length);
    unsigned int i;
    DWORD written = 0;
    DWORD total = (DWORD)length + FRAME_OVERHEAD;

    frame[0] = FRAME_MARKER_0;
    frame[1] = FRAME_MARKER_1;
    frame[2] = command;
    frame[3] = length;
    for(i = 0; i < length; i++)
    {
        frame[4 + i] = payload[i];
        checksum ^= payload[i];
    }
    frame[4 + length] = checksum;
    return WriteFile(handle, frame, total, &written, NULL) && written == total;
}

static bool status_field(const char *line, char key, unsigned long *value)
{
    const char *p;
    for(p = line; *p != '\0'; p++)
    {
        if(p[0] != key || p[1] != '=') continue;
        if(p != line && p[-1] != ' ') continue;
        {
            char *end = NULL;
            unsigned long parsed = strtoul(p + 2, &end, 10);
            if(end == p + 2) return false;
            *value = parsed;
            return true;
        }
    }
    return false;
}

static bool status_text_field(const char *line, char key, char *value, size_t size)
{
    const char *p;
    for(p = line; *p != '\0'; p++)
    {
        const char *end;
        size_t length;
        if(p[0] != key || p[1] != '=') continue;
        if(p != line && p[-1] != ' ') continue;
        p += 2;
        end = p;
        while(*end != '\0' && *end != ' ') end++;
        length = (size_t)(end - p);
        if(length == 0 || length >= size) return false;
        memcpy(value, p, length);
        value[length] = '\0';
        return true;
    }
    return false;
}

static void parse_status_line(const char *line, serial_status_t *status,
                              serial_log_fn log, void *context)
{
    unsigned long connected = 0;
    char target[32];
    const char *notice = strstr(line, "#N ");
    const char *body = strstr(line, "#S ");

    if(notice != NULL)
    {
        log(context, "%s", notice + 3);
        return;
    }
    if(body == NULL) return;
    if(status_field(body, 'c', &connected))
    {
        bool next = connected != 0;
        if(!status->connection_known || status->connected != next)
            log(context, "Connection state: %s", next ? "connected" : "disconnected");
        status->connected = next;
        status->connection_known = true;
        status->updated_ms = (int64_t)GetTickCount64();
    }
    if(status_text_field(body, 'h', target, sizeof(target)) &&
       (!status->target_known || strcmp(status->target, target) != 0))
    {
        strcpy(status->target, target);
        status->target_known = true;
        log(context, "Pairing target: %s", target);
    }
}

static void feed_serial(serial_reader_t *reader, const uint8_t *data, size_t count,
                        serial_status_t *status, serial_log_fn log, void *context)
{
    size_t i;
    for(i = 0; i < count; i++)
    {
        char c = (char)data[i];
        if(c == '\r') continue;
        if(c == '\n')
        {
            reader->buffer[reader->length] = '\0';
            parse_status_line(reader->buffer, status, log, context);
            reader->length = 0;
            continue;
        }
        if(reader->length + 1 < sizeof(reader->buffer))
            reader->buffer[reader->length++] = c;
    }
}

void serial_drain(HANDLE handle, serial_reader_t *reader, serial_status_t *status,
                  serial_log_fn log, void *context)
{
    uint8_t buffer[512];
    DWORD count = 0;
    while(ReadFile(handle, buffer, sizeof(buffer), &count, NULL) && count > 0)
    {
        feed_serial(reader, buffer, count, status, log, context);
        if(count < sizeof(buffer)) break;
    }
}

bool serial_status_connected(const serial_status_t *status)
{
    if(!status->connected) return false;
    return (int64_t)GetTickCount64() - status->updated_ms <= STATUS_STALE_MS;
}

bool serial_status_parser_self_check(void)
{
    const char line[] = "#S c=1";
    const char notice[] = "W (123) ns2_wake: #N NS2 wake data saved";
    unsigned long connected = 0;

    return status_field(line, 'c', &connected) && connected == 1 &&
           strcmp(strstr(notice, "#N ") + 3, "NS2 wake data saved") == 0;
}
