#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0A00
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "gamepad_input.h"
#include "ns1_protocol.h"
#include "serial_bridge.h"

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

#define INPUT_PERIOD_MS 8
#define SEARCH_INTERVAL_MS 1000
#define WM_APP_LOG (WM_APP + 1)
#define WM_APP_PORT_OPENED (WM_APP + 2)
#define WM_APP_WORKER_ENDED (WM_APP + 3)

#define IDC_PORT 100
#define IDC_REFRESH 101
#define IDC_OPEN 102
#define IDC_CLOSE 103
#define IDC_PAIR 104
#define IDC_RECONNECT 105
#define IDC_WAKE_UP 106
#define IDC_DISCONNECT 107
#define IDC_LOG 108
#define IDC_BLE_SCAN 109
#define IDC_WAKE_MODE_NS1 110
#define IDC_WAKE_MODE_NS2 111

#define COMMAND_QUEUE_SIZE 32

typedef enum {
    ACTION_PAIR,
    ACTION_RECONNECT,
    ACTION_WAKE_UP,
    ACTION_DISCONNECT,
    ACTION_BLE_SCAN,
    ACTION_NS2_WAKE_UP
} action_t;

typedef struct {
    HWND window;
    HWND port_combo;
    HWND refresh_button;
    HWND open_button;
    HWND close_button;
    HWND pair_button;
    HWND reconnect_button;
    HWND wake_button;
    HWND disconnect_button;
    HWND ble_scan_button;
    HWND wake_mode_ns1;
    HWND wake_mode_ns2;
    HWND log_edit;
    HANDLE worker;
    HANDLE stop_event;
    CRITICAL_SECTION command_lock;
    action_t command_queue[COMMAND_QUEUE_SIZE];
    unsigned int command_head;
    unsigned int command_tail;
    char port[32];
    bool port_open;
    volatile LONG ns2_mode_selected;
} app_state_t;

static app_state_t app;
static char settings_path[MAX_PATH];

static action_t selected_wake_action(void)
{
    return InterlockedCompareExchange(&app.ns2_mode_selected, 0, 0) ?
           ACTION_NS2_WAKE_UP : ACTION_WAKE_UP;
}

static void load_settings(void)
{
    DWORD length = GetModuleFileNameA(NULL, settings_path, sizeof(settings_path));
    char *name = length > 0 && length < sizeof(settings_path) ?
                 strrchr(settings_path, '\\') : NULL;
    name = name != NULL ? name + 1 : settings_path;
    snprintf(name, sizeof(settings_path) - (size_t)(name - settings_path),
             "NS Input Passthrough.ini");
    GetPrivateProfileStringA("settings", "port", "", app.port,
                             sizeof(app.port), settings_path);
    InterlockedExchange(&app.ns2_mode_selected,
                        GetPrivateProfileIntA("settings", "ns2", 0,
                                              settings_path) != 0);
}

static void save_port_setting(void)
{
    GetWindowTextA(app.port_combo, app.port, sizeof(app.port));
    WritePrivateProfileStringA("settings", "port", app.port, settings_path);
}

static void save_mode_setting(void)
{
    const char *mode = selected_wake_action() == ACTION_NS2_WAKE_UP ? "1" : "0";
    WritePrivateProfileStringA("settings", "ns2", mode, settings_path);
}

static void post_log(void *context, const char *format, ...)
{
    HWND window = (HWND)context;
    char text[256];
    char *copy;
    va_list args;

    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    copy = HeapAlloc(GetProcessHeap(), 0, strlen(text) + 1);
    if(copy == NULL) return;
    strcpy(copy, text);
    if(!PostMessageA(window, WM_APP_LOG, 0, (LPARAM)copy))
        HeapFree(GetProcessHeap(), 0, copy);
}

static HANDLE create_tick_timer(void)
{
    HANDLE timer = CreateWaitableTimerExW(NULL, NULL,
                                          CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                          TIMER_MODIFY_STATE | SYNCHRONIZE);
    if(timer == NULL)
        timer = CreateWaitableTimerExW(NULL, NULL, 0, TIMER_MODIFY_STATE | SYNCHRONIZE);
    return timer;
}

static bool pop_action(action_t *action)
{
    bool found = false;
    EnterCriticalSection(&app.command_lock);
    if(app.command_head != app.command_tail)
    {
        *action = app.command_queue[app.command_head];
        app.command_head = (app.command_head + 1) % COMMAND_QUEUE_SIZE;
        found = true;
    }
    LeaveCriticalSection(&app.command_lock);
    return found;
}

static bool run_action(HANDLE serial, action_t action, bool *connect_requested)
{
    uint8_t command;
    const uint8_t start = 1;
    const uint8_t *payload = NULL;
    uint8_t length = 0;

    switch(action)
    {
    case ACTION_PAIR:
        post_log(app.window, "Pairing attempt started");
        command = CMD_PAIR;
        break;
    case ACTION_RECONNECT:
        post_log(app.window, "Reconnect attempt started");
        command = CMD_RECONNECT;
        break;
    case ACTION_WAKE_UP:
        post_log(app.window, "Wake-up attempt started");
        command = CMD_WAKE_UP;
        payload = &start;
        length = 1;
        break;
    case ACTION_DISCONNECT:
        post_log(app.window, "Disconnect attempt started");
        command = CMD_DISCONNECT;
        break;
    case ACTION_BLE_SCAN:
        post_log(app.window, "BLE scan started; press HOME on the paired NS2 controller");
        command = CMD_BLE_SCAN;
        break;
    case ACTION_NS2_WAKE_UP:
        post_log(app.window, "NS2 wake-up attempt started");
        command = CMD_NS2_WAKE_UP;
        break;
    default:
        return true;
    }

    if(!serial_send_frame(serial, command, payload, length))
    {
        post_log(app.window, "Error: command write failed (code %lu)",
                 (unsigned long)GetLastError());
        return false;
    }
    if(action == ACTION_WAKE_UP) *connect_requested = true;
    return true;
}

static DWORD WINAPI worker_main(LPVOID parameter)
{
    HANDLE serial = INVALID_HANDLE_VALUE;
    HANDLE timer = NULL;
    SDL_Gamepad *gamepad = NULL;
    int accel_sensor = GAMEPAD_SENSOR_INVALID;
    int gyro_sensor = GAMEPAD_SENSOR_INVALID;
    serial_reader_t reader;
    serial_status_t status;
    bool home_was_pressed = false;
    bool other_button_was_pressed = false;
    bool connect_requested = false;
    bool stop_sent = false;
    unsigned int write_failures = 0;
    LARGE_INTEGER frequency;
    LARGE_INTEGER now;
    LARGE_INTEGER due;
    int64_t search_tick = 0;
    bool port_was_opened = false;
    HANDLE waits[2];

    (void)parameter;
    memset(&reader, 0, sizeof(reader));
    memset(&status, 0, sizeof(status));

    if(!gamepad_input_initialize())
    {
        post_log(app.window, "Error: SDL3 initialization failed. Check SDL3.dll.");
        goto cleanup;
    }
    if(WaitForSingleObject(app.stop_event, 0) == WAIT_OBJECT_0) goto cleanup;
    serial = serial_open(app.port);
    if(serial == INVALID_HANDLE_VALUE)
    {
        post_log(app.window, "Error: cannot open %s (code %lu)", app.port,
                 (unsigned long)GetLastError());
        goto cleanup;
    }
    port_was_opened = true;
    post_log(app.window, "Port opened: %s", app.port);
    PostMessageA(app.window, WM_APP_PORT_OPENED, 0, 0);

    timer = create_tick_timer();
    if(timer == NULL)
    {
        post_log(app.window, "Error: cannot create the input timer.");
        goto cleanup;
    }
    due.QuadPart = -(LONGLONG)INPUT_PERIOD_MS * 10000;
    if(!SetWaitableTimer(timer, &due, INPUT_PERIOD_MS, NULL, NULL, FALSE))
    {
        post_log(app.window, "Error: cannot start the input timer.");
        goto cleanup;
    }

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&now);
    search_tick = now.QuadPart - frequency.QuadPart;
    waits[0] = app.stop_event;
    waits[1] = timer;
    while(WaitForMultipleObjects(2, waits, FALSE, 100) != WAIT_OBJECT_0)
    {
        uint8_t payload[INPUT_PAYLOAD_SIZE];
        action_t action;
        serial_drain(serial, &reader, &status, post_log, app.window);
        while(pop_action(&action)) run_action(serial, action, &connect_requested);

        if(status.connection_known && status.connected &&
           !serial_status_connected(&status))
        {
            status.connected = false;
            post_log(app.window, "Connection state: disconnected");
        }

        gamepad_input_update();
        gamepad_input_poll_events();
        QueryPerformanceCounter(&now);

        if(gamepad == NULL)
        {
            if(now.QuadPart - search_tick >=
               frequency.QuadPart * SEARCH_INTERVAL_MS / 1000)
            {
                search_tick = now.QuadPart;
                gamepad = gamepad_input_find(&accel_sensor, &gyro_sensor);
                if(gamepad != NULL)
                {
                    home_was_pressed = false;
                    other_button_was_pressed = false;
                }
            }
        }
        else if(!gamepad_input_connected(gamepad))
        {
            gamepad_input_build_neutral_payload(payload);
            serial_send_frame(serial, CMD_INPUT_STATE, payload, INPUT_PAYLOAD_SIZE);
            gamepad_input_close(gamepad);
            gamepad = NULL;
            accel_sensor = GAMEPAD_SENSOR_INVALID;
            gyro_sensor = GAMEPAD_SENSOR_INVALID;
            home_was_pressed = false;
            other_button_was_pressed = false;
            search_tick = now.QuadPart;
        }
        else
        {
            bool home_pressed;
            bool other_button_pressed;
            bool connected;
            gamepad_input_build_payload(payload, gamepad, accel_sensor, gyro_sensor);
            connected = serial_status_connected(&status);
            if(connected)
            {
                if(serial_send_frame(serial, CMD_INPUT_STATE, payload,
                                      INPUT_PAYLOAD_SIZE))
                    write_failures = 0;
                else if(++write_failures >= 8)
                {
                    post_log(app.window, "Error: repeated writes to %s failed (code %lu)",
                             app.port, (unsigned long)GetLastError());
                    goto cleanup;
                }
            }
            home_pressed = gamepad_input_button(gamepad, GAMEPAD_BUTTON_GUIDE);
            other_button_pressed =
                (((uint32_t)payload[0] | ((uint32_t)payload[1] << 8) |
                  ((uint32_t)payload[2] << 16)) & ~(UINT32_C(1) << 12)) != 0;
            if(!connected)
            {
                if(home_pressed && !home_was_pressed)
                    run_action(serial, selected_wake_action(), &connect_requested);
                else if(other_button_pressed && !other_button_was_pressed &&
                        !connect_requested)
                    run_action(serial, ACTION_RECONNECT, &connect_requested);
            }
            home_was_pressed = home_pressed;
            other_button_was_pressed = other_button_pressed;
        }

        if(serial_status_connected(&status))
        {
            if(!stop_sent)
            {
                const uint8_t stop = 0x00;
                if(serial_send_frame(serial, CMD_WAKE_UP, &stop, 1))
                {
                    stop_sent = true;
                    connect_requested = false;
                }
            }
        }
        else
            stop_sent = false;
    }

cleanup:
    if(serial != INVALID_HANDLE_VALUE)
    {
        uint8_t payload[INPUT_PAYLOAD_SIZE];
        gamepad_input_build_neutral_payload(payload);
        serial_send_frame(serial, CMD_INPUT_STATE, payload, INPUT_PAYLOAD_SIZE);
        if(connect_requested)
        {
            const uint8_t stop = 0x00;
            serial_send_frame(serial, CMD_WAKE_UP, &stop, 1);
        }
        serial_close(serial);
        if(port_was_opened) post_log(app.window, "Port closed: %s", app.port);
    }
    if(timer != NULL)
    {
        CancelWaitableTimer(timer);
        CloseHandle(timer);
    }
    if(gamepad != NULL) gamepad_input_close(gamepad);
    gamepad_input_shutdown();
    PostMessageA(app.window, WM_APP_WORKER_ENDED, 0, 0);
    return 0;
}

static void append_log(const char *text)
{
    LRESULT length = GetWindowTextLengthA(app.log_edit);
    SendMessageA(app.log_edit, EM_SETSEL, (WPARAM)length, (LPARAM)length);
    SendMessageA(app.log_edit, EM_REPLACESEL, FALSE, (LPARAM)text);
    SendMessageA(app.log_edit, EM_REPLACESEL, FALSE, (LPARAM)"\r\n");
}

static void update_controls(void)
{
    BOOL running = app.worker != NULL;
    EnableWindow(app.port_combo, !running);
    EnableWindow(app.refresh_button, !running);
    EnableWindow(app.open_button, !running &&
                 SendMessageA(app.port_combo, CB_GETCOUNT, 0, 0) > 0);
    EnableWindow(app.close_button, running);
    EnableWindow(app.pair_button, app.port_open);
    EnableWindow(app.reconnect_button, app.port_open);
    EnableWindow(app.wake_button, app.port_open);
    EnableWindow(app.disconnect_button, app.port_open);
    EnableWindow(app.ble_scan_button, app.port_open &&
                 selected_wake_action() == ACTION_NS2_WAKE_UP);
}

static void refresh_ports(void)
{
    char selected[32] = "";
    char name[32];
    char target[512];
    int selected_index = -1;
    unsigned int number;

    GetWindowTextA(app.port_combo, selected, sizeof(selected));
    if(selected[0] == '\0' && app.port[0] != '\0')
        strcpy_s(selected, sizeof(selected), app.port);
    SendMessageA(app.port_combo, CB_RESETCONTENT, 0, 0);
    for(number = 1; number <= 256; number++)
    {
        int index;
        snprintf(name, sizeof(name), "COM%u", number);
        if(QueryDosDeviceA(name, target, sizeof(target)) == 0) continue;
        index = (int)SendMessageA(app.port_combo, CB_ADDSTRING, 0, (LPARAM)name);
        if(strcmp(name, selected) == 0) selected_index = index;
    }
    if(selected_index < 0 && SendMessageA(app.port_combo, CB_GETCOUNT, 0, 0) > 0)
        selected_index = 0;
    if(selected_index >= 0)
        SendMessageA(app.port_combo, CB_SETCURSEL, (WPARAM)selected_index, 0);
    update_controls();
}

static void start_worker(void)
{
    if(app.worker != NULL ||
       GetWindowTextA(app.port_combo, app.port, sizeof(app.port)) == 0) return;
    save_port_setting();
    app.command_head = 0;
    app.command_tail = 0;
    app.port_open = false;
    app.stop_event = CreateEventA(NULL, TRUE, FALSE, NULL);
    if(app.stop_event == NULL)
    {
        append_log("Error: cannot create the worker stop event.");
        return;
    }
    app.worker = CreateThread(NULL, 0, worker_main, NULL, 0, NULL);
    if(app.worker == NULL)
    {
        append_log("Error: cannot start the worker thread.");
        CloseHandle(app.stop_event);
        app.stop_event = NULL;
    }
    update_controls();
}

static void queue_action(action_t action)
{
    unsigned int next;
    bool queued = false;
    EnterCriticalSection(&app.command_lock);
    next = (app.command_tail + 1) % COMMAND_QUEUE_SIZE;
    if(next != app.command_head)
    {
        app.command_queue[app.command_tail] = action;
        app.command_tail = next;
        queued = true;
    }
    LeaveCriticalSection(&app.command_lock);
    if(!queued) append_log("Error: command queue is full.");
}

static HWND make_control(DWORD ex_style, const char *class_name, const char *text,
                         DWORD style, int x, int y, int width, int height, int id)
{
    HWND control = CreateWindowExA(ex_style, class_name, text,
                                   WS_CHILD | WS_VISIBLE | style,
                                   x, y, width, height, app.window,
                                   (HMENU)(INT_PTR)id, GetModuleHandleA(NULL), NULL);
    SendMessageA(control, WM_SETFONT, (WPARAM)GetStockObject(DEFAULT_GUI_FONT), TRUE);
    return control;
}

static LRESULT CALLBACK window_proc(HWND window, UINT message,
                                    WPARAM wparam, LPARAM lparam)
{
    switch(message)
    {
    case WM_CREATE:
        app.window = window;
        make_control(0, "STATIC", "Serial port:", 0, 16, 17, 75, 20, 0);
        app.port_combo = make_control(0, "COMBOBOX", "",
                                      CBS_DROPDOWNLIST | WS_VSCROLL | WS_TABSTOP,
                                      92, 13, 130, 250, IDC_PORT);
        app.refresh_button = make_control(0, "BUTTON", "Refresh", WS_TABSTOP,
                                          230, 12, 75, 25, IDC_REFRESH);
        app.open_button = make_control(0, "BUTTON", "Open Port", WS_TABSTOP,
                                       313, 12, 85, 25, IDC_OPEN);
        app.close_button = make_control(0, "BUTTON", "Close Port", WS_TABSTOP,
                                        406, 12, 85, 25, IDC_CLOSE);
        app.pair_button = make_control(0, "BUTTON", "Pair", WS_TABSTOP,
                                       16, 51, 105, 28, IDC_PAIR);
        app.reconnect_button = make_control(0, "BUTTON", "Reconnect", WS_TABSTOP,
                                            129, 51, 105, 28, IDC_RECONNECT);
        app.wake_button = make_control(0, "BUTTON", "Wake Up", WS_TABSTOP,
                                       242, 51, 105, 28, IDC_WAKE_UP);
        app.disconnect_button = make_control(0, "BUTTON", "Disconnect", WS_TABSTOP,
                                             355, 51, 136, 28, IDC_DISCONNECT);
        app.ble_scan_button = make_control(0, "BUTTON", "BLE Scan", WS_TABSTOP,
                                           16, 87, 105, 28, IDC_BLE_SCAN);
        make_control(0, "STATIC", "Wake mode:", 0, 145, 94, 75, 20, 0);
        app.wake_mode_ns1 = make_control(0, "BUTTON", "NS1",
                                         BS_AUTORADIOBUTTON | WS_GROUP | WS_TABSTOP,
                                         222, 90, 55, 24, IDC_WAKE_MODE_NS1);
        app.wake_mode_ns2 = make_control(0, "BUTTON", "NS2",
                                         BS_AUTORADIOBUTTON | WS_TABSTOP,
                                         285, 90, 55, 24, IDC_WAKE_MODE_NS2);
        SendMessageA(app.wake_mode_ns1, BM_SETCHECK,
                     selected_wake_action() == ACTION_WAKE_UP ? BST_CHECKED : BST_UNCHECKED,
                     0);
        SendMessageA(app.wake_mode_ns2, BM_SETCHECK,
                     selected_wake_action() == ACTION_NS2_WAKE_UP ? BST_CHECKED : BST_UNCHECKED,
                     0);
        app.log_edit = make_control(WS_EX_CLIENTEDGE, "EDIT", "",
                                    ES_MULTILINE | ES_AUTOVSCROLL | ES_READONLY |
                                    WS_VSCROLL | WS_TABSTOP,
                                    16, 128, 475, 238, IDC_LOG);
        SendMessageA(app.log_edit, EM_SETLIMITTEXT, 1024 * 1024, 0);
        refresh_ports();
        return 0;

    case WM_COMMAND:
        if(LOWORD(wparam) == IDC_PORT && HIWORD(wparam) == CBN_SELCHANGE)
        {
            save_port_setting();
            return 0;
        }
        if(HIWORD(wparam) != BN_CLICKED) break;
        switch(LOWORD(wparam))
        {
        case IDC_REFRESH: refresh_ports(); break;
        case IDC_OPEN: start_worker(); break;
        case IDC_CLOSE:
            if(app.stop_event != NULL) SetEvent(app.stop_event);
            app.port_open = false;
            update_controls();
            break;
        case IDC_PAIR: queue_action(ACTION_PAIR); break;
        case IDC_RECONNECT: queue_action(ACTION_RECONNECT); break;
        case IDC_WAKE_UP: queue_action(selected_wake_action()); break;
        case IDC_DISCONNECT: queue_action(ACTION_DISCONNECT); break;
        case IDC_BLE_SCAN: queue_action(ACTION_BLE_SCAN); break;
        case IDC_WAKE_MODE_NS1:
        case IDC_WAKE_MODE_NS2:
            InterlockedExchange(&app.ns2_mode_selected,
                                LOWORD(wparam) == IDC_WAKE_MODE_NS2);
            save_mode_setting();
            update_controls();
            break;
        }
        return 0;

    case WM_APP_LOG:
        append_log((const char *)lparam);
        HeapFree(GetProcessHeap(), 0, (void *)lparam);
        return 0;
    case WM_APP_PORT_OPENED:
        app.port_open = true;
        update_controls();
        return 0;
    case WM_APP_WORKER_ENDED:
        if(app.worker != NULL)
        {
            WaitForSingleObject(app.worker, INFINITE);
            CloseHandle(app.worker);
            app.worker = NULL;
        }
        if(app.stop_event != NULL)
        {
            CloseHandle(app.stop_event);
            app.stop_event = NULL;
        }
        app.port_open = false;
        update_controls();
        return 0;
    case WM_CLOSE:
        if(app.stop_event != NULL) SetEvent(app.stop_event);
        DestroyWindow(window);
        return 0;
    case WM_DESTROY:
        if(app.worker != NULL)
        {
            WaitForSingleObject(app.worker, INFINITE);
            CloseHandle(app.worker);
            app.worker = NULL;
        }
        if(app.stop_event != NULL)
        {
            CloseHandle(app.stop_event);
            app.stop_event = NULL;
        }
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcA(window, message, wparam, lparam);
}

int WINAPI WinMain(HINSTANCE instance, HINSTANCE previous,
                   LPSTR command_line, int show_command)
{
    const char class_name[] = "NSInputPassthrough";
    WNDCLASSA window_class;
    MSG message;

    (void)previous;
    (void)command_line;
    memset(&app, 0, sizeof(app));
    load_settings();
    InitializeCriticalSection(&app.command_lock);
    if(!serial_status_parser_self_check())
    {
        MessageBoxA(NULL, "Status parser self-check failed.", "NS Input Passthrough",
                    MB_OK | MB_ICONERROR);
        DeleteCriticalSection(&app.command_lock);
        return 1;
    }
    memset(&window_class, 0, sizeof(window_class));
    window_class.lpfnWndProc = window_proc;
    window_class.hInstance = instance;
    window_class.hCursor = LoadCursorA(NULL, IDC_ARROW);
    window_class.hIcon = LoadIconA(NULL, IDI_APPLICATION);
    window_class.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
    window_class.lpszClassName = class_name;
    if(!RegisterClassA(&window_class))
    {
        DeleteCriticalSection(&app.command_lock);
        return 1;
    }
    app.window = CreateWindowExA(0, class_name, "NS Input Passthrough",
                                  WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU |
                                  WS_MINIMIZEBOX,
                                  CW_USEDEFAULT, CW_USEDEFAULT, 525, 421,
                                  NULL, NULL, instance, NULL);
    if(app.window == NULL)
    {
        DeleteCriticalSection(&app.command_lock);
        return 1;
    }
    ShowWindow(app.window, show_command);
    UpdateWindow(app.window);
    while(GetMessageA(&message, NULL, 0, 0) > 0)
    {
        TranslateMessage(&message);
        DispatchMessageA(&message);
    }
    DeleteCriticalSection(&app.command_lock);
    return (int)message.wParam;
}
