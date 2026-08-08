#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0A00
#endif
#define WIN32_LEAN_AND_MEAN
#include "gamepad_input.h"

#include <windows.h>
#include <stddef.h>
#include <string.h>

#define SDL_INIT_GAMEPAD 0x00002000u
#define SDL_INIT_SENSOR  0x00008000u
#define SDL_SENSOR_ACCEL 1
#define SDL_SENSOR_GYRO 2
#define SDL_SENSOR_ACCEL_L 3
#define SDL_SENSOR_GYRO_L 4
#define SDL_SENSOR_ACCEL_R 5
#define SDL_SENSOR_GYRO_R 6

#define AXIS_LEFTX 0
#define AXIS_LEFTY 1
#define AXIS_RIGHTX 2
#define AXIS_RIGHTY 3
#define AXIS_LEFT_TRIGGER 4
#define AXIS_RIGHT_TRIGGER 5

#define BUTTON_SOUTH 0
#define BUTTON_EAST 1
#define BUTTON_WEST 2
#define BUTTON_NORTH 3
#define BUTTON_BACK 4
#define BUTTON_GUIDE 5
#define BUTTON_START 6
#define BUTTON_LEFT_STICK 7
#define BUTTON_RIGHT_STICK 8
#define BUTTON_LEFT_SHOULDER 9
#define BUTTON_RIGHT_SHOULDER 10
#define BUTTON_DPAD_UP 11
#define BUTTON_DPAD_DOWN 12
#define BUTTON_DPAD_LEFT 13
#define BUTTON_DPAD_RIGHT 14
#define BUTTON_MISC1 15
#define BUTTON_TOUCHPAD 20
#define TRIGGER_THRESHOLD 8192

typedef uint32_t SDL_JoystickID;

typedef union {
    uint32_t type;
    uint8_t padding[128];
} SDL_Event;

_Static_assert(sizeof(SDL_Event) == 128, "SDL_Event ABI mismatch");

typedef struct {
    HMODULE module;
    bool (*init)(uint32_t);
    bool (*set_hint)(const char *, const char *);
    void (*update_gamepads)(void);
    bool (*poll_event)(SDL_Event *);
    SDL_JoystickID *(*get_gamepads)(int *);
    SDL_Gamepad *(*open_gamepad)(SDL_JoystickID);
    void (*close_gamepad)(SDL_Gamepad *);
    bool (*gamepad_connected)(SDL_Gamepad *);
    int16_t (*get_gamepad_axis)(SDL_Gamepad *, int);
    bool (*get_gamepad_button)(SDL_Gamepad *, int);
    bool (*gamepad_has_sensor)(SDL_Gamepad *, int);
    bool (*set_gamepad_sensor_enabled)(SDL_Gamepad *, int, bool);
    bool (*get_gamepad_sensor_data)(SDL_Gamepad *, int, float *, int);
    void (*free_memory)(void *);
    void (*quit)(void);
} sdl_api_t;

static sdl_api_t sdl;

static HMODULE load_sdl3(void)
{
    wchar_t path[MAX_PATH];
    DWORD length = GetModuleFileNameW(NULL, path, MAX_PATH);
    if(length > 0 && length < MAX_PATH)
    {
        wchar_t *separator = wcsrchr(path, L'\\');
        if(separator != NULL)
        {
            wcscpy_s(separator + 1, MAX_PATH - (size_t)(separator + 1 - path), L"SDL3.dll");
            HMODULE module = LoadLibraryW(path);
            if(module != NULL) return module;
        }
    }
    length = GetEnvironmentVariableW(L"ProgramFiles(x86)", path, MAX_PATH);
    if(length > 0 && length < MAX_PATH &&
       wcscat_s(path, MAX_PATH, L"\\Steam\\SDL3.dll") == 0)
    {
        HMODULE module = LoadLibraryW(path);
        if(module != NULL) return module;
    }
    length = GetEnvironmentVariableW(L"ProgramFiles", path, MAX_PATH);
    if(length > 0 && length < MAX_PATH &&
       wcscat_s(path, MAX_PATH, L"\\Steam\\SDL3.dll") == 0)
        return LoadLibraryW(path);
    return NULL;
}

#define LOAD_SDL(symbol, field) do { \
    FARPROC procedure = GetProcAddress(sdl.module, symbol); \
    if(procedure == NULL) goto failed; \
    memcpy(&sdl.field, &procedure, sizeof(procedure)); \
} while(0)

static bool load_sdl(void)
{
    sdl.module = load_sdl3();
    if(sdl.module == NULL) return false;
    LOAD_SDL("SDL_Init", init);
    LOAD_SDL("SDL_SetHint", set_hint);
    LOAD_SDL("SDL_UpdateGamepads", update_gamepads);
    LOAD_SDL("SDL_PollEvent", poll_event);
    LOAD_SDL("SDL_GetGamepads", get_gamepads);
    LOAD_SDL("SDL_OpenGamepad", open_gamepad);
    LOAD_SDL("SDL_CloseGamepad", close_gamepad);
    LOAD_SDL("SDL_GamepadConnected", gamepad_connected);
    LOAD_SDL("SDL_GetGamepadAxis", get_gamepad_axis);
    LOAD_SDL("SDL_GetGamepadButton", get_gamepad_button);
    LOAD_SDL("SDL_GamepadHasSensor", gamepad_has_sensor);
    LOAD_SDL("SDL_SetGamepadSensorEnabled", set_gamepad_sensor_enabled);
    LOAD_SDL("SDL_GetGamepadSensorData", get_gamepad_sensor_data);
    LOAD_SDL("SDL_free", free_memory);
    LOAD_SDL("SDL_Quit", quit);
    return true;
failed:
    FreeLibrary(sdl.module);
    memset(&sdl, 0, sizeof(sdl));
    return false;
}

bool gamepad_input_initialize(void)
{
    return load_sdl() && sdl.set_hint("SDL_NO_SIGNAL_HANDLERS", "1") &&
           sdl.set_hint("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1") &&
           sdl.init(SDL_INIT_GAMEPAD | SDL_INIT_SENSOR);
}

void gamepad_input_update(void)
{
    sdl.update_gamepads();
}

void gamepad_input_poll_events(void)
{
    SDL_Event event;
    while(sdl.poll_event(&event)) { }
}

static int enable_sensor(SDL_Gamepad *gamepad, int a, int b, int c)
{
    int types[3] = { a, b, c };
    int i;
    for(i = 0; i < 3; i++)
    {
        if(sdl.gamepad_has_sensor(gamepad, types[i]) &&
           sdl.set_gamepad_sensor_enabled(gamepad, types[i], true)) return types[i];
    }
    return GAMEPAD_SENSOR_INVALID;
}

SDL_Gamepad *gamepad_input_find(int *accel_sensor, int *gyro_sensor)
{
    SDL_JoystickID *ids;
    SDL_Gamepad *gamepad = NULL;
    int count = 0;
    int i;

    sdl.update_gamepads();
    ids = sdl.get_gamepads(&count);
    for(i = 0; i < count; i++)
    {
        gamepad = sdl.open_gamepad(ids[i]);
        if(gamepad != NULL) break;
    }
    if(ids != NULL) sdl.free_memory(ids);
    if(gamepad != NULL)
    {
        *accel_sensor = enable_sensor(gamepad, SDL_SENSOR_ACCEL,
                                      SDL_SENSOR_ACCEL_L, SDL_SENSOR_ACCEL_R);
        *gyro_sensor = enable_sensor(gamepad, SDL_SENSOR_GYRO,
                                     SDL_SENSOR_GYRO_L, SDL_SENSOR_GYRO_R);
    }
    return gamepad;
}

bool gamepad_input_connected(SDL_Gamepad *gamepad)
{
    return sdl.gamepad_connected(gamepad);
}

bool gamepad_input_button(SDL_Gamepad *gamepad, int button)
{
    return sdl.get_gamepad_button(gamepad, button);
}

void gamepad_input_close(SDL_Gamepad *gamepad)
{
    sdl.close_gamepad(gamepad);
}

static uint16_t axis_to_switch(int16_t input, bool invert)
{
    int32_t value = input;
    if(invert) value = -value;
    if(value > 32767) value = 32767;
    if(value < -32768) value = -32768;
    return value >= 0 ? (uint16_t)(0x800 + value * 0x7ff / 32767)
                      : (uint16_t)(0x800 + value * 0x800 / 32768);
}

static int16_t clamp_sensor(float value)
{
    if(value > 32767.0f) return 32767;
    if(value < -32768.0f) return -32768;
    return (int16_t)(value + (value < 0.0f ? -0.5f : 0.5f));
}

static void pack_stick(uint8_t out[3], uint16_t horizontal, uint16_t vertical)
{
    out[0] = (uint8_t)horizontal;
    out[1] = (uint8_t)((horizontal >> 8) | ((vertical & 0x0f) << 4));
    out[2] = (uint8_t)(vertical >> 4);
}

static void put_i16le(uint8_t *out, int16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)((uint16_t)value >> 8);
}

static void read_motion(SDL_Gamepad *gamepad, int accel_sensor, int gyro_sensor,
                        int16_t accel_out[3], int16_t gyro_out[3])
{
    const float accel_scale = 4096.0f / 9.80665f;
    const float gyro_scale = 14.2842f * 180.0f / 3.14159265358979323846f;
    float data[3];

    accel_out[0] = 0;
    accel_out[1] = 0;
    accel_out[2] = 4096;
    gyro_out[0] = 0;
    gyro_out[1] = 0;
    gyro_out[2] = 0;
    if(accel_sensor != GAMEPAD_SENSOR_INVALID &&
       sdl.get_gamepad_sensor_data(gamepad, accel_sensor, data, 3))
    {
        accel_out[0] = clamp_sensor(-data[2] * accel_scale);
        accel_out[1] = clamp_sensor(-data[0] * accel_scale);
        accel_out[2] = clamp_sensor( data[1] * accel_scale);
    }
    if(gyro_sensor != GAMEPAD_SENSOR_INVALID &&
       sdl.get_gamepad_sensor_data(gamepad, gyro_sensor, data, 3))
    {
        gyro_out[0] = clamp_sensor(-data[2] * gyro_scale);
        gyro_out[1] = clamp_sensor(-data[0] * gyro_scale);
        gyro_out[2] = clamp_sensor( data[1] * gyro_scale);
    }
}

void gamepad_input_build_payload(uint8_t payload[INPUT_PAYLOAD_SIZE], SDL_Gamepad *gamepad,
                                 int accel_sensor, int gyro_sensor)
{
    uint32_t buttons = 0;
    int16_t accel[3];
    int16_t gyro[3];
    int i;

    if(sdl.get_gamepad_button(gamepad, BUTTON_SOUTH)) buttons |= 1u << 2;
    if(sdl.get_gamepad_button(gamepad, BUTTON_EAST)) buttons |= 1u << 3;
    if(sdl.get_gamepad_button(gamepad, BUTTON_WEST)) buttons |= 1u << 0;
    if(sdl.get_gamepad_button(gamepad, BUTTON_NORTH)) buttons |= 1u << 1;
    if(sdl.get_gamepad_button(gamepad, BUTTON_BACK)) buttons |= 1u << 8;
    if(sdl.get_gamepad_button(gamepad, BUTTON_GUIDE)) buttons |= 1u << 12;
    if(sdl.get_gamepad_button(gamepad, BUTTON_START)) buttons |= 1u << 9;
    if(sdl.get_gamepad_button(gamepad, BUTTON_LEFT_STICK)) buttons |= 1u << 11;
    if(sdl.get_gamepad_button(gamepad, BUTTON_RIGHT_STICK)) buttons |= 1u << 10;
    if(sdl.get_gamepad_button(gamepad, BUTTON_LEFT_SHOULDER)) buttons |= 1u << 22;
    if(sdl.get_gamepad_button(gamepad, BUTTON_RIGHT_SHOULDER)) buttons |= 1u << 6;
    if(sdl.get_gamepad_button(gamepad, BUTTON_DPAD_UP)) buttons |= 1u << 17;
    if(sdl.get_gamepad_button(gamepad, BUTTON_DPAD_DOWN)) buttons |= 1u << 16;
    if(sdl.get_gamepad_button(gamepad, BUTTON_DPAD_LEFT)) buttons |= 1u << 19;
    if(sdl.get_gamepad_button(gamepad, BUTTON_DPAD_RIGHT)) buttons |= 1u << 18;
    if(sdl.get_gamepad_button(gamepad, BUTTON_MISC1) ||
       sdl.get_gamepad_button(gamepad, BUTTON_TOUCHPAD)) buttons |= 1u << 13;
    if(sdl.get_gamepad_axis(gamepad, AXIS_LEFT_TRIGGER) > TRIGGER_THRESHOLD)
        buttons |= 1u << 23;
    if(sdl.get_gamepad_axis(gamepad, AXIS_RIGHT_TRIGGER) > TRIGGER_THRESHOLD)
        buttons |= 1u << 7;

    payload[0] = (uint8_t)buttons;
    payload[1] = (uint8_t)(buttons >> 8);
    payload[2] = (uint8_t)(buttons >> 16);
    pack_stick(&payload[3],
               axis_to_switch(sdl.get_gamepad_axis(gamepad, AXIS_LEFTX), false),
               axis_to_switch(sdl.get_gamepad_axis(gamepad, AXIS_LEFTY), true));
    pack_stick(&payload[6],
               axis_to_switch(sdl.get_gamepad_axis(gamepad, AXIS_RIGHTX), false),
               axis_to_switch(sdl.get_gamepad_axis(gamepad, AXIS_RIGHTY), true));
    read_motion(gamepad, accel_sensor, gyro_sensor, accel, gyro);
    for(i = 0; i < 3; i++)
    {
        put_i16le(&payload[9 + i * 2], accel[i]);
        put_i16le(&payload[15 + i * 2], gyro[i]);
    }
}

void gamepad_input_build_neutral_payload(uint8_t payload[INPUT_PAYLOAD_SIZE])
{
    memset(payload, 0, INPUT_PAYLOAD_SIZE);
    pack_stick(&payload[3], 0x800, 0x800);
    pack_stick(&payload[6], 0x800, 0x800);
    put_i16le(&payload[13], 4096);
}

void gamepad_input_shutdown(void)
{
    if(sdl.module != NULL)
    {
        sdl.quit();
        FreeLibrary(sdl.module);
        memset(&sdl, 0, sizeof(sdl));
    }
}
