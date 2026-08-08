#ifndef GAMEPAD_INPUT_H
#define GAMEPAD_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "ns1_protocol.h"

typedef struct SDL_Gamepad SDL_Gamepad;

#define GAMEPAD_SENSOR_INVALID (-1)
#define GAMEPAD_BUTTON_GUIDE 5

bool gamepad_input_initialize(void);
void gamepad_input_update(void);
void gamepad_input_poll_events(void);
SDL_Gamepad *gamepad_input_find(int *accel_sensor, int *gyro_sensor);
bool gamepad_input_connected(SDL_Gamepad *gamepad);
bool gamepad_input_button(SDL_Gamepad *gamepad, int button);
void gamepad_input_close(SDL_Gamepad *gamepad);
void gamepad_input_build_payload(uint8_t payload[INPUT_PAYLOAD_SIZE], SDL_Gamepad *gamepad,
                                 int accel_sensor, int gyro_sensor);
void gamepad_input_build_neutral_payload(uint8_t payload[INPUT_PAYLOAD_SIZE]);
void gamepad_input_shutdown(void);

#endif
