#ifndef SERIAL_REMOTE_H
#define SERIAL_REMOTE_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#define SERIAL_REMOTE_FRAME_SIZE 15U

typedef struct
{
    int8_t left_x;
    int8_t left_y;
    int8_t right_x;
    int8_t right_y;
    uint8_t button_1;
    uint8_t button_2;
    uint8_t button_3;
    uint8_t right_trigger;
    uint8_t left_trigger;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t recovery_count;
    uint32_t last_update_ms;
    bool connected;
} SerialRemoteData_s;

const SerialRemoteData_s *SerialRemoteInit(UART_HandleTypeDef *uart_handle);
bool SerialRemoteIsOnline(void);

#endif
