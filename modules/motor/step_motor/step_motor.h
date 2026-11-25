#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "bsp_can.h"
#include <stdbool.h>
#include <stdint.h>

#define ZDT_X42_MAX_RPM 5000U
#define ZDT_X42_PULSES_PER_REV 3200U

typedef enum
{
    ZDT_DIRECTION_CW = 0,
    ZDT_DIRECTION_CCW = 1,
} ZDTMotorDirection_e;

typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint8_t address;
    bool reverse;
    uint8_t default_acceleration;
    uint8_t tx_timeout_ms;
} ZDTMotorInitConfig_s;

typedef struct
{
    int16_t speed_rpm;
    int32_t position_pulses;
    uint8_t last_command;
    uint8_t last_status;
    uint32_t rx_count;
} ZDTMotorMeasure_s;

typedef struct
{
    CANInstance *can_instance;
    ZDTMotorMeasure_s measure;
    uint8_t address;
    uint8_t default_acceleration;
    uint8_t tx_timeout_ms;
    bool reverse;
    bool enabled;
} ZDTMotorInstance;

ZDTMotorInstance *ZDTMotorInit(const ZDTMotorInitConfig_s *config);
bool ZDTMotorEnable(ZDTMotorInstance *motor, bool enable, bool sync);
bool ZDTMotorSetSpeed(ZDTMotorInstance *motor, int16_t rpm, uint8_t acceleration, bool sync);
bool ZDTMotorSetRef(ZDTMotorInstance *motor, int16_t rpm);
bool ZDTMotorStop(ZDTMotorInstance *motor, bool sync);
bool ZDTMotorSetPosition(ZDTMotorInstance *motor, int32_t pulses, uint16_t rpm,
                         uint8_t acceleration, bool absolute, bool sync);
bool ZDTMotorRequestSpeed(ZDTMotorInstance *motor);
bool ZDTMotorRequestPosition(ZDTMotorInstance *motor);
bool ZDTMotorSyncStart(ZDTMotorInstance *motor);

#endif
