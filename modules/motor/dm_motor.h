#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "bsp_can.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint16_t can_id;
    uint16_t master_id;
    float position_max;
    float velocity_max;
    float torque_max;
    uint8_t tx_timeout_ms;
} DMMotorInitConfig_s;

typedef struct
{
    float position;
    float velocity;
    float torque;
    uint8_t state;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    uint32_t rx_count;
    uint32_t last_rx_ms;
} DMMotorMeasure_s;

typedef struct
{
    CANInstance *can_instance;
    DMMotorMeasure_s measure;
    float position_max;
    float velocity_max;
    float torque_max;
    uint8_t tx_timeout_ms;
    bool enabled;
} DMMotorInstance;

DMMotorInstance *DMMotorInit(const DMMotorInitConfig_s *config);
bool DMMotorEnable(DMMotorInstance *motor);
bool DMMotorDisable(DMMotorInstance *motor);
bool DMMotorSetMIT(DMMotorInstance *motor, float position, float velocity,
                   float kp, float kd, float torque);
bool DMMotorSetVelocity(DMMotorInstance *motor, float velocity, float kd);
bool DMMotorIsOnline(const DMMotorInstance *motor, uint32_t timeout_ms);

#endif
