#ifndef PWM_SERVO_H
#define PWM_SERVO_H

#include "bsp_pwm.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    float min_angle_deg;
    float max_angle_deg;
    float initial_angle_deg;
} PWMServoInitConfig_s;

typedef struct
{
    PWMInstance *pwm;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    float min_angle_deg;
    float max_angle_deg;
    float commanded_angle_deg;
} PWMServoInstance;

PWMServoInstance *PWMServoInit(const PWMServoInitConfig_s *config);
bool PWMServoSetAngle(PWMServoInstance *servo, float angle_deg);
float PWMServoGetCommandedAngle(const PWMServoInstance *servo);

#endif
