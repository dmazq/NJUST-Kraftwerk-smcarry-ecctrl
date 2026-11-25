#include "pwm_servo.h"
#include <stdlib.h>

#define PWM_SERVO_PERIOD_MS 20.0f
#define PWM_SERVO_PERIOD_US 20000.0f

static float Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

bool PWMServoSetAngle(PWMServoInstance *servo, float angle_deg)
{
    if (servo == NULL || servo->pwm == NULL)
        return false;

    float angle = Clamp(angle_deg, servo->min_angle_deg, servo->max_angle_deg);
    float ratio = (angle - servo->min_angle_deg) /
                  (servo->max_angle_deg - servo->min_angle_deg);
    float pulse_us = servo->min_pulse_us +
                     ratio * (servo->max_pulse_us - servo->min_pulse_us);

    PWMSetDutyRatio(servo->pwm, pulse_us * 100.0f / PWM_SERVO_PERIOD_US);
    servo->commanded_angle_deg = angle;
    return true;
}

PWMServoInstance *PWMServoInit(const PWMServoInitConfig_s *config)
{
    if (config == NULL || config->htim == NULL ||
        config->min_pulse_us >= config->max_pulse_us ||
        config->min_angle_deg >= config->max_angle_deg)
        return NULL;

    PWMServoInstance *servo = malloc(sizeof(PWMServoInstance));
    if (servo == NULL)
        return NULL;

    servo->min_pulse_us = config->min_pulse_us;
    servo->max_pulse_us = config->max_pulse_us;
    servo->min_angle_deg = config->min_angle_deg;
    servo->max_angle_deg = config->max_angle_deg;
    servo->commanded_angle_deg = config->initial_angle_deg;

    float initial_angle = Clamp(config->initial_angle_deg,
                               config->min_angle_deg, config->max_angle_deg);
    float initial_ratio = (initial_angle - config->min_angle_deg) /
                          (config->max_angle_deg - config->min_angle_deg);
    float initial_pulse_us = config->min_pulse_us +
                             initial_ratio * (config->max_pulse_us - config->min_pulse_us);
    PWM_Init_Config_s pwm_config = {
        .htim = config->htim,
        .channel = config->channel,
        .period = PWM_SERVO_PERIOD_MS,
        .dutyratio = initial_pulse_us * 100.0f / PWM_SERVO_PERIOD_US,
    };

    servo->pwm = PWMRegister(&pwm_config);
    if (servo->pwm == NULL)
    {
        free(servo);
        return NULL;
    }
    servo->commanded_angle_deg = initial_angle;
    return servo;
}

float PWMServoGetCommandedAngle(const PWMServoInstance *servo)
{
    return servo == NULL ? 0.0f : servo->commanded_angle_deg;
}
