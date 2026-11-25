#include "chassis.h"
#include "robot_def.h"
#include "message_center.h"
#include "step_motor.h"
#include <limits.h>

enum
{
    WHEEL_LF = 0,
    WHEEL_RF,
    WHEEL_LB,
    WHEEL_RB,
    WHEEL_COUNT,
};

#define CHASSIS_MOTOR_ACCELERATION 180U
#define CHASSIS_MAX_WHEEL_RPM 300.0f
#define CHASSIS_POWER_ON_DELAY_MS 1000U
#define CHASSIS_ENABLE_SETTLE_MS 100U
#define CHASSIS_CMD_TIMEOUT_MS 100U

// Adjust only these flags if a wheel rotates opposite to the expected direction.
#define CHASSIS_LF_REVERSE false
#define CHASSIS_RF_REVERSE true
#define CHASSIS_LB_REVERSE false
#define CHASSIS_RB_REVERSE true

typedef enum
{
    CHASSIS_WAIT_POWER_ON,
    CHASSIS_ENABLE_MOTORS,
    CHASSIS_WAIT_ENABLE,
    CHASSIS_RUNNING,
} ChassisState_e;

static ZDTMotorInstance *wheel_motor[WHEEL_COUNT];
static Subscriber_t *chassis_sub;
static Chassis_Ctrl_Cmd_s chassis_cmd;
static ChassisState_e chassis_state;
static uint32_t state_time_ms;
static uint32_t last_cmd_time_ms;
static uint8_t enable_index;
static int16_t last_wheel_ref[WHEEL_COUNT];
static bool stopped = true;

static int16_t FloatToRpm(float value)
{
    if (value > (float)INT16_MAX)
        return INT16_MAX;
    if (value < (float)INT16_MIN)
        return INT16_MIN;
    return (int16_t)value;
}

static void ChassisInverseKinematics(const Chassis_Ctrl_Cmd_s *cmd,
                                     int16_t wheel_ref[WHEEL_COUNT])
{
    // X-pattern mecanum: vx forward, vy right, wz clockwise.
    float raw[WHEEL_COUNT] = {
        cmd->vx + cmd->vy + cmd->wz,
        cmd->vx - cmd->vy - cmd->wz,
        cmd->vx - cmd->vy + cmd->wz,
        cmd->vx + cmd->vy - cmd->wz,
    };

    float max_abs = 0.0f;
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        float magnitude = raw[i] >= 0.0f ? raw[i] : -raw[i];
        if (magnitude > max_abs)
            max_abs = magnitude;
    }

    float scale = max_abs > CHASSIS_MAX_WHEEL_RPM ?
                  CHASSIS_MAX_WHEEL_RPM / max_abs : 1.0f;
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
        wheel_ref[i] = FloatToRpm(raw[i] * scale);
}

static void ChassisStopAll(void)
{
    if (stopped)
        return;

    bool all_sent = true;
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        if (!ZDTMotorStop(wheel_motor[i], false))
            all_sent = false;
        else
            last_wheel_ref[i] = 0;
    }
    stopped = all_sent;
}

static void ChassisSendWheelRefs(const int16_t wheel_ref[WHEEL_COUNT])
{
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        if (wheel_ref[i] != last_wheel_ref[i] &&
            ZDTMotorSetRef(wheel_motor[i], wheel_ref[i]))
            last_wheel_ref[i] = wheel_ref[i];
    }
    stopped = false;
}

void ChassisInit(void)
{
    const ZDTMotorInitConfig_s wheel_config[WHEEL_COUNT] = {
        {.can_handle = &hcan1, .address = 1, .reverse = CHASSIS_LF_REVERSE,
         .default_acceleration = CHASSIS_MOTOR_ACCELERATION, .tx_timeout_ms = 2},
        {.can_handle = &hcan1, .address = 2, .reverse = CHASSIS_RF_REVERSE,
         .default_acceleration = CHASSIS_MOTOR_ACCELERATION, .tx_timeout_ms = 2},
        {.can_handle = &hcan1, .address = 3, .reverse = CHASSIS_LB_REVERSE,
         .default_acceleration = CHASSIS_MOTOR_ACCELERATION, .tx_timeout_ms = 2},
        {.can_handle = &hcan1, .address = 4, .reverse = CHASSIS_RB_REVERSE,
         .default_acceleration = CHASSIS_MOTOR_ACCELERATION, .tx_timeout_ms = 2},
    };

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        wheel_motor[i] = ZDTMotorInit(&wheel_config[i]);
        last_wheel_ref[i] = INT16_MIN;
    }

    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_cmd.chassis_mode = CHASSIS_STOP;
    enable_index = 0U;
    stopped = true;
    state_time_ms = HAL_GetTick();
    last_cmd_time_ms = state_time_ms;
    chassis_state = CHASSIS_WAIT_POWER_ON;
}

void ChassisTask(void)
{
    uint32_t now = HAL_GetTick();
    if (SubGetMessage(chassis_sub, &chassis_cmd))
        last_cmd_time_ms = now;

    switch (chassis_state)
    {
    case CHASSIS_WAIT_POWER_ON:
        if (now - state_time_ms >= CHASSIS_POWER_ON_DELAY_MS)
            chassis_state = CHASSIS_ENABLE_MOTORS;
        break;

    case CHASSIS_ENABLE_MOTORS:
        if (wheel_motor[enable_index] != NULL &&
            ZDTMotorEnable(wheel_motor[enable_index], true, false))
        {
            ++enable_index;
            if (enable_index == WHEEL_COUNT)
            {
                state_time_ms = now;
                chassis_state = CHASSIS_WAIT_ENABLE;
            }
        }
        break;

    case CHASSIS_WAIT_ENABLE:
        if (now - state_time_ms >= CHASSIS_ENABLE_SETTLE_MS)
        {
            for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
                last_wheel_ref[i] = 0;
            stopped = false;
            ChassisStopAll();
            chassis_state = CHASSIS_RUNNING;
        }
        break;

    case CHASSIS_RUNNING:
        if (chassis_cmd.chassis_mode == CHASSIS_STOP ||
            now - last_cmd_time_ms > CHASSIS_CMD_TIMEOUT_MS)
        {
            ChassisStopAll();
        }
        else
        {
            int16_t wheel_ref[WHEEL_COUNT];
            ChassisInverseKinematics(&chassis_cmd, wheel_ref);
            ChassisSendWheelRefs(wheel_ref);
        }
        break;
    }
}
