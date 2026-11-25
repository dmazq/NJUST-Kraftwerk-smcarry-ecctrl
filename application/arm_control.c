#include "arm_control.h"
#include "dm_motor.h"
#include "step_motor.h"
#include "pwm_servo.h"
#include "message_center.h"
#include "robot_def.h"
#include <limits.h>

#define ARM_YAW_CAN_ID 0x08U
#define ARM_YAW_MASTER_ID 0x00U
#define ARM_YAW_P_MAX 12.5f
#define ARM_YAW_V_MAX 50.0f
#define ARM_YAW_T_MAX 10.0f
#define ARM_YAW_VELOCITY_LIMIT 1.5f
#define ARM_YAW_VELOCITY_KD 1.0f

#define ARM_Z_CAN_ID 5U
#define ARM_X_CAN_ID 6U
#define ARM_Z_SPEED_RPM 2500
#define ARM_X_SPEED_RPM 100
#define ARM_Z_ACCELERATION 250U
#define ARM_X_ACCELERATION 100U
#define ARM_Z_REVERSE true
#define ARM_X_REVERSE true

#define ARM_GRIPPER_OPEN_ANGLE_DEG 125.0f
#define ARM_GRIPPER_CLOSED_ANGLE_DEG 85.0f
#define ARM_GRIPPER_MIN_PULSE_US 500U
#define ARM_GRIPPER_MAX_PULSE_US 2500U

#define ARM_POWER_ON_DELAY_MS 1000U
#define ARM_ENABLE_DELAY_MS 100U
#define ARM_ENABLE_RETRY_INTERVAL_MS 10U
#define ARM_YAW_ENABLE_FRAME_COUNT 10U
#define ARM_STEP_ENABLE_FRAME_COUNT 3U
#define ARM_CMD_TIMEOUT_MS 100U

typedef enum
{
    ARM_WAIT_POWER_ON,
    ARM_ENABLE_YAW,
    ARM_ENABLE_Z,
    ARM_ENABLE_X,
    ARM_WAIT_ENABLE,
    ARM_RUNNING,
} ArmState_e;

static DMMotorInstance *yaw_motor;
static ZDTMotorInstance *z_motor;
static ZDTMotorInstance *x_motor;
static PWMServoInstance *gripper_servo;
static Subscriber_t *arm_cmd_sub;
static Arm_Ctrl_Cmd_s arm_cmd;
static ArmState_e arm_state;
static uint32_t state_time_ms;
static uint32_t last_cmd_time_ms;
static uint8_t yaw_enable_frame_count;
static uint8_t z_enable_frame_count;
static uint8_t x_enable_frame_count;
static int16_t last_z_ref;
static int16_t last_x_ref;
static uint8_t last_gripper_closed;

static float LimitYawVelocity(float velocity)
{
    if (velocity > ARM_YAW_VELOCITY_LIMIT)
        return ARM_YAW_VELOCITY_LIMIT;
    if (velocity < -ARM_YAW_VELOCITY_LIMIT)
        return -ARM_YAW_VELOCITY_LIMIT;
    return velocity;
}

static void UpdateStepAxis(ZDTMotorInstance *motor, int8_t direction,
                           int16_t speed_rpm, int16_t *last_ref)
{
    int16_t target = direction > 0 ? speed_rpm : direction < 0 ? -speed_rpm : 0;
    if (target == *last_ref)
        return;

    bool sent = target == 0 ? ZDTMotorStop(motor, false) : ZDTMotorSetRef(motor, target);
    if (sent)
        *last_ref = target;
}

void ArmControlInit(void)
{
    const DMMotorInitConfig_s yaw_config = {
        .can_handle = &hcan1,
        .can_id = ARM_YAW_CAN_ID,
        .master_id = ARM_YAW_MASTER_ID,
        .position_max = ARM_YAW_P_MAX,
        .velocity_max = ARM_YAW_V_MAX,
        .torque_max = ARM_YAW_T_MAX,
        .tx_timeout_ms = 2U,
    };
    const ZDTMotorInitConfig_s z_config = {
        .can_handle = &hcan1,
        .address = ARM_Z_CAN_ID,
        .reverse = ARM_Z_REVERSE,
        .default_acceleration = ARM_Z_ACCELERATION,
        .tx_timeout_ms = 2U,
    };
    const ZDTMotorInitConfig_s x_config = {
        .can_handle = &hcan1,
        .address = ARM_X_CAN_ID,
        .reverse = ARM_X_REVERSE,
        .default_acceleration = ARM_X_ACCELERATION,
        .tx_timeout_ms = 2U,
    };
    const PWMServoInitConfig_s gripper_config = {
        .htim = &htim2,
        .channel = TIM_CHANNEL_1,
        .min_pulse_us = ARM_GRIPPER_MIN_PULSE_US,
        .max_pulse_us = ARM_GRIPPER_MAX_PULSE_US,
        .min_angle_deg = 0.0f,
        .max_angle_deg = 180.0f,
        .initial_angle_deg = ARM_GRIPPER_OPEN_ANGLE_DEG,
    };

    yaw_motor = DMMotorInit(&yaw_config);
    z_motor = ZDTMotorInit(&z_config);
    x_motor = ZDTMotorInit(&x_config);
    gripper_servo = PWMServoInit(&gripper_config);
    arm_cmd_sub = SubRegister("arm_cmd", sizeof(Arm_Ctrl_Cmd_s));

    arm_cmd.yaw_velocity = 0.0f;
    arm_cmd.z_direction = 0;
    arm_cmd.x_direction = 0;
    arm_cmd.gripper_closed = 0U;
    arm_cmd.remote_online = 0U;
    last_z_ref = INT16_MIN;
    last_x_ref = INT16_MIN;
    last_gripper_closed = 0U;
    yaw_enable_frame_count = 0U;
    z_enable_frame_count = 0U;
    x_enable_frame_count = 0U;
    state_time_ms = HAL_GetTick();
    last_cmd_time_ms = state_time_ms;
    arm_state = ARM_WAIT_POWER_ON;
}

void ArmControlTask(void)
{
    if (yaw_motor == NULL || z_motor == NULL || x_motor == NULL || gripper_servo == NULL)
        return;

    uint32_t now = HAL_GetTick();
    if (SubGetMessage(arm_cmd_sub, &arm_cmd))
        last_cmd_time_ms = now;

    switch (arm_state)
    {
    case ARM_WAIT_POWER_ON:
        if (now - state_time_ms >= ARM_POWER_ON_DELAY_MS)
            arm_state = ARM_ENABLE_YAW;
        break;

    case ARM_ENABLE_YAW:
        if (now - state_time_ms >= ARM_ENABLE_RETRY_INTERVAL_MS && DMMotorEnable(yaw_motor))
        {
            state_time_ms = now;
            if (++yaw_enable_frame_count >= ARM_YAW_ENABLE_FRAME_COUNT)
                arm_state = ARM_ENABLE_Z;
        }
        break;

    case ARM_ENABLE_Z:
        if (now - state_time_ms >= ARM_ENABLE_RETRY_INTERVAL_MS &&
            ZDTMotorEnable(z_motor, true, false))
        {
            state_time_ms = now;
            if (++z_enable_frame_count >= ARM_STEP_ENABLE_FRAME_COUNT)
                arm_state = ARM_ENABLE_X;
        }
        break;

    case ARM_ENABLE_X:
        if (now - state_time_ms >= ARM_ENABLE_RETRY_INTERVAL_MS &&
            ZDTMotorEnable(x_motor, true, false))
        {
            state_time_ms = now;
            if (++x_enable_frame_count >= ARM_STEP_ENABLE_FRAME_COUNT)
                arm_state = ARM_WAIT_ENABLE;
        }
        break;

    case ARM_WAIT_ENABLE:
        if (now - state_time_ms >= ARM_ENABLE_DELAY_MS)
        {
            ZDTMotorStop(z_motor, false);
            ZDTMotorStop(x_motor, false);
            last_z_ref = 0;
            last_x_ref = 0;
            arm_state = ARM_RUNNING;
        }
        break;

    case ARM_RUNNING:
    {
        bool command_valid = arm_cmd.remote_online &&
                             now - last_cmd_time_ms <= ARM_CMD_TIMEOUT_MS;
        float yaw_velocity = command_valid ? LimitYawVelocity(arm_cmd.yaw_velocity) : 0.0f;
        int8_t z_direction = command_valid ? arm_cmd.z_direction : 0;
        int8_t x_direction = command_valid ? arm_cmd.x_direction : 0;

        DMMotorSetVelocity(yaw_motor, yaw_velocity, ARM_YAW_VELOCITY_KD);
        UpdateStepAxis(z_motor, z_direction, ARM_Z_SPEED_RPM, &last_z_ref);
        UpdateStepAxis(x_motor, x_direction, ARM_X_SPEED_RPM, &last_x_ref);
        if (arm_cmd.gripper_closed != last_gripper_closed)
        {
            PWMServoSetAngle(gripper_servo,
                             arm_cmd.gripper_closed ? ARM_GRIPPER_CLOSED_ANGLE_DEG
                                                   : ARM_GRIPPER_OPEN_ANGLE_DEG);
            last_gripper_closed = arm_cmd.gripper_closed;
        }
        break;
    }
    }
}
