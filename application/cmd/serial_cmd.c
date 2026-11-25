#include "serial_cmd.h"
#include "serial_remote.h"
#include "message_center.h"
#include "robot_def.h"
#include "usart.h"

#define SERIAL_CMD_UART_HANDLE (&huart7)
#define SERIAL_CMD_STICK_DEADZONE 8
#define SERIAL_CMD_MAX_TRANSLATION_RPM 300.0f
#define SERIAL_CMD_MAX_ROTATION_RPM 180.0f
#define SERIAL_CMD_FORWARD_SIGN 1.0f
#define SERIAL_CMD_RIGHT_SIGN 1.0f
#define SERIAL_CMD_CLOCKWISE_SIGN 1.0f
#define SERIAL_CMD_ARM_YAW_MAX_RAD_S 1.5f
#define SERIAL_CMD_ARM_YAW_SIGN 1.0f
#define SERIAL_CMD_BUTTON_A_MASK 0x01U

static const SerialRemoteData_s *remote_data;
static Publisher_t *chassis_cmd_pub;
static Publisher_t *arm_cmd_pub;
static Chassis_Ctrl_Cmd_s chassis_cmd;
static Arm_Ctrl_Cmd_s arm_cmd;
static uint8_t last_button_a;

static void DecodeDPad(uint8_t button_1, int8_t *z_direction, int8_t *x_direction)
{
    *z_direction = 0;
    *x_direction = 0;

    switch (button_1 & 0x0FU)
    {
    case 1U: // up
        *z_direction = 1;
        break;
    case 2U: // up-right
        *z_direction = 1;
        *x_direction = 1;
        break;
    case 3U: // right
        *x_direction = 1;
        break;
    case 4U: // down-right
        *z_direction = -1;
        *x_direction = 1;
        break;
    case 5U: // down
        *z_direction = -1;
        break;
    case 6U: // down-left
        *z_direction = -1;
        *x_direction = -1;
        break;
    case 7U: // left
        *x_direction = -1;
        break;
    case 8U: // up-left
        *z_direction = 1;
        *x_direction = -1;
        break;
    default: // released
        break;
    }
}

static float MapAxis(int8_t axis, float max_output, float direction_sign)
{
    int16_t input = axis;
    int16_t magnitude = input >= 0 ? input : -input;

    if (magnitude <= SERIAL_CMD_STICK_DEADZONE)
        return 0.0f;

    float normalized = (float)(magnitude - SERIAL_CMD_STICK_DEADZONE) /
                       (127.0f - SERIAL_CMD_STICK_DEADZONE);
    float direction = input >= 0 ? 1.0f : -1.0f;
    return direction * normalized * max_output * direction_sign;
}

void SerialCMDInit(void)
{
    remote_data = SerialRemoteInit(SERIAL_CMD_UART_HANDLE);
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    arm_cmd_pub = PubRegister("arm_cmd", sizeof(Arm_Ctrl_Cmd_s));
    chassis_cmd.chassis_mode = CHASSIS_STOP;
    chassis_cmd.vx = 0.0f;
    chassis_cmd.vy = 0.0f;
    chassis_cmd.wz = 0.0f;
    arm_cmd.yaw_velocity = 0.0f;
    arm_cmd.z_direction = 0;
    arm_cmd.x_direction = 0;
    arm_cmd.gripper_closed = 0U;
    arm_cmd.remote_online = 0U;
    last_button_a = 0U;
}

void SerialCMDTask(void)
{
    if (SerialRemoteIsOnline())
    {
        chassis_cmd.vx = MapAxis(remote_data->left_y, SERIAL_CMD_MAX_TRANSLATION_RPM,
                                 SERIAL_CMD_FORWARD_SIGN);
        chassis_cmd.vy = MapAxis(remote_data->left_x, SERIAL_CMD_MAX_TRANSLATION_RPM,
                                 SERIAL_CMD_RIGHT_SIGN);
        chassis_cmd.wz = MapAxis(remote_data->right_x, SERIAL_CMD_MAX_ROTATION_RPM,
                                 SERIAL_CMD_CLOCKWISE_SIGN);
        arm_cmd.yaw_velocity = MapAxis(remote_data->right_y, SERIAL_CMD_ARM_YAW_MAX_RAD_S,
                                       SERIAL_CMD_ARM_YAW_SIGN);
        DecodeDPad(remote_data->button_1, &arm_cmd.z_direction, &arm_cmd.x_direction);
        uint8_t button_a = (remote_data->button_2 & SERIAL_CMD_BUTTON_A_MASK) != 0U;
        if (button_a && !last_button_a)
            arm_cmd.gripper_closed = !arm_cmd.gripper_closed;
        last_button_a = button_a;
        arm_cmd.remote_online = 1U;
    }
    else
    {
        chassis_cmd.vx = 0.0f;
        chassis_cmd.vy = 0.0f;
        chassis_cmd.wz = 0.0f;
        arm_cmd.yaw_velocity = 0.0f;
        arm_cmd.z_direction = 0;
        arm_cmd.x_direction = 0;
        arm_cmd.remote_online = 0U;
        last_button_a = 0U;
    }
    chassis_cmd.chassis_mode =
        (chassis_cmd.vx == 0.0f && chassis_cmd.vy == 0.0f && chassis_cmd.wz == 0.0f) ?
        CHASSIS_STOP : CHASSIS_MOVE;
    PubPushMessage(chassis_cmd_pub, &chassis_cmd);
    PubPushMessage(arm_cmd_pub, &arm_cmd);
}
