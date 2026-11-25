#include "zdt_motor_test.h"
#include "step_motor.h"
#include "message_center.h"
#include "robot_def.h"
#include <limits.h>

#define ZDT_TEST_MOTOR_ID 1U
#define ZDT_TEST_ACCELERATION 100U
#define ZDT_TEST_POWER_ON_DELAY_MS 1000U
#define ZDT_TEST_ENABLE_DELAY_MS 100U

typedef enum
{
    ZDT_TEST_WAIT_POWER_ON,
    ZDT_TEST_ENABLE,
    ZDT_TEST_WAIT_ENABLE,
    ZDT_TEST_RUNNING,
} ZDTMotorTestState_e;

static ZDTMotorInstance *test_motor;
static ZDTMotorTestState_e test_state;
static uint32_t power_on_time_ms;
static uint32_t enable_time_ms;
static Subscriber_t *chassis_cmd_sub;
static Chassis_Ctrl_Cmd_s chassis_cmd;
static int16_t last_speed_ref;

static int16_t ClampSpeedRef(float speed)
{
    if (speed > (float)INT16_MAX)
        return INT16_MAX;
    if (speed < (float)INT16_MIN)
        return INT16_MIN;
    return (int16_t)speed;
}

void ZDTMotorTestInit(void)
{
    const ZDTMotorInitConfig_s config = {
        .can_handle = &hcan1,
        .address = ZDT_TEST_MOTOR_ID,
        .default_acceleration = ZDT_TEST_ACCELERATION,
        .tx_timeout_ms = 2,
    };

    test_motor = ZDTMotorInit(&config);
    chassis_cmd_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    last_speed_ref = INT16_MIN;
    power_on_time_ms = HAL_GetTick();
    test_state = ZDT_TEST_WAIT_POWER_ON;
}

void ZDTMotorTestTask(void)
{
    if (test_motor == NULL)
        return;

    switch (test_state)
    {
    case ZDT_TEST_WAIT_POWER_ON:
        if (HAL_GetTick() - power_on_time_ms >= ZDT_TEST_POWER_ON_DELAY_MS)
            test_state = ZDT_TEST_ENABLE;
        break;

    case ZDT_TEST_ENABLE:
        if (ZDTMotorEnable(test_motor, true, false))
        {
            enable_time_ms = HAL_GetTick();
            test_state = ZDT_TEST_WAIT_ENABLE;
        }
        break;

    case ZDT_TEST_WAIT_ENABLE:
        if (HAL_GetTick() - enable_time_ms >= ZDT_TEST_ENABLE_DELAY_MS &&
            ZDTMotorSetRef(test_motor, 0))
        {
            last_speed_ref = 0;
            test_state = ZDT_TEST_RUNNING;
        }
        break;

    case ZDT_TEST_RUNNING:
        if (SubGetMessage(chassis_cmd_sub, &chassis_cmd))
        {
            int16_t speed_ref = chassis_cmd.chassis_mode == CHASSIS_STOP ?
                                0 : ClampSpeedRef(chassis_cmd.vx);
            if (speed_ref != last_speed_ref)
            {
                bool command_sent = speed_ref == 0 ?
                                    ZDTMotorStop(test_motor, false) :
                                    ZDTMotorSetRef(test_motor, speed_ref);
                if (command_sent)
                    last_speed_ref = speed_ref;
            }
        }
        break;

    default:
        break;
    }
}
