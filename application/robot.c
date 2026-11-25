#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#include "serial_cmd.h"
#include "test/zdt_motor_test.h"
#include "arm_control.h"

#define ZDT_MOTOR_TEST_MODE 0

void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();
   
#if ZDT_MOTOR_TEST_MODE
    SerialCMDInit();
    ZDTMotorTestInit();
#else
    SerialCMDInit();
    ChassisInit();
    ArmControlInit();
#endif

    __enable_irq();
}

void RobotTask()
{
#if ZDT_MOTOR_TEST_MODE
    SerialCMDTask();
    ZDTMotorTestTask();
#else
    SerialCMDTask();
    ChassisTask();
    ArmControlTask();
#endif
}
