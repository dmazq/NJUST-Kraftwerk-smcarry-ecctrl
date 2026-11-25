// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "message_center.h"
#include "general_def.h"
#include "openmv.h"
#include "mpu6500.h"
#include "bsp_dwt.h"
#define Up_Pitch 100   // 上
#define Down_Pitch 124 // 下
#define Left_Yaw 90    // 左
#define Right_Yaw 63   // 右
#define mid_Pitch 112
#define mid_Yaw 77
typedef enum
{
    Start_RunScreen = 0,
    Left_Up2Right_up = 1,
    Right_Up2Right_Down = 2,
    Right_Down2Left_Down = 3,
    Left_Down2Left_Up = 4,
} Gimbal_Run_Screen_State_e;

static Openmv_Recv_s *openmv_data; // openmv数据,初始化时返回

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回

static attitude_t *IMU_data; // IMU数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 发送给云台应用的信息,包括控制信息和UI绘制相关
static Gimbal_Run_Screen_State_e gimbal_run_screen_state = Start_RunScreen;
void RobotCMDInit()
{
    IMU_data = INS_Init();
    rc_data = RemoteControlInit(&huart1); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    openmv_data = OpenmvInit(&huart6);    // 修改为对应串口
    gimbal_cmd_send.yaw = mid_Yaw;
    gimbal_cmd_send.pitch = mid_Pitch;
    HAL_GPIO_WritePin(GPIOG, D8_Pin | D7_Pin | D6_Pin | D5_Pin | D4_Pin | D3_Pin | D2_Pin | D1_Pin, GPIO_PIN_SET);
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}
static void GimbalRunScreen();
static void GimbalFollowBlack();
static void GimbalReset();
static void GimbalStop();
static void RemoteControlSet()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    gimbal_cmd_send.yaw += (float)rc_data->rc.rocker_l_ / 660.0 / 10;
    gimbal_cmd_send.pitch += (float)rc_data->rc.rocker_l1 / 660.0 / 10;
}
uint8_t GimbalRunScreenstart_flag = 0;
static void AutoControlSet()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    switch (rc_data->rc.switch_right)
    {
    case 1:
        GimbalRunScreen();
        break;
    case 2:
        GimbalReset();
        break;
    case 3:
        GimbalFollowBlack();
        break;
    default:
        break;
    }
}
static void GimbalFollowBlack()
{

}
// 走屏幕从左上角开始顺时针旋转
static void GimbalRunScreen()
{
    static float now_time, last_time = 0;
    static float wait_time_XY = 1, wait_time_X = 1, wait_time_Y = 1;
    now_time = DWT_GetTimeline_s();
    switch (gimbal_run_screen_state)
    {
    case Start_RunScreen:
        if (now_time - last_time > wait_time_XY)
        {
            last_time = DWT_GetTimeline_s();
            gimbal_run_screen_state = Left_Up2Right_up;
        }
        gimbal_cmd_send.pitch = Up_Pitch;
        gimbal_cmd_send.yaw = Left_Yaw;
        break;
    case Left_Up2Right_up:
        if (now_time - last_time > wait_time_X)
        {
            last_time = DWT_GetTimeline_s();
            gimbal_run_screen_state = Right_Up2Right_Down;
        }
        gimbal_cmd_send.yaw = Right_Yaw;
        break;
    case Right_Up2Right_Down:
        if (now_time - last_time > wait_time_Y)
        {
            last_time = DWT_GetTimeline_s();
            gimbal_run_screen_state = Right_Down2Left_Down;
        }
        gimbal_cmd_send.pitch = Down_Pitch;
        break;
    case Right_Down2Left_Down:
        if (now_time - last_time > wait_time_X)
        {
            last_time = DWT_GetTimeline_s();
            gimbal_run_screen_state = Left_Down2Left_Up;
        }
        gimbal_cmd_send.yaw = Left_Yaw;
        break;
    case Left_Down2Left_Up:
        if (now_time - last_time > wait_time_Y)
        {
            last_time = DWT_GetTimeline_s();
            gimbal_run_screen_state = Left_Up2Right_up;
        }
        gimbal_cmd_send.pitch = Up_Pitch;
        break;
    default:
        break;
    }
}
static void GimbalStop()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
}

static void GimbalReset()
{
    gimbal_cmd_send.yaw = mid_Yaw;
    gimbal_cmd_send.pitch = mid_Pitch;
}
static void EmergencyHandler()
{
    if (rc_data->rc.dial > 300)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.yaw_total_angle;
        gimbal_cmd_send.pitch = gimbal_fetch_data.pitch_total_angle;
    }
}
// 模式选择左键 模式内作用右键
void RobotCMDTask()
{

    switch (rc_data->rc.switch_left)
    {
    case 1:
        RemoteControlSet();
        break;
    case 3:
        GimbalStop();
        break;
    case 2:
        AutoControlSet();
        break;
    default:
        EmergencyHandler();
        break;
    }
    EmergencyHandler();
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
