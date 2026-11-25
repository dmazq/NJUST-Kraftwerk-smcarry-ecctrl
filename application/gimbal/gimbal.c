#include "gimbal.h"
#include "robot_def.h"

#include "general_def.h"
#include "message_center.h"
#include "mpu6500.h"
#include "dji_motor.h"
#include "usart.h"

static DJIMotorInstance *yaw_motor, *pitch_motor; 

static Publisher_t *gimbal_pub; 
static Subscriber_t *gimbal_sub;
//yaw 10-4 mid 7.4 pitch mid 7  
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

#define GIMBAL_PITCH_MAX_Duty 30
#define GIMBAL_PITCH_MIN_Duty -30
#define GIMBAL_YAW_MAX_Duty 30
#define GIMBAL_YAW_MIN_Duty -30 

void GimbalInit()
{
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 25, //0.55
                .Ki = 0,//0.05
                .Kd = 0,//0.02
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit ,
                .IntegralLimit = 0.5,
                .MaxOut = 400,
            },
            .speed_PID = {
                .Kp = 16,//22000
                .Ki = 0,//
                .Kd =0,
                .Improve = PID_Trapezoid_Intergral |PID_Integral_Limit ,
                .IntegralLimit = 5000,
                .MaxOut = 20000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = SPEED_FEEDFORWARD,
            
        },
        .motor_type = GM6020};
     // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp =22,//0.4
                .Ki = 0,//0.15
                .Kd = 0,//0.006
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                .IntegralLimit =1,//1
                .MaxOut = 600,//600
            },
            .speed_PID = {
                .Kp=13,//14000
                .Ki =0,//0
                .Kd =0,//0.0005
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit ,
                .IntegralLimit =3000,//3000
                .MaxOut = 20000,//20000
            },
        },
            .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
        .motor_type = GM6020,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        break;
    case GIMBAL_FREE_MODE: 
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }
    gimbal_feedback_data.pitch_total_angle = pitch_motor->measure.total_angle;
    gimbal_feedback_data.yaw_total_angle = yaw_motor->measure.total_angle;
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
   
}