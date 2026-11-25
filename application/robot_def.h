/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "master_process.h"
#include "stdint.h"
#include "mpu6500.h"


#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输

typedef enum
{
    CHASSIS_STOP = 0,
    CHASSIS_MOVE = 1,
}Chassis_Mode_e;

typedef struct 
{
    Chassis_Mode_e chassis_mode;
    float vx; // forward is positive
    float vy; // right is positive
    float wz; // clockwise is positive
}Chassis_Ctrl_Cmd_s;

typedef struct
{
    float yaw_velocity; // rad/s
    int8_t z_direction; // -1, 0, +1
    int8_t x_direction; // -1, 0, +1
    uint8_t gripper_closed;
    uint8_t remote_online;
} Arm_Ctrl_Cmd_s;
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;
typedef enum
{
    GIMBAL_Recover = 0,
    GIMBAL_Follow = 1,
}Gimbal_Mode_e;
typedef struct
{
    float yaw_total_angle;
    float pitch_total_angle;
} Gimbal_Upload_Data_s;
typedef struct 
{
    gimbal_mode_e gimbal_mode;
    float pitch;
    float yaw;
}Gimbal_Ctrl_Cmd_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H
