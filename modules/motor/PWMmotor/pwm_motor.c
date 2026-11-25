#include "pwm_motor.h"
#include "bsp_pwm.h"
#include "stdint.h"
#define TASK_FREQUENCE 500
#include "general_def.h"
#include"bsp_dwt.h"
#define MG130P45
 //#define JGB_520
// 如果使用霍尔编码器的电机 请修改任务频率为100~200Hz 500hz无法获取准确的精度 或者使用周期测量法 待添加
#ifdef MG130P45
#define PWMENCODER_RESOLUTION 500    /*编码器一圈的物理脉冲数*/
#define PWM_ENCODER_MULTIPLE 4       /*编码器倍频，通过定时器的编码器模式设置*/
#define PWM_MOTOR_REDUCTION_RATIO 90 /*电机的减速比*/
#define PWM_MOTOR_BIAS 1.0f          /*电机的误差系数*/
#endif

#ifdef JGB_520
#define PWMENCODER_RESOLUTION 11    /*编码器一圈的物理脉冲数*/
#define PWM_ENCODER_MULTIPLE 4      /*编码器倍频，通过定时器的编码器模式设置*/
#define PWM_MOTOR_REDUCTION_RATIO 6.3 /*电机的减速比*/
#define PWM_MOTOR_BIAS 1.0f          /*电机的误差系数*/
#endif 

#define PWM_TOTAL_RESOLUTION (PWM_MOTOR_BIAS*PWMENCODER_RESOLUTION*PWM_ENCODER_MULTIPLE*PWM_MOTOR_REDUCTION_RATIO/TASK_FREQUENCE)
//@todo:目前电机的编码器获取为使用tim8和tim5 ch1 ch2 通道 是否有必要编写bsp_tim？
static uint8_t idx = 0;

static PWM_MOTOR_Instance_s *PWM_MOTOR_Instance[PWM_MOTOR_CNT];

static float decode_tmp,decode_last,decode_time;

static void DecodePWMMotor(PWM_MOTOR_Instance_s *instance)
{   
    HAL_TIM_Encoder_Start(instance->encoder_tim, TIM_CHANNEL_ALL); 
    
    decode_tmp = DWT_GetTimeline_ms();
    decode_time =  decode_tmp - decode_last;
    int16_t Encoder_Pluse = __HAL_TIM_GET_COUNTER(instance->encoder_tim);
    instance->measure.speed = (float)Encoder_Pluse/PWM_TOTAL_RESOLUTION*0.85+0.15*instance->measure.speed;

    instance->measure.angle += (instance->measure.speed*360/TASK_FREQUENCE); //角度的计算 待优化
    __HAL_TIM_SET_COUNTER(instance->encoder_tim,0);
    decode_last = DWT_GetTimeline_ms();
}

PWM_MOTOR_Instance_s *PWMMotorInit(PWM_MOTOR_Init_Config_s *config)
{
    PWM_MOTOR_Instance_s *instance = (PWM_MOTOR_Instance_s *)malloc(sizeof(PWM_MOTOR_Instance_s));
    memset(instance, 0, sizeof(PWM_MOTOR_Instance_s));
    
    instance->direction = config->direction;
    instance->working_state = PWM_MOTOR_OFF; //默认为停止状态
    instance->encoder_tim = config->encoder_tim;
    instance->control_type = config->control_type;
    instance->close_loop_type = config->close_loop_type;
    
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);

    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->angle_feedback_source = config->angle_feedback_source;
    instance->speed_feedback_source = config->speed_feedback_source;

    instance->dir_gpio_1.gpio_port = config->dir_gpio_1.gpio_port;
    instance->dir_gpio_1.gpio_pin = config->dir_gpio_1.gpio_pin;
    instance->dir_gpio_2.gpio_port = config->dir_gpio_2.gpio_port;
    instance->dir_gpio_2.gpio_pin = config->dir_gpio_2.gpio_pin;

    instance->pwm_instance = PWMRegister(&config->pwm_init_config);
    
    PWM_MOTOR_Instance[idx++] = instance;

    return instance;
}
static void PWMMotorSetGPIOVol(PWM_MOTOR_Instance_s *instance,uint8_t config)
{
    
    HAL_GPIO_WritePin(instance->dir_gpio_1.gpio_port, instance->dir_gpio_1.gpio_pin, config & 0b000000010);
    HAL_GPIO_WritePin(instance->dir_gpio_2.gpio_port, instance->dir_gpio_2.gpio_pin, config & 0b000000001);
}

void PWMMotorSetWorkingState(PWM_MOTOR_Instance_s *instance, PWM_MOTOR_Working_State_e state)
{
    instance->working_state = state;
}
void PWMMotorSetControLoopType(PWM_MOTOR_Instance_s *instance, PWM_MOTOR_Control_Type_e type)
{
    instance->control_type = type;
}

void PWMMotorSetDirection(PWM_MOTOR_Instance_s *instance, PWM_MOTOR_Direction_e direction)
{
    instance->direction = direction;   
}
void PWMMotorSetSpeed(PWM_MOTOR_Instance_s *instance, float speed)
{
    instance->motor_controller.pid_ref = speed;
}
void PWMMotorSetClossingLoopType(PWM_MOTOR_Instance_s *instance, Closeloop_Type_e type)
{
    instance->close_loop_type = type;
}
// 设置参考值
void PWMMotorSetRef(PWM_MOTOR_Instance_s *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}
void PWMMotorChangeFeedBack(PWM_MOTOR_Instance_s *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
    {
        motor->angle_feedback_source = type;
    }
    else if (loop == SPEED_LOOP)
    {
        motor->speed_feedback_source = type;
    }
}
void PWMmotor_task()
{
    float set; // pwm占空比
    PWM_MOTOR_Instance_s *motor;
    Motor_Controller_s *motor_controller;
    PWM_MOTOR_Measure_s *motor_measure;
    float pid_measure, pid_ref; // 电机PID测量值和设定值
    for (uint8_t i = 0; i < idx; i++)
    {
        motor = PWM_MOTOR_Instance[i];
        motor_controller = &motor->motor_controller;
        motor_measure = &motor->measure;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        DecodePWMMotor(PWM_MOTOR_Instance[i]);
        //待添加PID控制器 目前选择闭环控制输出为0
        if(PWM_MOTOR_Instance[i]->working_state == PWM_MOTOR_OFF)
        {
            PWMSetDutyRatio(PWM_MOTOR_Instance[i]->pwm_instance, 0);
            //HAL_TIM_Encoder_Stop(motor->encoder_tim, TIM_CHANNEL_ALL);
            continue;
        }
        if(PWM_MOTOR_Instance[i]->control_type == PWM_MOTOR_OPENLOOP) //开环直接选择方向
        {
            set = pid_ref;
        }
        else{ 
             if(motor->close_loop_type & ANGLE_LOOP)
             {
                if(motor->angle_feedback_source == OTHER_FEED)
                    pid_measure = *motor_controller->other_angle_feedback_ptr;
                else
                    pid_measure = motor_measure->angle;
                if(motor->angle_feedback_dir == FEEDBACK_DIRECTION_NORMAL)
                    pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure,pid_ref);
                else
                    pid_ref = -PIDCalculate(&motor_controller->angle_PID, pid_measure,pid_ref);
             }
             if(motor->close_loop_type & SPEED_LOOP )
             {
                if(motor->speed_feedback_source == OTHER_FEED)
                    pid_measure = *motor_controller->other_speed_feedback_ptr;
                else
                    pid_measure = motor_measure->speed;
                if(motor->speed_feedback_dir == FEEDBACK_DIRECTION_NORMAL)
                    pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure,pid_ref);
                else
                    pid_ref = -PIDCalculate(&motor_controller->speed_PID, pid_measure,pid_ref);
             }
             
             if( pid_ref > 0)
             {
                PWMMotorSetDirection(PWM_MOTOR_Instance[i], PWM_MOTOR_DIRECTION_NORMAL);
             }
             else if(pid_ref < 0)
             {
                PWMMotorSetDirection(PWM_MOTOR_Instance[i], PWM_MOTOR_DIRECTION_REVERSE);
             }
             set = fabs(pid_ref);
        }
        switch (motor->direction)
        {
        case PWM_MOTOR_DIRECTION_NORMAL:
            PWMMotorSetGPIOVol(motor,0b00000010);
            break;
        
        default:
            PWMMotorSetGPIOVol(motor,0b00000001);
            break;
        }
             PWMSetDutyRatio(PWM_MOTOR_Instance[i]->pwm_instance, set);
        
    }
// 待添加pid控制器
//
}

