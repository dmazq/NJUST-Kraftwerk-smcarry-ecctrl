#ifndef PWM_MOTOR_H
#define PWM_MOTOR_H
#include "bsp_pwm.h"
#include "motor_def.h"
#include "gpio.h"
#include "controller.h"
#define PWM_MOTOR_CNT 4

typedef enum
{
    PWM_MOTOR_OFF = 0,
    PWM_MOTOR_ON = 1,
}PWM_MOTOR_Working_State_e;

typedef enum{
    PWM_MOTOR_DIRECTION_NORMAL = 0,
    PWM_MOTOR_DIRECTION_REVERSE = 1,
}PWM_MOTOR_Direction_e;

typedef struct 
{
    float speed; //单位为度每秒
    float angle; //单位为度
}PWM_MOTOR_Measure_s;

typedef enum
{
    PWM_MOTOR_OPENLOOP = 0,
    PWM_MOTOR_CLOSELOOP = 1,
}PWM_MOTOR_Control_Type_e;

typedef struct 
{
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;
}PWM_MOTOR_Dir_Gpio_s;

typedef struct 
{
    Feedback_Source_e angle_feedback_source; // 角度反馈类型
    Feedback_Source_e speed_feedback_source; // 速度反馈类型
    Motor_Controller_s motor_controller;    // 电机控制器
    PWMInstance *pwm_instance;
    PWM_MOTOR_Working_State_e working_state;
    PWM_MOTOR_Direction_e direction;
    PWM_MOTOR_Measure_s measure;
    TIM_HandleTypeDef *encoder_tim;
    PWM_MOTOR_Control_Type_e control_type;
    Closeloop_Type_e close_loop_type;        // 使用几个闭环(串级)
    PWM_MOTOR_Dir_Gpio_s dir_gpio_1;  // 1 为正引脚 2 为反引脚
    PWM_MOTOR_Dir_Gpio_s dir_gpio_2;
    Feedback_Reverse_Flag_e angle_feedback_dir ;
    Feedback_Reverse_Flag_e speed_feedback_dir;
    //待添加pid控制器
}PWM_MOTOR_Instance_s;

typedef struct 
{   
    Motor_Controller_Init_s controller_param_init_config;
    Feedback_Source_e angle_feedback_source; // 角度反馈类型
    Feedback_Source_e speed_feedback_source; // 速度反馈类型
    PWM_Init_Config_s pwm_init_config;
    PWM_MOTOR_Direction_e direction;
    TIM_HandleTypeDef *encoder_tim;
    PWM_MOTOR_Control_Type_e control_type;
    Closeloop_Type_e close_loop_type;        // 使用几个闭环(串级)
    PWM_MOTOR_Dir_Gpio_s dir_gpio_1;
    PWM_MOTOR_Dir_Gpio_s dir_gpio_2;
    Feedback_Reverse_Flag_e angle_feedback_dir ;
    Feedback_Reverse_Flag_e speed_feedback_dir;
}PWM_MOTOR_Init_Config_s;

void PWMMotorSetWorkingState(PWM_MOTOR_Instance_s *instance, PWM_MOTOR_Working_State_e state);
void PWMMotorSetControLoopType(PWM_MOTOR_Instance_s *instance, PWM_MOTOR_Control_Type_e type);
void PWMmotor_task();
void PWMMotorSetSpeed(PWM_MOTOR_Instance_s *instance, float speed);
void PWMMotorSetClossingLoopType(PWM_MOTOR_Instance_s *instance, Closeloop_Type_e type);
PWM_MOTOR_Instance_s *PWMMotorInit(PWM_MOTOR_Init_Config_s *config);
void PWMMotorSetDirection(PWM_MOTOR_Instance_s *instance, PWM_MOTOR_Direction_e direction);
void PWMMotorSetRef(PWM_MOTOR_Instance_s *motor, float ref);
void PWMMotorChangeFeedBack(PWM_MOTOR_Instance_s *motor, Closeloop_Type_e loop, Feedback_Source_e type);
#endif // !