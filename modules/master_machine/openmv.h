#ifndef __OPENMV_H__
#define __OPENMV_H__
#include "bsp_usart.h"

#define OPENMV_RECV_SIZE 11u 
#define OPENMV_SEND_SIZE 36u

#define OPENMV_FRAME_HEAD_1 0x2C
#define OPENMV_FRAME_HEAD_2 0x12
#define OPENMV_FRAME_TAIL   0x5B

#pragma pack(1)
// 测试用
typedef enum
{
    OPENMV_NO_TARGET = 0,
    OPENMV_TARGET_CONVERGING = 1,
}Openmv_Target_State_e;
typedef enum
{
    OPENMV_OFF = 0,
    OPENMV_ON = 1,
}OPENMV_USE_STATE_e;

typedef struct 
{   
    float yaw_angle;
    float pitch_angle;
}Openmv_Recv_s;
typedef struct 
{
    OPENMV_USE_STATE_e openmv_use_state;


}Openmv_Send_s;
#pragma pack()

Openmv_Recv_s *OpenmvInit(UART_HandleTypeDef *_handle);

void OpenmvSend(Openmv_Send_s *send_data);

#endif // !