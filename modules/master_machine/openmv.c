#include "openmv.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "string.h"
static Openmv_Recv_s recv_data;
static USARTInstance *openmv_usart_instance;

static DaemonInstance *openmv_daemon_instance;

static void DecodeOpenmv()
{
    DaemonReload(openmv_daemon_instance);
    if( openmv_usart_instance->recv_buff[0] == OPENMV_FRAME_HEAD_1 && 
        openmv_usart_instance->recv_buff[1] == OPENMV_FRAME_HEAD_2 )
    {
        memcpy(&recv_data, openmv_usart_instance->recv_buff + 2, OPENMV_RECV_SIZE - 3 );
    }
}

static void OpenmvOffineCallback()
{
    USARTServiceInit(openmv_usart_instance);
}

Openmv_Recv_s *OpenmvInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeOpenmv;
    conf.recv_buff_size = OPENMV_RECV_SIZE;
    conf.usart_handle = _handle;
    openmv_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = OpenmvOffineCallback,
        .owner_id = openmv_usart_instance,
        .reload_count = 10,
    };
    openmv_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}


void OpenmvSend(Openmv_Send_s *send_data)
{
    static uint8_t send_buff[OPENMV_SEND_SIZE]; 
    send_buff[0] = OPENMV_FRAME_HEAD_1;
    send_buff[1] = OPENMV_FRAME_HEAD_2;
    send_buff[OPENMV_SEND_SIZE-1] = OPENMV_FRAME_TAIL;
    
    memcpy(send_buff + 2, send_data, OPENMV_SEND_SIZE - 3);
    
    USARTSend(openmv_usart_instance, send_buff, OPENMV_SEND_SIZE, USART_TRANSFER_DMA); 
}