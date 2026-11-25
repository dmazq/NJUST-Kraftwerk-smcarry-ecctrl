#include "graysensor.h"


/*
#define HJ_L1_Pin GPIO_PIN_0
#define HJ_L1_GPIO_Port GPIOC
#define HJ_L2_Pin GPIO_PIN_1
#define HJ_L2_GPIO_Port GPIOC
#define HJ_M_Pin GPIO_PIN_4
#define HJ_M_GPIO_Port GPIOA
#define HJ_R1_Pin GPIO_PIN_1
#define HJ_R1_GPIO_Port GPIOB
#define HJ_R2_Pin GPIO_PIN_0
#define HJ_R2_GPIO_Port GPIOB
*/
uint8_t GetGrayCode(void)
{
    return HAL_GPIO_ReadPin(HJ_L2_GPIO_Port, HJ_L2_Pin) << 4 |
           HAL_GPIO_ReadPin(HJ_L1_GPIO_Port, HJ_L1_Pin) << 3 |
           HAL_GPIO_ReadPin(HJ_M_GPIO_Port, HJ_M_Pin) << 2 |
           HAL_GPIO_ReadPin(HJ_R1_GPIO_Port, HJ_R1_Pin) << 1 |
           HAL_GPIO_ReadPin(HJ_R2_GPIO_Port, HJ_R2_Pin) << 0;
}






