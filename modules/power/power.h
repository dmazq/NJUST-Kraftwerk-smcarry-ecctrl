#ifndef POWER_H
#define POWER_H

#define POWER_CNT 4

#include "bsp_gpio.h"

typedef enum
{
    POWER_OFF = 0,
    POWER_ON
} Power_State_e;

typedef struct
{
    Power_State_e power_state;
    GPIOInstance *power_gpio_ins;
} PowerInstance;

typedef struct
{
    Power_State_e power_state;
    GPIO_Init_Config_s power_gpio_config;
} Power_config_s;


PowerInstance *PowerRegister(Power_config_s *power_config);

void PowerSetState(PowerInstance *power, Power_State_e state);

void PowerTask();

#endif // !POWER_H