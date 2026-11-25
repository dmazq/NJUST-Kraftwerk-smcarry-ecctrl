#include "power.h"
#include "string.h"
#include "stdlib.h"

static PowerInstance *power_instance[POWER_CNT] = {NULL};
static uint8_t idx = 0;

// 电源注册函数
PowerInstance *PowerRegister(Power_config_s *power_config)
{
    if(idx >= POWER_CNT)   // 超过最大实例数
        while(1)
        ;

    PowerInstance *instance = (PowerInstance*)malloc(sizeof(PowerInstance));
    memset(instance, 0, sizeof(PowerInstance));

    // 初始化关闭电源
    instance->power_state = POWER_OFF;

    // 注册power到GPIO实例
    instance->power_gpio_ins = GPIORegister(&power_config->power_gpio_config);

    power_instance[idx++] = instance;
    return instance;
}

// 电源状态设定
void PowerSetState(PowerInstance *power, Power_State_e state)
{
    power->power_state = state;
}

// 电源开关任务
void PowerTask()
{
    PowerInstance *power;

    for(int i = 0; i < POWER_CNT; i++)
    {
        power = power_instance[i];

        if(power == NULL)
            return;

        if(power->power_state == POWER_ON)
            GPIOSet(power->power_gpio_ins);
        else
            GPIOReset(power->power_gpio_ins);
    }
}