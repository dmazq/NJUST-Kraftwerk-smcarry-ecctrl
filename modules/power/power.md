# Power
可用于A板上自带的四路可调控电源

# 使用范例
```c
Power_config_s power1_config = {
        .power_gpio_config = {
            .exti_mode = GPIO_EXTI_MODE_NONE,
            .GPIOx = Power1_GPIO_Port,
            .GPIO_Pin = Power1_Pin,
            .pin_state = GPIO_PIN_RESET,
        },
    };
    power1 = PowerRegister(&power1_config);

PowerSetState(power1, POWER_OFF);
PowerSetState(power1, POWER_ON);
```