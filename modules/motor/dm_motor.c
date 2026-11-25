#include "dm_motor.h"
#include <stdlib.h>
#include <string.h>

#define DM_KP_MAX 500.0f
#define DM_KD_MAX 5.0f

static float Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static uint32_t FloatToUint(float value, float minimum, float maximum, uint8_t bits)
{
    value = Clamp(value, minimum, maximum);
    return (uint32_t)((value - minimum) * (float)((1UL << bits) - 1UL) /
                      (maximum - minimum));
}

static float UintToFloat(uint32_t value, float minimum, float maximum, uint8_t bits)
{
    return (float)value * (maximum - minimum) / (float)((1UL << bits) - 1UL) + minimum;
}

static bool DMMotorSendSpecial(DMMotorInstance *motor, uint8_t command)
{
    if (motor == NULL)
        return false;
    memset(motor->can_instance->tx_buff, 0xFF, 7U);
    motor->can_instance->tx_buff[7] = command;
    return CANTransmit(motor->can_instance, motor->tx_timeout_ms) != 0U;
}

static void DMMotorDecode(CANInstance *can)
{
    DMMotorInstance *motor = (DMMotorInstance *)can->id;
    if (motor == NULL || can->rx_len < 6U)
        return;

    const uint8_t *data = can->rx_buff;
    uint16_t position = ((uint16_t)data[1] << 8U) | data[2];
    uint16_t velocity = ((uint16_t)data[3] << 4U) | (data[4] >> 4U);
    uint16_t torque = ((uint16_t)(data[4] & 0x0FU) << 8U) | data[5];

    motor->measure.state = data[0] >> 4U;
    motor->measure.position = UintToFloat(position, -motor->position_max,
                                          motor->position_max, 16U);
    motor->measure.velocity = UintToFloat(velocity, -motor->velocity_max,
                                          motor->velocity_max, 12U);
    motor->measure.torque = UintToFloat(torque, -motor->torque_max,
                                        motor->torque_max, 12U);
    if (can->rx_len >= 8U)
    {
        motor->measure.mos_temperature = data[6];
        motor->measure.rotor_temperature = data[7];
    }
    ++motor->measure.rx_count;
    motor->measure.last_rx_ms = HAL_GetTick();
}

DMMotorInstance *DMMotorInit(const DMMotorInitConfig_s *config)
{
    if (config == NULL || config->can_handle == NULL ||
        config->position_max <= 0.0f || config->velocity_max <= 0.0f ||
        config->torque_max <= 0.0f)
        return NULL;

    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    if (motor == NULL)
        return NULL;
    memset(motor, 0, sizeof(DMMotorInstance));

    motor->position_max = config->position_max;
    motor->velocity_max = config->velocity_max;
    motor->torque_max = config->torque_max;
    motor->tx_timeout_ms = config->tx_timeout_ms == 0U ? 2U : config->tx_timeout_ms;

    CAN_Init_Config_s can_config = {
        .can_handle = config->can_handle,
        .tx_id = config->can_id,
        .rx_id = config->master_id,
        .id_type = CAN_ID_STD,
        .can_module_callback = DMMotorDecode,
        .id = motor,
    };
    motor->can_instance = CANRegister(&can_config);
    return motor;
}

bool DMMotorEnable(DMMotorInstance *motor)
{
    bool sent = DMMotorSendSpecial(motor, 0xFCU);
    if (sent)
        motor->enabled = true;
    return sent;
}

bool DMMotorDisable(DMMotorInstance *motor)
{
    bool sent = DMMotorSendSpecial(motor, 0xFDU);
    if (sent)
        motor->enabled = false;
    return sent;
}

bool DMMotorSetMIT(DMMotorInstance *motor, float position, float velocity,
                   float kp, float kd, float torque)
{
    if (motor == NULL)
        return false;

    uint16_t p = (uint16_t)FloatToUint(position, -motor->position_max,
                                       motor->position_max, 16U);
    uint16_t v = (uint16_t)FloatToUint(velocity, -motor->velocity_max,
                                       motor->velocity_max, 12U);
    uint16_t kp_value = (uint16_t)FloatToUint(kp, 0.0f, DM_KP_MAX, 12U);
    uint16_t kd_value = (uint16_t)FloatToUint(kd, 0.0f, DM_KD_MAX, 12U);
    uint16_t t = (uint16_t)FloatToUint(torque, -motor->torque_max,
                                       motor->torque_max, 12U);

    uint8_t *data = motor->can_instance->tx_buff;
    data[0] = (uint8_t)(p >> 8U);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4U);
    data[3] = (uint8_t)((v << 4U) | (kp_value >> 8U));
    data[4] = (uint8_t)kp_value;
    data[5] = (uint8_t)(kd_value >> 4U);
    data[6] = (uint8_t)((kd_value << 4U) | (t >> 8U));
    data[7] = (uint8_t)t;
    return CANTransmit(motor->can_instance, motor->tx_timeout_ms) != 0U;
}

bool DMMotorSetVelocity(DMMotorInstance *motor, float velocity, float kd)
{
    return DMMotorSetMIT(motor, 0.0f, velocity, 0.0f, kd, 0.0f);
}

bool DMMotorIsOnline(const DMMotorInstance *motor, uint32_t timeout_ms)
{
    return motor != NULL && motor->measure.rx_count > 0U &&
           HAL_GetTick() - motor->measure.last_rx_ms <= timeout_ms;
}
