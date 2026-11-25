#include "step_motor.h"
#include <stdlib.h>
#include <string.h>

#define ZDT_CAN_RX_MASK 0x1FFFFF00U
#define ZDT_COMMAND_END 0x6BU

static bool ZDTMotorSend(ZDTMotorInstance *motor, const uint8_t *command, uint8_t length)
{
    uint8_t offset = 1;
    uint8_t packet = 0;

    while (offset < length)
    {
        uint8_t payload_len = length - offset > 7U ? 7U : length - offset;
        motor->can_instance->txconf.ExtId = ((uint32_t)motor->address << 8U) | packet++;
        motor->can_instance->txconf.DLC = payload_len + 1U;
        motor->can_instance->tx_buff[0] = command[0];
        memcpy(&motor->can_instance->tx_buff[1], &command[offset], payload_len);
        if (!CANTransmit(motor->can_instance, motor->tx_timeout_ms))
            return false;
        offset += payload_len;
    }
    return true;
}

static void ZDTMotorDecode(CANInstance *can)
{
    ZDTMotorInstance *motor = (ZDTMotorInstance *)can->id;
    if (can->rx_len == 0U)
        return;

    motor->measure.last_command = can->rx_buff[0];
    motor->measure.rx_count++;
    if (can->rx_len > 1U)
        motor->measure.last_status = can->rx_buff[1];

    if (can->rx_buff[0] == 0x35U && can->rx_len >= 5U)
    {
        int16_t speed = (int16_t)(((uint16_t)can->rx_buff[2] << 8U) | can->rx_buff[3]);
        motor->measure.speed_rpm = can->rx_buff[1] ? -speed : speed;
    }
    else if (can->rx_buff[0] == 0x36U && can->rx_len >= 7U)
    {
        int32_t position = (int32_t)(((uint32_t)can->rx_buff[2] << 24U) |
                                     ((uint32_t)can->rx_buff[3] << 16U) |
                                     ((uint32_t)can->rx_buff[4] << 8U) |
                                     can->rx_buff[5]);
        motor->measure.position_pulses = can->rx_buff[1] ? -position : position;
    }
}

ZDTMotorInstance *ZDTMotorInit(const ZDTMotorInitConfig_s *config)
{
    if (config == NULL || config->can_handle == NULL || config->address == 0U)
        return NULL;

    ZDTMotorInstance *motor = (ZDTMotorInstance *)malloc(sizeof(ZDTMotorInstance));
    if (motor == NULL)
        return NULL;
    memset(motor, 0, sizeof(ZDTMotorInstance));

    motor->address = config->address;
    motor->reverse = config->reverse;
    motor->default_acceleration = config->default_acceleration;
    motor->tx_timeout_ms = config->tx_timeout_ms == 0U ? 2U : config->tx_timeout_ms;

    CAN_Init_Config_s can_config = {
        .can_handle = config->can_handle,
        .tx_id = (uint32_t)config->address << 8U,
        .rx_id = (uint32_t)config->address << 8U,
        .rx_id_mask = ZDT_CAN_RX_MASK,
        .id_type = CAN_ID_EXT,
        .can_module_callback = ZDTMotorDecode,
        .id = motor,
    };
    motor->can_instance = CANRegister(&can_config);
    return motor;
}

bool ZDTMotorEnable(ZDTMotorInstance *motor, bool enable, bool sync)
{
    if (motor == NULL)
        return false;
    const uint8_t command[] = {0xF3U, 0xABU, enable ? 1U : 0U, sync ? 1U : 0U, ZDT_COMMAND_END};
    bool result = ZDTMotorSend(motor, command, sizeof(command));
    if (result)
        motor->enabled = enable;
    return result;
}

bool ZDTMotorSetSpeed(ZDTMotorInstance *motor, int16_t rpm, uint8_t acceleration, bool sync)
{
    if (motor == NULL)
        return false;
    int32_t signed_rpm = motor->reverse ? -(int32_t)rpm : rpm;
    uint16_t magnitude = signed_rpm < 0 ? (uint16_t)-signed_rpm : (uint16_t)signed_rpm;
    if (magnitude > ZDT_X42_MAX_RPM)
        magnitude = ZDT_X42_MAX_RPM;
    const uint8_t command[] = {
        0xF6U,
        signed_rpm < 0 ? ZDT_DIRECTION_CCW : ZDT_DIRECTION_CW,
        (uint8_t)(magnitude >> 8U), (uint8_t)magnitude,
        acceleration, sync ? 1U : 0U, ZDT_COMMAND_END};
    return ZDTMotorSend(motor, command, sizeof(command));
}

bool ZDTMotorSetRef(ZDTMotorInstance *motor, int16_t rpm)
{
    return motor != NULL && ZDTMotorSetSpeed(motor, rpm, motor->default_acceleration, false);
}

bool ZDTMotorStop(ZDTMotorInstance *motor, bool sync)
{
    if (motor == NULL)
        return false;
    const uint8_t command[] = {0xFEU, 0x98U, sync ? 1U : 0U, ZDT_COMMAND_END};
    return ZDTMotorSend(motor, command, sizeof(command));
}

bool ZDTMotorSetPosition(ZDTMotorInstance *motor, int32_t pulses, uint16_t rpm,
                         uint8_t acceleration, bool absolute, bool sync)
{
    if (motor == NULL)
        return false;
    int64_t target = motor->reverse ? -(int64_t)pulses : pulses;
    uint32_t magnitude = target < 0 ? (uint32_t)-target : (uint32_t)target;
    if (rpm > ZDT_X42_MAX_RPM)
        rpm = ZDT_X42_MAX_RPM;
    const uint8_t command[] = {
        0xFDU,
        target < 0 ? ZDT_DIRECTION_CCW : ZDT_DIRECTION_CW,
        (uint8_t)(rpm >> 8U), (uint8_t)rpm, acceleration,
        (uint8_t)(magnitude >> 24U), (uint8_t)(magnitude >> 16U),
        (uint8_t)(magnitude >> 8U), (uint8_t)magnitude,
        absolute ? 1U : 0U, sync ? 1U : 0U, ZDT_COMMAND_END};
    return ZDTMotorSend(motor, command, sizeof(command));
}

bool ZDTMotorRequestSpeed(ZDTMotorInstance *motor)
{
    if (motor == NULL)
        return false;
    const uint8_t command[] = {0x35U, ZDT_COMMAND_END};
    return ZDTMotorSend(motor, command, sizeof(command));
}

bool ZDTMotorRequestPosition(ZDTMotorInstance *motor)
{
    if (motor == NULL)
        return false;
    const uint8_t command[] = {0x36U, ZDT_COMMAND_END};
    return ZDTMotorSend(motor, command, sizeof(command));
}

bool ZDTMotorSyncStart(ZDTMotorInstance *motor)
{
    if (motor == NULL)
        return false;
    motor->can_instance->txconf.ExtId = 0U;
    motor->can_instance->tx_buff[0] = 0xFFU;
    motor->can_instance->tx_buff[1] = 0x66U;
    motor->can_instance->tx_buff[2] = ZDT_COMMAND_END;
    motor->can_instance->txconf.DLC = 3U;
    return CANTransmit(motor->can_instance, motor->tx_timeout_ms) != 0U;
}
