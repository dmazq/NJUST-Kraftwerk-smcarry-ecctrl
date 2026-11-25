#include "serial_remote.h"
#include "bsp_usart.h"
#include <string.h>

#define SERIAL_REMOTE_HEADER_1 0xAAU
#define SERIAL_REMOTE_HEADER_2 0x55U
#define SERIAL_REMOTE_SOURCE 0xA0U
#define SERIAL_REMOTE_TYPE_DISCONNECTED 0x00U
#define SERIAL_REMOTE_TYPE_STANDARD 0x01U
#define SERIAL_REMOTE_TIMEOUT_MS 200U
#define SERIAL_REMOTE_RECOVERY_INTERVAL_MS 500U
#define SERIAL_REMOTE_RX_BUFFER_SIZE 64U

static USARTInstance *remote_uart;
static SerialRemoteData_s remote_data;
static uint8_t rx_frame[SERIAL_REMOTE_FRAME_SIZE];
static uint8_t rx_index;
static uint32_t last_recovery_ms;

static uint8_t SerialRemoteChecksum(const uint8_t *frame)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < SERIAL_REMOTE_FRAME_SIZE - 1U; ++i)
        checksum = (uint8_t)(checksum + frame[i]);
    return checksum;
}

static void SerialRemoteClearControls(void)
{
    remote_data.left_x = 0;
    remote_data.left_y = 0;
    remote_data.right_x = 0;
    remote_data.right_y = 0;
    remote_data.button_1 = 0;
    remote_data.button_2 = 0;
    remote_data.button_3 = 0;
    remote_data.right_trigger = 0;
    remote_data.left_trigger = 0;
}

static void SerialRemoteParseFrame(const uint8_t *frame)
{
    if (frame[3] != SERIAL_REMOTE_SOURCE ||
        frame[SERIAL_REMOTE_FRAME_SIZE - 1U] != SerialRemoteChecksum(frame))
    {
        ++remote_data.invalid_frame_count;
        return;
    }

    if (frame[4] == SERIAL_REMOTE_TYPE_DISCONNECTED)
    {
        SerialRemoteClearControls();
        remote_data.connected = false;
    }
    else if (frame[4] == SERIAL_REMOTE_TYPE_STANDARD)
    {
        remote_data.left_x = (int8_t)frame[5];
        remote_data.left_y = (int8_t)frame[6];
        remote_data.right_x = (int8_t)frame[7];
        remote_data.right_y = (int8_t)frame[8];
        remote_data.button_1 = frame[9];
        remote_data.button_2 = frame[10];
        remote_data.button_3 = frame[11];
        remote_data.right_trigger = frame[12];
        remote_data.left_trigger = frame[13];
        remote_data.connected = true;
    }
    else
    {
        ++remote_data.invalid_frame_count;
        return;
    }

    remote_data.last_update_ms = HAL_GetTick();
    ++remote_data.valid_frame_count;
}

static void SerialRemoteAcceptByte(uint8_t byte)
{
    if (rx_index == 0U)
    {
        if (byte == SERIAL_REMOTE_HEADER_1)
            rx_frame[rx_index++] = byte;
        return;
    }

    if (rx_index == 1U && byte != SERIAL_REMOTE_HEADER_2)
    {
        rx_index = byte == SERIAL_REMOTE_HEADER_1 ? 1U : 0U;
        return;
    }

    rx_frame[rx_index++] = byte;
    if (rx_index == 3U && rx_frame[2] != SERIAL_REMOTE_FRAME_SIZE)
    {
        ++remote_data.invalid_frame_count;
        rx_index = 0U;
    }
    else if (rx_index == SERIAL_REMOTE_FRAME_SIZE)
    {
        SerialRemoteParseFrame(rx_frame);
        rx_index = 0U;
    }
}

static void SerialRemoteRxCallback(void)
{
    for (uint16_t i = 0; i < remote_uart->recv_data_len; ++i)
        SerialRemoteAcceptByte(remote_uart->recv_buff[i]);
}

const SerialRemoteData_s *SerialRemoteInit(UART_HandleTypeDef *uart_handle)
{
    USART_Init_Config_s config = {
        .recv_buff_size = SERIAL_REMOTE_RX_BUFFER_SIZE,
        .usart_handle = uart_handle,
        .module_callback = SerialRemoteRxCallback,
    };

    memset(&remote_data, 0, sizeof(remote_data));
    rx_index = 0U;
    last_recovery_ms = HAL_GetTick();
    remote_uart = USARTRegister(&config);
    return &remote_data;
}

bool SerialRemoteIsOnline(void)
{
    uint32_t now = HAL_GetTick();
    bool online = remote_data.connected &&
                  (now - remote_data.last_update_ms <= SERIAL_REMOTE_TIMEOUT_MS);

    if ((!online || remote_uart->rx_restart_pending) &&
        now - last_recovery_ms >= SERIAL_REMOTE_RECOVERY_INTERVAL_MS)
    {
        rx_index = 0U;
        USARTRestartReceive(remote_uart);
        last_recovery_ms = now;
        ++remote_data.recovery_count;
    }
    return online;
}
