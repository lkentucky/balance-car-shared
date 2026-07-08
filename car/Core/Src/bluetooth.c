#include "bluetooth.h"
#include "usart.h"

static uint8_t bt_rx_byte = 0;

static volatile uint8_t bt_rx_buffer[BLUETOOTH_RX_BUFFER_SIZE];
static volatile uint16_t bt_rx_head = 0;
static volatile uint16_t bt_rx_tail = 0;

static volatile uint32_t bt_rx_count = 0;
static volatile uint32_t bt_overflow_count = 0;

static volatile uint8_t bt_last_raw = 0;
static volatile BluetoothCommand_t bt_last_cmd = BT_CMD_NONE;

static void Bluetooth_PushByteFromISR(uint8_t data);

HAL_StatusTypeDef Bluetooth_Init(void)
{
    bt_rx_head = 0;
    bt_rx_tail = 0;
    bt_rx_count = 0;
    bt_overflow_count = 0;
    bt_last_raw = 0;
    bt_last_cmd = BT_CMD_NONE;

    return Bluetooth_StartReceiveIT();
}

HAL_StatusTypeDef Bluetooth_StartReceiveIT(void)
{
    return HAL_UART_Receive_IT(&huart3, &bt_rx_byte, 1);
}

void Bluetooth_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        Bluetooth_PushByteFromISR(bt_rx_byte);

        HAL_UART_Receive_IT(&huart3, &bt_rx_byte, 1);
    }
}

void Bluetooth_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        HAL_UART_AbortReceive_IT(&huart3);
        HAL_UART_Receive_IT(&huart3, &bt_rx_byte, 1);
    }
}

static void Bluetooth_PushByteFromISR(uint8_t data)
{
    uint16_t next_head;

    next_head = (uint16_t)((bt_rx_head + 1U) % BLUETOOTH_RX_BUFFER_SIZE);

    if (next_head != bt_rx_tail)
    {
        bt_rx_buffer[bt_rx_head] = data;
        bt_rx_head = next_head;

        bt_last_raw = data;
        bt_last_cmd = Bluetooth_DecodeCommand(data);
        bt_rx_count++;
    }
    else
    {
        bt_overflow_count++;
    }
}

uint8_t Bluetooth_ReadRaw(uint8_t *data)
{
    uint8_t ret = 0;
    uint32_t primask;

    if (data == 0)
    {
        return 0;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (bt_rx_head != bt_rx_tail)
    {
        *data = bt_rx_buffer[bt_rx_tail];
        bt_rx_tail = (uint16_t)((bt_rx_tail + 1U) % BLUETOOTH_RX_BUFFER_SIZE);
        ret = 1;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    return ret;
}

uint8_t Bluetooth_ReadCommand(BluetoothCommand_t *cmd, uint8_t *raw)
{
    uint8_t data;

    if (Bluetooth_ReadRaw(&data) == 0)
    {
        return 0;
    }

    if (raw != 0)
    {
        *raw = data;
    }

    if (cmd != 0)
    {
        *cmd = Bluetooth_DecodeCommand(data);
    }

    return 1;
}

BluetoothCommand_t Bluetooth_DecodeCommand(uint8_t raw)
{
    switch (raw)
    {
        case 0x41: return BT_CMD_FORWARD;
        case 0x42: return BT_CMD_FRONT_RIGHT;
        case 0x43: return BT_CMD_TURN_RIGHT;
        case 0x44: return BT_CMD_BACK_RIGHT;
        case 0x45: return BT_CMD_BACKWARD;
        case 0x46: return BT_CMD_BACK_LEFT;
        case 0x47: return BT_CMD_TURN_LEFT;
        case 0x48: return BT_CMD_FRONT_LEFT;

        case 0x49: return BT_CMD_GRAVITY;
        case 0x4A: return BT_CMD_JOYSTICK;
        case 0x4B: return BT_CMD_BUTTON;

        case 0x38: return BT_CMD_SPEED_UP;
        case 0x39: return BT_CMD_SPEED_DOWN;

        default:   return BT_CMD_NONE;
    }
}

const char *Bluetooth_GetCommandName(BluetoothCommand_t cmd)
{
    switch (cmd)
    {
        case BT_CMD_FORWARD:     return "FORWARD";
        case BT_CMD_FRONT_RIGHT: return "FRONT_RIGHT";
        case BT_CMD_TURN_RIGHT:  return "TURN_RIGHT";
        case BT_CMD_BACK_RIGHT:  return "BACK_RIGHT";
        case BT_CMD_BACKWARD:    return "BACKWARD";
        case BT_CMD_BACK_LEFT:   return "BACK_LEFT";
        case BT_CMD_TURN_LEFT:   return "TURN_LEFT";
        case BT_CMD_FRONT_LEFT:  return "FRONT_LEFT";

        case BT_CMD_GRAVITY:     return "GRAVITY";
        case BT_CMD_JOYSTICK:    return "JOYSTICK";
        case BT_CMD_BUTTON:      return "BUTTON";

        case BT_CMD_SPEED_UP:    return "SPEED_UP";
        case BT_CMD_SPEED_DOWN:  return "SPEED_DOWN";

        default:                 return "UNKNOWN";
    }
}

uint8_t Bluetooth_IsKnownCommand(uint8_t raw)
{
    return Bluetooth_DecodeCommand(raw) != BT_CMD_NONE;
}

BluetoothStatus_t Bluetooth_GetStatus(void)
{
    BluetoothStatus_t status;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    status.rx_count = bt_rx_count;
    status.overflow_count = bt_overflow_count;
    status.last_raw = bt_last_raw;
    status.last_cmd = bt_last_cmd;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return status;
}

/* 当前阶段不控制电机，只预留接口 */
__weak void Car_BluetoothCommand(BluetoothCommand_t cmd, uint8_t raw)
{
    (void)cmd;
    (void)raw;
}

/* by codex */
