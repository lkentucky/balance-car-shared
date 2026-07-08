#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"
#include <stdint.h>

#define BLUETOOTH_RX_BUFFER_SIZE 64U

typedef enum
{
    BT_CMD_NONE        = 0x00,

    BT_CMD_FORWARD     = 0x41,
    BT_CMD_FRONT_RIGHT = 0x42,
    BT_CMD_TURN_RIGHT  = 0x43,
    BT_CMD_BACK_RIGHT  = 0x44,
    BT_CMD_BACKWARD    = 0x45,
    BT_CMD_BACK_LEFT   = 0x46,
    BT_CMD_TURN_LEFT   = 0x47,
    BT_CMD_FRONT_LEFT  = 0x48,

    BT_CMD_GRAVITY     = 0x49,
    BT_CMD_JOYSTICK    = 0x4A,
    BT_CMD_BUTTON      = 0x4B,

    BT_CMD_SPEED_UP    = 0x38,
    BT_CMD_SPEED_DOWN  = 0x39

} BluetoothCommand_t;

typedef struct
{
    uint32_t rx_count;
    uint32_t overflow_count;
    uint8_t last_raw;
    BluetoothCommand_t last_cmd;
} BluetoothStatus_t;

HAL_StatusTypeDef Bluetooth_Init(void);
HAL_StatusTypeDef Bluetooth_StartReceiveIT(void);

void Bluetooth_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Bluetooth_UART_ErrorCallback(UART_HandleTypeDef *huart);

uint8_t Bluetooth_ReadRaw(uint8_t *data);
uint8_t Bluetooth_ReadCommand(BluetoothCommand_t *cmd, uint8_t *raw);

BluetoothCommand_t Bluetooth_DecodeCommand(uint8_t raw);
const char *Bluetooth_GetCommandName(BluetoothCommand_t cmd);
uint8_t Bluetooth_IsKnownCommand(uint8_t raw);

BluetoothStatus_t Bluetooth_GetStatus(void);

/* 后续接电机控制时再实现/调用 */
void Car_BluetoothCommand(BluetoothCommand_t cmd, uint8_t raw);

#endif

/* by codex */
