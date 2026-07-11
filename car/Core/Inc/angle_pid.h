/**
 * @file    angle_pid.h
 * @brief   角度环 PD 控制器接口
 *
 * 机械中值固定为俯仰角 0°，由 ANGLE_PID_MECHANICAL_MID_ANGLE_DEG 定义。
 * 如需调整，修改 angle_pid.c 中的对应宏即可，无需依赖 MpuApp 自动校准。
 */

#ifndef __ANGLE_PID_H__
#define __ANGLE_PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief 角度环状态结构体
 */
typedef struct
{
    float target_angle;   /*!< 目标角度（即机械中值），单位：度 */
    float actual_angle;   /*!< 当前俯仰角，单位：度 */
    float gyro;           /*!< 当前陀螺仪 Y 轴角速度，单位：度/秒 */

    float error;
    float last_error;
    float integral;

    float kp;
    float ki;
    float kd;

    float avepwm;         /*!< 角度环输出的平均 PWM */
    float difpwm;         /*!< 差动 PWM */

    int16_t left_pwm;
    int16_t right_pwm;

    uint8_t enabled;      /*!< 使能标志 */
    uint8_t protect_stop; /*!< 保护停机标志 */
} AnglePid_State_t;

void AnglePid_Init(void);
void AnglePid_Reset(void);
void AnglePid_SerialTask(void);
void Control_Task_10ms(void);
void AnglePid_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void AnglePid_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif

/* by codex */
