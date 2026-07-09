#ifndef __ANGLE_PID_H__
#define __ANGLE_PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
    float target_angle;   // 目标平衡角度
    float actual_angle;   // 当前角度
    float gyro;           // 当前角速度

    float error;
    float last_error;
    float integral;

    float kp;
    float ki;
    float kd;

    float avepwm;         // 平均 PWM，由角度环输出
    float difpwm;         // 差分 PWM，后续给转向环用

    int16_t left_pwm;
    int16_t right_pwm;

    uint8_t enabled;      // 控制使能
    uint8_t protect_stop; // 倒车保护标志
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
