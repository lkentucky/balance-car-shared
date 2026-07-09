#ifndef __ANGLE_PID_H__
#define __ANGLE_PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  float target_pitch;
  float kp;
  float ki;
  float kd;
  float last_pitch;
  float integral;
  int16_t output;
  uint8_t safe;
} AnglePidStatus_t;

void AnglePid_Init(void);
void AnglePid_Task(void);
void AnglePid_SerialTask(void);
void AnglePid_Stop(void);
void AnglePid_SetTarget(float target_pitch);
void AnglePid_SetGains(float kp, float ki, float kd);
AnglePidStatus_t AnglePid_GetStatus(void);
void AnglePid_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void AnglePid_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif

/* by codex */
