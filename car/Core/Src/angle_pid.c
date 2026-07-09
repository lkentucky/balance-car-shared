#include "angle_pid.h"

#include <stdio.h>
#include <stdlib.h>

#include "motor.h"
#include "mpu_app.h"
#include "usart.h"

#define ANGLE_PID_PERIOD_MS 10U
#define ANGLE_PID_DT_S 0.01f
#define ANGLE_PID_MPU_TIMEOUT_MS 30U
#define ANGLE_PID_STOP_ANGLE_DEG 55.0f
#define ANGLE_PID_OUTPUT_MAX_PERCENT 65.0f
#define ANGLE_PID_INTEGRAL_MAX 100.0f
#define ANGLE_PID_GYRO_LSB_PER_DPS 16.4f
#define ANGLE_PID_OUTPUT_SIGN -1.0f
#define ANGLE_PID_SERIAL_LINE_SIZE 64U

#define ANGLE_PID_DEFAULT_TARGET 0.0f
#define ANGLE_PID_DEFAULT_KP 8.0f
#define ANGLE_PID_DEFAULT_KI 0.0f
#define ANGLE_PID_DEFAULT_KD 8.0f

static AnglePidStatus_t angle_pid;
static uint32_t angle_pid_last_ms = 0U;
static uint8_t angle_pid_rx_byte = 0U;
static volatile uint8_t angle_pid_rx_index = 0U;
static volatile uint8_t angle_pid_line_ready = 0U;
static volatile uint8_t angle_pid_rx_overflow = 0U;
static volatile char angle_pid_rx_line[ANGLE_PID_SERIAL_LINE_SIZE];

static float AnglePid_Abs(float value);
static float AnglePid_LimitFloat(float value, float min_value, float max_value);
static int16_t AnglePid_RoundToDuty(float value);
static void AnglePid_ResetDynamicState(void);
static void AnglePid_StartUartReceive(void);
static void AnglePid_PushSerialByte(uint8_t data);
static void AnglePid_HandleCommand(char *line);
static void AnglePid_PrintStatus(const char *prefix);
static char *AnglePid_SkipSeparators(char *text);
static uint8_t AnglePid_EqualsIgnoreCase(const char *a, const char *b);
static uint8_t AnglePid_KeyStartsWith(char *line, const char *key, char **value_text);
static uint8_t AnglePid_ParseFloat(char **text, float *value);

void AnglePid_Init(void)
{
  angle_pid.target_pitch = ANGLE_PID_DEFAULT_TARGET;
  angle_pid.kp = ANGLE_PID_DEFAULT_KP;
  angle_pid.ki = ANGLE_PID_DEFAULT_KI;
  angle_pid.kd = ANGLE_PID_DEFAULT_KD;
  angle_pid.safe = 1U;
  AnglePid_ResetDynamicState();
  angle_pid_last_ms = HAL_GetTick();
  angle_pid_rx_index = 0U;
  angle_pid_line_ready = 0U;
  angle_pid_rx_overflow = 0U;
  AnglePid_StartUartReceive();
}

void AnglePid_Task(void)
{
  uint32_t now_ms;
  MpuApp_Attitude_t attitude;
  float pitch;
  float gyro_pitch_dps;
  float error;
  float output;
  int16_t duty;

  now_ms = HAL_GetTick();
  if ((now_ms - angle_pid_last_ms) < ANGLE_PID_PERIOD_MS)
  {
    return;
  }
  angle_pid_last_ms = now_ms;

  if (MpuApp_GetLatest(&attitude) == 0U)
  {
    AnglePid_Stop();
    return;
  }

  if ((now_ms - attitude.tick_ms) > ANGLE_PID_MPU_TIMEOUT_MS)
  {
    AnglePid_Stop();
    return;
  }

  pitch = attitude.pitch;
  angle_pid.last_pitch = pitch;

  if (AnglePid_Abs(pitch) > ANGLE_PID_STOP_ANGLE_DEG)
  {
    angle_pid.safe = 0U;
    AnglePid_ResetDynamicState();
    Motor_Stop();
    return;
  }

  angle_pid.safe = 1U;
  gyro_pitch_dps = ((float)attitude.gyro_y) / ANGLE_PID_GYRO_LSB_PER_DPS;
  error = angle_pid.target_pitch - pitch;

  angle_pid.integral += error * ANGLE_PID_DT_S;
  angle_pid.integral = AnglePid_LimitFloat(angle_pid.integral,
                                           -ANGLE_PID_INTEGRAL_MAX,
                                           ANGLE_PID_INTEGRAL_MAX);

  output = angle_pid.kp * error +
           angle_pid.ki * angle_pid.integral -
           angle_pid.kd * gyro_pitch_dps;
  output *= ANGLE_PID_OUTPUT_SIGN;
  output = AnglePid_LimitFloat(output,
                               -ANGLE_PID_OUTPUT_MAX_PERCENT,
                               ANGLE_PID_OUTPUT_MAX_PERCENT);

  duty = AnglePid_RoundToDuty(output);
  angle_pid.output = duty;
  Motor_SetDuty(duty, duty);
}

void AnglePid_SerialTask(void)
{
  char line[ANGLE_PID_SERIAL_LINE_SIZE];
  uint8_t index;
  uint8_t overflow;
  uint8_t got_line = 0U;
  uint32_t primask;

  if ((angle_pid_line_ready == 0U) && (angle_pid_rx_overflow == 0U))
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  overflow = angle_pid_rx_overflow;
  angle_pid_rx_overflow = 0U;
  index = angle_pid_rx_index;

  if (angle_pid_line_ready != 0U)
  {
    uint8_t i;

    if (index >= ANGLE_PID_SERIAL_LINE_SIZE)
    {
      index = ANGLE_PID_SERIAL_LINE_SIZE - 1U;
    }

    for (i = 0U; i <= index; i++)
    {
      line[i] = (char)angle_pid_rx_line[i];
    }

    angle_pid_line_ready = 0U;
    angle_pid_rx_index = 0U;
    got_line = 1U;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }

  if (overflow != 0U)
  {
    printf("ERR PID uart line too long\r\n");
  }

  if ((got_line == 0U) || (index == 0U))
  {
    return;
  }

  AnglePid_HandleCommand(line);
}

void AnglePid_Stop(void)
{
  AnglePid_ResetDynamicState();
  Motor_Stop();
}

void AnglePid_SetTarget(float target_pitch)
{
  angle_pid.target_pitch = target_pitch;
  AnglePid_ResetDynamicState();
}

void AnglePid_SetGains(float kp, float ki, float kd)
{
  angle_pid.kp = kp;
  angle_pid.ki = ki;
  angle_pid.kd = kd;
  AnglePid_ResetDynamicState();
}

AnglePidStatus_t AnglePid_GetStatus(void)
{
  return angle_pid;
}

static float AnglePid_Abs(float value)
{
  if (value < 0.0f)
  {
    return -value;
  }

  return value;
}

static float AnglePid_LimitFloat(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }

  if (value < min_value)
  {
    return min_value;
  }

  return value;
}

static int16_t AnglePid_RoundToDuty(float value)
{
  if (value >= 0.0f)
  {
    return (int16_t)(value + 0.5f);
  }

  return (int16_t)(value - 0.5f);
}

static void AnglePid_ResetDynamicState(void)
{
  angle_pid.last_pitch = 0.0f;
  angle_pid.integral = 0.0f;
  angle_pid.output = 0;
}

void AnglePid_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    AnglePid_PushSerialByte(angle_pid_rx_byte);
    AnglePid_StartUartReceive();
  }
}

void AnglePid_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_AbortReceive_IT(&huart1);
    AnglePid_StartUartReceive();
  }
}

static void AnglePid_StartUartReceive(void)
{
  HAL_UART_Receive_IT(&huart1, &angle_pid_rx_byte, 1U);
}

static void AnglePid_PushSerialByte(uint8_t data)
{
  if ((data == '\r') || (data == '\n'))
  {
    if ((angle_pid_rx_index > 0U) && (angle_pid_line_ready == 0U))
    {
      angle_pid_rx_line[angle_pid_rx_index] = '\0';
      angle_pid_line_ready = 1U;
    }
    return;
  }

  if (angle_pid_line_ready != 0U)
  {
    return;
  }

  if (angle_pid_rx_index < (ANGLE_PID_SERIAL_LINE_SIZE - 1U))
  {
    angle_pid_rx_line[angle_pid_rx_index] = (char)data;
    angle_pid_rx_index++;
  }
  else
  {
    angle_pid_rx_index = 0U;
    angle_pid_rx_overflow = 1U;
  }
}

static void AnglePid_HandleCommand(char *line)
{
  char *value_text;
  char *cursor;
  float kp;
  float ki;
  float kd;
  float value;

  if (AnglePid_KeyStartsWith(line, "KP", &value_text) != 0U)
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR KP value\r\n");
      return;
    }
    AnglePid_SetGains(value, angle_pid.ki, angle_pid.kd);
    AnglePid_PrintStatus("OK");
    return;
  }

  if (AnglePid_KeyStartsWith(line, "KI", &value_text) != 0U)
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR KI value\r\n");
      return;
    }
    AnglePid_SetGains(angle_pid.kp, value, angle_pid.kd);
    AnglePid_PrintStatus("OK");
    return;
  }

  if (AnglePid_KeyStartsWith(line, "KD", &value_text) != 0U)
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR KD value\r\n");
      return;
    }
    AnglePid_SetGains(angle_pid.kp, angle_pid.ki, value);
    AnglePid_PrintStatus("OK");
    return;
  }

  if (AnglePid_KeyStartsWith(line, "PID", &value_text) != 0U)
  {
    cursor = value_text;
    if ((AnglePid_ParseFloat(&cursor, &kp) == 0U) ||
        (AnglePid_ParseFloat(&cursor, &ki) == 0U) ||
        (AnglePid_ParseFloat(&cursor, &kd) == 0U))
    {
      printf("ERR PID format\r\n");
      return;
    }
    AnglePid_SetGains(kp, ki, kd);
    AnglePid_PrintStatus("OK");
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "TARGET", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "T", &value_text) != 0U))
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR TARGET value\r\n");
      return;
    }
    AnglePid_SetTarget(value);
    AnglePid_PrintStatus("OK");
    return;
  }

  line = AnglePid_SkipSeparators(line);
  if ((AnglePid_EqualsIgnoreCase(line, "GET") != 0U) ||
      (AnglePid_EqualsIgnoreCase(line, "?") != 0U))
  {
    AnglePid_PrintStatus("PID");
    return;
  }

  if (AnglePid_EqualsIgnoreCase(line, "STOP") != 0U)
  {
    AnglePid_Stop();
    printf("OK STOP\r\n");
    return;
  }

  printf("ERR cmd: KP=10.0 or KI=0.0 or KD=0.08 or PID=10,0,0.08 or GET\r\n");
}

static void AnglePid_PrintStatus(const char *prefix)
{
  printf("%s KP=%.3f KI=%.3f KD=%.3f T=%.2f OUT=%d SAFE=%u\r\n",
         prefix,
         (double)angle_pid.kp,
         (double)angle_pid.ki,
         (double)angle_pid.kd,
         (double)angle_pid.target_pitch,
         angle_pid.output,
         angle_pid.safe);
}

static char *AnglePid_SkipSeparators(char *text)
{
  while ((*text == ' ') || (*text == '\t') || (*text == '=') ||
         (*text == ':') || (*text == ','))
  {
    text++;
  }

  return text;
}

static uint8_t AnglePid_EqualsIgnoreCase(const char *a, const char *b)
{
  char ca;
  char cb;

  while ((*a != '\0') && (*b != '\0'))
  {
    ca = *a;
    cb = *b;

    if ((ca >= 'a') && (ca <= 'z'))
    {
      ca = (char)(ca - 'a' + 'A');
    }
    if ((cb >= 'a') && (cb <= 'z'))
    {
      cb = (char)(cb - 'a' + 'A');
    }
    if (ca != cb)
    {
      return 0U;
    }

    a++;
    b++;
  }

  return ((*a == '\0') && (*b == '\0')) ? 1U : 0U;
}

static uint8_t AnglePid_KeyStartsWith(char *line, const char *key, char **value_text)
{
  char *text;
  const char *key_pos;
  char ca;
  char cb;

  text = AnglePid_SkipSeparators(line);
  key_pos = key;

  while (*key_pos != '\0')
  {
    ca = *text;
    cb = *key_pos;

    if ((ca >= 'a') && (ca <= 'z'))
    {
      ca = (char)(ca - 'a' + 'A');
    }
    if (ca != cb)
    {
      return 0U;
    }

    text++;
    key_pos++;
  }

  if ((*text != '\0') && (*text != ' ') && (*text != '\t') &&
      (*text != '=') && (*text != ':') && (*text != ','))
  {
    return 0U;
  }

  *value_text = text;
  return 1U;
}

static uint8_t AnglePid_ParseFloat(char **text, float *value)
{
  char *start;
  char *end;

  start = AnglePid_SkipSeparators(*text);
  *value = (float)strtod(start, &end);

  if (end == start)
  {
    return 0U;
  }

  *text = end;
  return 1U;
}

/* by codex */
