#include "angle_pid.h"

#include <stdio.h>
#include <stdlib.h>

#include "motor.h"
#include "mpu_app.h"
#include "usart.h"

#define ANGLE_PID_DT_S 0.01f
#define ANGLE_PID_GAIN_SCALE -100.0f
#define ANGLE_PID_DEFAULT_KP 800.0f
#define ANGLE_PID_DEFAULT_KI 0.0f
#define ANGLE_PID_DEFAULT_KD 180.0f
#define ANGLE_PID_TARGET_ANGLE_DEG 0.0f
#define ANGLE_PID_MAX_PWM_DUTY_PERCENT 70.0f
#define ANGLE_PID_MAX_CONTROL_ANGLE_DEG 40.0f
#define ANGLE_PID_INTEGRAL_MAX 100.0f
#define ANGLE_PID_MPU_TIMEOUT_MS 30U
#define ANGLE_PID_GYRO_LSB_PER_DPS 16.4f
#define ANGLE_PID_SERIAL_LINE_SIZE 64U

static AnglePid_State_t angle_pid;
static float angle_pid_max_pwm = ANGLE_PID_MAX_PWM_DUTY_PERCENT;
static float angle_pid_max_control_angle = ANGLE_PID_MAX_CONTROL_ANGLE_DEG;
static uint8_t angle_pid_rx_byte = 0U;
static volatile uint8_t angle_pid_rx_index = 0U;
static volatile uint8_t angle_pid_line_ready = 0U;
static volatile uint8_t angle_pid_rx_overflow = 0U;
static volatile char angle_pid_rx_line[ANGLE_PID_SERIAL_LINE_SIZE];

static float AnglePid_Abs(float value);
static float AnglePid_LimitFloat(float value, float min_value, float max_value);
static int16_t AnglePid_RoundToDuty(float value);
static void AnglePid_UpdateMotorPwm(void);
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
  angle_pid.kp = ANGLE_PID_DEFAULT_KP;
  angle_pid.ki = ANGLE_PID_DEFAULT_KI;
  angle_pid.kd = ANGLE_PID_DEFAULT_KD;
  angle_pid.target_angle = ANGLE_PID_TARGET_ANGLE_DEG;
  angle_pid_max_pwm = ANGLE_PID_MAX_PWM_DUTY_PERCENT;
  angle_pid_max_control_angle = ANGLE_PID_MAX_CONTROL_ANGLE_DEG;
  angle_pid.enabled = 1U;
  angle_pid_rx_index = 0U;
  angle_pid_line_ready = 0U;
  angle_pid_rx_overflow = 0U;
  AnglePid_Reset();
  AnglePid_StartUartReceive();
}

void AnglePid_Reset(void)
{
  angle_pid.actual_angle = 0.0f;
  angle_pid.gyro = 0.0f;
  angle_pid.error = 0.0f;
  angle_pid.last_error = 0.0f;
  angle_pid.integral = 0.0f;
  angle_pid.avepwm = 0.0f;
  angle_pid.difpwm = 0.0f;
  angle_pid.left_pwm = 0;
  angle_pid.right_pwm = 0;
  angle_pid.protect_stop = 0U;
  Motor_Stop();
}

void Control_Task_10ms(void)
{
  uint32_t now_ms;
  MpuApp_Attitude_t attitude;
  float p_term;
  float i_term;
  float d_term;

  if (angle_pid.enabled == 0U)
  {
    AnglePid_Reset();
    return;
  }

  now_ms = HAL_GetTick();
  if (MpuApp_GetLatest(&attitude) == 0U)
  {
    AnglePid_Reset();
    return;
  }

  if ((now_ms - attitude.tick_ms) > ANGLE_PID_MPU_TIMEOUT_MS)
  {
    AnglePid_Reset();
    return;
  }

  angle_pid.actual_angle = attitude.pitch;
  angle_pid.gyro = ((float)attitude.gyro_y) / ANGLE_PID_GYRO_LSB_PER_DPS;

  if (AnglePid_Abs(angle_pid.actual_angle) > angle_pid_max_control_angle)
  {
    angle_pid.protect_stop = 1U;
    AnglePid_Reset();
    angle_pid.protect_stop = 1U;
    return;
  }

  angle_pid.protect_stop = 0U;
  angle_pid.error = angle_pid.target_angle - angle_pid.actual_angle;
  angle_pid.integral += angle_pid.error * ANGLE_PID_DT_S;
  angle_pid.integral = AnglePid_LimitFloat(angle_pid.integral,
                                           -ANGLE_PID_INTEGRAL_MAX,
                                           ANGLE_PID_INTEGRAL_MAX);

  p_term = angle_pid.kp / ANGLE_PID_GAIN_SCALE * angle_pid.error;
  i_term = angle_pid.ki / ANGLE_PID_GAIN_SCALE * angle_pid.integral;
  d_term = angle_pid.kd / ANGLE_PID_GAIN_SCALE * angle_pid.gyro;

  angle_pid.avepwm = p_term + i_term - d_term;
  angle_pid.avepwm = AnglePid_LimitFloat(angle_pid.avepwm,
                                         -angle_pid_max_pwm,
                                         angle_pid_max_pwm);
  angle_pid.difpwm = 0.0f;

  AnglePid_UpdateMotorPwm();
  angle_pid.last_error = angle_pid.error;
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

static void AnglePid_UpdateMotorPwm(void)
{
  float half_difpwm;
  float left_pwm;
  float right_pwm;

  half_difpwm = angle_pid.difpwm * 0.5f;
  left_pwm = angle_pid.avepwm + half_difpwm;
  right_pwm = angle_pid.avepwm - half_difpwm;

  left_pwm = AnglePid_LimitFloat(left_pwm,
                                 -angle_pid_max_pwm,
                                 angle_pid_max_pwm);
  right_pwm = AnglePid_LimitFloat(right_pwm,
                                  -angle_pid_max_pwm,
                                  angle_pid_max_pwm);

  angle_pid.left_pwm = AnglePid_RoundToDuty(left_pwm);
  angle_pid.right_pwm = AnglePid_RoundToDuty(right_pwm);
  Motor_SetDuty(angle_pid.left_pwm, angle_pid.right_pwm);
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
    angle_pid.kp = value;
    AnglePid_Reset();
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
    angle_pid.ki = value;
    AnglePid_Reset();
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
    angle_pid.kd = value;
    AnglePid_Reset();
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
    angle_pid.kp = kp;
    angle_pid.ki = ki;
    angle_pid.kd = kd;
    AnglePid_Reset();
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
    angle_pid.target_angle = value;
    AnglePid_Reset();
    AnglePid_PrintStatus("OK");
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "PWM", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "MAXPWM", &value_text) != 0U))
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR PWM value\r\n");
      return;
    }
    angle_pid_max_pwm = AnglePid_LimitFloat(value, 0.0f, 100.0f);
    AnglePid_Reset();
    AnglePid_PrintStatus("OK");
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "ANGLE", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "LIMIT", &value_text) != 0U))
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR ANGLE value\r\n");
      return;
    }
    angle_pid_max_control_angle = AnglePid_LimitFloat(value, 0.0f, 90.0f);
    AnglePid_Reset();
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
    angle_pid.enabled = 0U;
    AnglePid_Reset();
    AnglePid_PrintStatus("OK");
    return;
  }

  if (AnglePid_EqualsIgnoreCase(line, "START") != 0U)
  {
    angle_pid.enabled = 1U;
    AnglePid_Reset();
    AnglePid_PrintStatus("OK");
    return;
  }

  printf("ERR cmd: KP=1400 or KI=0 or KD=140 or PID=1400,0,140 or TARGET=0 or PWM=65 or ANGLE=40 or GET\r\n");
}

static void AnglePid_PrintStatus(const char *prefix)
{
  printf("%s KP=%.3f KI=%.3f KD=%.3f T=%.2f PWM=%.1f ANGLE=%.1f EN=%u STOP=%u L=%d R=%d\r\n",
         prefix,
         (double)angle_pid.kp,
         (double)angle_pid.ki,
         (double)angle_pid.kd,
         (double)angle_pid.target_angle,
         (double)angle_pid_max_pwm,
         (double)angle_pid_max_control_angle,
         angle_pid.enabled,
         angle_pid.protect_stop,
         angle_pid.left_pwm,
         angle_pid.right_pwm);
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

  text = AnglePid_SkipSeparators(line);
  key_pos = key;

  while (*key_pos != '\0')
  {
    ca = *text;
    if ((ca >= 'a') && (ca <= 'z'))
    {
      ca = (char)(ca - 'a' + 'A');
    }
    if (ca != *key_pos)
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
