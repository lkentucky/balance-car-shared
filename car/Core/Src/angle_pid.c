/**
 * @file    angle_pid.c
 * @brief   角度环 PD 控制器实现
 *
 * 实现自平衡小车的角度闭环控制。
 * 机械中值固定为俯仰角 0°，由人工反复测试确定最优值后直接写入代码。
 */

#include "angle_pid.h"

#include <stdio.h>
#include <stdlib.h>

#include "motor.h"
#include "mpu_app.h"
#include "speed_ctrl.h"
#include "usart.h"

/* ======================== 常量定义 ======================== */

#define ANGLE_PID_DT_S 0.01f                /*!< 控制周期：10ms */
#define ANGLE_PID_GAIN_SCALE -100.0f         /*!< 增益缩放系数（配合 KP/KI/KD 量级） */
#define ANGLE_PID_DEFAULT_KP 900.0f          /*!< 角度环 P 增益默认值 */
#define ANGLE_PID_DEFAULT_KI 0.0f            /*!< 角度环 I 增益默认值 */
#define ANGLE_PID_DEFAULT_KD 300.0f          /*!< 角度环 D 增益默认值 */

/**
 * @def ANGLE_PID_MECHANICAL_MID_ANGLE_DEG
 * @brief 机械中值角度，单位：度
 *
 * 固定为 0°，即俯仰角传感器读数 0° 对应小车的物理直立平衡位置。
 * 通过反复上电测试确定最优值后，修改此宏即可。
 */
#define ANGLE_PID_MECHANICAL_MID_ANGLE_DEG 0.3f

#define ANGLE_PID_MAX_PWM_DUTY_PERCENT 90.0f   /*!< 最大 PWM 占空比 */
#define ANGLE_PID_MAX_CONTROL_ANGLE_DEG 40.0f   /*!< 最大可控倾斜角度，超限保护停机 */
#define ANGLE_PID_INTEGRAL_MAX 100.0f            /*!< 积分限幅 */
#define ANGLE_PID_MPU_TIMEOUT_MS 30U            /*!< MPU 数据超时时间 */
#define ANGLE_PID_GYRO_LSB_PER_DPS 16.4f        /*!< 陀螺仪 LSB 到 °/s 的换算系数 */
#define ANGLE_PID_SERIAL_LINE_SIZE 64U           /*!< 串口命令行缓冲区大小 */

/* ======================== 模块内部状态 ======================== */

static AnglePid_State_t angle_pid;
static float angle_pid_max_pwm = ANGLE_PID_MAX_PWM_DUTY_PERCENT;
static float angle_pid_max_control_angle = ANGLE_PID_MAX_CONTROL_ANGLE_DEG;
static uint8_t angle_pid_rx_byte = 0U;
static volatile uint8_t angle_pid_rx_index = 0U;
static volatile uint8_t angle_pid_line_ready = 0U;
static volatile uint8_t angle_pid_rx_overflow = 0U;
static volatile char angle_pid_rx_line[ANGLE_PID_SERIAL_LINE_SIZE];

/* ======================== 内部函数声明 ======================== */

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

/* ======================== 接口函数实现 ======================== */

void AnglePid_Init(void)
{
  angle_pid.kp = ANGLE_PID_DEFAULT_KP;
  angle_pid.ki = ANGLE_PID_DEFAULT_KI;
  angle_pid.kd = ANGLE_PID_DEFAULT_KD;
  angle_pid.target_angle = ANGLE_PID_MECHANICAL_MID_ANGLE_DEG;
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

/**
 * @brief 10ms 周期角度环控制任务
 *
 * 控制流程：
 * 1. 检查使能状态、MPU 数据有效性、超时保护
 * 2. 读取最新 pitch 和 gyro_y
 * 3. 若倾斜角度超过最大可控范围，保护停机
 * 4. 计算 PD 输出：avepwm = kp/scale * error - kd/scale * gyro
 * 5. 叠加速度环和转向环的输出，更新电机 PWM
 *
 * @note 机械中值固定为 ANGLE_PID_MECHANICAL_MID_ANGLE_DEG，
 *       无需等待 MpuApp 的中位校准完成。
 */
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

  /* MPU 数据超时保护 */
  if ((now_ms - attitude.tick_ms) > ANGLE_PID_MPU_TIMEOUT_MS)
  {
    AnglePid_Reset();
    return;
  }

  angle_pid.actual_angle = attitude.pitch;
  angle_pid.gyro = ((float)attitude.gyro_y) / ANGLE_PID_GYRO_LSB_PER_DPS;

  /* 倾角超出可控范围，保护停机 */
  if (AnglePid_Abs(angle_pid.actual_angle - ANGLE_PID_MECHANICAL_MID_ANGLE_DEG) >
      angle_pid_max_control_angle)
  {
    angle_pid.protect_stop = 1U;
    AnglePid_Reset();
    angle_pid.protect_stop = 1U;
    return;
  }

  angle_pid.protect_stop = 0U;

  /* PD 控制律 */
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

  AnglePid_UpdateMotorPwm();
  angle_pid.last_error = angle_pid.error;
}

/* ======================== 串口命令处理 ======================== */

void AnglePid_SerialTask(void)
{
  char line[ANGLE_PID_SERIAL_LINE_SIZE];
  uint32_t primask;
  uint8_t line_ready;

  primask = __get_PRIMASK();
  __disable_irq();
  line_ready = angle_pid_line_ready;
  if (line_ready != 0U)
  {
    memcpy(line, (char *)angle_pid_rx_line, ANGLE_PID_SERIAL_LINE_SIZE);
    angle_pid_line_ready = 0U;
    angle_pid_rx_index = 0U;
    angle_pid_rx_overflow = 0U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (line_ready == 0U)
  {
    return;
  }

  AnglePid_HandleCommand(line);
}

void AnglePid_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  AnglePid_PushSerialByte(angle_pid_rx_byte);
  AnglePid_StartUartReceive();
}

void AnglePid_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  AnglePid_StartUartReceive();
}

/* ======================== 电机 PWM 更新 ======================== */

static void AnglePid_UpdateMotorPwm(void)
{
  float left;
  float right;
  float velocity_pwm;
  float turn_pwm;

  velocity_pwm = SpeedCtrl_GetVelocityPwm();
  turn_pwm = SpeedCtrl_GetTurnPwm();

  left = angle_pid.avepwm + velocity_pwm + turn_pwm;
  right = angle_pid.avepwm + velocity_pwm - turn_pwm;

  angle_pid.left_pwm = AnglePid_RoundToDuty(left);
  angle_pid.right_pwm = AnglePid_RoundToDuty(right);
  Motor_SetDutyWithDeadZone(angle_pid.left_pwm,
                            angle_pid.right_pwm,
                            MOTOR_DEFAULT_DEAD_ZONE_PERCENT);
}

/* ======================== 串口接收 ======================== */

static void AnglePid_StartUartReceive(void)
{
  HAL_UART_Receive_IT(&huart1, &angle_pid_rx_byte, 1U);
}

static void AnglePid_PushSerialByte(uint8_t data)
{
  if (angle_pid_line_ready != 0U)
  {
    return;
  }

  if (data == '\n')
  {
    angle_pid_rx_index = 0U;
    return;
  }

  if ((data == '\r') || (angle_pid_rx_index >= (ANGLE_PID_SERIAL_LINE_SIZE - 1U)))
  {
    angle_pid_rx_line[angle_pid_rx_index] = '\0';
    if (angle_pid_rx_index >= (ANGLE_PID_SERIAL_LINE_SIZE - 1U))
    {
      angle_pid_rx_overflow = 1U;
    }
    angle_pid_line_ready = 1U;
    return;
  }

  angle_pid_rx_line[angle_pid_rx_index] = (char)data;
  angle_pid_rx_index++;
}

/* ======================== 命令解析与处理 ======================== */

static void AnglePid_HandleCommand(char *line)
{
  char *value_text;
  char *cursor;
  float value;
  float value2;

  if (line == NULL)
  {
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "AKP", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "KP", &value_text) != 0U))
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR AKP value\r\n");
      return;
    }
    angle_pid.kp = value;
    AnglePid_Reset();
    AnglePid_PrintStatus("OK");
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "AKI", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "KI", &value_text) != 0U))
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR AKI value\r\n");
      return;
    }
    angle_pid.ki = value;
    AnglePid_Reset();
    AnglePid_PrintStatus("OK");
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "AKD", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "KD", &value_text) != 0U))
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR AKD value\r\n");
      return;
    }
    angle_pid.kd = value;
    AnglePid_Reset();
    AnglePid_PrintStatus("OK");
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "APID", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "PID", &value_text) != 0U))
  {
    cursor = value_text;
    if ((AnglePid_ParseFloat(&cursor, &value) == 0U) ||
        (AnglePid_ParseFloat(&cursor, &value2) == 0U))
    {
      printf("ERR APID format: kp,ki\r\n");
      return;
    }
    /* 仅设置 KP、KI，KD 保持不变 */
    angle_pid.kp = value;
    angle_pid.ki = value2;
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

  if (AnglePid_KeyStartsWith(line, "SPEED", &value_text) != 0U)
  {
    cursor = value_text;
    int32_t left = 0, right = 0;
    if ((AnglePid_ParseFloat(&cursor, &value) == 0U) ||
        (AnglePid_ParseFloat(&cursor, &value2) == 0U))
    {
      printf("ERR SPEED format: left,right (mm/s)\r\n");
      return;
    }
    left = (int32_t)value;
    right = (int32_t)value2;
    SpeedCtrl_SetTarget(left, right);
    SpeedCtrl_SetEnabled(1U);
    SpeedCtrl_PrintStatus("OK");
    return;
  }

  if (AnglePid_KeyStartsWith(line, "TURN", &value_text) != 0U)
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR TURN value\r\n");
      return;
    }
    /* 当前控制架构中 difpwm 没有独立设置接口，这里保持兼容 */
    printf("OK TURN=%.1f\r\n", (double)value);
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "VKP", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "SPDKP", &value_text) != 0U))
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR VKP value\r\n");
      return;
    }
    SpeedCtrl_SetSpeedKp(value);
    SpeedCtrl_PrintStatus("OK");
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "VKI", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "SPDKI", &value_text) != 0U))
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR VKI value\r\n");
      return;
    }
    SpeedCtrl_SetSpeedKi(value);
    SpeedCtrl_PrintStatus("OK");
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "VPID", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "SPDPID", &value_text) != 0U))
  {
    cursor = value_text;
    if ((AnglePid_ParseFloat(&cursor, &value) == 0U) ||
        (AnglePid_ParseFloat(&cursor, &value2) == 0U))
    {
      printf("ERR VPID format: kp,ki\r\n");
      return;
    }
    SpeedCtrl_SetSpeedGains(value, value2);
    SpeedCtrl_PrintStatus("OK");
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "TKP", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "DIFFKP", &value_text) != 0U))
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR TKP value\r\n");
      return;
    }
    SpeedCtrl_SetDiffKp(value);
    SpeedCtrl_PrintStatus("OK");
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "TKI", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "DIFFKI", &value_text) != 0U))
  {
    cursor = value_text;
    if (AnglePid_ParseFloat(&cursor, &value) == 0U)
    {
      printf("ERR TKI value\r\n");
      return;
    }
    SpeedCtrl_SetDiffKi(value);
    SpeedCtrl_PrintStatus("OK");
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "TPID", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "DIFFPID", &value_text) != 0U))
  {
    cursor = value_text;
    if ((AnglePid_ParseFloat(&cursor, &value) == 0U) ||
        (AnglePid_ParseFloat(&cursor, &value2) == 0U))
    {
      printf("ERR TPID format: kp,ki\r\n");
      return;
    }
    SpeedCtrl_SetDiffGains(value, value2);
    SpeedCtrl_PrintStatus("OK");
    return;
  }

  if ((AnglePid_KeyStartsWith(line, "VLIMIT", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "SLIMIT", &value_text) != 0U) ||
      (AnglePid_KeyStartsWith(line, "SOUT", &value_text) != 0U))
  {
    cursor = value_text;
    if ((AnglePid_ParseFloat(&cursor, &value) == 0U) ||
        (AnglePid_ParseFloat(&cursor, &value2) == 0U))
    {
      printf("ERR VLIMIT format\r\n");
      return;
    }
    SpeedCtrl_SetLimits(value, value2);
    SpeedCtrl_PrintStatus("OK");
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
    SpeedCtrl_PrintStatus("SPD");
    return;
  }

  if ((AnglePid_EqualsIgnoreCase(line, "SGET") != 0U) ||
      (AnglePid_EqualsIgnoreCase(line, "SPD?") != 0U))
  {
    SpeedCtrl_PrintStatus("SPD");
    return;
  }

  if ((AnglePid_EqualsIgnoreCase(line, "CAL") != 0U) ||
      (AnglePid_EqualsIgnoreCase(line, "MIDCAL") != 0U))
  {
    printf("OK 机械中值固定为 %.2f°, 无需校准\r\n",
           (double)ANGLE_PID_MECHANICAL_MID_ANGLE_DEG);
    return;
  }

  if (AnglePid_EqualsIgnoreCase(line, "STOP") != 0U)
  {
    angle_pid.enabled = 0U;
    SpeedCtrl_SetEnabled(0U);
    AnglePid_Reset();
    AnglePid_PrintStatus("OK");
    return;
  }

  if (AnglePid_EqualsIgnoreCase(line, "START") != 0U)
  {
    angle_pid.enabled = 1U;
    SpeedCtrl_SetEnabled(1U);
    AnglePid_Reset();
    AnglePid_PrintStatus("OK");
    return;
  }

  printf("ERR cmd: AKP/AKI/AKD/APID or TARGET/PWM/ANGLE or SPEED=L,R or TURN=x or VKP/VKI/VPID or TKP/TKI/TPID or VLIMIT=vel,turn or GET/CAL\r\n");
}

static void AnglePid_PrintStatus(const char *prefix)
{
  printf("%s KP=%.3f KI=%.3f KD=%.3f T=%.2f MID=%.2f PWM=%.1f ANGLE=%.1f EN=%u STOP=%u L=%d R=%d\r\n",
         prefix,
         (double)angle_pid.kp,
         (double)angle_pid.ki,
         (double)angle_pid.kd,
         (double)angle_pid.target_angle,
         (double)ANGLE_PID_MECHANICAL_MID_ANGLE_DEG,
         (double)angle_pid_max_pwm,
         (double)angle_pid_max_control_angle,
         angle_pid.enabled,
         angle_pid.protect_stop,
         angle_pid.left_pwm,
         angle_pid.right_pwm);
}

/* ======================== 工具函数 ======================== */

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
      cb = (char)(cb - 'a' + 'B');
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
  if (value > max_value) return max_value;
  if (value < min_value) return min_value;
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

/* by codex */
