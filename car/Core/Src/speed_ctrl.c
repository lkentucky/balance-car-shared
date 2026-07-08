#include "speed_ctrl.h"

#include <stdio.h>

#include "encoder.h"
#include "motor.h"

#define SPEED_CTRL_PERIOD_MS 10U
#define SPEED_CTRL_REPORT_PERIOD_MS 50U
#define SPEED_CTRL_ENABLE_VOFA_REPORT 1U

#define SPEED_CTRL_OUTPUT_MAX_X100 10000L
#define SPEED_CTRL_OUTPUT_MIN_X100 -10000L

#define SPEED_PI_KP 20
#define SPEED_PI_KI 2

typedef struct
{
  int32_t target_speed_mm_s_x100;
  int32_t measured_speed_mm_s_x100;

  int32_t error;
  int32_t last_error;

  int32_t output_x100;

  int32_t kp;
  int32_t ki;
} SpeedPI_t;

static SpeedPI_t left_pi;
static SpeedPI_t right_pi;

static uint32_t speed_ctrl_last_ms = 0U;
static uint32_t speed_ctrl_last_report_ms = 0U;

static void SpeedPI_Init(SpeedPI_t *pi);
static int32_t SpeedPI_Update(SpeedPI_t *pi, int32_t measured_speed_mm_s_x100);
static int32_t SpeedCtrl_LimitOutput(int32_t output_x100);
static int32_t SpeedCtrl_RoundX100ToInteger(int32_t value_x100);
static void SpeedCtrl_ReportVofa(void);

void SpeedCtrl_Init(void)
{
  SpeedPI_Init(&left_pi);
  SpeedPI_Init(&right_pi);

  speed_ctrl_last_ms = HAL_GetTick();
  speed_ctrl_last_report_ms = speed_ctrl_last_ms;
}

static void SpeedPI_Init(SpeedPI_t *pi)
{
  pi->target_speed_mm_s_x100 = 0;
  pi->measured_speed_mm_s_x100 = 0;

  pi->error = 0;
  pi->last_error = 0;

  pi->output_x100 = 0;

  pi->kp = SPEED_PI_KP;
  pi->ki = SPEED_PI_KI;
}

void SpeedCtrl_SetTarget(int32_t left_mm_s, int32_t right_mm_s)
{
  left_pi.target_speed_mm_s_x100 = left_mm_s * 100L;
  right_pi.target_speed_mm_s_x100 = right_mm_s * 100L;
}

static int32_t SpeedPI_Update(SpeedPI_t *pi, int32_t measured_speed_mm_s_x100)
{
  int32_t delta_output;

  pi->measured_speed_mm_s_x100 = measured_speed_mm_s_x100;

  pi->error = pi->target_speed_mm_s_x100 - pi->measured_speed_mm_s_x100;

  delta_output = (pi->kp * (pi->error - pi->last_error) +
                  pi->ki * pi->error) / 1000L;

  pi->output_x100 += delta_output;
  pi->output_x100 = SpeedCtrl_LimitOutput(pi->output_x100);

  pi->last_error = pi->error;

  return pi->output_x100;
}

static int32_t SpeedCtrl_LimitOutput(int32_t output_x100)
{
  if (output_x100 > SPEED_CTRL_OUTPUT_MAX_X100)
  {
    return SPEED_CTRL_OUTPUT_MAX_X100;
  }

  if (output_x100 < SPEED_CTRL_OUTPUT_MIN_X100)
  {
    return SPEED_CTRL_OUTPUT_MIN_X100;
  }

  return output_x100;
}

void SpeedCtrl_Task(void)
{
  uint32_t now_ms;
  int32_t left_output_x100;
  int32_t right_output_x100;

  now_ms = HAL_GetTick();

  if ((now_ms - speed_ctrl_last_ms) < SPEED_CTRL_PERIOD_MS)
  {
    return;
  }

  speed_ctrl_last_ms = now_ms;

  left_output_x100 = SpeedPI_Update(&left_pi, Encoder_GetLeftSpeedMmSX100());
  right_output_x100 = SpeedPI_Update(&right_pi, Encoder_GetRightSpeedMmSX100());

  Motor_SetDuty((int16_t)(left_output_x100 / 100L),
                (int16_t)(right_output_x100 / 100L));

#if SPEED_CTRL_ENABLE_VOFA_REPORT
  if ((now_ms - speed_ctrl_last_report_ms) >= SPEED_CTRL_REPORT_PERIOD_MS)
  {
    speed_ctrl_last_report_ms = now_ms;
    SpeedCtrl_ReportVofa();
  }
#endif
}

void SpeedCtrl_Stop(void)
{
  SpeedPI_Init(&left_pi);
  SpeedPI_Init(&right_pi);
  Motor_Stop();
}

static int32_t SpeedCtrl_RoundX100ToInteger(int32_t value_x100)
{
  if (value_x100 >= 0L)
  {
    return (value_x100 + 50L) / 100L;
  }

  return (value_x100 - 50L) / 100L;
}

static void SpeedCtrl_ReportVofa(void)
{
  int32_t left_target;
  int32_t left_actual;
  int32_t right_target;
  int32_t right_actual;
  int32_t left_pwm;
  int32_t right_pwm;

  left_target = SpeedCtrl_RoundX100ToInteger(left_pi.target_speed_mm_s_x100);
  left_actual = SpeedCtrl_RoundX100ToInteger(left_pi.measured_speed_mm_s_x100);
  right_target = SpeedCtrl_RoundX100ToInteger(right_pi.target_speed_mm_s_x100);
  right_actual = SpeedCtrl_RoundX100ToInteger(right_pi.measured_speed_mm_s_x100);
  left_pwm = SpeedCtrl_RoundX100ToInteger(left_pi.output_x100);
  right_pwm = SpeedCtrl_RoundX100ToInteger(right_pi.output_x100);

  printf("%ld,%ld,%ld,%ld,%ld,%ld\r\n",
         (long)left_target,
         (long)left_actual,
         (long)right_target,
         (long)right_actual,
         (long)left_pwm,
         (long)right_pwm);
}

/* by codex */

