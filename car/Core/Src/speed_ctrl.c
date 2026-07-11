#include "speed_ctrl.h"

#include <stdio.h>

#include "encoder.h"

#define SPEED_CTRL_ENABLE 1U
#define SPEED_CTRL_PERIOD_MS 10U
#define SPEED_CTRL_DT_S 0.01f
#define SPEED_CTRL_DEFAULT_VELOCITY_KP -0.003f
#define SPEED_CTRL_DEFAULT_VELOCITY_KI 0.0f
#define SPEED_CTRL_DEFAULT_TURN_KP 0.020f
#define SPEED_CTRL_DEFAULT_TURN_KI 0.000f
#define SPEED_CTRL_MAX_VELOCITY_PWM_PERCENT 40.0f
#define SPEED_CTRL_MAX_TURN_PWM_PERCENT 40.0f
#define SPEED_CTRL_INTEGRAL_MAX 2000.0f

typedef struct
{
  float target_speed_mm_s;
  float measured_speed_mm_s;
  float error;
  float integral;
  float kp;
  float ki;
  float output;
} SpeedCtrl_PI_t;

static SpeedCtrl_PI_t velocity_pi;
static SpeedCtrl_PI_t turn_pi;
static float speed_ctrl_max_velocity_pwm = SPEED_CTRL_MAX_VELOCITY_PWM_PERCENT;
static float speed_ctrl_max_turn_pwm = SPEED_CTRL_MAX_TURN_PWM_PERCENT;
static float velocity_pwm = 0.0f;
static float turn_pwm = 0.0f;
static uint8_t speed_ctrl_enabled = 1U;

static void SpeedCtrl_InitPi(SpeedCtrl_PI_t *pi, float kp, float ki);
static uint8_t SpeedCtrl_IsEnabled(void);
static float SpeedCtrl_UpdatePi(SpeedCtrl_PI_t *pi, float measured, float max_output);
static float SpeedCtrl_LimitFloat(float value, float min_value, float max_value);
static int32_t SpeedCtrl_RoundToInt32(float value);

void SpeedCtrl_Init(void)
{
  SpeedCtrl_InitPi(&velocity_pi, SPEED_CTRL_DEFAULT_VELOCITY_KP, SPEED_CTRL_DEFAULT_VELOCITY_KI);
  SpeedCtrl_InitPi(&turn_pi, SPEED_CTRL_DEFAULT_TURN_KP, SPEED_CTRL_DEFAULT_TURN_KI);
  speed_ctrl_max_velocity_pwm = SPEED_CTRL_MAX_VELOCITY_PWM_PERCENT;
  speed_ctrl_max_turn_pwm = SPEED_CTRL_MAX_TURN_PWM_PERCENT;
  velocity_pwm = 0.0f;
  turn_pwm = 0.0f;
  speed_ctrl_enabled = (SPEED_CTRL_ENABLE == 0U) ? 0U : 1U;
}

static void SpeedCtrl_InitPi(SpeedCtrl_PI_t *pi, float kp, float ki)
{
  pi->target_speed_mm_s = 0.0f;
  pi->measured_speed_mm_s = 0.0f;
  pi->error = 0.0f;
  pi->integral = 0.0f;
  pi->kp = kp;
  pi->ki = ki;
  pi->output = 0.0f;
}

void SpeedCtrl_SetTarget(int32_t left_mm_s, int32_t right_mm_s)
{
  velocity_pi.target_speed_mm_s = ((float)left_mm_s + (float)right_mm_s) * 0.5f;
  turn_pi.target_speed_mm_s = (float)left_mm_s - (float)right_mm_s;
}

void SpeedCtrl_SetSpeedGains(float kp, float ki)
{
  velocity_pi.kp = kp;
  velocity_pi.ki = ki;
  velocity_pi.integral = 0.0f;
}

void SpeedCtrl_SetSpeedKp(float kp)
{
  velocity_pi.kp = kp;
  velocity_pi.integral = 0.0f;
}

void SpeedCtrl_SetSpeedKi(float ki)
{
  velocity_pi.ki = ki;
  velocity_pi.integral = 0.0f;
}

void SpeedCtrl_SetDiffGains(float kp, float ki)
{
  turn_pi.kp = kp;
  turn_pi.ki = ki;
  turn_pi.integral = 0.0f;
}

void SpeedCtrl_SetDiffKp(float kp)
{
  turn_pi.kp = kp;
  turn_pi.integral = 0.0f;
}

void SpeedCtrl_SetDiffKi(float ki)
{
  turn_pi.ki = ki;
  turn_pi.integral = 0.0f;
}

void SpeedCtrl_SetLimits(float max_velocity_pwm, float max_turn_pwm)
{
  speed_ctrl_max_velocity_pwm = SpeedCtrl_LimitFloat(max_velocity_pwm, 0.0f, 100.0f);
  speed_ctrl_max_turn_pwm = SpeedCtrl_LimitFloat(max_turn_pwm, 0.0f, 100.0f);
}

void SpeedCtrl_SetEnabled(uint8_t enabled)
{
#if (SPEED_CTRL_ENABLE == 0U)
  (void)enabled;
  speed_ctrl_enabled = 0U;
  SpeedCtrl_Stop();
  return;
#else
  speed_ctrl_enabled = (enabled == 0U) ? 0U : 1U;
  if (speed_ctrl_enabled == 0U)
  {
    SpeedCtrl_Stop();
  }
#endif
}

void SpeedCtrl_Task(void)
{
  float left_speed;
  float right_speed;
  float average_speed;
  float diff_speed;

  if (SpeedCtrl_IsEnabled() == 0U)
  {
    return;
  }

  left_speed = ((float)Encoder_GetLeftSpeedMmSX100()) / 100.0f;
  right_speed = ((float)Encoder_GetRightSpeedMmSX100()) / 100.0f;
  average_speed = (left_speed + right_speed) * 0.5f;
  diff_speed = left_speed - right_speed;

  velocity_pwm = SpeedCtrl_UpdatePi(&velocity_pi, average_speed, speed_ctrl_max_velocity_pwm);
  turn_pwm = SpeedCtrl_UpdatePi(&turn_pi, diff_speed, speed_ctrl_max_turn_pwm);
}

void SpeedCtrl_Stop(void)
{
  velocity_pi.integral = 0.0f;
  velocity_pi.error = 0.0f;
  velocity_pi.output = 0.0f;
  turn_pi.integral = 0.0f;
  turn_pi.error = 0.0f;
  turn_pi.output = 0.0f;
  velocity_pwm = 0.0f;
  turn_pwm = 0.0f;
}

float SpeedCtrl_GetVelocityPwm(void)
{
  if (SpeedCtrl_IsEnabled() == 0U)
  {
    return 0.0f;
  }

  return velocity_pwm;
}

float SpeedCtrl_GetTurnPwm(void)
{
  if (SpeedCtrl_IsEnabled() == 0U)
  {
    return 0.0f;
  }

  return turn_pwm;
}

void SpeedCtrl_PrintStatus(const char *prefix)
{
  printf("%s VEL_T=%ld VEL=%ld TURN_T=%ld TURN=%ld VKP=%.4f VKI=%.4f TKP=%.4f TKI=%.4f VPWM=%.2f TPWM=%.2f VMAX=%.1f TMAX=%.1f EN=%u\r\n",
         prefix,
         (long)SpeedCtrl_RoundToInt32(velocity_pi.target_speed_mm_s),
         (long)SpeedCtrl_RoundToInt32(velocity_pi.measured_speed_mm_s),
         (long)SpeedCtrl_RoundToInt32(turn_pi.target_speed_mm_s),
         (long)SpeedCtrl_RoundToInt32(turn_pi.measured_speed_mm_s),
         (double)velocity_pi.kp,
         (double)velocity_pi.ki,
         (double)turn_pi.kp,
         (double)turn_pi.ki,
         (double)velocity_pwm,
         (double)turn_pwm,
         (double)speed_ctrl_max_velocity_pwm,
         (double)speed_ctrl_max_turn_pwm,
         SpeedCtrl_IsEnabled());
}

static uint8_t SpeedCtrl_IsEnabled(void)
{
  if (SPEED_CTRL_ENABLE == 0U)
  {
    return 0U;
  }

  return speed_ctrl_enabled;
}

static float SpeedCtrl_UpdatePi(SpeedCtrl_PI_t *pi, float measured, float max_output)
{
  pi->measured_speed_mm_s = measured;
  pi->error = pi->target_speed_mm_s - pi->measured_speed_mm_s;
  pi->integral += pi->error * SPEED_CTRL_DT_S;
  pi->integral = SpeedCtrl_LimitFloat(pi->integral,
                                      -SPEED_CTRL_INTEGRAL_MAX,
                                      SPEED_CTRL_INTEGRAL_MAX);
  pi->output = pi->kp * pi->error + pi->ki * pi->integral;
  pi->output = SpeedCtrl_LimitFloat(pi->output, -max_output, max_output);

  return pi->output;
}

static float SpeedCtrl_LimitFloat(float value, float min_value, float max_value)
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

static int32_t SpeedCtrl_RoundToInt32(float value)
{
  if (value >= 0.0f)
  {
    return (int32_t)(value + 0.5f);
  }

  return (int32_t)(value - 0.5f);
}

/* by codex */
