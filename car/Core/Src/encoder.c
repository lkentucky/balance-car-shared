#include "encoder.h"

#include <stdint.h>
#include <stdio.h>
#include "tim.h"

#define ENCODER_PI_X1000 3142L
#define ENCODER_LEFT_INVERT 0U
#define ENCODER_RIGHT_INVERT 1U
#define ENCODER_ENABLE_REPORT 0U

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint16_t last_counter;
  int32_t total_count;
  int32_t speed_mm_s_x100;
  int32_t rpm_x100;
  uint8_t inverted;
} Encoder_Channel_t;

static Encoder_Channel_t encoder_left = {&htim4, 0U, 0L, 0L, 0L, ENCODER_LEFT_INVERT};
static Encoder_Channel_t encoder_right = {&htim8, 0U, 0L, 0L, 0L, ENCODER_RIGHT_INVERT};
static uint32_t encoder_last_sample_ms = 0U;
static uint32_t encoder_last_report_ms = 0U;

static void Encoder_StartTimer(TIM_HandleTypeDef *htim);
static void Encoder_ResetChannel(Encoder_Channel_t *channel);
static void Encoder_UpdateChannel(Encoder_Channel_t *channel, uint32_t elapsed_ms);
static int32_t Encoder_DeltaToSpeedMmSX100(int16_t delta_count, uint32_t elapsed_ms);
static int32_t Encoder_DeltaToRpmX100(int16_t delta_count, uint32_t elapsed_ms);
static int32_t Encoder_RoundX100ToInteger(int32_t value_x100);
static int32_t Encoder_DivideRounded(int64_t numerator, int64_t denominator);
static void Encoder_Report(void);
static void Encoder_PrintSignedFixed2(int32_t value);

void Encoder_Init(void)
{
  Encoder_StartTimer(&htim4);
  Encoder_StartTimer(&htim8);
  Encoder_Reset();
#if ENCODER_ENABLE_REPORT
  printf("encoder init ok, cpr=%ld\r\n", (long)ENCODER_COUNTS_PER_WHEEL_REV);
#endif
}

void Encoder_Task(void)
{
  uint32_t now_ms;
  uint32_t elapsed_ms;

  now_ms = HAL_GetTick();
  elapsed_ms = now_ms - encoder_last_sample_ms;

  if (elapsed_ms > 1000U)
  {
    Encoder_Reset();
    return;
  }

  if (elapsed_ms < ENCODER_SAMPLE_PERIOD_MS)
  {
    return;
  }

  encoder_last_sample_ms = now_ms;
  Encoder_UpdateChannel(&encoder_left, elapsed_ms);
  Encoder_UpdateChannel(&encoder_right, elapsed_ms);

#if ENCODER_ENABLE_REPORT
  if ((now_ms - encoder_last_report_ms) >= ENCODER_REPORT_PERIOD_MS)
  {
    encoder_last_report_ms = now_ms;
    Encoder_Report();
  }
#else
  encoder_last_report_ms = now_ms;
#endif
}

void Encoder_Reset(void)
{
  Encoder_ResetChannel(&encoder_left);
  Encoder_ResetChannel(&encoder_right);
  encoder_last_sample_ms = HAL_GetTick();
  encoder_last_report_ms = encoder_last_sample_ms;
}

void Encoder_GetState(Encoder_State_t *state)
{
  if (state == NULL)
  {
    return;
  }

  state->left_count = encoder_left.total_count;
  state->right_count = encoder_right.total_count;
  state->left_speed_mm_s = Encoder_RoundX100ToInteger(encoder_left.speed_mm_s_x100);
  state->right_speed_mm_s = Encoder_RoundX100ToInteger(encoder_right.speed_mm_s_x100);
  state->left_speed_mm_s_x100 = encoder_left.speed_mm_s_x100;
  state->right_speed_mm_s_x100 = encoder_right.speed_mm_s_x100;
  state->left_rpm_x100 = encoder_left.rpm_x100;
  state->right_rpm_x100 = encoder_right.rpm_x100;
}

int32_t Encoder_GetLeftSpeedMmS(void)
{
  return Encoder_RoundX100ToInteger(encoder_left.speed_mm_s_x100);
}

int32_t Encoder_GetRightSpeedMmS(void)
{
  return Encoder_RoundX100ToInteger(encoder_right.speed_mm_s_x100);
}

int32_t Encoder_GetLeftSpeedMmSX100(void)
{
  return encoder_left.speed_mm_s_x100;
}

int32_t Encoder_GetRightSpeedMmSX100(void)
{
  return encoder_right.speed_mm_s_x100;
}

int32_t Encoder_GetLeftRpmX100(void)
{
  return encoder_left.rpm_x100;
}

int32_t Encoder_GetRightRpmX100(void)
{
  return encoder_right.rpm_x100;
}

int32_t Encoder_GetLeftCount(void)
{
  return encoder_left.total_count;
}

int32_t Encoder_GetRightCount(void)
{
  return encoder_right.total_count;
}

static void Encoder_StartTimer(TIM_HandleTypeDef *htim)
{
  if (HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Encoder_ResetChannel(Encoder_Channel_t *channel)
{
  channel->last_counter = (uint16_t)__HAL_TIM_GET_COUNTER(channel->htim);
  channel->total_count = 0L;
  channel->speed_mm_s_x100 = 0L;
  channel->rpm_x100 = 0L;
}

static void Encoder_UpdateChannel(Encoder_Channel_t *channel, uint32_t elapsed_ms)
{
  uint16_t current_counter;
  int16_t delta_count;

  current_counter = (uint16_t)__HAL_TIM_GET_COUNTER(channel->htim);
  delta_count = (int16_t)(current_counter - channel->last_counter);
  channel->last_counter = current_counter;

  if (channel->inverted != 0U)
  {
    delta_count = (int16_t)(-delta_count);
  }

  channel->total_count += delta_count;
  channel->speed_mm_s_x100 = Encoder_DeltaToSpeedMmSX100(delta_count, elapsed_ms);
  channel->rpm_x100 = Encoder_DeltaToRpmX100(delta_count, elapsed_ms);
}

static int32_t Encoder_DeltaToSpeedMmSX100(int16_t delta_count, uint32_t elapsed_ms)
{
  int64_t numerator;
  int64_t denominator;

  numerator = (int64_t)delta_count * ENCODER_PI_X1000 *
              ENCODER_WHEEL_DIAMETER_MM * 100L;
  denominator = (int64_t)ENCODER_COUNTS_PER_WHEEL_REV * elapsed_ms;

  return Encoder_DivideRounded(numerator, denominator);
}

static int32_t Encoder_DeltaToRpmX100(int16_t delta_count, uint32_t elapsed_ms)
{
  int64_t numerator;
  int64_t denominator;

  numerator = (int64_t)delta_count * 60000L * 100L;
  denominator = (int64_t)ENCODER_COUNTS_PER_WHEEL_REV * elapsed_ms;

  return Encoder_DivideRounded(numerator, denominator);
}

static int32_t Encoder_RoundX100ToInteger(int32_t value_x100)
{
  if (value_x100 >= 0L)
  {
    return (value_x100 + 50L) / 100L;
  }

  return (value_x100 - 50L) / 100L;
}

static int32_t Encoder_DivideRounded(int64_t numerator, int64_t denominator)
{
  if (denominator <= 0L)
  {
    return 0L;
  }

  if (numerator >= 0L)
  {
    return (int32_t)((numerator + (denominator / 2L)) / denominator);
  }

  return (int32_t)((numerator - (denominator / 2L)) / denominator);
}

static void Encoder_Report(void)
{
  printf("enc left cnt=%ld rpm=", (long)encoder_left.total_count);
  Encoder_PrintSignedFixed2(encoder_left.rpm_x100);
  printf(" line=");
  Encoder_PrintSignedFixed2(encoder_left.speed_mm_s_x100);
  printf("mm/s, right cnt=%ld rpm=", (long)encoder_right.total_count);
  Encoder_PrintSignedFixed2(encoder_right.rpm_x100);
  printf(" line=");
  Encoder_PrintSignedFixed2(encoder_right.speed_mm_s_x100);
  printf("mm/s\r\n");
}

static void Encoder_PrintSignedFixed2(int32_t value)
{
  long abs_value;

  if (value < 0L)
  {
    printf("-");
    value = -value;
  }

  abs_value = (long)value;
  printf("%ld.%02ld", abs_value / 100L, abs_value % 100L);
}

/* by codex */
