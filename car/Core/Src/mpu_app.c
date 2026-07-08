#include "mpu_app.h"

#include <math.h>
#include <stdio.h>
#include "MPU6050.h"

/* ── 卡尔曼参数（可调） ── */
#define KF_Q_ANGLE   0.001f   /* 角度过程噪声：越小=越信任陀螺 */
#define KF_Q_BIAS    0.003f   /* 零偏漂移噪声：越小=零偏变化越慢 */
#define KF_R_MEASURE 0.05f    /* 测量噪声：越小=越信任加速度计 */

#define MPU_APP_READ_PERIOD_MS  5U
#define CALIB_SAMPLES           100U

/* 灵敏度 */
static float         gyro_sens   = 16.4f;
static unsigned short accel_sens = 16384;

/* 陀螺零偏（上电校准） */
static float gyro_bias_y = 0.0f;

/* ── 卡尔曼滤波器（单轴 pitch）── */
static float kf_angle  = 0.0f;  /* 最优角度估计 */
static float kf_bias   = 0.0f;  /* 陀螺零偏估计 */
static float kf_P[2][2] = {{1, 0}, {0, 1}};

static uint32_t last_kf_ms = 0U;

/* 上电静止校准陀螺零偏 */
static void CalibrateGyro(void) {
  float sum = 0.0f;
  short gyro[3];
  uint32_t i;

  printf("gyro calib...\r\n");
  for (i = 0U; i < CALIB_SAMPLES; i++) {
    while (mpu_get_gyro_reg(gyro, NULL) != 0) {}
    sum += (float)gyro[1];  /* Y 轴 = pitch */
    HAL_Delay(5U);
  }
  gyro_bias_y = sum / (float)CALIB_SAMPLES / gyro_sens;
  printf("gyro bias=%.2f deg/s\r\n", (double)gyro_bias_y);
}

uint8_t MpuApp_Init(void) {
  i2cInit();
  HAL_Delay(100U);

  if (mpu_init() != 0) {
    printf("mpu init failed\r\n");
    return 0U;
  }
  if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) {
    printf("mpu set sensors failed\r\n");
    return 0U;
  }
  mpu_set_sample_rate(200U);
  mpu_get_gyro_sens(&gyro_sens);
  mpu_get_accel_sens(&accel_sens);

  CalibrateGyro();

  last_kf_ms = HAL_GetTick();
  printf("kalman,accel_pitch,gyro_rate\r\n");
  return 1U;
}

/* 读原始传感器 */
static uint8_t ReadRaw(short *gyro, short *accel) {
  if (mpu_get_gyro_reg(gyro, NULL) != 0) return 0U;
  if (mpu_get_accel_reg(accel, NULL) != 0) return 0U;
  return 1U;
}

/* 卡尔曼预测+更新，返回滤波后的角度 */
static float KalmanStep(float gyro_rate_dps, float accel_angle_deg, float dt) {
  float angle_err, S, K0, K1;

  /* ── 预测 ── */
  kf_angle += (gyro_rate_dps - kf_bias) * dt;

  kf_P[0][0] += dt * (dt * kf_P[1][1] - kf_P[0][1] - kf_P[1][0] + KF_Q_ANGLE);
  kf_P[0][1] -= dt * kf_P[1][1];
  kf_P[1][0] -= dt * kf_P[1][1];
  kf_P[1][1] += KF_Q_BIAS * dt;

  /* ── 更新（用加速度计角度修正）── */
  angle_err = accel_angle_deg - kf_angle;

  S = kf_P[0][0] + KF_R_MEASURE;
  K0 = kf_P[0][0] / S;
  K1 = kf_P[1][0] / S;

  kf_angle += K0 * angle_err;
  kf_bias  += K1 * angle_err;

  kf_P[0][0] -= K0 * kf_P[0][0];
  kf_P[0][1] -= K0 * kf_P[0][1];
  kf_P[1][0] -= K1 * kf_P[0][0];
  kf_P[1][1] -= K1 * kf_P[0][1];

  return kf_angle;
}

void MpuApp_Task(void) {
  uint32_t now_ms;
  short gyro[3], accel[3];
  float dt, gyro_rate_dps, accel_angle_deg, kalman_angle;

  now_ms = HAL_GetTick();
  if ((now_ms - last_kf_ms) < MPU_APP_READ_PERIOD_MS) return;

  dt = (float)(now_ms - last_kf_ms) / 1000.0f;
  last_kf_ms = now_ms;

  if (ReadRaw(gyro, accel) == 0U) return;

  /* 陀螺 Y 轴 = pitch 角速度 (°/s)，减零偏 */
  gyro_rate_dps = (float)gyro[1] / gyro_sens - gyro_bias_y;

  /* 加速度计 Y,Z → pitch 角 (°) */
  accel_angle_deg = atan2f((float)accel[1], (float)accel[2]) * 57.29578f;

  /* 卡尔曼融合 */
  kalman_angle = KalmanStep(gyro_rate_dps, accel_angle_deg, dt);

  /* 输出: 卡尔曼角度, 加速度计角度, 陀螺角速度 */
  printf("%.2f,%.2f,%.2f\r\n",
         (double)kalman_angle,
         (double)accel_angle_deg,
         (double)gyro_rate_dps);
}

uint8_t MpuApp_Read(MpuApp_Attitude_t *attitude) {
  short gyro[3], accel[3];
  if (ReadRaw(gyro, accel) == 0U) return 0U;
  if (attitude != NULL) {
    attitude->pitch   = kf_angle;
    attitude->roll    = 0.0f;
    attitude->yaw     = 0.0f;
    attitude->gyro_x  = gyro[0]; attitude->gyro_y  = gyro[1]; attitude->gyro_z  = gyro[2];
    attitude->accel_x = accel[0]; attitude->accel_y = accel[1]; attitude->accel_z = accel[2];
  }
  return 1U;
}

/* by codex */
