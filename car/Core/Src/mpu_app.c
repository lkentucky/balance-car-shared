#include "mpu_app.h"

#include <math.h>
#include <stdio.h>
#include "MPU6050.h"

#define MPU_APP_READ_PERIOD_MS  10U
#define MPU_APP_PRINT_PERIOD_MS 100U
#define MPU_APP_CALIB_SAMPLES    50U    /* 50 个样本，DMP 50Hz 约 1 秒 */

extern __IO float fAX, fAY, fAZ;
extern __IO short gx, gy, gz;

static uint32_t last_read_ms  = 0U;
static uint32_t last_print_ms = 0U;

/* 静止状态采集原始陀螺数据，计算零偏（均值）和噪声（标准差）。 */
static void MpuApp_CalibrateGyro(void) {
  float buf_x[MPU_APP_CALIB_SAMPLES];
  float buf_y[MPU_APP_CALIB_SAMPLES];
  float buf_z[MPU_APP_CALIB_SAMPLES];
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  float mean_x, mean_y, mean_z;
  float var_x = 0.0f, var_y = 0.0f, var_z = 0.0f;
  uint32_t i;

  printf("gyro calib: keep still, collecting %lu samples...\r\n",
         (unsigned long)MPU_APP_CALIB_SAMPLES);

  HAL_Delay(200U);  /* 等 DMP FIFO 稳定 */

  for (i = 0U; i < MPU_APP_CALIB_SAMPLES; i++) {
    uint32_t timeout = 100U;  /* 最多等 100ms */
    while (MPU_getdata() == 0U) {
      if (--timeout == 0U) break;
      HAL_Delay(1U);
    }
    if (timeout == 0U) {
      printf("calib timeout at sample %lu\r\n", (unsigned long)i);
      break;
    }
    /* DMP 输出 gx/gy/gz 是校准后的原始 LSB，÷16.4 = °/s（±2000°/s 量程） */
    buf_x[i] = (float)gx / 16.4f;
    buf_y[i] = (float)gy / 16.4f;
    buf_z[i] = (float)gz / 16.4f;
    sum_x += buf_x[i];
    sum_y += buf_y[i];
    sum_z += buf_z[i];
    HAL_Delay(20U);
  }

  mean_x = sum_x / (float)MPU_APP_CALIB_SAMPLES;
  mean_y = sum_y / (float)MPU_APP_CALIB_SAMPLES;
  mean_z = sum_z / (float)MPU_APP_CALIB_SAMPLES;

  for (i = 0U; i < MPU_APP_CALIB_SAMPLES; i++) {
    float dx = buf_x[i] - mean_x;
    float dy = buf_y[i] - mean_y;
    float dz = buf_z[i] - mean_z;
    var_x += dx * dx;
    var_y += dy * dy;
    var_z += dz * dz;
  }

  /* 打印数据表 */
  printf("--- raw gyro table (deg/s) ---\r\n");
  printf("  N     gx       gy       gz\r\n");
  for (i = 0U; i < MPU_APP_CALIB_SAMPLES; i++) {
    printf("%3lu  %7.2f  %7.2f  %7.2f\r\n",
           (unsigned long)(i + 1U),
           (double)buf_x[i], (double)buf_y[i], (double)buf_z[i]);
  }
  printf("--- result ---\r\n");
  printf("bias (mean):  x=%.2f  y=%.2f  z=%.2f deg/s\r\n",
         (double)mean_x, (double)mean_y, (double)mean_z);
  printf("noise (std):  x=%.2f  y=%.2f  z=%.2f deg/s\r\n",
         (double)sqrtf(var_x / (float)(MPU_APP_CALIB_SAMPLES - 1U)),
         (double)sqrtf(var_y / (float)(MPU_APP_CALIB_SAMPLES - 1U)),
         (double)sqrtf(var_z / (float)(MPU_APP_CALIB_SAMPLES - 1U)));
}

/* DMP 完整初始化，上电时采集静止数据计算零偏与噪声。 */
uint8_t MpuApp_Init(void) {
  i2cInit();
  HAL_Delay(100U);

  if (MPU_init() == 0) {
    printf("mpu dmp init failed\r\n");
    return 0U;
  }
  printf("mpu dmp init ok\r\n");

  MpuApp_CalibrateGyro();
  return 1U;
}

/* 读 DMP FIFO，返回欧拉角（度）。 */
uint8_t MpuApp_Read(MpuApp_Attitude_t *attitude) {
  if (MPU_getdata() == 0U) {
    return 0U;
  }

  if (attitude != NULL) {
    attitude->pitch   = fAX;   /* pitch 俯仰角 (°) */
    attitude->roll    = fAY;   /* roll  横滚角 (°) */
    attitude->yaw     = fAZ;   /* yaw   偏航角 (°) */
    attitude->accel_x = ax;
    attitude->accel_y = ay;
    attitude->accel_z = az;
    attitude->gyro_x  = gx;
    attitude->gyro_y  = gy;
    attitude->gyro_z  = gz;
  }

  return 1U;
}

static void MpuApp_PrintFixed2(int32_t value) {
  long abs_val;
  if (value < 0L) { printf("-"); value = -value; }
  abs_val = (long)value;
  printf("%ld.%02ld", abs_val / 100L, abs_val % 100L);
}

/* 每 10ms 读取 DMP，每 100ms 打印俯仰角。 */
void MpuApp_Task(void) {
  uint32_t now_ms;
  MpuApp_Attitude_t att;

  now_ms = HAL_GetTick();
  if ((now_ms - last_read_ms) < MPU_APP_READ_PERIOD_MS) {
    return;
  }
  last_read_ms = now_ms;

  if (MpuApp_Read(&att) == 0U) {
    return;
  }

  if ((now_ms - last_print_ms) < MPU_APP_PRINT_PERIOD_MS) {
    return;
  }
  last_print_ms = now_ms;

  /* 用加速度计直接算 pitch 作为对照 */
  float accel_pitch = atan2f((float)att.accel_x,
          sqrtf((float)att.accel_y * (float)att.accel_y +
                (float)att.accel_z * (float)att.accel_z)) * 57.3f;

  printf("dmp: p=");
  MpuApp_PrintFixed2((int32_t)(att.pitch * 100.0f));
  printf(" r=");
  MpuApp_PrintFixed2((int32_t)(att.roll * 100.0f));
  printf(" | accel pitch=");
  MpuApp_PrintFixed2((int32_t)(accel_pitch * 100.0f));
  printf(" | raw acc=%d,%d,%d gyro=%d,%d,%d\r\n",
         att.accel_x, att.accel_y, att.accel_z,
         att.gyro_x, att.gyro_y, att.gyro_z);
}

/* by codex */
