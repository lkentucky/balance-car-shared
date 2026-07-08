#include "mpu_app.h"

#include <math.h>
#include <stdio.h>
#include "MPU6050.h"

#define MPU_APP_READ_PERIOD_MS 5U

extern __IO float fAX, fAY, fAZ;
extern __IO short gx, gy, gz, ax, ay, az;

static uint32_t last_read_ms = 0U;

uint8_t MpuApp_Init(void) {
  i2cInit();
  HAL_Delay(100U);

  if (MPU_init() == 0) {
    printf("mpu dmp init failed\r\n");
    return 0U;
  }
  printf("pitch,roll,yaw\r\n");
  return 1U;
}

void MpuApp_Task(void) {
  uint32_t now_ms;

  now_ms = HAL_GetTick();
  if ((now_ms - last_read_ms) < MPU_APP_READ_PERIOD_MS) return;
  last_read_ms = now_ms;

  if (MPU_getdata() == 0U) return;

  printf("%.2f,%.2f,%.2f\r\n",
         (double)fAY, (double)fAX, (double)fAZ);
}

uint8_t MpuApp_Read(MpuApp_Attitude_t *attitude) {
  if (MPU_getdata() == 0U) return 0U;
  if (attitude != NULL) {
    attitude->pitch   = fAY;
    attitude->roll    = fAX;
    attitude->yaw     = fAZ;
    attitude->gyro_x  = gx; attitude->gyro_y  = gy; attitude->gyro_z  = gz;
    attitude->accel_x = ax; attitude->accel_y = ay; attitude->accel_z = az;
  }
  return 1U;
}

/* by codex */
