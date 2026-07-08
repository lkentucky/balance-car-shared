#include "mpu_app.h"

#include <math.h>
#include <stdio.h>
#include "MPU6050.h"

/* 输出周期: 0=实时数据流, 非0=调试模式(ms) */
#define MPU_APP_PRINT_PERIOD_MS 0U
#define MPU_APP_READ_PERIOD_MS  5U

extern __IO float fAX, fAY, fAZ;
extern __IO short gx, gy, gz, ax, ay, az;

static uint32_t last_read_ms  = 0U;

uint8_t MpuApp_Init(void) {
  i2cInit();
  HAL_Delay(100U);

  if (MPU_init() == 0) {
    printf("mpu dmp init failed\r\n");
    return 0U;
  }
  /* FireWater 通道名（VOFA+ 自动解析为图例） */
  printf("pitch,roll,yaw,gx,gy,gz,ax,ay,az\r\n");
  return 1U;
}

uint8_t MpuApp_Read(MpuApp_Attitude_t *attitude) {
  if (MPU_getdata() == 0U) return 0U;

  if (attitude != NULL) {
    attitude->pitch   = fAY;
    attitude->roll    = fAX;
    attitude->yaw     = fAZ;
    attitude->accel_x = ax; attitude->accel_y = ay; attitude->accel_z = az;
    attitude->gyro_x  = gx; attitude->gyro_y  = gy; attitude->gyro_z  = gz;
  }
  return 1U;
}

void MpuApp_Task(void) {
  uint32_t now_ms;
  MpuApp_Attitude_t att;

  now_ms = HAL_GetTick();
  if ((now_ms - last_read_ms) < MPU_APP_READ_PERIOD_MS) return;
  last_read_ms = now_ms;

  if (MpuApp_Read(&att) == 0U) return;

  /* 数据流格式: seq,ms,pitch,roll,yaw,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z */
  printf("%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d\r\n",
         (double)att.pitch, (double)att.roll, (double)att.yaw,
         att.gyro_x, att.gyro_y, att.gyro_z,
         att.accel_x, att.accel_y, att.accel_z);
}

/* by codex */
