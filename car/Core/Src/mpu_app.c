#include "mpu_app.h"

#include <math.h>
#include <stdio.h>
#include "MPU6050.h"

#define MPU_APP_PRINT_PERIOD_MS 20U

extern __IO float fAX, fAY, fAZ;
extern __IO short gx, gy, gz, ax, ay, az;

static uint32_t last_print_ms = 0U;
static volatile uint32_t latest_sample_id = 0U;
static volatile uint8_t latest_valid = 0U;
static MpuApp_Attitude_t latest_attitude;

static void MpuApp_UpdateLatest(MpuApp_Attitude_t *attitude);

uint8_t MpuApp_Init(void) {
  last_print_ms = 0U;
  latest_sample_id = 0U;
  latest_valid = 0U;

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
  MpuApp_Attitude_t attitude;

  now_ms = HAL_GetTick();
  if ((now_ms - last_print_ms) < MPU_APP_PRINT_PERIOD_MS) return;
  if (MpuApp_GetLatest(&attitude) == 0U) return;

 /* printf("%.2f,%.2f,%.2f\r\n",
         (double)attitude.pitch,
         (double)attitude.roll,
         (double)attitude.yaw); */
	
	printf("%.2f\r\n", (double)attitude.pitch); 
  last_print_ms = now_ms;
}

uint8_t MpuApp_Read(MpuApp_Attitude_t *attitude) {
  MpuApp_Attitude_t data;

  if (MPU_getdata() == 0U) return 0U;

  data.tick_ms = HAL_GetTick();
  data.sample_id = 0U;
  data.pitch   = fAY;
  data.roll    = fAX;
  data.yaw     = fAZ;
  data.gyro_x  = gx;
  data.gyro_y  = gy;
  data.gyro_z  = gz;
  data.accel_x = ax;
  data.accel_y = ay;
  data.accel_z = az;

  MpuApp_UpdateLatest(&data);

  if (attitude != NULL) {
    *attitude = data;
  }
  return 1U;
}

uint8_t MpuApp_GetLatest(MpuApp_Attitude_t *attitude) {
  uint32_t primask;
  uint8_t valid;

  if (attitude == NULL) {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  valid = latest_valid;
  if (valid != 0U) {
    *attitude = latest_attitude;
  }
  if (primask == 0U) {
    __enable_irq();
  }

  return valid;
}

static void MpuApp_UpdateLatest(MpuApp_Attitude_t *attitude) {
  if (attitude == NULL) {
    return;
  }

  latest_sample_id++;
  attitude->sample_id = latest_sample_id;
  latest_attitude = *attitude;
  latest_valid = 1U;
}

/* by codex */
