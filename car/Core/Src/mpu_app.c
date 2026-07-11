/**
 * @file    mpu_app.c
 * @brief   MPU6050 姿态数据应用层实现
 *
 * 功能概述：
 * - 初始化 MPU6050 DMP，读取欧拉角（pitch/roll/yaw）
 * - 提供中断安全的最近数据缓存（MpuApp_GetLatest）
 * - 自动校准机械中位点：小车在竖直静止状态下累计有效样本，
 *   经一阶低通滤波和稳定性判定后输出中位角
 * - 周期性通过串口打印俯仰角
 */

#include "mpu_app.h"

#include <math.h>
#include <stdio.h>

#include "MPU6050.h"

/* ======================== 常量定义 ======================== */

/**
 * @def MPU_APP_PRINT_PERIOD_MS
 * @brief 串口打印姿态角的周期，单位：ms
 */
#define MPU_APP_PRINT_PERIOD_MS 20U

/**
 * @def MPU_APP_MECHANICAL_MID_FILTER_ALPHA
 * @brief 机械中位一阶低通滤波器系数，越小越平滑
 */
#define MPU_APP_MECHANICAL_MID_FILTER_ALPHA 0.04f

/**
 * @def MPU_APP_MECHANICAL_MID_UPRIGHT_WINDOW_DEG
 * @brief 判定小车"大致竖直"的 pitch 角度范围，单位：度
 */
#define MPU_APP_MECHANICAL_MID_UPRIGHT_WINDOW_DEG 12.0f

/**
 * @def MPU_APP_MECHANICAL_MID_MAX_GYRO_DPS
 * @brief 中位校准时允许的最大陀螺仪角速度，单位：度/秒
 */
#define MPU_APP_MECHANICAL_MID_MAX_GYRO_DPS 2.0f

/**
 * @def MPU_APP_MECHANICAL_MID_MAX_FILTER_ERROR_DEG
 * @brief 单帧角度与滤波值的最大偏差，超过则重置校准，单位：度
 */
#define MPU_APP_MECHANICAL_MID_MAX_FILTER_ERROR_DEG 0.60f

/**
 * @def MPU_APP_MECHANICAL_MID_MAX_RANGE_DEG
 * @brief 校准期间角度最大波动范围，超过则重置，单位：度
 */
#define MPU_APP_MECHANICAL_MID_MAX_RANGE_DEG 0.80f

/**
 * @def MPU_APP_MECHANICAL_MID_STABLE_SAMPLES
 * @brief 完成校准所需的最小稳定样本数（约 300 帧 × 5ms ≈ 1.5s）
 */
#define MPU_APP_MECHANICAL_MID_STABLE_SAMPLES 300U

/**
 * @def MPU_APP_MECHANICAL_MID_MAX_INVALID_SAMPLES
 * @brief 连续无效帧数上限，超过则重置校准
 */
#define MPU_APP_MECHANICAL_MID_MAX_INVALID_SAMPLES 5U

/**
 * @def MPU_APP_GYRO_LSB_PER_DPS
 * @brief 陀螺仪满量程 ±2000°/s 时每度/秒对应的 LSB 数
 */
#define MPU_APP_GYRO_LSB_PER_DPS 16.4f

/**
 * @def MPU_APP_ACCEL_1G_LSB
 * @brief 加速度计满量程 ±2g 时 1g 对应的 LSB 数
 */
#define MPU_APP_ACCEL_1G_LSB 16384.0f

/**
 * @def MPU_APP_MECHANICAL_MID_ACCEL_MIN_G
 * @brief 中位校准时合加速度最小值（以 g 为单位），用于剔除运动干扰
 */
#define MPU_APP_MECHANICAL_MID_ACCEL_MIN_G 0.85f

/**
 * @def MPU_APP_MECHANICAL_MID_ACCEL_MAX_G
 * @brief 中位校准时合加速度最大值（以 g 为单位）
 */
#define MPU_APP_MECHANICAL_MID_ACCEL_MAX_G 1.15f

/* ======================== 外部引用 ======================== */

/* MPU6050 驱动输出的全局变量（定义于 MPU6050.c） */
extern __IO float fAX, fAY, fAZ;            /*!< 欧拉角：roll、pitch、yaw */
extern __IO short gx, gy, gz;               /*!< 陀螺仪原始值 */
extern __IO short ax, ay, az;               /*!< 加速度计原始值 */

/* ======================== 模块内部状态 ======================== */

static uint32_t last_print_ms = 0U;                         /*!< 上次打印时刻 */
static volatile uint32_t latest_sample_id = 0U;             /*!< 最新采样序号 */
static volatile uint8_t latest_valid = 0U;                  /*!< 缓存数据是否有效 */
static MpuApp_Attitude_t latest_attitude;                   /*!< 最近一次姿态数据缓存 */
static volatile MpuApp_MechanicalMidpoint_t mechanical_midpoint;  /*!< 机械中位校准状态 */
static volatile uint8_t mechanical_midpoint_filter_initialized = 0U;  /*!< 滤波器是否已初始化 */
static volatile uint8_t mechanical_midpoint_invalid_samples = 0U;  /*!< 连续无效帧计数 */
static volatile uint8_t mechanical_midpoint_ready_report_pending = 0U;  /*!< 中位就绪待上报标志 */
static volatile float mechanical_midpoint_candidate_min_angle = 0.0f;  /*!< 候选窗口最小角度 */
static volatile float mechanical_midpoint_candidate_max_angle = 0.0f;  /*!< 候选窗口最大角度 */

/* ======================== 内部函数声明 ======================== */

static void MpuApp_UpdateLatest(MpuApp_Attitude_t *attitude);
static void MpuApp_UpdateMechanicalMidpoint(const MpuApp_Attitude_t *attitude);
static void MpuApp_ResetMechanicalMidpointCandidate(void);
static uint8_t MpuApp_TakeMechanicalMidpointReadyReport(MpuApp_MechanicalMidpoint_t *midpoint);
static float MpuApp_Abs(float value);
static uint8_t MpuApp_IsMechanicalMidpointSampleValid(const MpuApp_Attitude_t *attitude);

/* ======================== 接口函数实现 ======================== */

/**
 * @brief 初始化 MPU6050 和 DMP
 *
 * 执行流程：
 * 1. 重置内部状态
 * 2. 初始化 I2C 总线
 * 3. 调用 MPU_init() 启动 DMP
 * 4. 打印提示信息，告知用户需要将小车竖直静止以便后续中位校准
 *
 * @return 1 成功，0 失败
 */
uint8_t MpuApp_Init(void)
{
  last_print_ms = 0U;
  latest_sample_id = 0U;
  latest_valid = 0U;
  MpuApp_ResetMechanicalMidpoint();

  i2cInit();
  HAL_Delay(100U);

  if (MPU_init() == 0)
  {
    printf("mpu dmp init failed\r\n");
    return 0U;
  }
  printf("pitch,roll,yaw\r\n");
  printf("keep car upright and still for mechanical midpoint calibration\r\n");
  return 1U;
}

/**
 * @brief 周期性任务
 *
 * 每 MPU_APP_PRINT_PERIOD_MS（20ms）通过串口打印一次俯仰角。
 * 同时检查机械中位校准是否完成，若完成则打印一次 MID_OK 提示。
 */
void MpuApp_Task(void)
{
  uint32_t now_ms;
  MpuApp_Attitude_t attitude;
  MpuApp_MechanicalMidpoint_t midpoint;

  /* 检查是否有待上报的中位校准就绪事件 */
  if (MpuApp_TakeMechanicalMidpointReadyReport(&midpoint) != 0U)
  {
    printf("MID_OK=1 MID=%.2f\r\n", (double)midpoint.angle_deg);
  }

  now_ms = HAL_GetTick();
  if ((now_ms - last_print_ms) < MPU_APP_PRINT_PERIOD_MS) return;
  if (MpuApp_GetLatest(&attitude) == 0U) return;

  /* 串口输出俯仰角，供 VOFA+ / 串口助手实时观察 */
  printf("%.2f\r\n", (double)attitude.pitch);
  last_print_ms = now_ms;
}

/**
 * @brief 读取 MPU6050 最新数据
 *
 * 调用 MPU_getdata() 触发 DMP 数据读取，更新：
 * - 姿态角缓存
 * - 机械中位校准状态
 *
 * @param[out] attitude  姿态数据输出指针，可为 NULL
 * @return 1 成功，0 失败（DMP 数据未就绪）
 */
uint8_t MpuApp_Read(MpuApp_Attitude_t *attitude)
{
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

  /* 更新机械中位校准、更新最新缓存 */
  MpuApp_UpdateMechanicalMidpoint(&data);
  MpuApp_UpdateLatest(&data);

  if (attitude != NULL)
  {
    *attitude = data;
  }
  return 1U;
}

/**
 * @brief 获取最近一次有效的姿态数据（中断安全）
 *
 * 在关中断环境下拷贝缓存数据，避免中断和主循环的竞态。
 *
 * @param[out] attitude  姿态数据输出指针
 * @return 1 缓存有效，0 无有效数据
 */
uint8_t MpuApp_GetLatest(MpuApp_Attitude_t *attitude)
{
  uint32_t primask;
  uint8_t valid;

  if (attitude == NULL)
  {
    return 0U;
  }

  /* 关中断保护临界区 */
  primask = __get_PRIMASK();
  __disable_irq();
  valid = latest_valid;
  if (valid != 0U)
  {
    *attitude = latest_attitude;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  return valid;
}

/**
 * @brief 获取机械中位点校准结果
 *
 * 中断安全，适用于在主循环中查询当前校准状态。
 *
 * @param[out] midpoint  中位点结果指针
 * @return 1 校准已完成，0 仍在进行中
 */
uint8_t MpuApp_GetMechanicalMidpoint(MpuApp_MechanicalMidpoint_t *midpoint)
{
  uint32_t primask;

  if (midpoint == NULL)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  midpoint->angle_deg = mechanical_midpoint.angle_deg;
  midpoint->filtered_vertical_angle_deg = mechanical_midpoint.filtered_vertical_angle_deg;
  midpoint->stable_samples = mechanical_midpoint.stable_samples;
  midpoint->ready = mechanical_midpoint.ready;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return midpoint->ready;
}

/**
 * @brief 重置机械中位点校准
 *
 * 清除所有校准状态和候选窗口，重新开始采集。
 */
void MpuApp_ResetMechanicalMidpoint(void)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  mechanical_midpoint.angle_deg = 0.0f;
  mechanical_midpoint.filtered_vertical_angle_deg = 0.0f;
  mechanical_midpoint.stable_samples = 0U;
  mechanical_midpoint.ready = 0U;
  mechanical_midpoint_ready_report_pending = 0U;
  MpuApp_ResetMechanicalMidpointCandidate();
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/* ======================== 内部函数实现 ======================== */

/**
 * @brief 更新最新姿态数据缓存
 *
 * 自增 sample_id，覆盖缓存。
 *
 * @param attitude  新读取的姿态数据
 */
static void MpuApp_UpdateLatest(MpuApp_Attitude_t *attitude)
{
  if (attitude == NULL)
  {
    return;
  }

  latest_sample_id++;
  attitude->sample_id = latest_sample_id;
  latest_attitude = *attitude;
  latest_valid = 1U;
}

/**
 * @brief 更新机械中位点校准状态机
 *
 * 校准流程：
 * 1. 检查单帧数据是否有效（竖直角度范围、陀螺仪静止、合加速度接近 1g）
 * 2. 有效则进入一阶低通滤波器，累积稳定样本数
 * 3. 累计达到 MPU_APP_MECHANICAL_MID_STABLE_SAMPLES 时，校准完成
 * 4. 无效帧连续超过上限则重置整个校准
 *
 * @param attitude  当前帧的姿态数据
 */
static void MpuApp_UpdateMechanicalMidpoint(const MpuApp_Attitude_t *attitude)
{
  float filtered_angle;

  if ((attitude == NULL) || (mechanical_midpoint.ready != 0U))
  {
    return;
  }

  /* 无效帧处理 */
  if (MpuApp_IsMechanicalMidpointSampleValid(attitude) == 0U)
  {
    if (mechanical_midpoint_invalid_samples < MPU_APP_MECHANICAL_MID_MAX_INVALID_SAMPLES)
    {
      mechanical_midpoint_invalid_samples++;
    }
    if (mechanical_midpoint_invalid_samples >= MPU_APP_MECHANICAL_MID_MAX_INVALID_SAMPLES)
    {
      MpuApp_ResetMechanicalMidpointCandidate();
    }
    return;
  }

  mechanical_midpoint_invalid_samples = 0U;

  /* 首次有效帧：初始化滤波器 */
  if (mechanical_midpoint_filter_initialized == 0U)
  {
    mechanical_midpoint.filtered_vertical_angle_deg = attitude->pitch;
    mechanical_midpoint.stable_samples = 1U;
    mechanical_midpoint_filter_initialized = 1U;
    mechanical_midpoint_candidate_min_angle = attitude->pitch;
    mechanical_midpoint_candidate_max_angle = attitude->pitch;
    return;
  }

  /* 更新角度波动窗口 */
  if (attitude->pitch < mechanical_midpoint_candidate_min_angle)
  {
    mechanical_midpoint_candidate_min_angle = attitude->pitch;
  }
  if (attitude->pitch > mechanical_midpoint_candidate_max_angle)
  {
    mechanical_midpoint_candidate_max_angle = attitude->pitch;
  }

  /* 波动过大：认为小车被移动，重置校准 */
  if ((mechanical_midpoint_candidate_max_angle -
       mechanical_midpoint_candidate_min_angle) >
      MPU_APP_MECHANICAL_MID_MAX_RANGE_DEG)
  {
    MpuApp_ResetMechanicalMidpointCandidate();
    return;
  }

  /* 一阶低通滤波 */
  filtered_angle = mechanical_midpoint.filtered_vertical_angle_deg;
  filtered_angle += MPU_APP_MECHANICAL_MID_FILTER_ALPHA *
                    (attitude->pitch - filtered_angle);

  /* 单帧偏差过大：认为受干扰，重置 */
  if (MpuApp_Abs(attitude->pitch - filtered_angle) >
      MPU_APP_MECHANICAL_MID_MAX_FILTER_ERROR_DEG)
  {
    MpuApp_ResetMechanicalMidpointCandidate();
    return;
  }

  mechanical_midpoint.filtered_vertical_angle_deg = filtered_angle;
  if (mechanical_midpoint.stable_samples < MPU_APP_MECHANICAL_MID_STABLE_SAMPLES)
  {
    mechanical_midpoint.stable_samples++;
  }

  /* 稳定样本数达到阈值，完成校准 */
  if (mechanical_midpoint.stable_samples >= MPU_APP_MECHANICAL_MID_STABLE_SAMPLES)
  {
    mechanical_midpoint.angle_deg = filtered_angle;
    mechanical_midpoint.ready = 1U;
    mechanical_midpoint_ready_report_pending = 1U;
  }
}

/**
 * @brief 重置机械中位候选窗口
 *
 * 清空滤波器状态、样本计数、角度范围等所有候选数据。
 * 保留 mechanical_midpoint.ready 不变，由上层调用 Reset 来重置。
 */
static void MpuApp_ResetMechanicalMidpointCandidate(void)
{
  mechanical_midpoint.filtered_vertical_angle_deg = 0.0f;
  mechanical_midpoint.stable_samples = 0U;
  mechanical_midpoint_filter_initialized = 0U;
  mechanical_midpoint_invalid_samples = 0U;
  mechanical_midpoint_candidate_min_angle = 0.0f;
  mechanical_midpoint_candidate_max_angle = 0.0f;
}

/**
 * @brief 获取并清除中位校准就绪上报（一次性）
 *
 * 在 MpuApp_Task 中调用，确保校准完成事件只被打印一次。
 *
 * @param[out] midpoint  中位点结果指针
 * @return 1 有新的就绪事件，0 无
 */
static uint8_t MpuApp_TakeMechanicalMidpointReadyReport(MpuApp_MechanicalMidpoint_t *midpoint)
{
  uint32_t primask;
  uint8_t report_ready;

  if (midpoint == NULL)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  report_ready = 0U;
  if ((mechanical_midpoint_ready_report_pending != 0U) &&
      (mechanical_midpoint.ready != 0U))
  {
    midpoint->angle_deg = mechanical_midpoint.angle_deg;
    midpoint->filtered_vertical_angle_deg = mechanical_midpoint.filtered_vertical_angle_deg;
    midpoint->stable_samples = mechanical_midpoint.stable_samples;
    midpoint->ready = mechanical_midpoint.ready;
    mechanical_midpoint_ready_report_pending = 0U;
    report_ready = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  return report_ready;
}

/**
 * @brief 计算浮点数的绝对值
 */
static float MpuApp_Abs(float value)
{
  if (value < 0.0f)
  {
    return -value;
  }
  return value;
}

/**
 * @brief 判定当前帧是否适合用于机械中位校准
 *
 * 三个判定条件：
 * 1. pitch 在竖直窗口内（±12°）
 * 2. 陀螺仪 Y 轴角速度小于阈值（约 2°/s），确保小车静止
 * 3. 合加速度接近 1g（0.85g ~ 1.15g），排除运动或倾斜干扰
 *
 * @param attitude  当前帧姿态数据
 * @return 1 有效，0 无效
 */
static uint8_t MpuApp_IsMechanicalMidpointSampleValid(const MpuApp_Attitude_t *attitude)
{
  float gyro_limit;
  float accel_x;
  float accel_y;
  float accel_z;
  float accel_magnitude_squared;
  float accel_min;
  float accel_max;

  /* 条件 1：pitch 在竖直窗口范围内 */
  if (!((attitude->pitch >= -MPU_APP_MECHANICAL_MID_UPRIGHT_WINDOW_DEG) &&
        (attitude->pitch <= MPU_APP_MECHANICAL_MID_UPRIGHT_WINDOW_DEG)))
  {
    return 0U;
  }

  /* 条件 2：陀螺仪 Y 轴静止 */
  gyro_limit = MPU_APP_MECHANICAL_MID_MAX_GYRO_DPS * MPU_APP_GYRO_LSB_PER_DPS;
  if (MpuApp_Abs((float)attitude->gyro_y) > gyro_limit)
  {
    return 0U;
  }

  /* 条件 3：合加速度接近 1g */
  accel_x = (float)attitude->accel_x;
  accel_y = (float)attitude->accel_y;
  accel_z = (float)attitude->accel_z;
  accel_magnitude_squared = accel_x * accel_x +
                              accel_y * accel_y +
                              accel_z * accel_z;
  accel_min = MPU_APP_ACCEL_1G_LSB * MPU_APP_MECHANICAL_MID_ACCEL_MIN_G;
  accel_max = MPU_APP_ACCEL_1G_LSB * MPU_APP_MECHANICAL_MID_ACCEL_MAX_G;

  if ((accel_magnitude_squared < (accel_min * accel_min)) ||
      (accel_magnitude_squared > (accel_max * accel_max)))
  {
    return 0U;
  }

  return 1U;
}

/* by codex */
