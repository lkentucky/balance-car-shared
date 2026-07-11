/**
 * @file    mpu_app.h
 * @brief   MPU6050 姿态数据应用层接口
 *
 * 提供姿态角读取、最新数据缓存、机械中位点校准等功能。
 * 底层调用 MPU6050 DMP 驱动获取原始四元数，转换为欧拉角。
 */

#ifndef __MPU_APP_H__
#define __MPU_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"        /* HAL 库类型、uintX_t 等 */

/**
 * @brief 姿态角及传感器原始数据结构体
 */
typedef struct
{
  uint32_t tick_ms;      /*!< 采样时刻（HAL_GetTick） */
  uint32_t sample_id;    /*!< 自增采样序号 */
  float pitch;           /*!< 俯仰角，单位：度 */
  float roll;            /*!< 横滚角，单位：度 */
  float yaw;             /*!< 偏航角，单位：度 */
  int16_t accel_x;       /*!< 加速度计 X 轴原始值 */
  int16_t accel_y;       /*!< 加速度计 Y 轴原始值 */
  int16_t accel_z;       /*!< 加速度计 Z 轴原始值 */
  int16_t gyro_x;        /*!< 陀螺仪 X 轴原始值 */
  int16_t gyro_y;        /*!< 陀螺仪 Y 轴原始值 */
  int16_t gyro_z;        /*!< 陀螺仪 Z 轴原始值 */
} MpuApp_Attitude_t;

/**
 * @brief 机械中位点校准结果
 *
 * 小车在竖直静止状态下自动采集多帧 pitch 角度，
 * 经滤波和稳定性判定后得到机械中位角。
 */
typedef struct
{
  float angle_deg;                     /*!< 最终中位角度，单位：度 */
  float filtered_vertical_angle_deg;   /*!< 一阶低通滤波后的角度 */
  uint16_t stable_samples;             /*!< 已累积的稳定样本数 */
  uint8_t ready;                       /*!< 中位校准是否已完成（非零表示就绪） */
} MpuApp_MechanicalMidpoint_t;

/**
 * @brief 初始化 MPU6050 及 DMP
 * @return 1 成功，0 失败
 */
uint8_t MpuApp_Init(void);

/**
 * @brief 读取最新姿态数据，更新内部缓存和中位校准
 * @param[out] attitude  姿态数据指针，传入 NULL 则仅更新内部状态
 * @return 1 读取成功，0 失败
 */
uint8_t MpuApp_Read(MpuApp_Attitude_t *attitude);

/**
 * @brief 获取最近一次有效的姿态数据（中断安全）
 * @param[out] attitude  姿态数据指针
 * @return 1 数据有效，0 无最新数据
 */
uint8_t MpuApp_GetLatest(MpuApp_Attitude_t *attitude);

/**
 * @brief 获取机械中位点校准结果
 * @param[out] midpoint  中位点结构体指针
 * @return 1 校准已完成，0 仍在进行中
 */
uint8_t MpuApp_GetMechanicalMidpoint(MpuApp_MechanicalMidpoint_t *midpoint);

/**
 * @brief 重置机械中位点校准
 */
void MpuApp_ResetMechanicalMidpoint(void);

/**
 * @brief 周期性任务：打印姿态角、检查中位校准报告
 */
void MpuApp_Task(void);

#ifdef __cplusplus
}
#endif

#endif

/* by codex */
