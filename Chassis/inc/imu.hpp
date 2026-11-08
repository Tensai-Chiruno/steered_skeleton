/**
 *******************************************************************************
 * @file      :imu.hpp
 * @brief     :IMU 封装：BMI088 原始数据 + Mahony 姿态 + 开机零漂
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "BMI088.hpp"
#include "mahony.hpp"

#if defined(HAL_SPI_MODULE_ENABLED) && defined(HAL_GPIO_MODULE_ENABLED)

namespace hello_world {
namespace imu {

/**
 * @brief IMU 封装（考核第 4 项）。
 *
 * 职责：
 *   1. 管理 BMI088 的 SPI 句柄与两片 CS 引脚，产出 acc / gyro / temp 原始数据；
 *   2. 用 Mahony 补融合成欧拉角，对外只暴露 roll / pitch / yaw；
 *   3. 开机静置零漂标定：上电后前 kCalibMs 毫秒只累加不输出，
 *      结束后把陀螺仪三轴均值当作零偏扣掉，再把 yaw 清零。
 *
 * 安全约定：标定完成前 isReady() 为 false，调用方（状态机）不得进入 Working。
 */
class Imu {
 public:
  struct Params {
    float samp_freq = 1000.0f;    ///< 采样频率，与调用周期一致（1 kHz）
    float mahony_kp = 1.0f;       ///< 姿态收敛速度
    float mahony_ki = 0.02f;      ///< 零偏在线修正
    uint32_t calib_ms = 1000;     ///< 开机静置标定时长
    float acc_norm_tol = 0.35f;   ///< 标定期间 |a| 允许偏离 1 g 的容差
    uint32_t calib_timeout_ms = 3000;  ///< 标定超时：超过则放弃零偏标定并以 bias=0 就绪（S-13）
  };

  /** @brief 初始化 SPI 与传感器；HAL_SPI_Init 必须已由 CubeMX 完成。 */
  void init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *acc_cs_port, uint16_t acc_cs_pin,
            GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin,
            const Params &params);

  /** @brief 每个控制周期调用一次。 */
  void update(void);

  bool isReady(void) const { return ready_; }
  bool isOk(void) const { return sensor_ok_; }
  /** @brief 是否发生过「标定超时降级」——为 true 表示零偏没标上，姿态只能短期可信。 */
  bool isCalibTimeout(void) const { return calib_timeout_; }

  float roll(void) const { return euler_[0]; }
  float pitch(void) const { return euler_[1]; }
  float yaw(void) const { return euler_[2]; }
  const float *euler(void) const { return euler_; }
  const float *gyro(void) const { return gyro_; }
  const float *acc(void) const { return acc_; }
  const float *gyroBias(void) const { return gyro_bias_; }

  /** @brief 把当前 yaw 记为零点（对中/回正时用），不改变 roll/pitch。 */
  void resetYaw(void);

 private:
  static constexpr int kAxis = 3;

  BMI088 bmi088_{};
  hello_world::ahrs::Mahony ahrs_{};

  Params params_{};
  bool inited_ = false;
  bool sensor_ok_ = false;
  bool ready_ = false;

  float acc_[kAxis] = {0.0f, 0.0f, 0.0f};
  float gyro_[kAxis] = {0.0f, 0.0f, 0.0f};
  float euler_[kAxis] = {0.0f, 0.0f, 0.0f};
  float temp_ = 0.0f;

  float gyro_bias_[kAxis] = {0.0f, 0.0f, 0.0f};
  float bias_sum_[kAxis] = {0.0f, 0.0f, 0.0f};
  uint32_t calib_count_ = 0;
  uint32_t calib_ticks_ = 0;
  float raw_yaw_ = 0.0f;  ///< 未经 yaw_offset_ 修正的原始 yaw，resetYaw() 用它当新零点
  bool calib_timeout_ = false;
  float yaw_offset_ = 0.0f;
};

}  // namespace imu
}  // namespace hello_world

#endif /* HAL_SPI_MODULE_ENABLED && HAL_GPIO_MODULE_ENABLED */

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */
