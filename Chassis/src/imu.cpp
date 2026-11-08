/**
 *******************************************************************************
 * @file      :imu.cpp
 * @brief     :IMU 封装实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "imu.hpp"

#if defined(HAL_SPI_MODULE_ENABLED) && defined(HAL_GPIO_MODULE_ENABLED)

/* Private variables ---------------------------------------------------------*/
namespace {
/* BMI088 安装方向：单位阵表示芯片坐标即机器人坐标。
 * 若上板后发现 roll/pitch 与实车不符，只改这 9 个数，不动逻辑。 */
const float kRotMatUnit[9] = {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f};
constexpr float kGravity = 9.80665f;
}  // namespace

namespace hello_world {
namespace imu {

void Imu::init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *acc_cs_port, uint16_t acc_cs_pin,
               GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin,
               const Params &params) {
  params_ = params;

  BMI088HWConfig hw_config;
  hw_config.hspi = hspi;
  hw_config.acc_cs_port = acc_cs_port;
  hw_config.acc_cs_pin = acc_cs_pin;
  hw_config.gyro_cs_port = gyro_cs_port;
  hw_config.gyro_cs_pin = gyro_cs_pin;

  bmi088_.init(hw_config, kRotMatUnit, BMI088Config());
  const uint8_t err = bmi088_.imuInit();
  sensor_ok_ = (err == kBMI088ErrStateNoErr);

  ahrs_.init(params_.samp_freq, params_.mahony_kp, params_.mahony_ki);

  for (int i = 0; i < kAxis; ++i) {
    gyro_bias_[i] = 0.0f;
    bias_sum_[i] = 0.0f;
    acc_[i] = 0.0f;
    gyro_[i] = 0.0f;
    euler_[i] = 0.0f;
  }
  calib_count_ = 0;
  yaw_offset_ = 0.0f;
  ready_ = false;
  inited_ = true;
}

void Imu::update(void) {
  if (!inited_) {
    return;
  }

  bmi088_.getData(acc_, gyro_, &temp_);

  /* 陀螺仪单位换算：BMI088 库输出 rad/s；Mahony 也按 rad/s 使用 */
  for (int i = 0; i < kAxis; ++i) {
    gyro_[i] -= gyro_bias_[i];
  }

  /* ---- 开机静置零漂标定 ---- */
  if (!ready_) {
    float acc_norm = 0.0f;
    for (int i = 0; i < kAxis; ++i) {
      acc_norm += acc_[i] * acc_[i];
    }
    acc_norm = sqrtf(acc_norm);
    const bool acc_still = (acc_norm > (kGravity - params_.acc_norm_tol)) &&
                           (acc_norm < (kGravity + params_.acc_norm_tol));
    const uint32_t need =
        static_cast<uint32_t>(params_.samp_freq * 0.001f * params_.calib_ms);
    if (acc_still && calib_count_ < need) {
      for (int i = 0; i < kAxis; ++i) {
        bias_sum_[i] += gyro_[i];
      }
      ++calib_count_;
      /* 标定期间照常跑姿态解算，保证姿态先收敛 */
      ahrs_.update(acc_, gyro_);
      ahrs_.getEulerAngle(euler_);
      return;
    }
    if (calib_count_ >= need && need > 0) {
      const float n = static_cast<float>(calib_count_);
      for (int i = 0; i < kAxis; ++i) {
        gyro_bias_[i] = bias_sum_[i] / n;
      }
      ahrs_.getEulerAngle(euler_);
      yaw_offset_ = euler_[2];  /* 开机位置即 yaw 零点 */
      ready_ = true;
      return;
    }
    /* 静置条件不满足：这一拍不累加，继续等（不会误标定） */
    ahrs_.update(acc_, gyro_);
    ahrs_.getEulerAngle(euler_);
    return;
  }

  ahrs_.update(acc_, gyro_);
  ahrs_.getEulerAngle(euler_);

  /* yaw 归一化到 (-pi, pi] 并扣掉开机零点 */
  float yaw = euler_[2] - yaw_offset_;
  while (yaw > 3.14159265f) yaw -= 6.28318531f;
  while (yaw < -3.14159265f) yaw += 6.28318531f;
  euler_[2] = yaw;
}

void Imu::resetYaw(void) {
  yaw_offset_ += euler_[2];
  euler_[2] = 0.0f;
}

}  // namespace imu
}  // namespace hello_world

#endif /* HAL_SPI_MODULE_ENABLED && HAL_GPIO_MODULE_ENABLED */
