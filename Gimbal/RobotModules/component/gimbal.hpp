/**
 *******************************************************************************
 * @file      :gimbal.hpp
 * @brief     :云台 yaw / pitch 关节空间控制
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

/* Includes ------------------------------------------------------------------*/
#include "module_state.hpp"

namespace robot {

/**
 * @brief 云台两轴关节空间位置环（考核第 7 项）。
 *
 * - yaw 允许 360° 连续转动，误差按最短路径折算；
 * - pitch 为有限角度，带软件限位，且限位后再做一次机械量程保护；
 * - 未 enable 时两轴力矩指令恒为 0。
 */
class Gimbal {
 public:
  struct Params {
    float pitch_min = -0.60f;      ///< rad，软件下限（向上仰为负，实测确认）
    float pitch_max = 0.90f;       ///< rad，软件上限
    float max_torq = 2.0f;         ///< N·m，两轴力矩限幅
    float yaw_deadband = 0.002f;   ///< rad
    float pitch_deadband = 0.002f; ///< rad
  };

  void init(void);
  void setTarget(const GimbalMotion &tar) { target_ = tar; }
  const GimbalMotion &target(void) const { return target_; }
  const GimbalMotion &current(void) const { return current_; }

  void setEnabled(bool en);
  bool enabled(void) const { return enabled_; }
  void setOutputZero(void);
  void update(void);

 private:
  static float Clamp(float v, float lo, float hi);
  static float WrapPi(float a);

  Params params_{};
  GimbalMotion target_{};
  GimbalMotion current_{};
  bool enabled_ = false;
  bool inited_ = false;
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
