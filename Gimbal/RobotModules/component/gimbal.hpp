/**
 *******************************************************************************
 * @file      :gimbal.hpp
 * @brief     :云台 yaw / pitch 关节空间控制
 * @category : 功能 | 考核第 7 项（云台两轴位置环）
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
    float max_torq = 1.5f;         ///< N·m，两轴力矩限幅
    /* 【现场方案 2026-11-17】pitch 重力前馈（N·m）。
     * 为什么不用积分：用户一句话点破了——"有重力的情况下 I 会一直积累"。
     * 云台被重力/垫块顶住时误差不消失，积分一路涨到限幅；一旦摆脱阻力，
     * 攒满的积分继续满力矩推 -> 猛砸（现场已砸三次）。
     * 正确做法是按重力做**前馈**：重力矩 ≈ m·g·l·cos(pitch)，P 只修残差。
     * 现场反推：kp=4 时云台在 -9° 稳住，说明该角重力矩 ≈ 4×0.157 ≈ 0.63 N·m。 */
    float gravity_torq = 0.65f;    ///< N·m，pitch 重力前馈幅值
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
  float SlewLimit(float want, float &state) const;

  Params params_{};
  GimbalMotion target_{};
  GimbalMotion current_{};
  bool enabled_ = false;
  bool inited_ = false;
  /* 【现场修复 2026-11-17】输出力矩变化率限幅（N·m/s）。
   *
   * 为什么需要：云台头被垫块/静摩擦"搭住"时，误差会一直很大，
   * P（以及 I）已经饱和；一旦它突破静摩擦摆脱垫块，这一瞬间就是满力矩，
   * 云台会"猛砸"出去。现场反复出现，是三件事里最危险的一件。
   *
   * 现在每一拍最多改变 kTorqSlew * dt 的力矩，冲不起来：
   *   40 N·m/s × 1 ms = 0.04 N·m/拍 -> 从 0 加到 1.5 N·m 需要约 38 ms。
   * 代价是响应变慢一点，换来的是"任何情况下都不会突然给满力矩"。
   */
  static constexpr float kTorqSlew = 40.0f;  ///< N·m/s
  float out_yaw_ = 0.0f;
  float out_pitch_ = 0.0f;
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
