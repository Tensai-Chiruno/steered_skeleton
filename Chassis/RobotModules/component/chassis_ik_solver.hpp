/**
 *******************************************************************************
 * @file      :chassis_ik_solver.hpp
 * @brief     :舵向轮底盘逆运动学（考核第 5 项）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 四轮独立转向逆解 + 舵角连续性
 *******************************************************************************
 * @attention :
 *  【零外部依赖，可脱离硬件编译】
 *    本文件只用 <cmath>/<cstdint>，不引用 HAL、不引用任何骨架组件。
 *    原因是它必须能在电脑上单测：见 pc_test/test_chassis_ik.cpp。
 *    逆解是全套代码里唯一「纯数学、又能决定车是否走直线的」部分，
 *    把它放在硬件循环里调试是浪费实验室时间的典型方式。
 *
 *  【坐标系】
 *    x 向前、y 向左、z 向上（与 ahrs_base.hpp 一致）。
 *    vx/v_y 为底盘中心线速度（m/s），w 为绕 z 轴角速度（rad/s，逆时针为正）。
 *
 *  【几何（考核说明给定）】
 *    轮半径 66 mm；左右轮距 0.334 m、前后轴距 0.334 m（即半距各 0.167 m）。
 *    轮位（x, y）：左前 (+0.167, +0.167)、左后 (-0.167, +0.167)、
 *                  右后 (-0.167, -0.167)、右前 (+0.167, -0.167)。
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#ifndef HELLO_WORLD_CHASSIS_IK_SOLVER_HPP_
#define HELLO_WORLD_CHASSIS_IK_SOLVER_HPP_

#include <cmath>
#include <cstdint>

namespace hello_world {
namespace chassis_ik_solver {

/**
 * @brief 四轮独立转向（舵向轮）底盘逆解器。
 * @note  有状态：内部保存每个轮子上一次的舵角，用于消解 180° 二义性。
 *        因此非线程安全，只应在 1 ms 主任务里单线程调用。
 */
class ChassisIkSolver {
 public:
  /** 轮序：全代码统一用这个顺序，接线/方向按 id 配，不靠数组顺序 */
  enum WheelIndex : uint8_t {
    kLeftFront = 0,
    kLeftBack = 1,
    kRightBack = 2,
    kRightFront = 3,
    kWheelNum = 4,
  };

  struct Params {
    float wheel_radius = 0.066f;    ///< 轮半径，m
    float half_wheelbase = 0.167f;  ///< 前后半轴距，m
    float half_track = 0.167f;      ///< 左右半轮距，m

    /** 输出端最大轮角速度，rad/s。<= 0 表示不限。
     *  超限时按比例缩放全部轮速（保持运动方向不变），而不是逐轮各自削顶——
     *  逐轮削顶会让四个轮子的合成速度方向歪掉，车会走斜线。 */
    float max_wheel_speed = 0.0f;

    /** 低于此轮心线速度（m/s）视为静止：保持舵角不动，轮速给 0。 */
    float min_linear_speed = 1.0e-4f;
  };

  /** 单轮指令 */
  struct WheelState {
    float angle = 0.0f;  ///< 舵角 rad，已回绕到 [-π, π)
    float speed = 0.0f;  ///< 轮速 rad/s（输出端）。负值表示反向转
  };

  ChassisIkSolver(void) = default;

  /** 设置几何参数并清零内部状态。 */
  void init(const Params &params);

  /** 只清零舵角历史（切换工作模式后调用，避免沿用旧姿态的最近解）。 */
  void reset(void);

  /**
   * @brief  逆解：底盘速度 → 四轮舵角 + 轮速
   * @param  vx  前向速度，m/s
   * @param  vy  左向速度，m/s
   * @param  w   绕 z 角速度，rad/s（逆时针为正）
   * @param  out 长度 4 的输出，按 WheelIndex 排列
   * @retval None
   */
  void solve(float vx, float vy, float w, WheelState out[kWheelNum]);

  const WheelState &wheel(int idx) const { return wheel_[idx & 3]; }
  const Params &params(void) const { return params_; }

  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float kHalfPi = 1.57079632679489661923f;

  /** 把任意角度回绕到 [-π, π)。用 fmod 而非循环加减：1 ms 硬实时里循环次数
   *  不确定是不可接受的（多圈累计时可能要循环几十次）。 */
  static float wrapPi(float angle);

 private:
  Params params_{};
  WheelState wheel_[kWheelNum];
};

}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* HELLO_WORLD_CHASSIS_IK_SOLVER_HPP_ */
