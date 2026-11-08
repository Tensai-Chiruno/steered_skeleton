/**
 *******************************************************************************
 * @file      :chassis.hpp
 * @brief     :底盘运动模块（舵向轮逆解 + 8 个闭环）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "chassis_ik_solver.hpp"
#include "module_state.hpp"

namespace robot {

/**
 * @brief 底盘运动模块：把 (vx, vy, w) 变成 4 个舵角 + 4 个轮速，并闭 8 个环。
 *
 * 安全约定（考核硬性）：
 *   - 未 enable 时所有电机输入恒为 0；
 *   - target 会被限幅到参数上限，任何输入都不会产生越界指令。
 */
class Chassis {
 public:
  struct Params {
    float max_linear_speed = 3.0f;    ///< 平移速度限幅，m/s
    float max_angular_speed = 10.0f;  ///< 角速度限幅，rad/s
    float max_wheel_vel = 120.0f;     ///< 轮电机输出端角速度限幅，rad/s
    float steer_raw_limit = 20000.0f; ///< 舵电机 raw 指令限幅（GM6020 满量程 25000）
    float wheel_raw_limit = 16000.0f; ///< 轮电机 raw 指令限幅（M3508 满量程 16384）
    float steer_deadband = 0.01f;     ///< 舵角误差死区，rad（抑制静止抖动）
  };

  void init(void);

  void registerIkSolver(hello_world::chassis_ik_solver::ChassisIkSolver *ik) { ik_ = ik; }

  void setTarget(const ChassisMotion &tar) { target_ = tar; }
  const ChassisMotion &target(void) const { return target_; }

  /** @brief 使能 / 失能；失能时立即把所有电机指令清零。 */
  void setEnabled(bool en);
  bool enabled(void) const { return enabled_; }

  /** @brief 所有电机输入置 0（急停 / 未就绪时调用）。 */
  void setOutputZero(void);

  /** @brief 每个控制周期调用一次：限幅 → 逆解 → 8 个 PID → 下发。 */
  void update(void);

 private:
  Params params_{};
  hello_world::chassis_ik_solver::ChassisIkSolver *ik_ = nullptr;
  ChassisMotion target_{};
  bool enabled_ = false;
  bool inited_ = false;
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */
