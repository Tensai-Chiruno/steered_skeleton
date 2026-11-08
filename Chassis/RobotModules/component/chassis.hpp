/**
 *******************************************************************************
 * @file      :chassis.hpp
 * @brief     :底盘运动模块（舵向轮逆解 + 8 个闭环）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *  V1.1.0      2026-11-09      Tensai-Chiruno  1. 按第二轮复查报告修 P0/P1
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
 * 量纲契约（P0-1 修复后的唯一约定，改这里必须同时改 ins_motor.cpp）：
 *   - 两台电机的 input_type 都是 kRaw（电压/原始帧域）；
 *   - 因此本模块的 PID 输出、限幅、送给 setInput() 的数值**全部是 raw**；
 *   - 舵电机走 GM6020 电压帧 0x1FF（满量程 25000），
 *     轮电机走 M3508 控制帧 0x200（满量程 16384）。
 *   若将来改成 kCurr（电流环），必须把下面两个限幅换成安培、帧 ID 换成 0x1FE，
 *   三处一起改，否则就是「指令很小却剧烈动作」。
 *
 * 安全约定（考核硬性）：
 *   - 未 enable 时所有电机输入恒为 0；
 *   - target 会被限幅，任何输入都不会产生越界指令。
 */
class Chassis {
 public:
  struct Params {
    float max_linear_speed = 2.0f;    ///< 平移速度限幅，m/s（与 Robot 侧一致）
    float max_angular_speed = 10.0f;  ///< 角速度限幅，rad/s（仅兜底）
    float max_wheel_vel = 120.0f;     ///< 轮电机输出端角速度限幅，rad/s
    float steer_raw_limit = 20000.0f; ///< 舵电机 raw 限幅（GM6020 帧满量程 25000）
    float wheel_raw_limit = 16000.0f; ///< 轮电机 raw 限幅（M3508 帧满量程 16384）
    float steer_deadband = 0.01f;     ///< 舵角误差死区，rad
  };

  void init(void);

  void registerIkSolver(hello_world::chassis_ik_solver::ChassisIkSolver *ik) { ik_ = ik; }

  void setTarget(const ChassisMotion &tar) { target_ = tar; }
  const ChassisMotion &target(void) const { return target_; }

  void setEnabled(bool en);
  bool enabled(void) const { return enabled_; }

  /** @brief 所有电机输入置 0（急停 / 未就绪时调用）。 */
  void setOutputZero(void);

  /** @brief 反解得到的 4 组舵角/轮速，供调试与测试读取。 */
  const hello_world::chassis_ik_solver::ChassisIkSolver::WheelState *wheelState(void) const {
    return wheel_state_;
  }

  void update(void);

 private:
  Params params_{};
  hello_world::chassis_ik_solver::ChassisIkSolver *ik_ = nullptr;
  ChassisMotion target_{};
  hello_world::chassis_ik_solver::ChassisIkSolver::WheelState wheel_state_[4]{};
  float steer_hold_[4] = {0.0f, 0.0f, 0.0f, 0.0f};  ///< 死区内保持的舵输出
  bool enabled_ = false;
  bool inited_ = false;
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */
