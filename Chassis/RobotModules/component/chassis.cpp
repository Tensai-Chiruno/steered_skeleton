/**
 *******************************************************************************
 * @file      :chassis.cpp
 * @brief     :底盘运动模块实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *  V1.1.0      2026-11-09      Tensai-Chiruno  1. 按第二轮复查报告修 P0/P1
 *  V1.2.0      2026-11-09      Tensai-Chiruno  1. 舵角就近解 + 前进方向变换接入
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis.hpp"

#include "ins_chassis_iksolver.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include <cmath>

#include "pid.hpp"

/* Private macro -------------------------------------------------------------*/
namespace {
constexpr float kDt = 1e-3f;  ///< TIM6 控制周期 1 kHz
constexpr float kPi = 3.14159265358979f;

float Clamp(float v, float lo, float hi) {
  if (!(v == v)) return 0.0f;  /* NaN 直接折 0，绝不让它进 float->int */
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
}  // namespace

namespace robot {

void Chassis::init(void) {
  if (!ik_) {
    ik_ = GetChassisIkSolver();
  }
  for (int i = 0; i < 4; ++i) {
    auto *ps = GetPidMotorSteer(i);
    auto *pw = GetPidMotorWheel(i);
    if (ps) ps->reset();
    if (pw) pw->reset();
    steer_hold_[i] = 0.0f;
  }
  target_.reset();
  inited_ = true;
  setOutputZero();
}

void Chassis::setEnabled(bool en) {
  if (enabled_ == en) return;
  enabled_ = en;
  if (!en) setOutputZero();
}

void Chassis::setOutputZero(void) {
  for (int i = 0; i < 4; ++i) {
    auto *steer = GetMotorSteer(i);
    auto *wheel = GetMotorWheel(i);
    if (steer) steer->setInput(0.0f);
    if (wheel) wheel->setInput(0.0f);
    auto *ps = GetPidMotorSteer(i);
    auto *pw = GetPidMotorWheel(i);
    if (ps) ps->reset();
    if (pw) pw->reset();
    steer_hold_[i] = 0.0f;
  }
}

void Chassis::update(void) {
  if (!inited_ || ik_ == nullptr) {
    return;
  }
  if (!enabled_) {
    setOutputZero();
    return;
  }

  /* 1) 目标限幅：任何遥控器输入都不能变成越界指令 */
  ChassisMotion t;
  t.vx = Clamp(target_.vx, -params_.max_linear_speed, params_.max_linear_speed);
  t.vy = Clamp(target_.vy, -params_.max_linear_speed, params_.max_linear_speed);
  t.w = Clamp(target_.w, -params_.max_angular_speed, params_.max_angular_speed);

  /* 2) 前进方向变换（考核第 6 项）：遥控给的 (vx, vy) 定义在**云台坐标系**，
   *    而逆解吃的是车头坐标系，所以先转到车体系再解算。
   *    δ 由 Robot 每拍下发；δ == 0 时是恒等变换，单板台架无需对端也能跑。 */
  float vx_c = t.vx;
  float vy_c = t.vy;
  if (params_.rotate_velocity_by_delta && frame_delta_ != 0.0f) {
    hello_world::chassis_ik_solver::ChassisIkSolver::gimbalToChassis(
        t.vx, t.vy, frame_delta_, &vx_c, &vy_c);
  }

  /* 3) 逆解：速度 -> 每个舵的目标转角 + 每个轮输出端角速度（rad/s） */
  ik_->solve(vx_c, vy_c, t.w, wheel_state_);

  for (int i = 0; i < 4; ++i) {
    auto *steer = GetMotorSteer(i);
    auto *wheel = GetMotorWheel(i);
    if (steer == nullptr || wheel == nullptr) {
      continue;
    }
    const float cur_angle = steer->angle();

    /* ---- 舵角就近解（IK 不播种，舵角本来就多解）----
     * 舵向轮有两个等价解：角度 +π 且轮速取反。
     * 逆解每拍独立算，若直接采用它给的角度，上电时可能让舵轮跨 149° 去一个
     * 「数学上等价、机械上很远」的位置。这里以当前实测角为基准挑近的那个。 */
    float target_angle = wheel_state_[i].angle;
    float target_speed = wheel_state_[i].speed;
    const float err_fwd = std::fmod(target_angle - cur_angle + 3.14159265f, 6.28318531f) - 3.14159265f;
    const float err_rev = std::fmod(target_angle + 3.14159265f - cur_angle + 3.14159265f,
                                    6.28318531f) - 3.14159265f;
    if (std::fabs(err_rev) < std::fabs(err_fwd)) {
      target_angle += 3.14159265f;
      target_speed = -target_speed;
    }
    target_angle = std::fmod(target_angle + 3.14159265f, 6.28318531f) - 3.14159265f;
    wheel_state_[i].angle = target_angle;
    wheel_state_[i].speed = target_speed;

    /* ---- 舵向角度环 ---- */
    const float steer_raw_limit = params_.steer_raw_limit;
    const float err = std::fmod(target_angle - cur_angle + 3.14159265f, 6.28318531f) - 3.14159265f;

    auto *pid = GetPidMotorSteer(i);
    float steer_raw = 0.0f;
    if (pid != nullptr) {
      /* 无条件调用，保证 PID 的 dt 契约（P2-14 / G-2：跳拍会把 D 项放大 N 倍） */
      steer_raw = pid->update(target_angle, cur_angle, kDt);
    }
    if (err < params_.steer_deadband && err > -params_.steer_deadband) {
      /* 死区内不切断输出，而是保持上一拍的值：
       * 舵轮带 caster 力，切 0 会让它漂回零位、再被拉回，形成极限环抖动。 */
      steer_raw = steer_hold_[i];
    } else {
      steer_hold_[i] = steer_raw;
    }
    steer->setInput(Clamp(steer_raw, -steer_raw_limit, steer_raw_limit));

    /* ---- 轮速环 ----
     * 逆解已经把轮心线速度按 params_.wheel_radius 折算成「输出端 rad/s」，
     * 与 wheel->vel() 同域。这里**绝不能**再除一次轮半径
     * （旧代码这样写导致整定当天实际车速为指令的约 15 倍）。 */
    const float wheel_vel_target =
        Clamp(target_speed, -params_.max_wheel_vel, params_.max_wheel_vel);
    auto *pidw = GetPidMotorWheel(i);
    float wheel_raw = 0.0f;
    if (pidw != nullptr) {
      wheel_raw = pidw->update(wheel_vel_target, wheel->vel(), kDt);
    }
    wheel->setInput(Clamp(wheel_raw, -params_.wheel_raw_limit, params_.wheel_raw_limit));
  }
}

}  // namespace robot
