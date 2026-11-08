/**
 *******************************************************************************
 * @file      :chassis.cpp
 * @brief     :底盘运动模块实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *  V1.1.0      2026-11-09      Tensai-Chiruno  1. 按第二轮复查报告修 P0/P1
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

  /* 2) 逆解：速度 -> 每个舵的目标转角 + 每个轮输出端角速度（rad/s） */
  ik_->solve(t.vx, t.vy, t.w, wheel_state_);

  for (int i = 0; i < 4; ++i) {
    auto *steer = GetMotorSteer(i);
    auto *wheel = GetMotorWheel(i);
    if (steer == nullptr || wheel == nullptr) {
      continue;
    }

    /* ---- 舵向角度环 ---- */
    const float steer_raw_limit = params_.steer_raw_limit;
    float err = wheel_state_[i].angle - steer->angle();
    while (err > kPi) err -= 2.0f * kPi;
    while (err < -kPi) err += 2.0f * kPi;

    auto *pid = GetPidMotorSteer(i);
    float steer_raw = 0.0f;
    if (pid != nullptr) {
      /* 无条件调用，保证 PID 的 dt 契约（P2-14 / G-2：跳拍会把 D 项放大 N 倍） */
      steer_raw = pid->update(wheel_state_[i].angle, steer->angle(), kDt);
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
    const float wheel_vel_target = Clamp(wheel_state_[i].speed,
                                         -params_.max_wheel_vel, params_.max_wheel_vel);
    auto *pidw = GetPidMotorWheel(i);
    float wheel_raw = 0.0f;
    if (pidw != nullptr) {
      wheel_raw = pidw->update(wheel_vel_target, wheel->vel(), kDt);
    }
    wheel->setInput(Clamp(wheel_raw, -params_.wheel_raw_limit, params_.wheel_raw_limit));
  }
}

}  // namespace robot
