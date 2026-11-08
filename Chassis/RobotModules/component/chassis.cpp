/**
 *******************************************************************************
 * @file      :chassis.cpp
 * @brief     :底盘运动模块实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
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

float Clamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
}  // namespace

namespace robot {

/* Exported function definitions ---------------------------------------------*/
void Chassis::init(void) {
  if (!ik_) {
    ik_ = GetChassisIkSolver();
  }
  for (int i = 0; i < 4; ++i) {
    auto *ps = GetPidMotorSteer(i);
    auto *pw = GetPidMotorWheel(i);
    if (ps) ps->reset();
    if (pw) pw->reset();
  }
  target_.reset();
  inited_ = true;
  setOutputZero();
}

void Chassis::setEnabled(bool en) {
  enabled_ = en;
  if (!en) {
    setOutputZero();
  }
}

void Chassis::setOutputZero(void) {
  for (int i = 0; i < 4; ++i) {
    auto *steer = GetMotorSteer(i);
    auto *wheel = GetMotorWheel(i);
    if (steer) steer->setInput(0.0f);
    if (wheel) wheel->setInput(0.0f);
  }
  for (int i = 0; i < 4; ++i) {
    auto *ps = GetPidMotorSteer(i);
    auto *pw = GetPidMotorWheel(i);
    if (ps) ps->reset();
    if (pw) pw->reset();
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

  /* 2) 逆解：速度 -> 每个舵的目标转角 + 每个轮的轮心线速度 */
  hello_world::chassis_ik_solver::ChassisIkSolver::WheelState ws[4];
  ik_->solve(t.vx, t.vy, t.w, ws);

  /* 3) 舵向角度环 + 轮速环，输出 raw 直接喂电机 */
  for (int i = 0; i < 4; ++i) {
    auto *steer = GetMotorSteer(i);
    auto *wheel = GetMotorWheel(i);
    if (steer == nullptr || wheel == nullptr) {
      continue;
    }

    /* 舵向：目标角度已在 inverse kinematics 里做过「就近转」与限速，
     * 反馈角度已由 ins_motor.cpp 的 angle_offset 折算到零位为 0。 */
    float steer_raw = 0.0f;
    float err = ws[i].angle - steer->angle();
    if (err > 3.14159265f) err -= 6.28318531f;
    if (err < -3.14159265f) err += 6.28318531f;
    if (err > params_.steer_deadband || err < -params_.steer_deadband) {
      auto *pid = GetPidMotorSteer(i);
      if (pid != nullptr) {
        steer_raw = pid->update(ws[i].angle, steer->angle(), kDt);
      }
    }
    steer_raw = Clamp(steer_raw, -params_.steer_raw_limit, params_.steer_raw_limit);
    steer->setInput(steer_raw);

    /* 轮速：逆解给的是轮心线速度 m/s，折算成电机输出端角速度 rad/s */
    float wheel_vel_target = ws[i].speed / 0.066f;
    wheel_vel_target =
        Clamp(wheel_vel_target, -params_.max_wheel_vel, params_.max_wheel_vel);
    float wheel_raw = 0.0f;
    auto *pidw = GetPidMotorWheel(i);
    if (pidw != nullptr) {
      wheel_raw = pidw->update(wheel_vel_target, wheel->vel(), kDt);
    }
    wheel_raw = Clamp(wheel_raw, -params_.wheel_raw_limit, params_.wheel_raw_limit);
    wheel->setInput(wheel_raw);
  }
}

}  // namespace robot
