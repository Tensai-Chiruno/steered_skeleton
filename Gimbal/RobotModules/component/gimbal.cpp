/**
 *******************************************************************************
 * @file      :gimbal.cpp
 * @brief     :云台两轴位置环实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "gimbal.hpp"

#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "pid.hpp"

namespace {
constexpr float kDt = 1e-3f;
}  // namespace

namespace robot {

float Gimbal::Clamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float Gimbal::WrapPi(float a) {
  while (a > 3.14159265f) a -= 6.28318531f;
  while (a < -3.14159265f) a += 6.28318531f;
  return a;
}

void Gimbal::init(void) {
  {
    auto *py = GetPidMotorYaw();
    auto *pp = GetPidMotorPitch();
    if (py) py->reset();
    if (pp) pp->reset();
  }
  target_.reset();
  current_.reset();
  inited_ = true;
  setOutputZero();
}

void Gimbal::setEnabled(bool en) {
  enabled_ = en;
  if (!en) {
    setOutputZero();
  }
}

void Gimbal::setOutputZero(void) {
  auto *yaw = GetMotorYaw();
  auto *pitch = GetMotorPitch();
  if (yaw) yaw->setInput(0.0f);
  if (pitch) pitch->setInput(0.0f);
  {
    auto *py = GetPidMotorYaw();
    auto *pp = GetPidMotorPitch();
    if (py) py->reset();
    if (pp) pp->reset();
  }
}

void Gimbal::update(void) {
  if (!inited_) {
    return;
  }
  auto *yaw = GetMotorYaw();
  auto *pitch = GetMotorPitch();
  if (yaw == nullptr || pitch == nullptr) {
    return;
  }
  current_.yaw = yaw->angle();
  current_.pitch = pitch->angle();

  if (!enabled_) {
    setOutputZero();
    return;
  }

  /* ---- yaw：360° 连续，误差走最短路径 ---- */
  auto *pid_yaw = GetPidMotorYaw();
  float yaw_raw = 0.0f;
  float yaw_err = WrapPi(target_.yaw - current_.yaw);
  if (pid_yaw != nullptr) {
    /* 无条件调用：PID 的 dt 契约要求每拍都调用（G-2）。
     * 曾经「死区内跳过调用、恢复时仍传固定 dt」会把 D 项放大 N 倍。 */
    yaw_raw = pid_yaw->update(target_.yaw, current_.yaw, kDt);
  }
  if (yaw_err < params_.yaw_deadband && yaw_err > -params_.yaw_deadband) {
    yaw_raw = 0.0f;           /* 死区只清输出，不动 PID 的调用节拍 */
    if (pid_yaw) pid_yaw->reset();
  }
  yaw->setInput(Clamp(yaw_raw, -params_.max_torq, params_.max_torq));

  /* ---- pitch：有限角度 + 软件限位 ---- */
  float pitch_tar = Clamp(target_.pitch, params_.pitch_min, params_.pitch_max);
  auto *pid_pitch = GetPidMotorPitch();
  float pitch_raw = 0.0f;
  float pitch_err = pitch_tar - current_.pitch;
  if (pid_pitch != nullptr) {
    pitch_raw = pid_pitch->update(pitch_tar, current_.pitch, kDt);
  }
  if (pitch_err < params_.pitch_deadband && pitch_err > -params_.pitch_deadband) {
    pitch_raw = 0.0f;
    if (pid_pitch) pid_pitch->reset();
  }
  pitch->setInput(Clamp(pitch_raw, -params_.max_torq, params_.max_torq));
}

}  // namespace robot
