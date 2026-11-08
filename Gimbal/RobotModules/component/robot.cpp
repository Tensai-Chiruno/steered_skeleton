/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     :云台板顶层状态机实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "robot.hpp"

#include "ins_comm_state.hpp"
#include "tick.hpp"

namespace robot {

namespace {
constexpr float kDt = 1e-3f;
float Clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
}  // namespace

void Robot::init(void) {
  life_state_ = LifeState::kDead;
  mode_ = GimbalMode::kJoint;
  motion_.reset();
  rc_online_ = false;
  estop_ = true;
  inited_ = true;
  if (gimbal_ != nullptr) {
    gimbal_->setTarget(motion_);
    gimbal_->setEnabled(false);
  }
}

float Robot::Deadband(float v, float db) {
  return (v > -db && v < db) ? 0.0f : v;
}

void Robot::updateRemote(void) {
  rc_online_ = IsRemoteOnline();
  if (!rc_online_ || rc_ == nullptr) {
    estop_ = true;
    return;
  }
  estop_ = (rc_->rc_l_switch() == hello_world::remote_control::kSwitchStateDown);
}

void Robot::enterDead(void) {
  life_state_ = LifeState::kDead;
  motion_.reset();
  if (gimbal_ != nullptr) {
    gimbal_->setTarget(motion_);
    gimbal_->setEnabled(false);
    gimbal_->setOutputZero();
  }
}

void Robot::enterResurrection(uint32_t now) {
  life_state_ = LifeState::kResurrection;
  resurrection_start_ms_ = now;
  /* 以当前实测角为目标，避免使能瞬间跳变 */
  if (gimbal_ != nullptr) {
    motion_ = gimbal_->current();
    gimbal_->setTarget(motion_);
    gimbal_->setEnabled(true);
  }
}

void Robot::enterWorking(void) { life_state_ = LifeState::kWorking; }

void Robot::updateLifeState(uint32_t now) {
  switch (life_state_) {
    case LifeState::kDead:
      enterDead();
      if (rc_online_ && !estop_) enterResurrection(now);
      break;
    case LifeState::kResurrection:
      if (!rc_online_ || estop_) { enterDead(); break; }
      if (now - resurrection_start_ms_ >= params_.resurrection_hold_ms) enterWorking();
      break;
    case LifeState::kWorking:
      if (!rc_online_ || estop_) enterDead();
      break;
    default:
      enterDead();
      break;
  }
}

void Robot::updateMotion(void) {
  if (life_state_ != LifeState::kWorking || rc_ == nullptr) {
    return;
  }
  /* 增量式摇杆控制：右摇杆左右 -> yaw，左摇杆上下 -> pitch */
  float dyaw = Deadband(rc_->rc_rh(), params_.remote_deadband) * params_.max_yaw_rate * kDt;
  float dpitch = Deadband(rc_->rc_lv(), params_.remote_deadband) * params_.max_pitch_rate * kDt;
  motion_.yaw += dyaw;
  motion_.pitch = Clampf(motion_.pitch + dpitch, params_.pitch_min, params_.pitch_max);
  /* yaw 保持 360° 连续，只在内部限幅到 ±100 圈避免浮点漂移 */
  if (motion_.yaw > 628.318531f) motion_.yaw -= 628.318531f;
  if (motion_.yaw < -628.318531f) motion_.yaw += 628.318531f;
}

void Robot::update(void) {
  if (!inited_) return;
  const uint32_t now = hello_world::tick::GetTickMs();

  updateRemote();
  updateLifeState(now);
  mode_ = GimbalMode::kJoint;
  updateMotion();

  if (gimbal_ != nullptr) {
    gimbal_->setTarget(motion_);
    gimbal_->setEnabled(life_state_ == LifeState::kWorking);
    gimbal_->update();
  }
}

}  // namespace robot
