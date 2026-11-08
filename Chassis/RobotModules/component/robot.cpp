/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     :底盘板顶层状态机实现
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

/* Private macro -------------------------------------------------------------*/
namespace {
constexpr float kDt = 1e-3f;
}  // namespace

/* Exported function definitions ---------------------------------------------*/
void Robot::init(void) {
  life_state_ = LifeState::kDead;
  mode_ = ChassisMode::kDepart;
  ctrl_source_ = CtrlSource::kNone;
  motion_.reset();
  yaw_error_ = 0.0f;
  rc_online_ = false;
  estop_ = true;
  inited_ = true;
  if (chassis_ != nullptr) {
    chassis_->setTarget(motion_);
    chassis_->setEnabled(false);
  }
}

float Robot::Deadband(float v, float db) {
  return (v > -db && v < db) ? 0.0f : v;
}

void Robot::updateRemote(void) {
  rc_online_ = IsRemoteOnline();
  if (!rc_online_) {
    ctrl_source_ = CtrlSource::kNone;
    estop_ = true;  // 遥控器离线，一律视为急停
    return;
  }
  ctrl_source_ = CtrlSource::kRemote;
  if (rc_ != nullptr) {
    /* 左拨杆最下 = 急停（考核硬性项） */
    estop_ = (rc_->rc_l_switch() == hello_world::remote_control::kSwitchStateDown);
  } else {
    estop_ = true;
  }
}

void Robot::enterDead(void) {
  life_state_ = LifeState::kDead;
  motion_.reset();
  if (chassis_ != nullptr) {
    chassis_->setTarget(motion_);
    chassis_->setEnabled(false);
    chassis_->setOutputZero();
  }
}

void Robot::enterResurrection(uint32_t now) {
  life_state_ = LifeState::kResurrection;
  resurrection_start_ms_ = now;
  motion_.reset();
  if (chassis_ != nullptr) {
    chassis_->setTarget(motion_);
    chassis_->setEnabled(true);
  }
}

void Robot::enterWorking(void) {
  life_state_ = LifeState::kWorking;
}

void Robot::updateLifeState(uint32_t now) {
  switch (life_state_) {
    case LifeState::kDead:
      enterDead();
      if (rc_online_ && !estop_) {
        enterResurrection(now);
      }
      break;

    case LifeState::kResurrection:
      if (!rc_online_ || estop_) {
        enterDead();
        break;
      }
      if (now - resurrection_start_ms_ >= params_.resurrection_hold_ms) {
        enterWorking();
      }
      break;

    case LifeState::kWorking:
      if (!rc_online_ || estop_) {
        enterDead();
      }
      break;

    default:
      enterDead();
      break;
  }
}

void Robot::updateMode(void) {
  if (rc_ == nullptr) {
    mode_ = ChassisMode::kDepart;
    return;
  }
  switch (rc_->rc_r_switch()) {
    case hello_world::remote_control::kSwitchStateUp:
      mode_ = ChassisMode::kDepart;
      break;
    case hello_world::remote_control::kSwitchStateMid:
      mode_ = ChassisMode::kFollow;
      break;
    case hello_world::remote_control::kSwitchStateDown:
      mode_ = ChassisMode::kGyro;
      break;
    default:
      mode_ = ChassisMode::kDepart;
      break;
  }
}

void Robot::updateMotion(void) {
  motion_.reset();
  if (life_state_ != LifeState::kWorking || rc_ == nullptr) {
    return;
  }
  if (ctrl_source_ != CtrlSource::kRemote) {
    return;
  }

  /* 平移：左摇杆（上=前，右=右）。注意 lv 上推为正，故 vx 直接取 lv。 */
  motion_.vx = Deadband(rc_->rc_lv(), params_.remote_deadband) * params_.max_vx;
  motion_.vy = Deadband(rc_->rc_lh(), params_.remote_deadband) * params_.max_vy;

  /* 自转按模式区分 */
  switch (mode_) {
    case ChassisMode::kDepart:
      motion_.w = Deadband(rc_->rc_rh(), params_.remote_deadband) * params_.max_w_manual;
      break;
    case ChassisMode::kFollow:
      /* δ 由双板帧（云台 yaw 相对车头的角度）喂入；未接入前 δ 恒 0，w 恒 0，
       * 属于安全缺省值，不会让车自己转。 */
      motion_.w = params_.follow_kp * yaw_error_;
      break;
    case ChassisMode::kGyro:
      motion_.w = params_.gyro_w;
      break;
    default:
      motion_.w = 0.0f;
      break;
  }
}

void Robot::update(void) {
  if (!inited_) {
    return;
  }
  const uint32_t now = hello_world::tick::GetTickMs();

  updateRemote();
  updateLifeState(now);
  updateMode();
  updateMotion();

  if (chassis_ != nullptr) {
    chassis_->setTarget(motion_);
    chassis_->setEnabled(life_state_ == LifeState::kWorking);
    chassis_->update();
  }
}

}  // namespace robot
