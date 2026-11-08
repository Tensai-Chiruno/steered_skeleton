/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     :底盘板顶层状态机实现
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
#include "robot.hpp"

#include "ins_comm_state.hpp"
#include "tick.hpp"

namespace {
constexpr float kDt = 1e-3f;
}  // namespace

namespace robot {

float Robot::Deadband(float v, float db) {
  return (v > -db && v < db) ? 0.0f : v;
}

/* 摇杆原始值域实测可能是 ±1.55（P1-11），在这里统一夹到 ±1，
 * 保证「满舵 = 设计车速」这件事与 DT7 的实际量程无关。 */
float Robot::ClampNorm(float v) {
  if (!(v == v)) return 0.0f;
  if (v > 1.0f) return 1.0f;
  if (v < -1.0f) return -1.0f;
  return v;
}


void Robot::init(void) {
  life_state_ = LifeState::kDead;
  mode_ = ChassisMode::kDepart;
  ctrl_source_ = CtrlSource::kNone;
  motion_.reset();
  yaw_error_ = 0.0f;
  rc_online_ = false;
  estop_ = true;
  peer_offline_ = true;
  imu_ready_ = false;
  inited_ = true;
  if (chassis_ != nullptr) {
    chassis_->setTarget(motion_);
    chassis_->setEnabled(false);
  }
}

void Robot::updateRemote(void) {
  rc_online_ = IsRemoteOnline();
  if (!rc_online_ || rc_ == nullptr) {
    ctrl_source_ = CtrlSource::kNone;
    estop_ = true;
    return;
  }
  ctrl_source_ = CtrlSource::kRemote;

  /* 急停判据用白名单（P2-22 / N-4）：
   * 只有明确落在 Up / Mid 才不算急停；未知编码（含 0）一律按急停处理。
   * 若左拨杆实际是两档开关，只产生 Up/Down，这条依然正确。 */
  const auto sw = rc_->rc_l_switch();
  estop_ = !(sw == hello_world::remote_control::kSwitchStateUp ||
             sw == hello_world::remote_control::kSwitchStateMid);

  /* 双板链路（P1-4）：对端离线 = 数据不可信 = 急停。 */
  peer_offline_ = (comm_ == nullptr) ? true : comm_->isOffline();
  if (peer_offline_) {
    estop_ = true;
  }

  /* IMU 就绪（P1-2）：未注册、标定未完成、SPI 读失败都不许进 Working。 */
  imu_ready_ = (imu_ != nullptr) && imu_->isReady() && imu_->isOk();
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
      if (rc_online_ && !estop_) enterResurrection(now);
      break;

    case LifeState::kResurrection:
      if (!rc_online_ || estop_) { enterDead(); break; }
      /* 两道条件同时满足才准进 Working：计时到了 且 IMU 真的就绪 */
      if ((now - resurrection_start_ms_) >= params_.resurrection_hold_ms && imu_ready_) {
        enterWorking();
      }
      break;

    case LifeState::kWorking:
      if (!rc_online_ || estop_ || !imu_ready_) enterDead();
      break;

    default:
      enterDead();
      break;
  }
}

void Robot::updateMode(void) {
  if (rc_ == nullptr) { mode_ = ChassisMode::kDepart; return; }
  switch (rc_->rc_r_switch()) {
    case hello_world::remote_control::kSwitchStateUp:   mode_ = ChassisMode::kDepart; break;
    case hello_world::remote_control::kSwitchStateMid:  mode_ = ChassisMode::kFollow; break;
    case hello_world::remote_control::kSwitchStateDown: mode_ = ChassisMode::kGyro; break;
    default: mode_ = ChassisMode::kDepart; break;
  }
}

void Robot::updateMotion(void) {
  motion_.reset();
  if (life_state_ != LifeState::kWorking || rc_ == nullptr) return;
  if (ctrl_source_ != CtrlSource::kRemote) return;

  motion_.vx = Deadband(ClampNorm(rc_->rc_lv()), params_.remote_deadband) * params_.max_vx;
  motion_.vy = Deadband(ClampNorm(rc_->rc_lh()), params_.remote_deadband) * params_.max_vy;

  switch (mode_) {
    case ChassisMode::kDepart:
      motion_.w = Deadband(ClampNorm(rc_->rc_rh()), params_.remote_deadband) * params_.max_w_manual;
      break;
    case ChassisMode::kFollow:
      /* δ 由双板帧喂入；未接入前 δ 恒 0 → w 恒 0（安全缺省②）。 */
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
  if (!inited_) return;
  /* 姿态链必须每拍真的更新一次：只查 isReady() 而不调 update() 的话，
   * 零偏标定永远不推进，状态机永远进不了 Working（这也是 nm 查出来的死代码）。 */
  if (imu_ != nullptr) {
    imu_->update();
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

  /* 双板帧语义字段的唯一写者（P1-3）。底盘板的 local 帧只承载状态，
   * 角度由云台板填，所以这里不写 yaw/pitch。 */
  if (comm_ != nullptr) {
    auto &f = comm_->local();
    f.online = (life_state_ != LifeState::kDead) ? 1u : 0u;
    f.ready = imu_ready_ ? 1u : 0u;
    f.work_mode = ToByte(mode_);
    f.ctrl_mode = ToByte(life_state_);
  }
}

}  // namespace robot
