/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     :云台板顶层状态机实现
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

#include "HW_can.hpp"
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
  mode_ = GimbalMode::kJoint;
  motion_.reset();
  rc_online_ = false;
  estop_ = true;
  peer_offline_ = true;
  imu_ready_ = false;
  inited_ = true;
  if (gimbal_ != nullptr) {
    gimbal_->setTarget(motion_);
    gimbal_->setEnabled(false);
  }
}

void Robot::updateRemote(void) {
  rc_online_ = IsRemoteOnline();
  if (!rc_online_ || rc_ == nullptr) {
    estop_ = true;
    return;
  }
  const auto sw = rc_->rc_l_switch();
  estop_ = !(sw == hello_world::remote_control::kSwitchStateUp ||
             sw == hello_world::remote_control::kSwitchStateMid);
  peer_offline_ = (comm_ == nullptr) ? true : comm_->isOffline();
  if (peer_offline_) estop_ = true;

  /* P1-8①：CAN 发送持续失败 = 对端在、但发不出去（bus-off / 邮箱被重传占满）。
   * 只在对端在线时才判：单板台架没有 ACK，发送失败是常态，不能因此刹车。 */
  {
    const uint32_t fail_now = CAN_GetTxFailCount();
    if (fail_now != last_tx_fail_) {
      last_tx_fail_ = fail_now;
      ++tx_fail_streak_;
    } else if (tx_fail_streak_ > 0u) {
      --tx_fail_streak_;
    }
    if (!peer_offline_ && tx_fail_streak_ >= 20u) {
      estop_ = true;
    }
  }

  imu_ready_ = (imu_ != nullptr) && imu_->isReady() && imu_->isOk();
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
  if (gimbal_ != nullptr) {
    motion_ = gimbal_->current();   /* 以实测角为目标，使能瞬间不跳变 */
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

void Robot::updateMotion(void) {
  if (life_state_ != LifeState::kWorking || rc_ == nullptr) return;
  const float dyaw = Deadband(ClampNorm(rc_->rc_rh()), params_.remote_deadband) *
                     params_.max_yaw_rate * kDt;
  const float dpitch = -Deadband(ClampNorm(rc_->rc_lv()), params_.remote_deadband) *
                       params_.max_pitch_rate * kDt;
  motion_.yaw += dyaw;
  float p = motion_.pitch + dpitch;
  if (p < params_.pitch_min) p = params_.pitch_min;
  if (p > params_.pitch_max) p = params_.pitch_max;
  motion_.pitch = p;
  if (motion_.yaw > 628.318531f) motion_.yaw -= 628.318531f;
  if (motion_.yaw < -628.318531f) motion_.yaw += 628.318531f;
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

  /* G-1：位置保持接口只在 Working 期间开放（motor 层会断言这个标志）。 */
  robot::g_mit_input_allowed = (life_state_ == LifeState::kWorking);

  if (gimbal_ != nullptr) {
    gimbal_->setTarget(motion_);
    gimbal_->setEnabled(life_state_ == LifeState::kWorking);
    gimbal_->update();
  }

  /* 双板帧语义字段的唯一写者（P1-3 / G-3）。
   * 角度必须先回绕到 [-pi, pi) 再交出去：codec 的 AngleToRaw 是钳位不是回绕，
   * 把 ±628 rad 的连续 yaw 直接写进去会被夹成「永远贴在顶的假角度」。 */
  if (comm_ != nullptr) {
    auto &f = comm_->local();
    f.online = (life_state_ != LifeState::kDead) ? 1u : 0u;
    f.ready = imu_ready_ ? 1u : 0u;
    f.work_mode = ToByte(mode_);
    f.ctrl_mode = ToByte(life_state_);
    if (gimbal_ != nullptr) {
      float y = gimbal_->current().yaw;
      y = fmodf(y + 3.14159265f, 6.28318531f);
      if (y < 0.0f) y += 6.28318531f;
      f.yaw = y - 3.14159265f;
      f.pitch = gimbal_->current().pitch;
    }
  }
}

}  // namespace robot
