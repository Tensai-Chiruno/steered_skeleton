/**
 *******************************************************************************
 * @file      :robot.hpp
 * @brief     :云台板顶层状态机
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifndef GIMBAL_ROBOT_MODULES_ROBOT_HPP_
#define GIMBAL_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"
#include "gimbal.hpp"
#include "module_state.hpp"

namespace robot {

/**
 * @brief 云台板顶层状态机：1 kHz 入口。
 *
 * 生命状态迁移与底盘板一致（Dead / Resurrection / Working），
 * 但输出对象换成云台两轴力矩。右摇杆控制 yaw，左摇杆上下控制 pitch。
 */
class Robot {
 public:
  struct Params {
    uint32_t rc_timeout_ms = 100;
    uint32_t resurrection_hold_ms = 500;
    float max_yaw_rate = 3.0f;    ///< rad/s
    float max_pitch_rate = 2.0f;  ///< rad/s
    float pitch_min = -0.60f;     ///< rad
    float pitch_max = 0.90f;      ///< rad
    float remote_deadband = 0.02f;
  };

  void init(void);
  void registerRemoteControl(hello_world::remote_control::DT7 *rc) { rc_ = rc; }
  void registerGimbal(Gimbal *gimbal) { gimbal_ = gimbal; }
  void update(void);

  LifeState lifeState(void) const { return life_state_; }
  GimbalMode gimbalMode(void) const { return mode_; }
  const GimbalMotion &gimbalMotion(void) const { return motion_; }
  bool isEmergencyStop(void) const { return estop_; }

 private:
  void updateRemote(void);
  void updateLifeState(uint32_t now);
  void enterDead(void);
  void enterResurrection(uint32_t now);
  void enterWorking(void);
  void updateMotion(void);
  static float Deadband(float v, float db);

  Params params_{};
  hello_world::remote_control::DT7 *rc_ = nullptr;
  Gimbal *gimbal_ = nullptr;

  LifeState life_state_ = LifeState::kDead;
  GimbalMode mode_ = GimbalMode::kJoint;
  GimbalMotion motion_{};
  bool rc_online_ = false;
  bool estop_ = true;
  uint32_t resurrection_start_ms_ = 0;
  bool inited_ = false;
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_ROBOT_HPP_ */
