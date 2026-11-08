/**
 *******************************************************************************
 * @file      :robot.hpp
 * @brief     :云台板顶层状态机
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *  V1.1.0      2026-11-09      Tensai-Chiruno  1. 按第二轮复查报告修 P0/P1
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
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "module_state.hpp"

namespace robot {

/**
 * @brief 云台板顶层状态机：1 kHz 入口。
 *
 * 与底盘板同一套迁移（Dead / Resurrection / Working），三道门全部接上：
 * 遥控在线、无急停、双板在线、IMU 就绪。
 * 急停判据同样用**白名单**（非 Up/Mid 一律急停）。
 */
class Robot {
 public:
  struct Params {
    uint32_t rc_timeout_ms = 100;
    uint32_t resurrection_hold_ms = 1200;  ///< >= Imu::calib_ms(1000)
    float max_yaw_rate = 3.0f;             ///< rad/s
    float max_pitch_rate = 2.0f;           ///< rad/s
    float pitch_min = -0.60f;              ///< rad，与 Gimbal::Params 同源数值
    float pitch_max = 0.90f;               ///< rad
    float remote_deadband = 0.02f;
  };

  void init(void);
  void registerRemoteControl(hello_world::remote_control::DT7 *rc) { rc_ = rc; }
  void registerGimbal(Gimbal *gimbal) { gimbal_ = gimbal; }
  void registerImu(hello_world::imu::Imu *imu) { imu_ = imu; }
  void registerComm(GimbalChassisComm *comm) { comm_ = comm; }

  void update(void);

  LifeState lifeState(void) const { return life_state_; }
  GimbalMode gimbalMode(void) const { return mode_; }
  const GimbalMotion &gimbalMotion(void) const { return motion_; }
  bool isEmergencyStop(void) const { return estop_; }
  bool isRemoteOnline(void) const { return rc_online_; }
  bool isPeerOffline(void) const { return peer_offline_; }
  bool isImuReady(void) const { return imu_ready_; }

 private:
  void updateRemote(void);
  void updateLifeState(uint32_t now);
  void enterDead(void);
  void enterResurrection(uint32_t now);
  void enterWorking(void);
  void updateMode(void) { mode_ = GimbalMode::kJoint; }
  void updateMotion(void);

  static float Deadband(float v, float db);
  static float ClampNorm(float v);

  Params params_{};
  hello_world::remote_control::DT7 *rc_ = nullptr;
  Gimbal *gimbal_ = nullptr;
  hello_world::imu::Imu *imu_ = nullptr;
  GimbalChassisComm *comm_ = nullptr;

  LifeState life_state_ = LifeState::kDead;
  GimbalMode mode_ = GimbalMode::kJoint;
  GimbalMotion motion_{};
  bool rc_online_ = false;
  bool estop_ = true;
  bool peer_offline_ = true;
  bool imu_ready_ = false;
  uint32_t resurrection_start_ms_ = 0;
  bool inited_ = false;
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_ROBOT_HPP_ */
