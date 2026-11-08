/**
 *******************************************************************************
 * @file      :robot.hpp
 * @brief     :底盘板顶层状态机
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifndef CHASSIS_ROBOT_MODULES_ROBOT_HPP_
#define CHASSIS_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"
#include "chassis.hpp"
#include "module_state.hpp"

namespace robot {

/**
 * @brief 底盘板顶层状态机：1 kHz 入口，负责「读数 → 状态 → 模式 → 指令」。
 *
 * 生命状态迁移：
 *   Dead --(遥控在线 且 左拨杆不在最下)--> Resurrection --(保持 500 ms)--> Working
 *   Working --(遥控离线 或 左拨杆最下)--> Dead
 * 急停：左拨杆最下。任何非 Working 状态都强制底盘输出为 0。
 */
class Robot {
 public:
  struct Params {
    uint32_t rc_timeout_ms = 100;       ///< 遥控器离线阈值
    uint32_t resurrection_hold_ms = 500;///< 恢复自检保持时间
    float max_vx = 2.0f;                ///< m/s
    float max_vy = 2.0f;                ///< m/s
    float max_w_manual = 6.0f;          ///< rad/s，遥控器手动自转
    float gyro_w = 8.0f;                ///< rad/s，小陀螺常值
    float follow_kp = 3.0f;             ///< 跟随模式 δ -> w 的比例系数
    float remote_deadband = 0.02f;      ///< 摇杆死区（归一化）
  };

  void init(void);
  void registerRemoteControl(hello_world::remote_control::DT7 *rc) { rc_ = rc; }
  void registerChassis(Chassis *chassis) { chassis_ = chassis; }

  /** @brief 跟随模式的角度误差 δ（云台朝向 - 车头朝向），由双板帧喂入。 */
  void setYawError(float delta) { yaw_error_ = delta; }

  void update(void);

  LifeState lifeState(void) const { return life_state_; }
  ChassisMode chassisMode(void) const { return mode_; }
  CtrlSource ctrlSource(void) const { return ctrl_source_; }
  const ChassisMotion &chassisMotion(void) const { return motion_; }
  bool isEmergencyStop(void) const { return estop_; }
  bool isRemoteOnline(void) const { return rc_online_; }

 private:
  void updateRemote(void);
  void updateLifeState(uint32_t now);
  void enterDead(void);
  void enterResurrection(uint32_t now);
  void enterWorking(void);
  void updateMode(void);
  void updateMotion(void);

  static float Deadband(float v, float db);

  Params params_{};
  hello_world::remote_control::DT7 *rc_ = nullptr;
  Chassis *chassis_ = nullptr;

  LifeState life_state_ = LifeState::kDead;
  ChassisMode mode_ = ChassisMode::kDepart;
  CtrlSource ctrl_source_ = CtrlSource::kNone;
  ChassisMotion motion_{};
  float yaw_error_ = 0.0f;

  bool rc_online_ = false;
  bool estop_ = false;
  uint32_t resurrection_start_ms_ = 0;
  bool inited_ = false;
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_ROBOT_HPP_ */
