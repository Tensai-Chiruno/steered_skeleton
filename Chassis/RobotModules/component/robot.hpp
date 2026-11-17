/**
 *******************************************************************************
 * @file      :robot.hpp
 * @brief     :底盘板顶层状态机
 * @category : 功能 | 考核第 9 项（状态机）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *  V1.1.0      2026-11-09      Tensai-Chiruno  1. 按第二轮复查报告修 P0/P1
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
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "module_state.hpp"

namespace robot {

/**
 * @brief 底盘板顶层状态机：1 kHz 入口。
 *
 * 生命状态迁移（V1.1.0 起三道门全部接上）：
 *   Dead --(遥控在线 && 无急停 && 双板在线)--> Resurrection
 *   Resurrection --(保持 >= resurrection_hold_ms && IMU 就绪)--> Working
 *   Working --(遥控离线 || 急停 || 双板离线 || IMU 未就绪)--> Dead
 *
 * 急停判据用**白名单**：左拨杆只有明确落在 Up / Mid 才不算急停，
 * 其余任何编码（含未定义的 0）一律视为急停——失效安全方向必须朝这里。
 */
class Robot {
 public:
  struct Params {
    /* 遥控离线阈值不在这里：唯一来源是 ins_comm_state.cpp 的 kRemoteTimeoutMs */
    uint32_t resurrection_hold_ms = 1200;///< >= Imu::calib_ms(1000)，与之重复但不矛盾
    float max_vx = 2.0f;                 ///< m/s
    float max_vy = 2.0f;                 ///< m/s
    float max_w_manual = 6.0f;           ///< rad/s，遥控器手动自转
    float gyro_w = 2.0f;                 ///< 小陀螺常值 rad/s（2026-11-18 底盘验证后放开，原 0 是台架保护）
    float follow_kp = 1.0f;              ///< 跟随 δ->w 增益：δ 越大自转越快，w 限幅 ±max_w_manual
    float remote_deadband = 0.02f;       ///< 摇杆死区（归一化）
  };

  void init(void);
  void registerRemoteControl(hello_world::remote_control::DT7 *rc) { rc_ = rc; }
  void registerChassis(Chassis *chassis) { chassis_ = chassis; }
  void registerImu(hello_world::imu::Imu *imu) { imu_ = imu; }
  void registerComm(GimbalChassisComm *comm) { comm_ = comm; }

  /** @brief 跟随模式的角度误差 δ（云台朝向 - 车头朝向），由双板帧喂入。 */
  void setYawError(float delta) { yaw_error_ = delta; }

  void update(void);

  LifeState lifeState(void) const { return life_state_; }
  ChassisMode chassisMode(void) const { return mode_; }
  CtrlSource ctrlSource(void) const { return ctrl_source_; }
  const ChassisMotion &chassisMotion(void) const { return motion_; }
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
  void updateMode(void);
  void updateMotion(void);

  /* 【现场修复 2026-11-17】遥控数据来源选择：本地 DBUS 或云台板转发（0x212）。
   * 接收机只接在云台板，底盘收不到；硬件不能改线，所以由云台转发。
   * 下面这几个访问器把"从哪来"的差异收在一处，控制逻辑不用到处写 if。 */
  hello_world::remote_control::SwitchState LeftSwitch(void) const;
  hello_world::remote_control::SwitchState RightSwitch(void) const;
  /* 摇杆通道（分配见 robot.cpp updateMotion 里的说明）：
   *   lv = 云台 pitch（底盘不用）, lh = 云台 yaw（底盘不用）
   *   rv = 底盘 vx, rh = 底盘 vy, wheel = 分离模式自转
   * 这里保留 lv/lh 的访问器只是为了调试读数时不绕路。 */
  float StickLv(void) const;
  float StickLh(void) const;
  float StickRv(void) const;
  float StickRh(void) const;
  float StickWheel(void) const;
  /** 本拍遥控是否取自云台转发帧（新鲜 + 校验过 + online=1）。
   *  承载优先级：转发 > 本地 DBUS。见 robot.cpp 里的说明。 */
  bool UseRelay(void) const;
  /** 【调试·临时】模拟遥控是否生效（g_dbg_rc_sim != 0）。 */
  bool UseSim(void) const;
  bool rc_from_relay_ = false;   ///< 本拍遥控是否来自转发（诊断用，不参与判据）

  static float Deadband(float v, float db);
  static float ClampNorm(float v);

  Params params_{};
  hello_world::remote_control::DT7 *rc_ = nullptr;
  Chassis *chassis_ = nullptr;
  hello_world::imu::Imu *imu_ = nullptr;
  GimbalChassisComm *comm_ = nullptr;

  LifeState life_state_ = LifeState::kDead;
  ChassisMode mode_ = ChassisMode::kDepart;
  CtrlSource ctrl_source_ = CtrlSource::kNone;
  ChassisMotion motion_{};
  float yaw_error_ = 0.0f;

  bool rc_online_ = false;
  bool estop_ = true;
  bool peer_offline_ = true;
  bool imu_ready_ = false;
  uint32_t last_tx_fail_ = 0;
  uint32_t tx_fail_streak_ = 0;
  uint32_t resurrection_start_ms_ = 0;
  /* 【现场修复 2026-11-18】"不安全"信号的确认去抖。
   * 现场现象：拨模式拨杆（右）时，状态机会 2(Working)->1(自检)->2 跳一次，
   * 也就是中间进过一帧 Dead —— 只要某一帧 estop / rc_online / imu_ready 抖一下，
   * 就会莫名其妙停约 1 秒。真实故障会持续存在，所以用"持续 unsafe_confirm_ms 才认"，
   * 既不会漏掉真急停（100 ms 远快于 200 ms 看门狗），也不会被一帧噪声打断。 */
  static constexpr uint32_t kUnsafeConfirmMs = 100u;
  bool unsafe_active_ = false;
  uint32_t unsafe_since_ms_ = 0;
  bool unsafe_confirmed_ = false;
  bool inited_ = false;
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_ROBOT_HPP_ */
