/**
 *******************************************************************************
 * @file      :module_state.hpp
 * @brief     :两板共享的状态枚举与运动结构（考核第 5 项）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifndef ROBOT_COMPONENTS_MODULE_STATE_HPP_
#define ROBOT_COMPONENTS_MODULE_STATE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

/* ============================================================================
 * 本文件被底盘板与云台板同时编译，只允许纯类型 / 枚举 / 内联函数，
 * 严禁依赖任何硬件（电机 / CAN / HAL）——否则另一块板编不过。
 *
 * 字段与取值必须与三处保持一致：
 *   1. RobotModules 的状态机
 *   2. RobotComponents 的双板帧（gimbal_chassis_comm）
 *   3. Instance 层的接线
 * 从枚举取值到帧字节的映射写在 ToByte()/FromByte() 里，只有这一处定义。
 * ==========================================================================*/

namespace robot {

/** 生命状态：上电未就绪 → 恢复自检 → 正常工作（考核 3.5） */
enum class LifeState : uint8_t {
  kDead = 0,          ///< 上电未就绪：所有输出归零
  kResurrection = 1,  ///< 恢复自检：使能电机、等 IMU 零漂收敛、舵角回中
  kWorking = 2,       ///< 正常工作
};

/** 控制源（本次只有遥控器，预留扩展） */
enum class CtrlSource : uint8_t { kNone = 0, kRemote = 1 };

/** 底盘工作模式（考核 3.1） */
enum class ChassisMode : uint8_t {
  kDepart = 0,  ///< 分离：底盘速度全部来自遥控器直接映射，不自动回正
  kFollow = 1,  ///< 跟随：车头自动对准云台（w 由 δ 闭环给出）
  kGyro = 2,    ///< 小陀螺：w 为限幅后的常值，平移方向仍以云台为基准
};

/** 云台工作模式（本次只有关节空间控制） */
enum class GimbalMode : uint8_t { kJoint = 0 };

/** 底盘运动状态：平移速度定义在**云台坐标系**（考核 3.2 硬性约定） */
struct ChassisMotion {
  float vx = 0.0f;  ///< 前向，m/s（x = 云台朝向）
  float vy = 0.0f;  ///< 左向，m/s（y = 云台左方）
  float w = 0.0f;   ///< 绕 z 角速度，rad/s（逆时针为正）

  ChassisMotion operator+(const ChassisMotion &o) const {
    return ChassisMotion{vx + o.vx, vy + o.vy, w + o.w};
  }
  ChassisMotion operator-(const ChassisMotion &o) const {
    return ChassisMotion{vx - o.vx, vy - o.vy, w - o.w};
  }
  ChassisMotion operator*(float k) const {
    return ChassisMotion{vx * k, vy * k, w * k};
  }
  ChassisMotion &operator+=(const ChassisMotion &o) {
    vx += o.vx; vy += o.vy; w += o.w; return *this;
  }
  void reset(void) { vx = 0.0f; vy = 0.0f; w = 0.0f; }
};

/** 云台关节状态，rad */
struct GimbalMotion {
  float yaw = 0.0f;
  float pitch = 0.0f;

  GimbalMotion operator+(const GimbalMotion &o) const {
    return GimbalMotion{yaw + o.yaw, pitch + o.pitch};
  }
  GimbalMotion operator-(const GimbalMotion &o) const {
    return GimbalMotion{yaw - o.yaw, pitch - o.pitch};
  }
  GimbalMotion operator*(float k) const {
    return GimbalMotion{yaw * k, pitch * k};
  }
  void reset(void) { yaw = 0.0f; pitch = 0.0f; }
};

/* ---- 枚举 ↔ 帧字节的唯一映射（双板协议两边都调这两个，避免取值不一致）---- */
inline uint8_t ToByte(LifeState s) { return static_cast<uint8_t>(s); }
inline uint8_t ToByte(CtrlSource s) { return static_cast<uint8_t>(s); }
inline uint8_t ToByte(ChassisMode s) { return static_cast<uint8_t>(s); }
inline uint8_t ToByte(GimbalMode s) { return static_cast<uint8_t>(s); }

inline LifeState LifeStateFromByte(uint8_t v) {
  return (v <= 2u) ? static_cast<LifeState>(v) : LifeState::kDead;
}
inline ChassisMode ChassisModeFromByte(uint8_t v) {
  return (v <= 2u) ? static_cast<ChassisMode>(v) : ChassisMode::kDepart;
}

}  // namespace robot

#endif /* ROBOT_COMPONENTS_MODULE_STATE_HPP_ */
