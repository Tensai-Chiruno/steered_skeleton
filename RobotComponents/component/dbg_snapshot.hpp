/**
 *******************************************************************************
 * @file      :dbg_snapshot.hpp
 * @brief     :只读调试快照（给现场用调试器读数，不参与控制）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-11      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#ifndef ROBOT_COMPONENTS_COMPONENT_DBG_SNAPSHOT_HPP_
#define ROBOT_COMPONENTS_COMPONENT_DBG_SNAPSHOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace robot {

/**
 * @brief 现场调试用的只读快照。
 *
 * 目的：上板时用调试器（pyocd / STM32_Programmer）直接读这一个结构，
 * 就能看到舵角、轮速、IMU、状态机、双板链路等所有关键量，
 * 不必逐个去猜匿名命名空间里静态对象的符号名，也不用往串口塞 printf。
 *
 * 约定：
 *   - 由 Task 层每拍最后一次写入，控制逻辑**只写不读**，不参与任何判断；
 *   - magic 用来确认"这块内存确实是本结构、且固件版本对得上"；
 *   - 字段顺序/类型改动必须同步改 tools/read_vars.py。
 */
struct DbgSnapshot {
  uint32_t magic;          ///< 固定 0x5A5ADBG? 见 kDbgMagic
  uint32_t tick_ms;        ///< 写入时刻（GetTickMs）
  float steer[4];          ///< 4 个舵的实测角，rad
  float steer_target[4];   ///< 4 个舵的目标角，rad
  float wheel_target[4];   ///< 4 个轮的轮心线速度目标，m/s
  float wheel_vel[4];      ///< 4 个轮的实测输出端角速度，rad/s
  float imu_yaw;           ///< rad
  float imu_pitch;         ///< rad
  float frame_delta;       ///< δ（云台相对车头），rad
  uint32_t tx_fail_count;  ///< CAN 发送失败累计
  uint8_t life_state;      ///< 0 Dead / 1 Resurrection / 2 Working
  uint8_t chassis_mode;    ///< 0 分离 / 1 跟随 / 2 小陀螺（云台板固定 0）
  uint8_t ctrl_source;     ///< 0 None / 1 Remote
  uint8_t imu_ready;       ///< 0/1
  uint8_t peer_offline;    ///< 0/1
  uint8_t estop;           ///< 0/1
  uint8_t remote_online;   ///< 0/1
  uint8_t reserved0;       ///< 对齐
};

inline constexpr uint32_t kDbgMagic = 0x5A5ADB91u;

/** @brief 全局唯一实例；两板各一份，名字相同，读法相同。 */
inline volatile DbgSnapshot g_dbg{};

}  // namespace robot

#endif /* ROBOT_COMPONENTS_COMPONENT_DBG_SNAPSHOT_HPP_ */
