/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     :PID 实例（Gimbal）
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
#include "ins_pid.hpp"

#include "pid.hpp"

/* Private constants ---------------------------------------------------------*/
namespace {
/* 与 Gimbal::Params::max_torq 同值（N·m）：见 G-2 / G-7 */
constexpr float kTorqOutLimit = 2.0f;
constexpr float kIntegralLimit = 0.5f;
}  // namespace

/* Private variables ---------------------------------------------------------*/
namespace {
hello_world::pid::Pid g_pid_yaw;
hello_world::pid::Pid g_pid_pitch;
bool g_inited = false;

void EnsureInit(void) {
  if (g_inited) return;
  hello_world::pid::Pid::Params p;
  p.kp = 0.0f;
  p.ki = 0.0f;
  p.kd = 0.0f;
  p.integral_limit = kIntegralLimit;  /* 单位 = 误差×秒（P2-5） */
  p.output_limit = kTorqOutLimit;     /* 单位 = N·m，与执行器一致 */
  p.diff_on_measurement = true;
  p.angle_wrap = true;
  g_pid_yaw.init(p);
  g_pid_pitch.init(p);
  g_inited = true;
}
}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorYaw() {
  EnsureInit();
  return &g_pid_yaw;
}

hello_world::pid::Pid *GetPidMotorPitch() {
  EnsureInit();
  return &g_pid_pitch;
}
