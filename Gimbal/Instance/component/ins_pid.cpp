/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     :PID 实例（Gimbal）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"

#include "pid.hpp"

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
  p.integral_limit = 5.0f;
  p.output_limit = 0.0f;
  p.diff_on_measurement = true;
  p.angle_wrap = true;   /* 两轴都是角度环 */
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
