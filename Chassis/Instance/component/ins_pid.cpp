/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     :PID 实例（Chassis）
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

#include "assert.hpp"
#include "pid.hpp"

/* Private constants ---------------------------------------------------------*/
namespace {
constexpr int kNum = 4;
}  // namespace

/* Private variables ---------------------------------------------------------*/
namespace {
/* 8 个底盘回路的 PID：4 个驱动轮速度环 + 4 个舵向角度环。
 * 增益全部留 0，等台架整定后再填（考核 3.5：各回路「稳、快、无超调」）。
 * 现在留 0 是刻意的失效安全：未整定的 PID 输出恒 0，不会让车乱动。 */
hello_world::pid::Pid g_pid_wheel[kNum];
hello_world::pid::Pid g_pid_steer[kNum];
bool g_inited = false;

void EnsureInit(void) {
  if (g_inited) {
    return;
  }
  for (int i = 0; i < kNum; ++i) {
    hello_world::pid::Pid::Params p;
    p.kp = 0.0f;
    p.ki = 0.0f;
    p.kd = 0.0f;
    p.integral_limit = 5.0f;   // 与 RobotComponents/pid.hpp 的建议一致
    p.output_limit = 0.0f;     // 0 = 不限，整定时再收紧
    p.diff_on_measurement = true;
    p.angle_wrap = false;      // 速度环不需要回绕
    g_pid_wheel[i].init(p);

    hello_world::pid::Pid::Params q = p;
    q.angle_wrap = true;       // 舵向是角度环，必须回绕（-179° 与 +179° 相邻）
    g_pid_steer[i].init(q);
  }
  g_inited = true;
}
}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorWheel(int idx) {
  HW_ASSERT(idx >= 0 && idx < kNum, "Error wheel pid idx: %d", idx);
  EnsureInit();
  return &g_pid_wheel[idx];
}

hello_world::pid::Pid *GetPidMotorSteer(int idx) {
  HW_ASSERT(idx >= 0 && idx < kNum, "Error steer pid idx: %d", idx);
  EnsureInit();
  return &g_pid_steer[idx];
}
