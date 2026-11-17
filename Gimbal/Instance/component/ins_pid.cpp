/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     :PID 实例（Gimbal）
 * @category : 功能 | 考核第 3 项（PID 实例与限幅）
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
/* 积分限幅：单位是"误差×秒"。当前 ki=0.3，1.5 对应的最大积分力矩贡献 = 0.45 N·m。
 * 2026-11-17 现场教训：先给 ki=1.0 + limit=0.5 → 积分饱和后一次冲击砸到极限。
 * 现在改成"小 ki + 大限幅"，靠时间慢慢爬，宁可慢也不要冲。 */
constexpr float kIntegralLimit = 2.0f;
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
  /* yaw 暂不开环：现场一直读到"使能=0、反馈恒 0"，先保持 kp=0，等查清电机再开。 */
  g_pid_yaw.init(p);

  /* pitch 整定（2026-11-17 现场实测曲线）
   *   kp=2.5/kd=2.0 -> 稳在 -24.1°（抬不动，力矩差一点）
   *   kp=4.0/kd=2.0 -> 从 -25° 抬到 -9° 并稳住（±0.1°，无抖动），但残留 9~14° 静态误差
   *   ki=1.0 / 0.5 / 0.3 三次尝试都在"摆脱阻力"的瞬间猛砸（积分饱和）
   * 结论：**不用积分**，改用"重力前馈 + PD"（见 gimbal.hpp 的 gravity_torq）。 */
  p.kp = 4.0f;
  p.kd = 2.0f;
  p.ki = 0.0f;
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
