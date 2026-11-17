/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     :PID 实例（Chassis）
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

#include "assert.hpp"
#include "pid.hpp"

/* Private constants ---------------------------------------------------------*/
namespace {
constexpr int kNum = 4;
/* 限幅必须与执行域一致：底盘走 kRaw，舵 = 0x1FF(25000) / 轮 = 0x200(16384)。
 * 与 chassis.hpp 的 steer_raw_limit / wheel_raw_limit 同量纲，
 * 这样 Pid 的「输出饱和」判据才用得上，条件积分抗饱和才真正生效。 */
constexpr float kSteerOutLimit = 20000.0f;
constexpr float kWheelOutLimit = 16000.0f;
}  // namespace

/* Private variables ---------------------------------------------------------*/
namespace {
hello_world::pid::Pid g_pid_wheel[kNum];
hello_world::pid::Pid g_pid_steer[kNum];
bool g_inited = false;

void EnsureInit(void) {
  if (g_inited) return;
  for (int i = 0; i < kNum; ++i) {
    hello_world::pid::Pid::Params p;
    /* 轮速环现场整定 2026-11-17：先只加 P（kp=10），ki/kd 保持 0。
     * 第一版先用小限幅（见 chassis.hpp 的 wheel_raw_limit=5000）兜住，
     * 看实测轮速能否平稳跟随目标再谈加速度环。 */
    p.kp = 0.0f;   /* 轮速环：现场已验证到 kp=100/ki=20（见现场调试记录），收工时归零保证安全 */
    p.ki = 0.0f;   /* 轮速环积分：整定时给到 20（轮子无重力负载，可以放心用 I） */
    p.kd = 0.0f;
    /* integral_limit 的单位是「误差×秒」，不是输出单位（P2-5）。
     * 这里给 2.0，配合 ki 才有意义；整定时按 ki 反算出想要的输出上限。 */
    p.integral_limit = 100.0f;
    p.output_limit = kWheelOutLimit;
    p.diff_on_measurement = true;
    p.angle_wrap = false;   /* 速度环不回绕 */
    g_pid_wheel[i].init(p);

    hello_world::pid::Pid::Params q = p;
    q.kp = 0.0f;   /* 舵角环：电流模式(0x1FE)下 3000 会自激，收工归零，明天从 200 起重新整定 */
    q.output_limit = kSteerOutLimit;
    q.angle_wrap = true;    /* 舵向是角度环，必须回绕 */
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
