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
/* 限幅必须与**执行域**一致。⚠ 两路电机的执行域**不同**：
 *   舵 GM6020：ins_motor.cpp 里 kSteerInputType = kCurr -> 入参是 **安培**（满量程 3 A）
 *   轮 M3508 ：ins_motor.cpp 里 kWheelInputType = kRaw  -> 入参是 **raw**（满量程 16384）
 * 因此下面两个限幅一个是安培、一个是 raw，名字里都写明了。
 * 【现场踩坑 2026-11-18】原来两个都写成 20000/16000（raw 数字），舵那侧的安培域
 * 等于"永不饱和"，Pid 条件积分抗饱和完全失效；反过来把轮改成安培（5）又让轮子
 * 只拿到 raw=5 ≈ 0.006 A，悬空都不转。改这里之前先确认 ins_motor.cpp 的 input_type。 */
constexpr float kSteerOutLimitA = 2.0f;    /* 安培：GM6020 满量程 3 A */
constexpr float kWheelOutLimitRaw = 8000.0f;  /* raw：M3508 满量程 16384（8000 ≈ 9.8 A） */
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
    /* 轮速环：恢复 2026-11-17 现场整定并验证过的值（目标 30 rad/s -> 实测 29.0~29.8）。
     * 增益是 **raw 域**（轮电机 kWheelInputType = kRaw）：误差 30 rad/s -> 3000 raw ≈ 3.7 A。 */
    p.kp = 100.0f;   /* raw per (rad/s) */
    p.ki = 20.0f;    /* 轮子无重力负载，可以用 I 消静差 */
    p.kd = 0.0f;
    /* integral_limit 的单位是「误差×秒」，不是输出单位（P2-5）。
     * 联调期压到 20：轮子被限幅卡住时不让积分越积越大，松杆时不至于冲一下。 */
    p.integral_limit = 20.0f;
    p.output_limit = kWheelOutLimitRaw;
    p.diff_on_measurement = true;
    p.angle_wrap = false;   /* 速度环不回绕 */
    g_pid_wheel[i].init(p);

    hello_world::pid::Pid::Params q = p;
    /* 舵角环**不使用**这个 PID 对象：Chassis::update() 里改用手写 PD
     * （误差回绕 + 直接用电机反馈角速度做阻尼，避开 ±180° 差分爆炸）。
     * 这里保留对象只是为了 GetPidMotorSteer()/reset() 的接口一致性，增益置 0。 */
    q.kp = 0.0f;
    q.kd = 0.0f;
    q.ki = 0.0f;
    q.output_limit = kSteerOutLimitA;
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
