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
/* 限幅必须与**执行域**一致。本工程两个电机的 input_type 都是 kCurr，
 * 而 Motor::setInput() 在 kCurr 下的入参单位是「安培」，所以 Pid 的输出限幅
 * 也必须是安培（GM6020 满量程 3 A，M3508 满量程 20 A）。
 * 【现场踩坑 2026-11-18】这里原来写的是 20000 / 16000（raw 域的数字），
 * 在安培域等于"永不饱和"，Pid 的条件积分抗饱和完全失效，
 * 舵角环因此长期以满电流开关方式工作 -> 四台 GM6020 满力高频抖动 + 啸叫。 */
constexpr float kSteerOutLimit = 1.5f;   /* A，GM6020 满量程 3 A */
constexpr float kWheelOutLimit = 5.0f;   /* A，M3508 满量程 20 A */
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
     * 注意输出限幅已经是**安培域**（kWheelOutLimit=5 A），所以条件积分抗饱和终于能生效；
     * 之前 output_limit 写成 16000（raw 数字）在安培域等于永不饱和，积分会一直顶死。 */
    p.kp = 100.0f;   /* A per (rad/s)：误差 0.01 rad/s -> 1 A */
    p.ki = 20.0f;    /* 轮子无重力负载，可以用 I 消静差 */
    p.kd = 0.0f;
    /* integral_limit 的单位是「误差×秒」，不是输出单位（P2-5）。
     * 这里给 2.0，配合 ki 才有意义；整定时按 ki 反算出想要的输出上限。 */
    p.integral_limit = 100.0f;
    p.output_limit = kWheelOutLimit;
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
