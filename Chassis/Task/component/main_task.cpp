/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     :任务层入口（1 ms 控制/通讯调度）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 阶段 1 最小接线：初始化 + 周期分发
 *******************************************************************************
 * @attention :
 *  【本文件当前只做到「能跑起来」的程度】
 *    阶段 1 的目标是电机反馈能读、指令能发，所以这里只接了：
 *      MainTaskInit() → 建立电机对象 + 通讯初始化
 *      htim6 回调     → MainTask() + CommTask()
 *    MainTask() 内部留空，等第 6/7 项把 Robot/状态机/各管理器接进来。
 *    现在就在 MainTask 里写控制逻辑，会在没有状态机的前提下堆积一次性代码，
 *    后面必须整段重写——这是刻意不做的。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "ins_comm.hpp"
#include "ins_imu.hpp"
#include "main.h"
#include "HW_can.hpp"
#include "gimbal_chassis_comm.hpp"
#include "ins_robot.hpp"
#include "robot.hpp"
#include "chassis.hpp"

#include "comm_task.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_motor.hpp"
#include "tick.hpp"
#include "tim.h"

/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void) {
  /* 1) 建立全部电机对象并完成参数配置。
   *    EnsureInit() 在第一次访问 GetMotorXxx 时懒执行，这里主动访问一次，
   *    让「初始化」发生在 while(1) 之前，而不是在第一个中断里。 */
  for (int i = 0; i < 4; ++i) {
    (void)GetMotorWheel(i);
    (void)GetMotorSteer(i);
  }

  /* 2) 双板通讯对象（构造时就绑定 0x210/0x211 与 CAN2） */
  (void)GetGimbalChassisComm();
  (void)GetRemoteControl();  // 两板都收 DBUS

  /* 3) 通讯初始化：CAN 滤波器 + 启动外设 + 打开接收中断 */
  CommTaskInit();

  /* TODO【第 3 项】: IMU 零漂采集（须在电机使能之后、状态机之前） */
  /* TODO【第 6 项】: 把各模块注册进 Robot */
  /* ---- 状态机与运动模块接线（遥控在线判定 + 生命状态机） ---- */
  GetChassis()->init();
  GetRobot()->registerChassis(GetChassis());
  GetRobot()->registerRemoteControl(GetRemoteControl());
  /* IMU 在主上下文预热一次：imuInit() 内含 HAL_Delay，绝不能第一次就在中断里跑（G-4） */
  (void)GetImu();
  GetRobot()->registerImu(GetImu());
  GetRobot()->registerComm(GetGimbalChassisComm());
  GetRobot()->init();

}

namespace {
volatile uint32_t g_main_loop_ms = 0xFFFFFFFFu;
bool g_main_loop_seen = false;
}  // namespace

void MainLoopHeartbeat(void) {
  g_main_loop_ms = hello_world::tick::GetTickMs();
  g_main_loop_seen = true;
}

uint32_t MainLoopIdleMs(void) {
  if (!g_main_loop_seen) {
    return 0xFFFFFFFFu;
  }
  return hello_world::tick::GetTickMs() - g_main_loop_ms;
}

namespace {

/* ---------------------------------------------------------------------------
 * 板载状态灯（PC8，只有底盘板有这根脚；云台板 gpio.c 里没配，别去找）。
 *
 * 闪码（1 kHz 拍子里数脉冲，不用 HAL_Delay，不放在 CAN/USART 中断里）：
 *   常亮     = 全绿
 *   1 闪     = IMU 未 ready
 *   2 闪     = IMU 标定超时（已降级，姿态只能短期可信）
 *   3 闪     = CAN 发送失败计数在涨（含 bus-off / 无 ACK；单板台架非零属正常）
 *   4 闪     = 主循环心跳停了（P1-8 的另一半：TIM6 还在跳但主上下文卡死）
 *   5 闪     = 对端板离线
 *   6 闪     = 对端在但不在 Working（对面卡在哪个生命状态就看它）
 *   7 闪     = 舵角零位自检不过（P0-4 的 offset 没对上，见 g_steer_bad_mask）
 * ------------------------------------------------------------------------- */
constexpr uint32_t kLedOnMs = 120u;
constexpr uint32_t kLedOffMs = 140u;
constexpr uint32_t kLedCyclePauseMs = 1000u;
constexpr uint32_t kSteerCheckDelayMs = 2000u;
constexpr float kSteerZeroTol = 0.2f;

uint32_t g_start_ms = 0u;
bool g_started = false;
uint32_t g_last_tx_fail = 0u;
bool g_tx_fail_flag = false;
uint32_t g_tx_fail_flag_ms = 0u;
uint32_t g_steer_bad_mask = 0u;
bool g_steer_checked = false;

uint32_t LedCode(void) {
  hello_world::imu::Imu *imu = GetImu();
  if (imu == nullptr || !imu->isReady()) {
    return 1u;
  }
  if (imu->isCalibTimeout()) {
    return 2u;
  }
  if (g_tx_fail_flag) {
    return 3u;
  }
  /* 上电 500 ms 内主循环还没跑到第一圈，不算"停" */
  if ((hello_world::tick::GetTickMs() - g_start_ms) > 500u && MainLoopIdleMs() >= 200u) {
    return 4u;
  }
  robot::GimbalChassisComm *bc = GetGimbalChassisComm();
  if (bc == nullptr || bc->isOffline()) {
    return 5u;
  }
  if (robot::LifeStateFromByte(bc->remote().ctrl_mode) != robot::LifeState::kWorking) {
    return 6u;
  }
  if (g_steer_bad_mask != 0u) {
    return 7u;
  }
  return 0u;
}

void SteerZeroSelfCheck(void) {
  if (g_steer_checked) {
    return;
  }
  if ((hello_world::tick::GetTickMs() - g_start_ms) < kSteerCheckDelayMs) {
    return;
  }
  g_steer_checked = true;
  uint32_t mask = 0u;
  for (int i = 0; i < 4; ++i) {
    auto *m = GetMotorSteer(i);
    if (m == nullptr) {
      continue;
    }
    const float a = m->angle();
    if (a > kSteerZeroTol || a < -kSteerZeroTol) {
      mask |= (1u << i);
    }
  }
  g_steer_bad_mask = mask;
}

void StatusLedUpdate(void) {
  const uint32_t now = hello_world::tick::GetTickMs();
  if (!g_started) {
    g_started = true;
    g_start_ms = now;
  }
  /* 3 号灯：发送失败计数只要在涨就点亮 1 s（不够就续），不需要锁存 */
  const uint32_t fail = CAN_GetTxFailCount();
  if (fail != g_last_tx_fail) {
    g_last_tx_fail = fail;
    g_tx_fail_flag = true;
    g_tx_fail_flag_ms = now;
  } else if (g_tx_fail_flag && (now - g_tx_fail_flag_ms) > 1000u) {
    g_tx_fail_flag = false;
  }

  const uint32_t code = LedCode();
  bool high = true;
  if (code != 0u) {
    const uint32_t unit = kLedOnMs + kLedOffMs;
    const uint32_t cycle = code * unit + kLedCyclePauseMs;
    const uint32_t pos = (now - g_start_ms) % cycle;
    high = (pos < code * unit) && ((pos % unit) < kLedOnMs);
  }
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

}  // namespace

void MainTask(void) {
  /* 1 ms 控制周期：读数据 -> 状态迁移 -> 生成指令 -> 下发。
   * 安全前提（急停/模式/在线判定）全部在 Robot 里，这里只做一次调用。 */
  GetRobot()->update();

  /* 舵零位自检必须在填第一个 kp 之前看过一眼（P0-4 那四个 offset 对不对）。
   * 纯读、不开环、不给输出。 */
  if (!g_started) {
    g_started = true;
    g_start_ms = hello_world::tick::GetTickMs();
  }
  SteerZeroSelfCheck();
  StatusLedUpdate();
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief  TIM6 周期中断回调（1 ms）
 * @note   TIM2 也用 HAL 定时器中断（底盘板 main.c 里启动了 htim2 与 htim6），
 *         所以必须先判来源，否则别的定时器溢出会当成 1 ms tick，
 *         表现为「PID 周期莫名其妙变快」。
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {
    MainTask();
    CommTask();
  }
}
