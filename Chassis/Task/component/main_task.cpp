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
#include "ins_robot.hpp"
#include "robot.hpp"
#include "chassis.hpp"

#include "comm_task.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_motor.hpp"
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

void MainTask(void) {
  /* 1 ms 控制周期：读数据 -> 状态迁移 -> 生成指令 -> 下发。
   * 安全前提（急停/模式/在线判定）全部在 Robot 里，这里只做一次调用。 */
  GetRobot()->update();
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
