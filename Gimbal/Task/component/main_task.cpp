/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     :云台板任务层入口（1 ms 控制/通讯调度）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 阶段 1 最小接线：初始化 + 周期分发
 *******************************************************************************
 * @attention :
 *  【本文件当前只做到「能跑起来」的程度】
 *    阶段 1 的目标是 yaw/pitch 反馈能读、指令能发，所以这里只接了：
 *      MainTaskInit() → 建立电机对象 + 通讯初始化
 *      htim6 回调     → MainTask() + CommTask()
 *    MainTask() 内部留空，等第 6/7/8 项把 Robot/状态机/云台位置环接进来。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "ins_comm.hpp"
#include "ins_robot.hpp"
#include "robot.hpp"
#include "gimbal.hpp"

#include "comm_task.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_motor.hpp"
#include "tim.h"

/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void) {
  /* 1) 建立 yaw / pitch 电机对象（DM 电机失能状态下不会有反馈，
   *    使能指令由 encode() 在 CommTask 周期里自动插入） */
  (void)GetMotorYaw();
  (void)GetMotorPitch();

  /* 2) 双板通讯对象 + 遥控器解码对象 */
  (void)GetGimbalChassisComm();
  (void)GetRemoteControl();

  /* 3) 通讯初始化：CAN 滤波器/启动/中断 + DBUS 的 UART 空闲 DMA */
  CommTaskInit();

  /* TODO【第 3 项】: IMU 零漂采集 */
  /* TODO【第 9 项】: 双板通讯对象创建 */
  /* ---- 状态机与运动模块接线（遥控在线判定 + 生命状态机） ---- */
  GetGimbal()->init();
  GetRobot()->registerGimbal(GetGimbal());
  GetRobot()->registerRemoteControl(GetRemoteControl());
  GetRobot()->init();

}

void MainTask(void) {
  /* 1 ms 控制周期。
   * TODO【第 6/8 项】: 云台关节位置环（yaw/pitch 位置环，pitch 需反馈限位）。
   * 目前为空：没有状态机与 PID 实例时下发位置指令缺安全前提。 */
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief  TIM6 周期中断回调（1 ms）
 * @note   先判来源：云台板同样启动了多个定时器，误判会让「1 ms 周期」失真。
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {
    MainTask();
    CommTask();
  }
}
