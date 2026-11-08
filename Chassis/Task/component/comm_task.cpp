/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :通讯层初始化与周期发送（底盘板）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

#include "HW_can.hpp"
#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_motor.hpp"
#include "iwdg.h"
#include "iwdg_policy.hpp"

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void) {
  /* 1) 两条总线都配置全收滤波器 */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  /* 2) 启动外设并打开接收中断。
   *    CanFilter_Init 只配滤波器、不启动外设；漏掉 HAL_CAN_Start 的现象是
   *    「滤波器配好、中断也使能，但一帧都收不到」。 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* 底盘板不接 DBUS：遥控数据由云台板经 0x102 下发。DBUS 接收在云台板实现。 */
}

void CommTask(void) {
  /* 1) 4 轮 M3508 + 4 舵 GM6020 指令打包发出（CAN1） */
  MotorSendAllCommands();

  /* 2) 双板帧：把本板状态编码后发到 CAN2 */
  robot::GimbalChassisComm *bc = GetGimbalChassisComm();
  bc->local().online = 1;
  bc->local().ready = 0;
  uint8_t buf[8];
  bc->encode(buf);
  CAN_Send_Msg(&hcan2, buf, bc->tx_id(), 8);

#if IWDG_UNCONDITIONAL
  /* 台架保活。验收时把 iwdg_policy.hpp 改成 0，
   * 喂狗改由 BoardCommDispatch() 在收到云台帧时完成。 */
  HAL_IWDG_Refresh(&hiwdg);
#endif
}

/* Private function definitions ----------------------------------------------*/
