/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :通讯层初始化与周期发送（考核第 1 / 2 项）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. CAN 滤波器 + 启动 + 电机帧发送
 *******************************************************************************
 * @attention :
 *  【云台板是 DBUS 的落点】
 *    遥控器接收机接在云台板 USART3（100k、9bit、偶校验、只收）。
 *    底盘板没有 DBUS，遥控数据由云台板经双板协议 0x102 下发（第 9 项）。
 *    所以第 2 项（UART 空闲+DMA）在这块板上做，底盘板只留接线位置。
 *
 *  【IWDG 现在是无条件喂的，这是临时措施】
 *    云台板 IWDG = Prescaler 128 / Reload 50，LSI 约 32 kHz，
 *    超时 = (50+1)*128/32000 ≈ 204 ms。不喂就会 204 ms 复位一次。
 *    第 10 项要求「关遥控器即杀程序」——那时要把本函数末尾这一行删掉，
 *    改成在遥控器解包处 / 下板信息解包处刷新（见考核说明 3.2）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

#include "HW_can.hpp"
#include "can.h"
#include "ins_motor.hpp"
#include "iwdg.h"

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

  /* TODO【第 2 项】: 启动 DBUS 接收（本板 USART3）
   *   HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dbus_buf_, 18);
   *   需先 MX_DMA_Init 已由 main.c 调用；回调见文件末尾 TODO。 */
}

void CommTask(void) {
  /* 1) yaw / pitch 指令打包发出（DM 电机的使能指令也由 encode 自动插入） */
  MotorSendAllCommands();

  /* 2) TODO【第 9 项】: 双板通讯对象 encode 后发送 0x102 */

  /* 3) 喂狗（临时，见文件头说明） */
  HAL_IWDG_Refresh(&hiwdg);
}

/* Private function definitions ----------------------------------------------*/
/* TODO【第 2 项】: 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback
 *   - 收到 18 字节后调用 GetRemoteControl()->decode(buf)，并重启下一轮接收；
 *   - 该回调是 C 链接，需 extern "C" 包裹。
 */
