/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :通讯层初始化与周期发送（考核第 1 项）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. CAN 滤波器 + 启动 + 电机帧发送
 *******************************************************************************
 * @attention :
 *  【本阶段（第 1 项）只做 CAN】
 *    - 遥控器 UART（第 2 项）与双板协议（第 9 项）未接入，留 TODO 标记。
 *    - CAN 接收的「按 ID 分发」不在这里，在 src/HW_can.cpp 的回调里、
 *      由 Instance/component/ins_motor.cpp 的 MotorDispatchCanFrame() 完成。
 *      此处重复实现会形成两条分发路径，是后面「一个电机两处解码」类 bug 的来源。
 *
 *  【IWDG 现在是无条件喂的，这是临时措施】
 *    底盘板 IWDG = Prescaler 128 / Reload 50，LSI 约 32 kHz，
 *    超时 = (50+1)*128/32000 ≈ 204 ms。若无人喂狗，板子会每 204 ms 复位一次，
 *    连 Ozone 都连不上。所以阶段 1 先在本任务里无条件刷新，保证能调试。
 *    第 10 项要求「关遥控器即杀程序」——那时要把这一行删掉，改成在
 *    遥控器解包处与下板信息解包处刷新（见考核说明 3.2 的看门狗一行）。
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

  /* 2) 启动 CAN 外设并打开接收中断。
   *    注意：CanFilter_Init 只配滤波器，不启动外设；漏掉 HAL_CAN_Start 的现象是
   *    「滤波器配好了、中断也使能了，但一帧都收不到」。 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* TODO【第 2 项】: 启动遥控器 UART 空闲 + DMA 接收
   *   HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dbus_buf, 18);
   *   DBUS 在云台板 USART3；底盘板此处只保留接线位置。 */
}

void CommTask(void) {
  /* 1) 把各电机当前输入打包成共享帧发出 */
  MotorSendAllCommands();

  /* 2) TODO【第 9 项】: 双板通讯对象 encode 后同样发送 */

  /* 3) 喂狗（临时，见文件头说明） */
  HAL_IWDG_Refresh(&hiwdg);
}

/* Private function definitions ----------------------------------------------*/
/* TODO【第 2 项】: 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback
 *   - 收到 18 字节后调用 GetRemoteControl()->decode(buf)，并重启下一轮接收；
 *   - 该回调是 C 链接，需 extern "C" 包裹。
 */
