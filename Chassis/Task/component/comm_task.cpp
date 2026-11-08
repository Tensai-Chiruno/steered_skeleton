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
#include "ins_comm_state.hpp"

#include "HW_can.hpp"
#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_motor.hpp"
#include "usart.h"
#include "iwdg.h"
#include "iwdg_policy.hpp"

/* Private variables ---------------------------------------------------------*/
namespace {
constexpr uint16_t kDbusFrameLen = 18u;  ///< DBUS 固定 18 字节
uint8_t g_dbus_buf[kDbusFrameLen];
}  // namespace

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

  /* 3) DBUS：考核说明任务 2 明确「两板」都要做。两板的 USART3 都是
   *    100k / 9bit / 偶校验 / 仅接收，DMA1_Stream1 循环模式，所以启动一次即可。
   *    关掉半传输中断：DBUS 每 14 ms 才来一帧，半满中断只增加负载。 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf, sizeof(g_dbus_buf));
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
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

#if IWDG_FEED_IN_COMM_TASK
  /* 正常形态：CommTask 每 1ms 刷新。验收演示复位时把
   * iwdg_policy.hpp 的 IWDG_FEED_IN_COMM_TASK 改成 0。 */
  HAL_IWDG_Refresh(&hiwdg);
#endif
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief UART 空闲/DMA 接收完成回调（底盘板）
 * @note  必须 extern "C"，否则覆盖不到 HAL 的弱符号，现象是「收到了但回调不进」。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size) {
  if (huart != &huart3) {
    return;
  }
  if (Size >= kDbusFrameLen) {
    GetRemoteControl()->decode(g_dbus_buf);
    NotifyRemoteFrame();
  }
  if (huart3.RxState != HAL_UART_STATE_BUSY_RX) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf, sizeof(g_dbus_buf));
  }
}
