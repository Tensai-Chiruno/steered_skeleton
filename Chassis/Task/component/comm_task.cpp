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
#include "main_task.hpp"
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
  /* P1-3：online / ready / work_mode / ctrl_mode 由数据拥有者（Robot）每拍写入，
   * 这里**不能**再无条件覆盖——旧代码每 1 ms 写一次 online=1 / ready=0，
   * 表现为「online 永远自报在线（反诊断）」且「别人写的 ready 下一拍就被清掉」。 */
  uint8_t buf[8];
  bc->encode(buf);
  CAN_Send_Msg(&hcan2, buf, bc->tx_id(), 8);

#if IWDG_FEED_IN_COMM_TASK
  /* 正常形态：CommTask 每 1ms 刷新。验收演示复位时把
   * iwdg_policy.hpp 的 IWDG_FEED_IN_COMM_TASK 改成 0。 */
  /* P1-8②：喂狗条件里加一条「主上下文还活着」。
   * 只证明 TIM6 在跳是不够的：Error_Handler() 这类主上下文死循环发生时，
   * TIM6 照样每 1 ms 抢占并喂狗，看门狗等于关闭。 */
  if (MainLoopIdleMs() < 200u) {
    HAL_IWDG_Refresh(&hiwdg);
  }
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

/**
 * @brief UART 错误回调（P1-7）
 * @note  PE/FE/NE/ORE 任一发生时 HAL 会中止 DMA 接收并且**不再**进 RxEventCallback，
 *        所以「在 RxEventCallback 里检查 RxState 重新挂载」这条兜底永远等不到机会 ——
 *        遥控器会因为一次噪声错误而永久失联，直到重启。这里必须重新挂载。
 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart != &huart3) {
    return;
  }
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);
  HAL_UART_DMAStop(huart);
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf, sizeof(g_dbus_buf));
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}
