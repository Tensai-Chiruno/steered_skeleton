/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :通讯层初始化与周期发送（云台板，含 DBUS）
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
#include "ins_comm.hpp"
#include "ins_motor.hpp"
#include "usart.h"
#include "iwdg.h"
#include "iwdg_policy.hpp"

/* Private variables ---------------------------------------------------------*/
namespace {
/* DBUS 一帧固定 18 字节。缓冲区必须是静态的：DMA 直写内存，
 * 放栈上函数一返回就被覆盖。 */
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

  /* 3) DBUS：USART3 已由 CubeMX 配成 100k / 9bit / 偶校验 / 仅接收，
   *    且 DMA1_Stream1 是【循环模式】（见 usart.c），所以只需要启动一次，
   *    不需要每帧重启接收。
   *    关掉半传输中断：DBUS 每 14 ms 才来一帧，半满中断只会制造无谓负载。 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf, sizeof(g_dbus_buf));
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

void CommTask(void) {
  /* 1) yaw / pitch 指令打包发出（CAN1；DM 使能指令由 encode 自动插入） */
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
   * 喂狗改由 DBUS 解包回调完成（关遥控器即杀程序）。 */
  HAL_IWDG_Refresh(&hiwdg);
#endif
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief UART 空闲/DMA 接收完成回调
 * @note  必须 extern "C"：HAL 的弱符号是 C 链接，不加就覆盖不到它，
 *        现象是「UART 收到了但回调从不进」。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size) {
  if (huart != &huart3) {
    return;
  }
  if (Size >= kDbusFrameLen) {
    GetRemoteControl()->decode(g_dbus_buf);

#if !IWDG_UNCONDITIONAL
    /* 第 10 项验收形态：收到遥控器数据才喂狗 */
    HAL_IWDG_Refresh(&hiwdg);
#endif
  }
  /* 循环 DMA 下不需要重启；只有出错导致 RxState 掉出 BUSY 时才补一次 */
  if (huart3.RxState != HAL_UART_STATE_BUSY_RX) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf, sizeof(g_dbus_buf));
  }
}
