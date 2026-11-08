/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *                                                全量提供，接收分发回调掏空
 *******************************************************************************
 * @attention :
 *  接收中断回调 HAL_CAN_RxFifo0/1MsgPendingCallback 已在此预留，仅完成「取 8 字节 +
 *  再次使能接收中断」，具体「按帧 ID 分发」逻辑留给新人：
 *    - M3508 / DM 电机反馈：调用对应电机的 decode(can?_rx_data)
 *    - 双板通讯帧：调用新人自写的协议解码
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_motor.hpp"

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static uint32_t p_tx_mailbox;
static uint32_t g_tx_fail_count = 0;
static bool g_last_tx_ok = true;

/* P1-9：只接受「标准帧 + 数据帧 + 恰好 8 字节」。
 * HAL_CAN_GetRxMessage 不管实际 DLC 是几都会读满 8 字节，DLC<8 时尾部是上一帧残值；
 * 扩展帧的 StdId 只是 29 位 ID 的高 11 位；远程帧没有数据场。全部拒掉。 */
static bool RxFrameUsable(const CAN_RxHeaderTypeDef &h) {
  return (h.DLC == 8u) && (h.IDE == CAN_ID_STD) && (h.RTR == CAN_RTR_DATA);
}

/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  /* P0-2：必须零初始化并显式给出 CAN1/CAN2 的分组边界。
   * CAN_FilterTypeDef 少填一个成员时 HAL 会把栈残值写进 CAN1->FMR.CAN2SB，
   * 残值不是 14 就会「随机能收、随机不能收」，且构建层完全看不出来。 */
  CAN_FilterTypeDef can_filter = {0};
  can_filter.SlaveStartFilterBank = 14;  /* bank 0~13 归 CAN1，14~27 归 CAN2 */

  /* 全收：掩码全 0，任何 ID 均通过 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterActivation = ENABLE;

  if (hcan == &hcan1) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
  } else if (hcan == &hcan2) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterBank = 14;
  } else {
    return;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
}

bool CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  if (HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox) != HAL_OK) {
    ++g_tx_fail_count;   /* 邮箱满/bus-off 时会走到这里，不再静默丢帧 */
    g_last_tx_ok = false;
    return false;
  }
  g_last_tx_ok = true;
  return true;
}

uint32_t CAN_GetTxFailCount(void) { return g_tx_fail_count; }
bool CAN_LastTxOk(void) { return g_last_tx_ok; }

/**
 * @brief   CAN1 FIFO0 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    此处只取了 8 字节并重新使能接收中断，具体分发由新人实现
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                             can1_rx_data) == HAL_OK &&
        RxFrameUsable(rx_header1)) {
      /* 按帧 ID 路由到对应电机；未命中的 ID 由分发函数静默丢弃。
       * 真正的「ID → 哪个电机」映射表在 Instance/component/ins_motor.cpp，
       * 这里只负责把帧原样交出去——通信原语不该知道总线上挂了哪些设备。 */
      MotorDispatchCanFrame(hcan, rx_header1.StdId, can1_rx_data);
      BoardCommDispatch(hcan, rx_header1.StdId, can1_rx_data);
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN2 FIFO1 接收中断回调
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    路由逻辑与 FIFO0 相同，只是总线不同
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                             can2_rx_data) == HAL_OK &&
        RxFrameUsable(rx_header2)) {
      MotorDispatchCanFrame(hcan, rx_header2.StdId, can2_rx_data);
      BoardCommDispatch(hcan, rx_header2.StdId, can2_rx_data);
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}
