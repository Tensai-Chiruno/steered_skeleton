/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.cpp
 * @brief     :本板双板通讯对象实现（Chassis）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_gimbal_comm.hpp"

#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "iwdg.h"
#include "iwdg_policy.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace {

/* 底盘板：发送 0x101（期望云台角），接收 0x102（云台实际角） */
constexpr uint32_t kC2gId = 0x101;
constexpr uint32_t kG2cId = 0x102;

/* 双板通讯固定在 CAN2：CAN1 留给电机。
 * 这样「哪条总线出问题」可以直接从现象判断：电机不动看 CAN1，双板不同步看 CAN2。 */
CAN_HandleTypeDef *const kBoardBus = &hcan2;

robot::GimbalChassisComm g_comm(robot::GimbalChassisComm::CodePart::Chassis, kC2gId, kG2cId);

}  // namespace

/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm *GetGimbalChassisComm() { return &g_comm; }

void BoardCommDispatch(CAN_HandleTypeDef *hcan, uint32_t std_id,
                       const uint8_t data[8]) {
  if (hcan != kBoardBus || std_id != g_comm.rx_id()) {
    return;
  }
  g_comm.decode(data);

#if !IWDG_UNCONDITIONAL
  /* 第 10 项验收形态：对方帧到了才喂狗。对面停发 → 本板 204 ms 后复位。 */
  HAL_IWDG_Refresh(&hiwdg);
#endif
}

/* Private function definitions ----------------------------------------------*/
