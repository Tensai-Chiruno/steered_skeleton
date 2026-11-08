/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.cpp
 * @brief     :本板双板通讯对象实现（Gimbal）
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

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace {

/* 云台板：发送 0x211（实际角反馈），接收 0x210（期望角） */
constexpr uint32_t kC2gId = 0x210;  // 考核说明 3.4 建议值
constexpr uint32_t kG2cId = 0x211;  // 必须避开 0x200/0x201~0x208/0x01~0x1F

/* 双板通讯固定在 CAN2：CAN2 是主干（yaw/pitch 电机帧 + 双板帧共用）。
 * 判据：电机不动 = 查 CAN2 上的电机帧；双板不同步 = 查 CAN2 上的双板帧。
 *   两者都在同一根线上，所以 CAN2 出错时两种现象会同时出现，别当成两个独立故障。
 * CAN1 本板未使用：它虽然被配了全收滤波 + HAL_CAN_Start + FIFO0 通知，
 *   但 BoardCommDispatch / MotorDispatchCanFrame 对 hcan1 一律直接 return，
 *   所以**误接到 CAN1 的任何设备表现为"完全没反应、也不报错"**，不要到 CAN1 上找问题。 */
CAN_HandleTypeDef *const kBoardBus = &hcan2;

robot::GimbalChassisComm g_comm(robot::GimbalChassisComm::CodePart::Gimbal, kG2cId, kC2gId);

}  // namespace

/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm *GetGimbalChassisComm() { return &g_comm; }

void BoardCommDispatch(CAN_HandleTypeDef *hcan, uint32_t std_id,
                       const uint8_t data[8]) {
  if (hcan != kBoardBus || std_id != g_comm.rx_id()) {
    return;
  }
  g_comm.decode(data);

}

/* Private function definitions ----------------------------------------------*/
