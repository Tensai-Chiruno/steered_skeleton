/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.hpp
 * @brief     :本板双板通讯对象单例（Chassis）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifndef CHASSIS_INSTANCE_INS_CHASSIS_GIMBAL_COMM_HPP_
#define CHASSIS_INSTANCE_INS_CHASSIS_GIMBAL_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace robot {
class GimbalChassisComm;
}  // namespace robot
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取本板双板通讯对象（robot::GimbalChassisComm）单例。 */
robot::GimbalChassisComm *GetGimbalChassisComm();

/**
 * @brief  把收到的帧尝试交给双板协议解码
 * @param  hcan:  该帧来自哪条 CAN
 * @param  std_id 标准帧 ID
 * @param  data   8 字节
 * @retval None
 * @note   由 CAN 接收中断调用。只认 CAN2 上、ID 等于「本方接收 ID」的帧，
 *         其余直接返回——电机帧在上一步已经被 MotorDispatchCanFrame 消化掉了。
 */
void BoardCommDispatch(CAN_HandleTypeDef *hcan, uint32_t std_id,
                       const uint8_t data[8]);

#endif /* CHASSIS_INSTANCE_INS_CHASSIS_GIMBAL_COMM_HPP_ */
