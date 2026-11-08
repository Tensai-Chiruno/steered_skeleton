/**
 *******************************************************************************
 * @file      :ins_motor.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_INSTANCE_INS_MOTOR_HPP_
#define GIMBAL_INSTANCE_INS_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"
#include "stm32_hal.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取云台 yaw 关节电机单例（DM-J4310）。 */
hello_world::motor::Motor *GetMotorYaw();
/* 获取云台 pitch 关节电机单例（DM-J4310）。 */
hello_world::motor::Motor *GetMotorPitch();

/* ---------------------------------------------------------------------------
 * CAN 层的两个入口（由 src/HW_can.cpp 与 Task/component/comm_task.cpp 调用）
 * 说明同底盘板：HW_can 只管收发，ID→设备的映射表留在 Instance 层。
 * ------------------------------------------------------------------------- */

/**
 * @brief       把一帧接收到的反馈分发给对应电机
 * @param       hcan:   该帧来自哪条 CAN
 * @param       std_id: 标准帧 ID
 * @param       data:   8 字节反馈数据
 * @retval      None
 * @note        由 CAN 接收中断调用：短小、不可阻塞、不可打印。
 *              DM 电机反馈帧 ID 恒为 id + 0x10（yaw 0x11 / pitch 0x12）。
 */
void MotorDispatchCanFrame(CAN_HandleTypeDef *hcan, uint32_t std_id,
                           const uint8_t data[8]);

/**
 * @brief       把 yaw/pitch 当前输入打包发出（1 ms 周期调用）
 * @retval      None
 * @note        DM 电机上电失能，encode() 会自动插入使能指令，
 *              因此本函数必须被周期调用，电机才会被真正使能。
 */
void MotorSendAllCommands(void);

#endif /* GIMBAL_INSTANCE_INS_MOTOR_HPP_ */
