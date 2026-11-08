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
#ifndef CHASSIS_INSTANCE_INS_MOTOR_HPP_
#define CHASSIS_INSTANCE_INS_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"
#include "stm32_hal.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取第 idx 号轮电机单例（M3508，共 4 个，idx ∈ [0,4)）。 */
hello_world::motor::Motor *GetMotorWheel(int idx);
/* 获取第 idx 号舵电机单例（GM6020，共 4 个，idx ∈ [0,4)）。 */
hello_world::motor::Motor *GetMotorSteer(int idx);

/* ---------------------------------------------------------------------------
 * CAN 层的两个入口（由 src/HW_can.cpp 与 Task/component/comm_task.cpp 调用）
 *
 * 为什么放在 ins_motor 而不是 HW_can：
 *   HW_can 只该知道「怎么收、怎么发」，不该知道「总线上挂了哪些电机」。
 *   把「帧 ID → 哪个电机对象」这张表留在 Instance 层，加一台电机就只改这里，
 *   不用碰通信原语。
 * ------------------------------------------------------------------------- */

/**
 * @brief       把一帧接收到的反馈分发给对应电机
 * @param       hcan:   该帧来自哪条 CAN（用于区分两条总线上的同名 ID）
 * @param       std_id: 标准帧 ID
 * @param       data:   8 字节反馈数据
 * @retval      None
 * @note        由 CAN 接收中断调用，因此必须短小、不可阻塞、不可打印。
 *              未命中的 ID 直接忽略（双板协议帧在阶段 4 接入这里）。
 */
void MotorDispatchCanFrame(CAN_HandleTypeDef *hcan, uint32_t std_id,
                           const uint8_t data[8]);

/**
 * @brief       把所有电机当前输入打包成 CAN 帧并发出（1 ms 周期调用）
 * @retval      None
 * @note        M3508 / GM6020 是「同一发送 ID 下 4 台共用一个 8 字节缓冲区」，
 *              所以必须先清零缓冲、再让 4 台分别 encode 到各自的 2 字节段，
 *              最后用共享 ID 发一次。逐台各发一帧会互相覆盖，只有一台响应。
 */
void MotorSendAllCommands(void);

#endif /* CHASSIS_INSTANCE_INS_MOTOR_HPP_ */
