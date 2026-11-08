/**
 *******************************************************************************
 * @file      :ins_comm_state.hpp
 * @brief     :遥控器接收状态（Gimbal）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifndef INSTANCE_COMPONENT_INS_COMM_STATE_HPP_
#define INSTANCE_COMPONENT_INS_COMM_STATE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

/* Exported function prototypes ----------------------------------------------*/

/** @brief  每收到一帧合法 DBUS 就登记一次时间戳，由 UART 回调调用。 */
void NotifyRemoteFrame(void);

/** @brief  上电以来是否收到过任何一帧。 */
bool RemoteHasFrame(void);

/** @brief  距离最近一帧的毫秒数；从未收到过时返回 UINT32_MAX。 */
uint32_t RemoteIdleMs(void);

/** @brief  遥控器是否在线：收到过帧且空闲时间小于阈值。 */
bool IsRemoteOnline(void);

/**
 * @brief  主动作废"在线"状态（用于连续可疑帧：按离线走现成的急停路径）。
 * @note   不是新状态，只是把 has_frame 清掉；下一帧合法数据会自动恢复。
 */
void InvalidateRemote(void);

#endif /* INSTANCE_COMPONENT_INS_COMM_STATE_HPP_ */
