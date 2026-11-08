/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.cpp
 * @brief     :底盘↔云台双板 CAN 通讯实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 帧打包/解包 + 离线判断
 *******************************************************************************
 * @attention :
 *  字节布局与缩放全部委托给 gimbal_chassis_comm_codec，本文件只做：
 *    - 按 code_part 选方向
 *    - 维护序列号与离线时间戳
 *    - 校验失败时丢弃且不更新缓存
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal_chassis_comm.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm::GimbalChassisComm(
    robot::GimbalChassisComm::CodePart code_part, uint32_t tx_id, uint32_t rx_id)
    : code_part_(code_part),
      tx_id_(tx_id),
      rx_id_(rx_id),
      last_rx_tick_(hello_world::tick::GetTickMs()) {}

void robot::GimbalChassisComm::encode(uint8_t tx_data[8]) {
  hello_world::comm::PackFrame(local(), seq_, tx_data);
  seq_ = static_cast<uint8_t>(seq_ + 1);  // 溢出回绕是有意的：对面只比差异
}

void robot::GimbalChassisComm::decode(const uint8_t rx_data[8]) {
  Frame tmp;
  if (!hello_world::comm::UnpackFrame(rx_data, &tmp)) {
    ++bad_frame_count_;
    return;  // 坏帧不刷新时间戳，让离线判断照常超时
  }

  /* 写入「对方」那份缓存：底盘板上收到的来自云台，反之亦然 */
  if (code_part_ == CodePart::Chassis) {
    gimbal_data_ = tmp;
  } else {
    chassis_data_ = tmp;
  }
  last_rx_tick_ = hello_world::tick::GetTickMs();
}

bool robot::GimbalChassisComm::isOffline() const {
  /* GetTickMs 是 32 位、1 ms 分辨率，减法在无符号域里天然处理回绕 */
  return (hello_world::tick::GetTickMs() - last_rx_tick_) >
         kOfflineTimeoutMs;
}

/* Private function definitions ----------------------------------------------*/
