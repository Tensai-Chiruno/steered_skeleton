/**
 *******************************************************************************
 * @file      :ins_comm_state.cpp
 * @brief     :遥控器接收状态实现（Chassis）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ins_comm_state.hpp"

#include "tick.hpp"

/* Private macro -------------------------------------------------------------*/
namespace {
/* DBUS 帧率约 143 Hz（7 ms 一帧），100 ms 无帧即判离线，与 GIMBAL_CHASSIS 一致 */
constexpr uint32_t kRemoteTimeoutMs = 100;
}  // namespace

/* Private variables ---------------------------------------------------------*/
namespace {
volatile uint32_t g_last_rx_ms = 0;
volatile bool g_has_frame = false;
}  // namespace

/* Exported function definitions ---------------------------------------------*/
void NotifyRemoteFrame(void) {
  g_last_rx_ms = hello_world::tick::GetTickMs();
  g_has_frame = true;
}

bool RemoteHasFrame(void) { return g_has_frame; }

uint32_t RemoteIdleMs(void) {
  if (!g_has_frame) {
    return 0xFFFFFFFFu;
  }
  return hello_world::tick::GetTickMs() - g_last_rx_ms;
}

void InvalidateRemote(void) { g_has_frame = false; }

bool IsRemoteOnline(void) {
  return g_has_frame && (RemoteIdleMs() < kRemoteTimeoutMs);
}
