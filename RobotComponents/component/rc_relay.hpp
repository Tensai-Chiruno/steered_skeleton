/**
 *******************************************************************************
 * @file      :rc_relay.hpp
 * @brief     :遥控数据"云台→底盘"转发（单帧 8 字节，走 0x212）
 * @category  : 功能 | 考核第 2 项（DBUS）+ 第 8 项（双板通讯）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-17      Tensai-Chiruno  1. 首版：实车只有云台板接得到接收机
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#ifndef ROBOT_COMPONENTS_COMPONENT_RC_RELAY_HPP_
#define ROBOT_COMPONENTS_COMPONENT_RC_RELAY_HPP_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "tick.hpp"

namespace robot {

/* 为什么需要这一层（2026-11-17 现场实测）：
 *   接收机的信号线**只接到云台板**（底盘板 PC11 800 次采样零跳变、USART3 零字节），
 *   而底盘要动又必须有遥控数据（进 Working、急停判据都依赖它）。
 *   硬件不允许改线，所以由云台板把遥控数据打包成 8 字节走双板 CAN 转发给底盘。
 *
 * 帧格式（ID = 0x212，云台 → 底盘，1 kHz）：
 *   [0] 0xA5 帧头
 *   [1] online(bit0) | seq(bit4..7)
 *   [2] lv   [3] lh   [4] rv   [5] rh      —— 各 int8，范围 -100..100
 *   [6] s1(bit4..5) | s2(bit6..7)          —— 拨杆 1/2/3
 *   [7] 前 7 字节异或校验
 */
constexpr uint32_t kRcRelayTxId = 0x212u;
constexpr uint8_t kRcRelayHeader = 0xA5u;
/* 超过这个时间没收到转发帧就当作"转发来的遥控也失效"（与本地 DBUS 同一量级） */
constexpr uint32_t kRcRelayTimeoutMs = 300u;

/** @brief 转发中的遥控量（底盘侧使用；云台侧只写不读） */
struct RcRelayState {
  volatile uint8_t online = 0;   ///< 云台侧遥控是否在线
  volatile int8_t lv = 0;        ///< -100..100
  volatile int8_t lh = 0;
  volatile int8_t rv = 0;
  volatile int8_t rh = 0;
  volatile uint8_t s1 = 3;       ///< 1/2/3
  volatile uint8_t s2 = 3;
  volatile uint32_t last_rx_ms = 0;
  volatile bool has = false;     ///< 是否收到过至少一帧
};

/** @brief 全局唯一实例（两板各一份：云台只写、底盘只读） */
inline RcRelayState g_rc_relay;

/** @brief 转发来的遥控数据现在是否可用 */
inline bool RcRelayFresh(void) {
  if (!g_rc_relay.has) {
    return false;
  }
  return (hello_world::tick::GetTickMs() - g_rc_relay.last_rx_ms) < kRcRelayTimeoutMs;
}

/** @brief 把 -1..1 量化成 -100..100 */
inline int8_t QuantizeStick(float v) {
  float q = v * 100.0f;
  if (q > 100.0f) q = 100.0f;
  if (q < -100.0f) q = -100.0f;
  return static_cast<int8_t>(q);
}

/** @brief 把量化值还原成 -1..1 */
inline float DequantizeStick(int8_t q) { return static_cast<float>(q) / 100.0f; }

/** @brief 打包 8 字节（云台侧） */
inline void PackRcRelay(uint8_t online, float lv, float lh, float rv, float rh,
                        uint8_t s1, uint8_t s2, uint8_t seq, uint8_t out[8]) {
  out[0] = kRcRelayHeader;
  out[1] = static_cast<uint8_t>((online ? 1u : 0u) | ((seq & 0x0Fu) << 4));
  out[2] = static_cast<uint8_t>(QuantizeStick(lv));
  out[3] = static_cast<uint8_t>(QuantizeStick(lh));
  out[4] = static_cast<uint8_t>(QuantizeStick(rv));
  out[5] = static_cast<uint8_t>(QuantizeStick(rh));
  out[6] = static_cast<uint8_t>(((s1 & 0x03u) << 4) | ((s2 & 0x03u) << 6));
  uint8_t x = 0;
  for (uint8_t i = 0; i < 7u; ++i) {
    x ^= out[i];
  }
  out[7] = x;
}

/** @brief 解包 8 字节（底盘侧）；成功返回 true 并写 g_rc_relay */
inline bool UnpackRcRelay(const uint8_t in[8]) {
  if (in[0] != kRcRelayHeader) {
    return false;
  }
  uint8_t x = 0;
  for (uint8_t i = 0; i < 7u; ++i) {
    x ^= in[i];
  }
  if (x != in[7]) {
    return false;
  }
  g_rc_relay.online = static_cast<uint8_t>(in[1] & 0x01u);
  g_rc_relay.lv = static_cast<int8_t>(in[2]);
  g_rc_relay.lh = static_cast<int8_t>(in[3]);
  g_rc_relay.rv = static_cast<int8_t>(in[4]);
  g_rc_relay.rh = static_cast<int8_t>(in[5]);
  g_rc_relay.s1 = static_cast<uint8_t>((in[6] >> 4) & 0x03u);
  g_rc_relay.s2 = static_cast<uint8_t>((in[6] >> 6) & 0x03u);
  g_rc_relay.last_rx_ms = hello_world::tick::GetTickMs();
  g_rc_relay.has = true;
  return true;
}

}  // namespace robot

#endif /* ROBOT_COMPONENTS_COMPONENT_RC_RELAY_HPP_ */
