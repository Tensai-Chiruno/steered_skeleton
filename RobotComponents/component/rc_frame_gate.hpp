/**
 *******************************************************************************
 * @file      :rc_frame_gate.hpp
 * @brief     :DBUS 摇杆跳变闸门（错帧保护）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifndef ROBOT_COMPONENTS_COMPONENT_RC_FRAME_GATE_HPP_
#define ROBOT_COMPONENTS_COMPONENT_RC_FRAME_GATE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace hello_world {
namespace comm {

/** @brief 只取四个摇杆（拨轮是瞬跳量，刻意不纳入）。 */
struct RcSticks {
  float lv = 0.0f;
  float lh = 0.0f;
  float rv = 0.0f;
  float rh = 0.0f;
};

/**
 * @brief DBUS 错帧保护的纯逻辑部分。
 *
 * 背景：DBUS 帧**没有固定帧头**（前 11 bit 就是右摇杆通道），掉一个字节后
 * 接收缓冲会永久错位，而错位帧长度仍然是 18，`Size >= 18` 判不出来。
 * 一个错位的摇杆帧 = 一脚随机的车指令，但"摇杆单帧跳变"在物理上几乎不可能：
 * 143 Hz 采样下真摇杆从 −1 到 +1 至少要几十毫秒。
 *
 * 策略（与 interrupt 层无关，便于主机单测）：
 *   - 任一摇杆两帧之间跳变 > kJumpLimit → 这一帧不可信；
 *   - 不可信时**保持上一帧的值**（不清零，清零本身也是一脚跳变）；
 *   - 连续 kBadLimit 帧不可信 → shouldDropOnline()，由调用方按离线处理（现成急停）。
 */
class RcFrameGate {
 public:
  static constexpr float kJumpLimit = 0.45f;
  static constexpr uint32_t kBadLimit = 3;

  void reset(void) {
    primed_ = false;
    bad_streak_ = 0;
  }

  /** @return true = 这一帧可用（调用方应更新 rc_*_ 并刷新在线时间戳）。 */
  bool accept(const RcSticks &now) {
    if (primed_ && Jumped(prev_, now)) {
      ++bad_streak_;
      return false;  /* prev_ 不动 = 保持上一帧 */
    }
    prev_ = now;
    primed_ = true;
    bad_streak_ = 0;
    return true;
  }

  uint32_t badStreak(void) const { return bad_streak_; }
  bool shouldDropOnline(void) const { return bad_streak_ >= kBadLimit; }
  bool primed(void) const { return primed_; }

 private:
  /** 注意用 !(d <= lim)：这样 NaN 也会被判成"跳变"，而不是静默放过。 */
  static bool Jumped(const RcSticks &a, const RcSticks &b) {
    return !(Fabs(a.lv - b.lv) <= kJumpLimit) || !(Fabs(a.lh - b.lh) <= kJumpLimit) ||
           !(Fabs(a.rv - b.rv) <= kJumpLimit) || !(Fabs(a.rh - b.rh) <= kJumpLimit);
  }
  static float Fabs(float v) { return (v < 0.0f) ? -v : v; }

  RcSticks prev_{};
  uint32_t bad_streak_ = 0;
  bool primed_ = false;
};

}  // namespace comm
}  // namespace hello_world

#endif /* ROBOT_COMPONENTS_COMPONENT_RC_FRAME_GATE_HPP_ */
