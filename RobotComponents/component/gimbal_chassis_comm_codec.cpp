/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm_codec.cpp
 * @brief     :双板通讯帧编解码实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal_chassis_comm_codec.hpp"

#include <cmath>

namespace hello_world {
namespace comm {

namespace {

constexpr float kAngleLimit = 3.2767f;  ///< 32767 / 10000，int16 可表示的最大角

/** 角度 → int16。夹在可表示范围内，避免溢出回绕成反向的大角度。 */
int16_t AngleToRaw(float angle) {
  if (angle > kAngleLimit) {
    angle = kAngleLimit;
  } else if (angle < -kAngleLimit) {
    angle = -kAngleLimit;
  }
  const float scaled = angle * kAngleScale;
  const float rounded = (scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f);
  return static_cast<int16_t>(rounded);
}

/** int16 → 角度 */
float RawToAngle(int16_t raw) { return static_cast<float>(raw) / kAngleScale; }

void PutInt16(uint8_t *p, int16_t v) {
  const uint16_t u = static_cast<uint16_t>(v);
  p[0] = static_cast<uint8_t>(u >> 8);  // 大端：高字节在前
  p[1] = static_cast<uint8_t>(u & 0xFF);
}

int16_t GetInt16(const uint8_t *p) {
  const uint16_t u = static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) |
                                           static_cast<uint16_t>(p[1]));
  return static_cast<int16_t>(u);
}

uint8_t XorChecksum(const uint8_t *p, int n) {
  uint8_t s = 0;
  for (int i = 0; i < n; ++i) {
    s = static_cast<uint8_t>(s ^ p[i]);
  }
  return s;
}

}  // namespace

void PackFrame(const Frame &frame, uint8_t seq, uint8_t out[8]) {
  uint8_t flags = 0;
  if (frame.online != 0) {
    flags = static_cast<uint8_t>(flags | 0x01);
  }
  if (frame.ready != 0) {
    flags = static_cast<uint8_t>(flags | 0x02);
  }

  out[0] = flags;
  out[1] = static_cast<uint8_t>(((frame.work_mode & 0x0F) << 4) |
                                (frame.ctrl_mode & 0x0F));
  PutInt16(&out[2], AngleToRaw(frame.yaw));
  PutInt16(&out[4], AngleToRaw(frame.pitch));
  out[6] = seq;
  out[7] = XorChecksum(out, 7);
}

bool UnpackFrame(const uint8_t in[8], Frame *out) {
  if (out == nullptr) {
    return false;
  }
  if (XorChecksum(in, 7) != in[7]) {
    return false;  // 校验失败：不写 out，避免半更新
  }

  Frame tmp;
  tmp.online = static_cast<uint8_t>(in[0] & 0x01);
  tmp.ready = static_cast<uint8_t>((in[0] >> 1) & 0x01);
  tmp.work_mode = static_cast<uint8_t>((in[1] >> 4) & 0x0F);
  tmp.ctrl_mode = static_cast<uint8_t>(in[1] & 0x0F);
  tmp.yaw = RawToAngle(GetInt16(&in[2]));
  tmp.pitch = RawToAngle(GetInt16(&in[4]));

  *out = tmp;
  return true;
}

}  // namespace comm
}  // namespace hello_world
