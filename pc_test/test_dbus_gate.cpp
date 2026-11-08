/**
 *******************************************************************************
 * @file      :test_dbus_gate.cpp
 * @brief     :DBUS 错帧保护（RcFrameGate）主机侧测试
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-09      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* 为什么这个测试值得写：
 * DBUS 帧**没有固定帧头**（前 11 bit 就是右摇杆通道），掉 1 字节后接收缓冲会永久
 * 错位，而错位帧长度仍然是 18 字节，`Size >= 18` 完全判不出来。错位后的解码结果就是
 * 一脚随机的摇杆指令。这里用"单帧跳变"当判据，验证三件事：
 *   1. 正常帧与轻微抖动必须放行（不能把正常操作当成错帧）；
 *   2. 整体错位（等价于丢 1 字节）必须拒绝，且连续 3 帧判离线；
 *   3. 拨轮是瞬跳量，刻意**不**参与判据，它跳不能导致整帧被拒。
 *
 * 注意一个反直觉的点：只把某一个字节右移 1 位，通常只改变该通道的十几到几十个 LSB
 * （≈0.02~0.1 归一化），**达不到 0.45 的阈值**。真正致命的是整帧错位，所以测试按
 * "整体移位 1 字节"构造，这与现场丢字节的真实后果一致。
 */

#include <cstdio>
#include <cstring>

#include "../Chassis/inc/DT7.hpp"
#include "../RobotComponents/component/rc_frame_gate.hpp"

using hello_world::remote_control::DT7;
using hello_world::comm::RcFrameGate;
using hello_world::comm::RcSticks;

namespace {

int g_pass = 0;
int g_fail = 0;

void Check(bool ok, const char *name)
{
  if (ok) {
    ++g_pass;
    std::printf("  [PASS] %s\n", name);
  } else {
    ++g_fail;
    std::printf("  [FAIL] %s\n", name);
  }
}

RcSticks Sticks(const uint8_t frame[18])
{
  DT7 rc;
  rc.decode(frame);
  RcSticks s;
  s.lv = rc.rc_lv();
  s.lh = rc.rc_lh();
  s.rv = rc.rc_rv();
  s.rh = rc.rc_rh();
  return s;
}

/* 一条"看起来正常"的基准帧：六个通道字节给中段值，其余补 0 */
void BaseFrame(uint8_t out[18])
{
  std::memset(out, 0, 18);
  out[0] = 0x12;
  out[1] = 0x34;
  out[2] = 0x56;
  out[3] = 0x78;
  out[4] = 0x9A;
  out[5] = 0xBC;
}

void ShiftLeftOneByte(const uint8_t in[18], uint8_t out[18])
{
  /* 等价于"丢了最前面 1 个字节后按 18 字节重新对齐" */
  for (int i = 0; i < 17; ++i) {
    out[i] = in[i + 1];
  }
  out[17] = 0;
}

}  // namespace

int main(void)
{
  uint8_t base[18];
  BaseFrame(base);

  std::printf("---- DBUS 错帧保护 ----\n");

  /* 1) 首帧建立基准，必须放行 */
  {
    RcFrameGate gate;
    Check(gate.accept(Sticks(base)), "首帧建立基准并放行");
    Check(!gate.primed() == false, "首帧后 gate 已 primed");
    Check(gate.badStreak() == 0u, "首帧后 bad_streak = 0");
  }

  /* 2) 同一帧重复喂 → 不跳变，必须放行 */
  {
    RcFrameGate gate;
    gate.accept(Sticks(base));
    Check(gate.accept(Sticks(base)), "重复同一帧放行（无跳变）");
  }

  /* 3) 轻微抖动（每通道 ±0.05 以内）必须放行 */
  {
    RcFrameGate gate;
    RcSticks a = Sticks(base);
    gate.accept(a);
    RcSticks b = a;
    b.lv += 0.05f;
    b.lh -= 0.05f;
    b.rh += 0.04f;
    Check(gate.accept(b), "±0.05 抖动放行");
  }

  /* 4) 整体错位 1 字节（丢 1 字节）必须拒绝，且保持上一帧值 */
  {
    uint8_t bad[18];
    ShiftLeftOneByte(base, bad);
    RcSticks a = Sticks(base);
    RcSticks b = Sticks(bad);
    RcFrameGate gate;
    gate.accept(a);
    Check(gate.accept(b) == false, "整体错位 1 字节被拒绝");
    Check(gate.badStreak() == 1u, "错位后 bad_streak = 1");
    Check(gate.shouldDropOnline() == false, "仅 1 帧错位还不判离线");
    Check(gate.accept(b) == false, "第二帧错位仍被拒绝");
    Check(gate.accept(b) == false, "第三帧错位仍被拒绝");
    Check(gate.shouldDropOnline(), "连续 3 帧错位 → 判离线（走现成急停）");
    /* 恢复合法帧：应重新放行并把 streak 清零 */
    Check(gate.accept(a), "恢复合法帧重新放行");
    Check(gate.badStreak() == 0u, "恢复后 bad_streak 归零");
    Check(gate.shouldDropOnline() == false, "恢复后不再判离线");
  }

  /* 5) 拨轮单独变化不参与判据 */
  {
    uint8_t wheel_moved[18];
    std::memcpy(wheel_moved, base, 18);
    wheel_moved[16] = 0xFF;
    wheel_moved[17] = 0xFF;
    RcFrameGate gate;
    gate.accept(Sticks(base));
    Check(gate.accept(Sticks(wheel_moved)), "拨轮瞬跳不影响接受（拨轮不纳入判据）");
  }

  /* 6) 全 0 帧（接收机掉电/无信号时常见）→ 相对基准是大跳变，必须拒绝 */
  {
    uint8_t zeros[18];
    std::memset(zeros, 0, 18);
    RcFrameGate gate;
    gate.accept(Sticks(base));
    Check(gate.accept(Sticks(zeros)) == false, "全 0 帧被拒绝（不是合法操作）");
  }

  std::printf("--------------------------------------------------\n");
  std::printf(" SUMMARY: %d passed, %d failed\n", g_pass, g_fail);
  std::printf("--------------------------------------------------\n");
  return (g_fail == 0) ? 0 : 1;
}
