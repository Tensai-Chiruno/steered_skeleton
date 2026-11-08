/**
 *******************************************************************************
 * @file      : test_chassis_comm.cpp
 * @brief     : 主机端双板通讯帧编解码验证（考核第 9 项的证据）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 * @attention :
 *  只测编解码（纯函数），不测 GimbalChassisComm（它依赖 tick/HAL）。
 *  双板联调最常见的 bug 就是「一端偏移或缩放写错」，症状是数据看起来像对的、
 *  但整体差一个比例或错半个字节——那种问题在实车上极难归因，在电脑上却是几行断言。
 *******************************************************************************
 */

#include <cmath>
#include <cstdio>

#include "../RobotComponents/component/gimbal_chassis_comm_codec.hpp"

namespace {

using hello_world::comm::Frame;
using hello_world::comm::PackFrame;
using hello_world::comm::UnpackFrame;
using hello_world::comm::kAngleScale;

int g_pass = 0;
int g_fail = 0;

void Check(bool cond, const char *name, const char *detail) {
  if (cond) {
    ++g_pass;
    std::printf("  [PASS] %-44s %s\n", name, detail);
  } else {
    ++g_fail;
    std::printf("  [FAIL] %-44s %s\n", name, detail);
  }
}

void Dump(const char *tag, const uint8_t b[8]) {
  std::printf("      %s:", tag);
  for (int i = 0; i < 8; ++i) {
    std::printf(" %02X", b[i]);
  }
  std::printf("\n");
}

/** 1. 往返：任意合法值 encode 后 decode 应原值还原（精度 1/10000 rad） */
void TestRoundTrip(void) {
  std::printf("\n[1] round trip: encode then decode restores the frame\n");

  struct Case {
    float yaw;
    float pitch;
  } cases[] = {
      {0.0f, 0.0f},      {1.5708f, -0.7854f}, {-3.1415f, 3.1415f},
      {0.0001f, -0.0001f}, {2.0f, 0.5f},
  };

  bool ok = true;
  float worst = 0.0f;
  for (const Case &c : cases) {
    Frame src;
    src.online = 1;
    src.ready = 1;
    src.ctrl_mode = 1;
    src.work_mode = 2;
    src.yaw = c.yaw;
    src.pitch = c.pitch;

    uint8_t buf[8];
    PackFrame(src, 7, buf);

    Frame dst;
    if (!UnpackFrame(buf, &dst)) {
      ok = false;
      continue;
    }
    if (dst.online != 1 || dst.ready != 1 || dst.ctrl_mode != 1 || dst.work_mode != 2) {
      ok = false;
    }
    const float dy = std::fabs(dst.yaw - src.yaw);
    const float dp = std::fabs(dst.pitch - src.pitch);
    if (dy > worst) worst = dy;
    if (dp > worst) worst = dp;
    if (dy > 1.0f / kAngleScale || dp > 1.0f / kAngleScale) {
      ok = false;
    }
  }

  char buf2[96];
  std::snprintf(buf2, sizeof(buf2), "max error=%.6f rad (limit %.6f)",
                static_cast<double>(worst), static_cast<double>(1.0 / kAngleScale));
  Check(ok, "all cases restore within 1e-4 rad", buf2);
}

/** 2. 字节布局固定：钉死一份已知输入的期望字节，防偏移写错 */
void TestFixedLayout(void) {
  std::printf("\n[2] fixed layout: known frame -> known bytes\n");

  Frame f;
  f.online = 1;      // -> flags bit0
  f.ready = 1;       // -> flags bit1
  f.ctrl_mode = 1;   // -> byte1 低 4 位
  f.work_mode = 2;   // -> byte1 高 4 位
  f.yaw = 0.5f;      // 5000 = 0x1388
  f.pitch = -0.25f;  // -2500 = 0xF63C

  uint8_t buf[8];
  PackFrame(f, 0xA5, buf);
  Dump("bytes", buf);

  char d[96];
  std::snprintf(d, sizeof(d), "flags=0x%02X modes=0x%02X yaw=%02X%02X pitch=%02X%02X seq=%02X",
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
  Check(buf[0] == 0x03 && buf[1] == 0x21 && buf[2] == 0x13 && buf[3] == 0x88 &&
            buf[4] == 0xF6 && buf[5] == 0x3C && buf[6] == 0xA5,
        "byte layout matches spec", d);

  /* 校验字节 = 前 7 字节异或，自己再算一遍 */
  uint8_t x = 0;
  for (int i = 0; i < 7; ++i) {
    x = static_cast<uint8_t>(x ^ buf[i]);
  }
  Check(buf[7] == x, "checksum equals XOR of first 7", "");
}

/** 3. online / ready 两个标志位互不干扰 */
void TestFlagBits(void) {
  std::printf("\n[3] online / ready are independent bits\n");
  bool ok = true;
  for (int on = 0; on < 2; ++on) {
    for (int rd = 0; rd < 2; ++rd) {
      Frame f;
      f.online = static_cast<uint8_t>(on);
      f.ready = static_cast<uint8_t>(rd);
      uint8_t b[8];
      PackFrame(f, 0, b);
      Frame g;
      if (!UnpackFrame(b, &g)) {
        ok = false;
        continue;
      }
      if (g.online != on || g.ready != rd) {
        ok = false;
      }
    }
  }
  Check(ok, "all four combinations survive", "");
}

/** 4. 模式：工作模式在高 4 位、控制模式在低 4 位 */
void TestModes(void) {
  std::printf("\n[4] work_mode high nibble, ctrl_mode low nibble\n");
  bool ok = true;
  for (int w = 0; w <= 3; ++w) {
    for (int c = 0; c <= 1; ++c) {
      Frame f;
      f.work_mode = static_cast<uint8_t>(w);
      f.ctrl_mode = static_cast<uint8_t>(c);
      uint8_t b[8];
      PackFrame(f, 0, b);
      if (b[1] != static_cast<uint8_t>((w << 4) | c)) {
        ok = false;
      }
      Frame g;
      if (!UnpackFrame(b, &g) || g.work_mode != w || g.ctrl_mode != c) {
        ok = false;
      }
    }
  }
  Check(ok, "modes pack and unpack cleanly", "");
}

/** 5. ★ 校验失败：任一字节被篡改都必须被拒，且不写坏 out */
void TestChecksumRejectsCorruption(void) {
  std::printf("\n[5] corrupted byte must be rejected, output untouched\n");

  Frame f;
  f.online = 1;
  f.yaw = 1.0f;
  f.pitch = -1.0f;
  uint8_t good[8];
  PackFrame(f, 1, good);

  Frame sentinel;
  sentinel.yaw = 12.345f;  // 用于确认「失败时不写 out」
  sentinel.pitch = -6.789f;

  bool all_rejected = true;
  bool out_untouched = true;
  for (int pos = 0; pos < 8; ++pos) {
    for (uint8_t flip = 0x01; flip != 0; flip = static_cast<uint8_t>(flip << 1)) {
      uint8_t bad[8];
      for (int i = 0; i < 8; ++i) {
        bad[i] = good[i];
      }
      bad[pos] = static_cast<uint8_t>(bad[pos] ^ flip);

      Frame dst = sentinel;
      const bool ok = UnpackFrame(bad, &dst);
      /* 单比特翻转后若仍通过校验，说明校验太弱（XOR 对成对错误无力，
       * 但单比特必然改变校验值，所以这里应当全部被拒） */
      if (ok) {
        all_rejected = false;
      }
      if (dst.yaw != sentinel.yaw || dst.pitch != sentinel.pitch) {
        out_untouched = false;
      }
    }
  }
  Check(all_rejected, "all 64 single-bit corruptions rejected", "");
  Check(out_untouched, "output frame untouched on failure", "");
}

/** 6. ★ 角度限幅：超出 int16 表示范围时必须夹住，不能溢出回绕 */
void TestAngleClamp(void) {
  std::printf("\n[6] out-of-range angle clamps instead of wrapping\n");

  Frame f;
  f.yaw = 10.0f;    // 远超 3.2767
  f.pitch = -10.0f;
  uint8_t b[8];
  PackFrame(f, 0, b);

  Frame g;
  const bool ok = UnpackFrame(b, &g);
  std::printf("      yaw=10.0 -> %.4f rad ; pitch=-10.0 -> %.4f rad\n",
              static_cast<double>(g.yaw), static_cast<double>(g.pitch));

  Check(ok, "frame still valid", "");
  Check(g.yaw > 3.2f && g.yaw < 3.3f, "positive overflow clamped to ~3.2767", "");
  Check(g.pitch < -3.2f && g.pitch > -3.3f, "negative overflow clamped to ~-3.2767", "");
}

/** 7. 序列号按传入值原样落到 byte6 */
void TestSequence(void) {
  std::printf("\n[7] sequence byte carries through\n");
  Frame f;
  bool ok = true;
  for (int s = 0; s < 256; s += 17) {
    uint8_t b[8];
    PackFrame(f, static_cast<uint8_t>(s), b);
    if (b[6] != static_cast<uint8_t>(s)) {
      ok = false;
    }
  }
  Check(ok, "sequence byte intact", "");
}

}  // namespace

int main(void) {
  std::printf("==================================================\n");
  std::printf(" Dual-board frame codec - host verification\n");
  std::printf(" 8 bytes: flags | modes | yaw(i16) | pitch(i16) | seq | xor\n");
  std::printf("==================================================\n");

  TestRoundTrip();
  TestFixedLayout();
  TestFlagBits();
  TestModes();
  TestChecksumRejectsCorruption();
  TestAngleClamp();
  TestSequence();

  std::printf("\n--------------------------------------------------\n");
  std::printf(" SUMMARY: %d passed, %d failed\n", g_pass, g_fail);
  std::printf("--------------------------------------------------\n");
  return (g_fail == 0) ? 0 : 1;
}
