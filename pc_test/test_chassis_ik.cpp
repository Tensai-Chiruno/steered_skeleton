/**
 *******************************************************************************
 * @file      : test_chassis_ik.cpp
 * @brief     : 主机端舵向轮逆解验证（考核第 5 项的证据）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 * @attention :
 *  逆解是纯数学，不依赖 HAL，所以能在电脑上判定对错。这是本题最该先做完的部分：
 *  它决定车走不走直线，而实车上「走斜了」很难归因到具体一行代码。
 *
 *  重点不是把公式算对（公式只有三行），而是两个边界：
 *    - 180° 二义性：舵角不能甩超过 90°
 *    - 零速：atan2(0,0) 不能产生随机舵角
 *******************************************************************************
 */

#include <cmath>
#include <cstdio>

#include "../Chassis/RobotModules/component/chassis_ik_solver.hpp"

namespace {

using hello_world::chassis_ik_solver::ChassisIkSolver;

constexpr float kPi = ChassisIkSolver::kPi;
constexpr float kDeg = 180.0f / kPi;

int g_pass = 0;
int g_fail = 0;

void Check(bool cond, const char *name, const char *detail) {
  if (cond) {
    ++g_pass;
    std::printf("  [PASS] %-42s %s\n", name, detail);
  } else {
    ++g_fail;
    std::printf("  [FAIL] %-42s %s\n", name, detail);
  }
}

bool Near(float a, float b, float tol = 1.0e-3f) {
  return std::fabs(a - b) <= tol;
}

ChassisIkSolver MakeSolver(float max_speed = 0.0f) {
  ChassisIkSolver s;
  ChassisIkSolver::Params p;
  p.wheel_radius = 0.066f;
  p.half_wheelbase = 0.167f;
  p.half_track = 0.167f;
  p.max_wheel_speed = max_speed;
  s.init(p);
  return s;
}

const char *kName[4] = {"LF", "LB", "RB", "RF"};

void Print(const char *tag, const ChassisIkSolver::WheelState w[4]) {
  std::printf("      %s:", tag);
  for (int i = 0; i < 4; ++i) {
    std::printf("  %s %6.1f°/%6.2f", kName[i],
                static_cast<double>(w[i].angle * kDeg),
                static_cast<double>(w[i].speed));
  }
  std::printf("\n");
}

/** 1. 纯前进：四轮舵角都应指向正前方（0°），轮速 = vx / R */
void TestForward(void) {
  std::printf("\n[1] pure forward: all steer angles 0, speed = vx/R\n");
  ChassisIkSolver s = MakeSolver();
  ChassisIkSolver::WheelState w[4];
  s.solve(1.0f, 0.0f, 0.0f, w);
  Print("fwd vx=1", w);

  const float expect = 1.0f / 0.066f;
  bool ok = true;
  for (int i = 0; i < 4; ++i) {
    if (!Near(w[i].angle, 0.0f)) ok = false;
    if (!Near(w[i].speed, expect, 0.05f)) ok = false;
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "speed=%.2f (expect %.2f)", static_cast<double>(w[0].speed),
                static_cast<double>(expect));
  Check(ok, "forward is straight", buf);
}

/** 2. 纯左移：四轮都应该转到 +90°，轮速相同（舵向轮能横移，麦轮不能） */
void TestStrafeLeft(void) {
  std::printf("\n[2] pure strafe left: all steer angles +90 deg\n");
  ChassisIkSolver s = MakeSolver();
  ChassisIkSolver::WheelState w[4];
  s.solve(0.0f, 1.0f, 0.0f, w);
  Print("strafe vy=1", w);

  bool ok = true;
  for (int i = 0; i < 4; ++i) {
    if (!Near(w[i].angle, kPi / 2.0f)) ok = false;
  }
  Check(ok, "all wheels +90 deg", "");
}

/**
 * 判定「解算结果是否等于指令」的正确方式：比较轮心速度矢量，而不是直接比舵角。
 * 因为 (θ, ω) 与 (θ+π, −ω) 是同一个运动，解算器按「就近原则」可能给出后者。
 * 拿角度硬比会把正确解判成错误——这正是第一版测试写错的地方。
 */
bool WheelsMatchCommand(const ChassisIkSolver::WheelState w[4], float vx, float vy,
                        float gw, float tol = 1.0e-3f) {
  const float sign_x[4] = {+1.0f, -1.0f, -1.0f, +1.0f};
  const float sign_y[4] = {+1.0f, +1.0f, -1.0f, -1.0f};
  for (int i = 0; i < 4; ++i) {
    const float rx = sign_x[i] * 0.167f;
    const float ry = sign_y[i] * 0.167f;
    const float evx = vx - gw * ry;
    const float evy = vy + gw * rx;
    const float avx = w[i].speed * std::cos(w[i].angle) * 0.066f;
    const float avy = w[i].speed * std::sin(w[i].angle) * 0.066f;
    if (std::fabs(avx - evx) > tol || std::fabs(avy - evy) > tol) {
      return false;
    }
  }
  return true;
}

/** 3. 纯自转：解出的四个轮心速度必须与「绕中心旋转」的期望矢量一致 */
void TestSpin(void) {
  std::printf("\n[3] pure rotation w=+1: wheel velocities must match\n");
  ChassisIkSolver s = MakeSolver();
  ChassisIkSolver::WheelState w[4];
  s.solve(0.0f, 0.0f, 1.0f, w);
  Print("spin w=1", w);

  Check(WheelsMatchCommand(w, 0.0f, 0.0f, 1.0f), "velocity vectors match", "");

  /* 四轮轮速大小应当相等（|v| = w * |r|，四个 r 等长） */
  float lo = 1.0e9f, hi = 0.0f;
  for (int i = 0; i < 4; ++i) {
    const float a = std::fabs(w[i].speed);
    if (a < lo) lo = a;
    if (a > hi) hi = a;
  }
  char buf[80];
  std::snprintf(buf, sizeof(buf), "|speed| in [%.3f, %.3f]", static_cast<double>(lo),
                static_cast<double>(hi));
  Check(hi - lo < 1.0e-3f, "all four wheel speeds equal", buf);
}

/**
 * 4. ★ 180° 二义性。
 *    (a) 方向平滑绕一圈时，舵角每步变化必须 <= 90°；
 *    (b) 指令瞬时反向时，必须保持舵角不动、轮速取反，而不是让舵角转 180°。
 *    第二点才是这条规则真正的用武之地——平滑旋转时每步只差几度，用不上。
 */
void TestNoWhipAtPi(void) {
  std::printf("\n[4a] direction swept 360 deg: no steer jump > 90 deg/step\n");
  ChassisIkSolver s = MakeSolver();
  ChassisIkSolver::WheelState w[4];
  ChassisIkSolver::WheelState prev[4];

  const int kSteps = 360;
  s.solve(1.0f, 0.0f, 0.0f, prev);
  float worst = 0.0f;
  int worst_step = -1;

  for (int n = 1; n <= kSteps; ++n) {
    const float a = 2.0f * kPi * static_cast<float>(n) / static_cast<float>(kSteps);
    s.solve(std::cos(a), std::sin(a), 0.0f, w);
    for (int i = 0; i < 4; ++i) {
      const float d = std::fabs(ChassisIkSolver::wrapPi(w[i].angle - prev[i].angle));
      if (d > worst) {
        worst = d;
        worst_step = n;
      }
      prev[i] = w[i];
    }
  }

  char buf[96];
  std::snprintf(buf, sizeof(buf), "max step = %.1f deg (at step %d, limit 90)",
                static_cast<double>(worst * kDeg), worst_step);
  Check(worst <= kPi / 2.0f + 1.0e-3f, "steer never whips around", buf);

  std::printf("\n[4b] instant reversal: steer holds, wheel speed flips sign\n");
  ChassisIkSolver r = MakeSolver();
  ChassisIkSolver::WheelState fwd[4];
  ChassisIkSolver::WheelState bwd[4];
  r.solve(1.0f, 0.0f, 0.0f, fwd);
  r.solve(-1.0f, 0.0f, 0.0f, bwd);
  Print("cmd +1", fwd);
  Print("cmd -1", bwd);

  bool angle_held = true;
  bool speed_flipped = true;
  for (int i = 0; i < 4; ++i) {
    if (std::fabs(ChassisIkSolver::wrapPi(bwd[i].angle - fwd[i].angle)) > 1.0e-3f) {
      angle_held = false;
    }
    if (bwd[i].speed * fwd[i].speed >= 0.0f) {
      speed_flipped = false;
    }
  }
  Check(angle_held, "steer angle unchanged on reversal", "");
  Check(speed_flipped, "wheel speed sign flipped", "");
  Check(WheelsMatchCommand(bwd, -1.0f, 0.0f, 0.0f), "reversal is still correct motion", "");
}

/** 5. 零速：保持上次舵角、轮速为 0（atan2(0,0) 不能产生乱跳） */
void TestZeroKeepsAngle(void) {
  std::printf("\n[5] zero command: hold last angle, speed 0\n");
  ChassisIkSolver s = MakeSolver();
  ChassisIkSolver::WheelState w[4];
  s.solve(0.0f, 1.0f, 0.0f, w);  // 先掰到 +90°
  const float held = w[0].angle;
  s.solve(0.0f, 0.0f, 0.0f, w);  // 再给零速
  Print("zero cmd", w);

  bool ok = true;
  for (int i = 0; i < 4; ++i) {
    if (!Near(w[i].angle, held, 1.0e-4f)) ok = false;
    if (!Near(w[i].speed, 0.0f, 1.0e-4f)) ok = false;
  }
  Check(ok, "angle held, speed zeroed", "");
}

/** 6. 限速：超限时四轮按同一比例缩放，舵角不变（否则车会走斜线） */
void TestSpeedLimitScalesUniformly(void) {
  std::printf("\n[6] speed limit scales all wheels by one ratio\n");
  ChassisIkSolver free_s = MakeSolver(0.0f);
  ChassisIkSolver lim_s = MakeSolver(5.0f);

  ChassisIkSolver::WheelState a[4];
  ChassisIkSolver::WheelState b[4];
  // 走斜向 + 自转，让四个轮速本来就不相等
  free_s.solve(1.0f, 1.0f, 0.5f, a);
  lim_s.solve(1.0f, 1.0f, 0.5f, b);
  Print("unlimited", a);
  Print("limit=5", b);

  float ratio0 = 0.0f;
  bool same_ratio = true;
  bool angle_ok = true;
  float max_speed = 0.0f;
  for (int i = 0; i < 4; ++i) {
    const float f = std::fabs(a[i].speed);
    if (f > 1.0e-6f) {
      const float r = b[i].speed / a[i].speed;
      if (ratio0 == 0.0f) {
        ratio0 = r;
      } else if (!Near(r, ratio0, 1.0e-3f)) {
        same_ratio = false;
      }
    }
    if (!Near(a[i].angle, b[i].angle, 1.0e-4f)) angle_ok = false;
    const float bs = std::fabs(b[i].speed);
    if (bs > max_speed) max_speed = bs;
  }

  char buf[96];
  std::snprintf(buf, sizeof(buf), "ratio=%.3f, max speed after=%.2f (limit 5)",
                static_cast<double>(ratio0), static_cast<double>(max_speed));
  Check(same_ratio && ratio0 > 0.0f && ratio0 < 1.0f, "one scale factor for all wheels", buf);
  Check(angle_ok, "steer angles untouched by limiting", "");
  Check(max_speed <= 5.0f + 1.0e-3f, "limit respected", "");
}

}  // namespace

int main(void) {
  std::printf("==================================================\n");
  std::printf(" Chassis inverse kinematics - host verification\n");
  std::printf(" geometry: R=66mm, wheelbase=track=334mm, 4-wheel steer\n");
  std::printf("==================================================\n");

  TestForward();
  TestStrafeLeft();
  TestSpin();
  TestNoWhipAtPi();
  TestZeroKeepsAngle();
  TestSpeedLimitScalesUniformly();

  std::printf("\n--------------------------------------------------\n");
  std::printf(" SUMMARY: %d passed, %d failed\n", g_pass, g_fail);
  std::printf("--------------------------------------------------\n");
  return (g_fail == 0) ? 0 : 1;
}
