/**
 *******************************************************************************
 * @file      : test_pid.cpp
 * @brief     : 主机端 PID 验证台（考核第 4 项的证据）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版：阶跃/饱和/回绕/边界
 *******************************************************************************
 * @attention :
 *  本文件不属于任何一板的 CMake glob 路径（在 pc_test/ 下），因此不会被编进固件。
 *  它唯一的作用是：在不接硬件的前提下，证明 RobotComponents/component/pid.hpp
 *  的行为与设计意图一致。
 *
 *  被控对象用一阶惯性环节近似（时间常数 50 ms），对应"电流输入 → 转速输出"
 *  这类实际环节的简化模型。所有时间步长固定 1 ms，与主任务周期一致。
 *
 *  编译：见同目录 build_and_run.bat（调用 MSVC 的 vcvars64）
 *******************************************************************************
 */

#include <cmath>
#include <cstdio>

#include "../RobotComponents/component/pid.hpp"

namespace {

using hello_world::pid::Pid;

constexpr float kDt = 0.001f;      // 1 ms，与 htim6 主任务周期一致
constexpr float kPiF = 3.14159265358979323846f;

int g_pass = 0;
int g_fail = 0;

void Check(bool cond, const char *name, const char *detail) {
  if (cond) {
    ++g_pass;
    std::printf("  [PASS] %-38s %s\n", name, detail);
  } else {
    ++g_fail;
    std::printf("  [FAIL] %-38s %s\n", name, detail);
  }
}

/** 一阶惯性环节：y' = (K*u - y) / tau */
class Plant {
 public:
  explicit Plant(float tau = 0.05f, float gain = 1.0f) : tau_(tau), gain_(gain) {}
  float Step(float u, float dt) {
    y_ += (gain_ * u - y_) / tau_ * dt;
    return y_;
  }
  void Reset(void) { y_ = 0.0f; }
  float y(void) const { return y_; }

 private:
  float tau_;
  float gain_;
  float y_ = 0.0f;
};

struct StepMetrics {
  float overshoot_pct;  // 相对目标值的最大超调（%）
  float settle_ms;      // 首次进入 ±2% 带且此后再未离开的时间
  float sse;            // 末态误差 target - y(末)
};

StepMetrics RunStep(Pid &pid, Plant &plant, float target, float seconds) {
  const int n = static_cast<int>(seconds / kDt);
  plant.Reset();
  pid.reset();

  float peak = -1.0e9f;
  float last_y = 0.0f;
  int enter_band = -1;
  int leave_band = -1;
  const float band = 0.02f * std::fabs(target);

  for (int i = 0; i < n; ++i) {
    const float u = pid.update(target, plant.y(), kDt);
    const float y = plant.Step(u, kDt);
    if (y > peak) peak = y;
    if (enter_band < 0 && std::fabs(y - target) <= band) enter_band = i;
    if (enter_band >= 0 && std::fabs(y - target) > band) {
      enter_band = -1;  // 又出带了，重新计
    } else if (enter_band >= 0 && leave_band < enter_band) {
      leave_band = enter_band;
    }
    last_y = y;
  }

  StepMetrics m;
  m.overshoot_pct = (peak - target) / target * 100.0f;
  m.settle_ms = (leave_band < 0) ? -1.0f : static_cast<float>(leave_band) * kDt * 1000.0f;
  m.sse = target - last_y;
  return m;
}

// ---------------------------------------------------------------- 用例

/** 用例 1：只有 P，必然存在稳态误差——这解释了为什么需要 I 项。 */
void TestProportionalLeavesSteadyStateError(void) {
  std::printf("\n[1] P-only step: steady-state error must be non-zero\n");
  Pid pid;
  Pid::Params p;
  p.kp = 2.0f;
  p.ki = 0.0f;
  p.kd = 0.0f;
  p.output_limit = 0.0f;  // 不限幅
  pid.init(p);

  Plant plant;
  const StepMetrics m = RunStep(pid, plant, 10.0f, 2.0f);
  std::printf("      overshoot=%.2f%%  sse=%.3f\n", static_cast<double>(m.overshoot_pct),
              static_cast<double>(m.sse));
  char buf[64];
  std::snprintf(buf, sizeof(buf), "sse=%.3f (expect > 0.05)", static_cast<double>(m.sse));
  Check(m.sse > 0.05f, "steady-state error present", buf);
}

/** 用例 2：加 I 后稳态误差应被消掉，且超调可控。 */
void TestIntegralRemovesSteadyStateError(void) {
  std::printf("\n[2] PI step: integral removes steady-state error\n");
  Pid pid;
  Pid::Params p;
  p.kp = 2.0f;
  p.ki = 40.0f;
  p.kd = 0.0f;
  p.integral_limit = 50.0f;
  p.output_limit = 200.0f;
  pid.init(p);

  Plant plant;
  const StepMetrics m = RunStep(pid, plant, 10.0f, 3.0f);
  std::printf("      overshoot=%.2f%%  settle=%s  sse=%.4f\n",
              static_cast<double>(m.overshoot_pct),
              (m.settle_ms < 0.0f) ? "never" : "reached",
              static_cast<double>(m.sse));
  char buf[80];
  std::snprintf(buf, sizeof(buf), "sse=%.5f (expect < 0.01)", static_cast<double>(m.sse));
  Check(std::fabs(m.sse) < 0.01f, "steady-state error ~ 0", buf);
  Check(m.overshoot_pct < 30.0f, "overshoot < 30%", "");
}

/**
 * 用例 3a：饱和时"条件积分抗饱和"必须拒绝积分。
 * 输出撞限幅、且误差仍在把输出往同一方向推 → 每一拍都拒绝累加，积分应保持 0。
 * 这正是"先试探算输出再决定是否接受积分"那个分支的意义。
 */
void TestConditionalAntiWindup(void) {
  std::printf("\n[3a] saturated + error pushes deeper -> integral rejected\n");

  Pid pid;
  Pid::Params p;
  p.kp = 1.0f;
  p.ki = 20.0f;
  p.output_limit = 1.0f;     // 输出只能到 1，target=100 必然饱和
  p.integral_limit = 50.0f;  // 给足余量，排除了"被硬限幅"这个解释
  pid.init(p);

  Plant plant;
  for (int i = 0; i < 1000; ++i) {  // 持续 1 s 饱和
    pid.update(100.0f, plant.y(), kDt);
    plant.Step(pid.last_output(), kDt);
  }

  const float integ = pid.integral();
  std::printf("      after 1000 saturated steps: integral=%.6f  output=%.3f\n",
              static_cast<double>(integ), static_cast<double>(pid.last_output()));

  char buf[80];
  std::snprintf(buf, sizeof(buf), "integral=%.6f (expect 0)", static_cast<double>(integ));
  Check(integ == 0.0f, "integral never accumulates while saturated", buf);
  Check(pid.saturated(), "output pinned at limit", "");
}

/**
 * 用例 3b：不饱和时，integral_limit 把积分项本身钉在上限。
 * 用 gain=0 的假对象让误差恒定，从而把"积分项"从"控制效果"里隔离出来单独观察。
 */
void TestIntegralLimit(void) {
  std::printf("\n[3b] unsaturated -> integral_limit caps the integral term\n");

  const float target = 10.0f;

  Pid free_run;
  Pid::Params a;
  a.kp = 0.0f;               // 只看积分项
  a.ki = 1.0f;
  a.output_limit = 0.0f;     // 不限输出 → 永不饱和 → 条件抗饱和不介入
  a.integral_limit = 0.0f;   // 不限积分
  free_run.init(a);

  Pid clamped;
  Pid::Params b = a;
  b.integral_limit = 5.0f;
  clamped.init(b);

  Plant stub(0.05f, 0.0f);  // gain=0：输出不影响反馈，误差恒为 target

  const int n = 2000;  // 2000 × 10 × 0.001 = 20
  for (int i = 0; i < n; ++i) {
    free_run.update(target, stub.y(), kDt);
    clamped.update(target, stub.y(), kDt);
  }

  const float i_free = free_run.integral();
  const float i_clamped = clamped.integral();
  std::printf("      integral(unlimited)=%.3f  integral(clamped)=%.3f\n",
              static_cast<double>(i_free), static_cast<double>(i_clamped));

  char buf[80];
  std::snprintf(buf, sizeof(buf), "free=%.2f (expect ~20)", static_cast<double>(i_free));
  Check(std::fabs(i_free - 20.0f) < 0.1f, "unlimited integral grows linearly", buf);
  std::snprintf(buf, sizeof(buf), "clamped=%.3f (expect 5)", static_cast<double>(i_clamped));
  Check(std::fabs(i_clamped - 5.0f) < 1.0e-4f, "clamped integral pinned at limit", buf);
}

/**
 * 用例 4：角度回绕。
 * 注意符号：+179° 减去 -179° 得 +358°，而 -179° 等价于 +181°，其实已经越过目标
 * 2°，所以回绕后的误差是 **-2°**（不是 +2°）。不回绕则得到 +358°，云台会几乎
 * 转一整圈——这才是第 8 项真正会踩的坑。
 */
void TestAngleWrap(void) {
  std::printf("\n[4] angle wrap: +179 deg target vs -179 deg measured\n");
  const float target = 179.0f / 180.0f * kPiF;
  const float measured = -179.0f / 180.0f * kPiF;

  Pid wrap;
  Pid::Params p;
  p.kp = 1.0f;
  p.angle_wrap = true;
  wrap.init(p);
  const float u_wrap = wrap.update(target, measured, kDt);

  Pid nowrap;
  Pid::Params q;
  q.kp = 1.0f;
  q.angle_wrap = false;
  nowrap.init(q);
  const float u_nowrap = nowrap.update(target, measured, kDt);

  std::printf("      wrapped=%.4f rad (-2 deg)   unwrapped=%.4f rad (+358 deg)\n",
              static_cast<double>(u_wrap), static_cast<double>(u_nowrap));

  char buf[88];
  std::snprintf(buf, sizeof(buf), "wrapped=%.4f (expect ~-0.0349)", static_cast<double>(u_wrap));
  Check(std::fabs(u_wrap + 2.0f / 180.0f * kPiF) < 1.0e-3f, "wrapped error is -2 deg", buf);
  std::snprintf(buf, sizeof(buf), "unwrapped=%.4f (expect ~+6.2483)", static_cast<double>(u_nowrap));
  Check(std::fabs(u_nowrap - 358.0f / 180.0f * kPiF) < 1.0e-3f, "unwrapped error is +358 deg", buf);

  // 回绕后的绝对值必须 <= pi
  bool all_in_range = true;
  for (int deg = -720; deg <= 720; deg += 7) {
    const float x = static_cast<float>(deg) / 180.0f * kPiF;
    const float w = Pid::WrapPi(x);
    if (w < -kPiF - 1.0e-4f || w >= kPiF + 1.0e-4f) all_in_range = false;
  }
  Check(all_in_range, "WrapPi output always in [-pi, pi)", "");
}

/** 用例 5：dt<=0 时保持上一拍输出，不produce巨大微分。 */
void TestNonPositiveDt(void) {
  std::printf("\n[5] dt <= 0: hold last output\n");
  Pid pid;
  Pid::Params p;
  p.kp = 1.0f;
  p.kd = 1.0f;
  pid.init(p);

  const float u1 = pid.update(1.0f, 0.0f, kDt);
  const float u2 = pid.update(5.0f, 0.0f, 0.0f);   // 非法 dt
  const float u3 = pid.update(5.0f, 0.0f, -kDt);   // 非法 dt
  std::printf("      u(valid)=%.4f  u(dt=0)=%.4f  u(dt<0)=%.4f\n",
              static_cast<double>(u1), static_cast<double>(u2), static_cast<double>(u3));
  Check(u2 == u1 && u3 == u1, "output held on invalid dt", "");
}

/** 用例 6：未 init 时增益全 0 → 输出恒 0（失效安全）。 */
void TestFailSafeBeforeInit(void) {
  std::printf("\n[6] not initialised: output must stay 0\n");
  Pid pid;
  float max_abs = 0.0f;
  for (int i = 0; i < 100; ++i) {
    const float u = pid.update(10.0f, 0.0f, kDt);
    if (std::fabs(u) > max_abs) max_abs = std::fabs(u);
  }
  std::printf("      max|u| = %.6f\n", static_cast<double>(max_abs));
  Check(max_abs == 0.0f, "output is zero before init", "");
}

}  // namespace

int main(void) {
  std::printf("==================================================\n");
  std::printf(" PID host-side verification (plant: 1st order, tau=50ms)\n");
  std::printf("==================================================\n");

  TestProportionalLeavesSteadyStateError();
  TestIntegralRemovesSteadyStateError();
  TestConditionalAntiWindup();
  TestIntegralLimit();
  TestAngleWrap();
  TestNonPositiveDt();
  TestFailSafeBeforeInit();

  std::printf("\n--------------------------------------------------\n");
  std::printf(" SUMMARY: %d passed, %d failed\n", g_pass, g_fail);
  std::printf("--------------------------------------------------\n");
  return (g_fail == 0) ? 0 : 1;
}
