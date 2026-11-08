/**
 *******************************************************************************
 * @file      : pid.hpp
 * @brief     : 通用单轴 PID 控制器（考核第 4 项）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-04      Hello World     1. 位置式 PID + 条件积分抗饱和 +
 *                                                 测量值微分 + 角度回绕 + 微分滤波
 *******************************************************************************
 * @attention :
 *  【为什么一个类要能服务三处】
 *    - 舵机（GM6020）角度位置环：输入 rad，输出 电流 A
 *    - 轮电机（M3508）速度环：  输入 rad/s，输出 电流 A
 *    - 云台（DM-J4310）位置环： 输入 rad，输出 力矩 N·m
 *  三处的数学完全相同，只有量纲/限幅/是否角度回绕不同。考核要求"PID 写成一个
 *  类"，而"良好"档看的是"模块化封装清晰"——写三个长得几乎一样的类，就是把
 *  同一个 bug 修三遍的开始。
 *
 *  【为什么放在 RobotComponents/（两板共享）】
 *    底盘板要它、云台板也要它。放各自目录会逼出两份 .cpp；两板行为一旦出现细微
 *    差别（比如一板忘了限幅），联调时你只会在实车上看到"云台比底盘抖"。
 *
 *  【为什么不用增量式】
 *    增量式省掉积分累加，但它把"积分限幅"变成隐性约束、且每次输出依赖上一次输出，
 *    急停/切换工作模式时要做额外处理才能归零。位置式的 integral_ 是显式状态，
 *    reset() 一下就是干净的——本任务第 6 项急停要求"全部电机指令归零"，位置式
 *    更贴合这个需求。
 *
 *  【本文件可以脱离硬件在电脑上验证】
 *    纯数学，不依赖 HAL。见 pc_test/test_pid.cpp：喂阶跃/正弦，检查超调、稳态
 *    误差、饱和时的积分行为。这一层的正确性不需要等上车。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#ifndef ROBOT_COMPONENTS_PID_HPP_
#define ROBOT_COMPONENTS_PID_HPP_

#include <cmath>
#include <cstdint>

namespace hello_world {
namespace pid {

/**
 * @brief 单轴位置式 PID。
 * @note  非线程安全：只在 1 ms 主任务里被单线程调用（见 Task/main_task.cpp）。
 *        若将来要在 CAN 中断里用，必须先加 ENTER_CRITICAL() 保护。
 */
class Pid {
 public:
  struct Params {
    /* ---- 增益 ----
     * 增益在这里只是"容器"，真实数值写在各板的 ins_config.hpp。原因：这些数
     * 只有上车才定得下来，集中到一张表，现场整定时的 diff 才只有一处。 */
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    /* ---- 积分限幅（作用于积分"项"本身，单位同输出）----
     * 为什么不能省：堵转/顶限位时误差长期同向，integral_ 会一路累加到几万。
     * 误差反向后要先花同样长的时间把它泄掉，外在表现就是"大幅超调 + 回得慢"。
     * 设 0 表示不限幅（不推荐，仅留给调试）。 */
    float integral_limit = 0.0f;

    /* ---- 输出限幅（单位 = setInput() 的输入域）----
     * kCurr → 电流 A（M3508 上限 20 A，GM6020 见其手册）
     * kTorq → 力矩 N·m（DM-J4310 上限 7.5 N·m）
     * 台架阶段先用小值（ins_config.hpp 的 bench_* 段），确认方向后再放开。 */
    float output_limit = 0.0f;

    /* ---- 微分作用对象 ----
     * true（默认）: D = -d(measured)/dt。
     * 为什么默认这样：目标值阶跃时 d(error) 是一个冲激，输出瞬间打满再回落，
     * 电机"抖一下"。云台期望角由遥控映射而来、经常阶跃，必须用测量值微分。
     * 代价：反馈噪声被放大，所以要配 deriv_filter。 */
    bool diff_on_measurement = true;

    /* ---- 角度回绕 ----
     * true: 误差先回绕到 [-π, π)。
     * 为什么必需（第 8 项头号坑）：DM-J4310 用 kNegPiToPosPi 角度域。目标 +179°、
     * 实测 -179°（等价于 +181°，其实已越过目标 2°）时，直接相减得 +358°，云台会
     * 几乎转一整圈；回绕后得到 -2°，反向退 2° 就到。 */
    bool angle_wrap = false;

    /* ---- 微分低通保持系数 [0,1]，0=不滤波 ----
     * 差分项是四个环节里最容易被反馈噪声打爆的：GM6020 角度反馈 12 bit，在 1 ms
     * 周期上做差分，量化噪声会被放大成很大的毛刺。0.2 是一个保守起点。 */
    float deriv_filter_keep = 0.2f;
  };

  Pid(void) = default;

  /**
   * @brief 初始化：存参数并清空全部内部状态。
   * @note  切换工作模式（分离↔随动↔小陀螺）后应当对受影响的环调用 reset()，
   *        否则旧模式的 integral_ 会作为扰动带进新模式。
   */
  void init(const Params &params) {
    params_ = params;
    reset();
  }

  /** @brief 运行时改增益（不 reset，保留当前积分），用于台架整定。 */
  void set_params(const Params &params) { params_ = params; }

  const Params &params(void) const { return params_; }

  /** @brief 清空全部内部状态：积分、微分滤波态、上一次的测量/误差。 */
  void reset(void) {
    integral_ = 0.0f;
    deriv_state_ = 0.0f;
    last_measured_ = 0.0f;
    last_error_ = 0.0f;
    output_ = 0.0f;
    primed_ = false;
    saturated_ = false;
  }

  /**
   * @brief 计算一次输出。
   * @param dt       本次调用与上次的间隔，单位 s（1 ms 任务里传 0.001f）。
   *                 用真实 dt 而不是常数，是为了让"任务被别的东西拖慢"时增益
   *                 不至于整体失真；但它不替代"任务必须周期执行"这个前提。
   * @param target   期望值（量纲与 measured 相同）
   * @param measured 反馈值
   * @retval 控制量，已按 output_limit 限幅
   */
  float update(float target, float measured, float dt) {
    /* dt 非法时保持上一次输出，而不是算出一个巨大的微分。
     * 这条不是防御性冗余：GetTickMs() 在 49.7 天会绕回，绕回那一拍 dt<=0。 */
    if (!(dt > 0.0f)) {
      return output_;
    }

    float error = target - measured;
    if (params_.angle_wrap) {
      error = WrapPi(error);
    }

    /* 第一拍只建立基准，不产生微分——否则 last_measured_ 的初值 0 会被当成
     * 一次真实的巨大跳变。 */
    if (!primed_) {
      last_measured_ = measured;
      last_error_ = error;
      deriv_state_ = 0.0f;
      primed_ = true;
    }

    /* ---------------- 微分项 ---------------- */
    float deriv_raw = params_.diff_on_measurement
                          ? -((measured - last_measured_) / dt)
                          : ((error - last_error_) / dt);
    last_measured_ = measured;
    last_error_ = error;

    float keep = params_.deriv_filter_keep;
    if (keep < 0.0f) keep = 0.0f;
    if (keep > 1.0f) keep = 1.0f;
    /* 一阶低通：新态 = 旧态·keep + 采样·(1-keep) */
    deriv_state_ += (1.0f - keep) * (deriv_raw - deriv_state_);

    /* ---------------- 积分项（条件积分抗饱和）---------------- */
    float integral_trial = integral_ + error * dt;
    if (params_.integral_limit > 0.0f) {
      integral_trial = Clamp(integral_trial, params_.integral_limit);
    }

    /* 先试探算输出，看它是否撞限幅：
     *   没饱和        → 接受积分
     *   饱和且误差仍在推它更深 → 拒绝（回滚），这是抗饱和的关键一步
     *   饱和但误差已反向      → 接受，让积分能泄掉，否则卡在限幅上出不来 */
    float out_open =
        params_.kp * error + params_.ki * integral_trial + params_.kd * deriv_state_;
    bool saturated = Saturates(out_open);

    if (!saturated || (error * out_open < 0.0f)) {
      integral_ = integral_trial;
    }
    saturated_ = saturated;

    /* ---------------- 输出 ---------------- */
    output_ = params_.kp * error + params_.ki * integral_ + params_.kd * deriv_state_;
    output_ = Clamp(output_, params_.output_limit);
    return output_;
  }

  /* ---------------- 调试访问（RTT 打印用）---------------- */
  float integral(void) const { return integral_; }
  float derivative_state(void) const { return deriv_state_; }
  float last_output(void) const { return output_; }
  bool saturated(void) const { return saturated_; }

  /* ---------------- 工具：给运动学与云台限位复用 ---------------- */
  /**
   * @brief 本类自带的 π 常量。
   * @note  为什么不复用 base.hpp 的 PI 宏、也不用 <cmath> 的 M_PI：
   *          - M_PI 不是标准 C++（GCC 靠 GNU 扩展、MSVC 要 _USE_MATH_DEFINES），
   *            本文件还要在 pc_test 里用 MSVC 编译，不赌编译器宽宏。
   *          - base.hpp 的 PI 是 float 且会把 allocator/system/HAL 间接层一起
   *            拉进来。PID 保持**零外部依赖**是刻意的：它在电脑上能单独编译、
   *            单独跑测试，这是第 4 项今晚就能拿到证据的前提。
   */
  static constexpr float kPi = 3.141592653589793f;

  /** @brief 把任意角度回绕到 [-π, π)。 */
  static float WrapPi(float angle) {
    /* 用 fmod 而不是"±2π 循环加减"：后者在角度很大（多圈累计）时要循环很多次，
     * 而且循环次数不确定，不适合出现在 1 ms 硬实时任务里。 */
    float a = std::fmod(angle + kPi, 2.0f * kPi);
    if (a < 0.0f) {
      a += 2.0f * kPi;
    }
    return a - kPi;
  }

  /** @brief 对称限幅。limit<=0 时原值返回（= 不限幅）。 */
  static float Clamp(float value, float limit) {
    if (limit <= 0.0f) {
      return value;
    }
    if (value > limit) {
      return limit;
    }
    if (value < -limit) {
      return -limit;
    }
    return value;
  }

 private:
  bool Saturates(float out_open) const {
    if (params_.output_limit <= 0.0f) {
      return false;
    }
    return (out_open > params_.output_limit) || (out_open < -params_.output_limit);
  }

  Params params_;

  /* 内部状态：全部有初值，构造后即可安全调用（未 init 时增益为 0 → 输出 0，
   * 这是一个刻意的"失效安全"设计：忘记 init 的后果是"不动"而不是"乱动"。 */
  float integral_ = 0.0f;
  float deriv_state_ = 0.0f;
  float last_measured_ = 0.0f;
  float last_error_ = 0.0f;
  float output_ = 0.0f;
  bool primed_ = false;
  bool saturated_ = false;
};

}  // namespace pid
}  // namespace hello_world

#endif /* ROBOT_COMPONENTS_PID_HPP_ */
