/**
 *******************************************************************************
 * @file      : pid.hpp
 * @brief     : 通用单轴 PID 控制器（考核第 3 项）
 * @category : 功能 | 考核第 3 项（PID 类）
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

    // 位置式 PID 的误差定义：期望 − 实测。正误差 = 需要往正方向推。
    float error = target - measured;
    // 角度环必须回绕：+179° 与 -179° 实际只差 2°，直接相减会得到 358°。
    if (params_.angle_wrap) {
      error = WrapPi(error);
    }

    /* 第一拍只建立基准，不产生微分——否则 last_measured_ 的初值 0 会被当成
     * 一次真实的巨大跳变。 */
    if (!primed_) {
      // 用第一拍的实测值当作"上一次"，这样本拍 d_meas = 0，微分项为 0。
      last_measured_ = measured;
      last_error_ = error;
      deriv_state_ = 0.0f;   // 滤波态也清零，避免从 0 开始缓慢爬升
      primed_ = true;        // 之后才允许真正计算微分
    }

    /* ---------------- 微分项 ---------------- */
    /* 角度环里 measured 会跨 ±pi（舵角、云台 pitch 都会），
     * 直接用 measured - last_measured_ 会在过零那一拍产生 ±2*pi 的假跳变，
     * 变成一脚巨大的 D 输出。测量差同样要回绕。 */
    float d_meas = measured - last_measured_;
    if (params_.angle_wrap) {
      d_meas = WrapPi(d_meas);
    }
    // 两种微分取法：
    //   diff_on_measurement = true  → -d(measured)/dt：目标阶跃不会打出冲激，
    //                                 代价是把反馈噪声也一起微分放大；
    //   diff_on_measurement = false → d(error)/dt   ：跟随更快，但目标一跳就抖。
    // 负号的作用：测量值上升时输出要减小，方向与"误差减小"一致。
    float deriv_raw = params_.diff_on_measurement
                          ? -(d_meas / dt)
                          : ((error - last_error_) / dt);
    // 先更新"上一次"，再做滤波与输出：保证下一拍的差分基于本拍实测。
    last_measured_ = measured;
    last_error_ = error;

    // 滤波系数做一次防御性夹取：配置写错成 -1 或 2 时不会把状态推飞。
    float keep = params_.deriv_filter_keep;
    if (keep < 0.0f) keep = 0.0f;
    if (keep > 1.0f) keep = 1.0f;
    /* 一阶低通：新态 = 旧态·keep + 采样·(1-keep) */
    deriv_state_ += (1.0f - keep) * (deriv_raw - deriv_state_);

    /* ---------------- 积分项（条件积分抗饱和）---------------- */
    // 先算出"如果接受本拍积分会变成多少"，但**先不写回**：
    // 下面要根据是否饱和来决定到底接不接受（条件积分 / 反算抗饱和）。
    float integral_trial = integral_ + error * dt;   // 积分状态的单位是"误差×秒"
    if (params_.integral_limit > 0.0f) {
      // 硬夹一次，防止长期同向误差把状态累到几万（泄起来要好几秒）。
      integral_trial = Clamp(integral_trial, params_.integral_limit);
    }

    /* 先试探算输出，看它是否撞限幅：
     *   没饱和        → 接受积分
     *   饱和且误差仍在推它更深 → 拒绝（回滚），这是抗饱和的关键一步
     *   饱和但误差已反向      → 接受，让积分能泄掉，否则卡在限幅上出不来 */
    // 用"试探积分"开环算一次输出，只是为了判断它会不会撞限幅。
    float out_open =
        params_.kp * error + params_.ki * integral_trial + params_.kd * deriv_state_;
    // output_limit <= 0 时 Saturates() 恒返回 false（= 不限幅）。
    bool saturated = Saturates(out_open);

    // 三种情况：
    //   1) 没饱和                        → 接受（正常积分）
    //   2) 饱和 且 误差与输出同号（还在往深处推）→ 拒绝，rollback 到旧积分
    //   3) 饱和 但 误差已反向（error*out_open<0）→ 接受，让积分能泄掉，
    //      否则会一直卡在限幅上出不来，脱限后表现为长时间超调
    if (!saturated || (error * out_open < 0.0f)) {
      integral_ = integral_trial;
    }
    saturated_ = saturated;   // 只作为观测量，供调试/状态灯使用

    /* ---------------- 输出 ---------------- */
    // 用**最终被接受的** integral_ 重算一次输出（不是直接用 out_open）：
    // 抗饱和回滚时，输出必须反映回滚后的积分值，否则回滚就白做了。
    output_ = params_.kp * error + params_.ki * integral_ + params_.kd * deriv_state_;
    // 最后再硬夹一次。output_limit 与执行器量纲一致（raw / A / N·m）。
    output_ = Clamp(output_, params_.output_limit);
    return output_;
  }

  /* ---------------- 调试访问（RTT 打印用）---------------- */
  /// 积分状态（单位"误差×秒"）。整定时用它看积分有没有一直涨。
  float integral(void) const { return integral_; }
  /// 滤波后的微分项，撞限位看它是否被噪声打爆。
  float derivative_state(void) const { return deriv_state_; }
  /// 上一次输出，用于"dt 非法时保持输出"。
  float last_output(void) const { return output_; }
  /// 上一拍是否撞了输出限幅（抗饱和是否被触发）。
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
    // 先把区间平移到 [0, 2π)：加 π 让 [-π,π) 变成 [0,2π)，再取模。
    float a = std::fmod(angle + kPi, 2.0f * kPi);
    // fmod 对负数返回负值（C++ 语义），补一次把结果拉回 [0, 2π)。
    if (a < 0.0f) {
      a += 2.0f * kPi;
    }
    // 再平移回 [-π, π)。
    return a - kPi;
  }

  /** @brief 对称限幅。limit<=0 时原值返回（= 不限幅）。 */
  static float Clamp(float value, float limit) {
    // limit <= 0 约定为"不限幅"。这是刻意的：出厂默认 limit=0，
    // 配合"增益全 0"，未整定时输出恒 0（失效安全）。
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
  /// 判断"开环输出是否撞限幅"。output_limit<=0 时视为不限幅，永不饱和。
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
