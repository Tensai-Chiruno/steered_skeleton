/**
 *******************************************************************************
 * @file      :chassis.hpp
 * @brief     :底盘运动模块（舵向轮逆解 + 8 个闭环）
 * @category : 功能 | 考核第 6 项（底盘闭环；含 1 处调试访问器）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *  V1.1.0      2026-11-09      Tensai-Chiruno  1. 按第二轮复查报告修 P0/P1
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "chassis_ik_solver.hpp"
#include "module_state.hpp"

namespace robot {

/**
 * @brief 底盘运动模块：把 (vx, vy, w) 变成 4 个舵角 + 4 个轮速，并闭 8 个环。
 *
 * 量纲契约（2026-11-18 修正后的唯一约定，改这里必须同时改 ins_motor.cpp / ins_pid.cpp）：
 *   - 两台电机的 input_type 都是 **kCurr（电流环）**；
 *   - 因此本模块的 PID 输出、限幅、送给 setInput() 的数值**全部是安培**；
 *   - 舵电机走 GM6020 电流帧 0x1FE（满量程 3 A），
 *     轮电机走 M3508 控制帧 0x200（满量程 20 A）。
 *   ⚠ 这张卡原来写的是 "kRaw + 满量程 25000/16384，若改 kCurr 必须把限幅换成安培"。
 *     2026-11-17 我把 input_type 改成了 kCurr、帧 ID 换成了 0x1FE，**但没有换限幅**，
 *     于是 steer_raw_limit=250 被 setInput() 当成 250 A、钳到 3 A 满电流，
 *     四台 GM6020 变成满电流开关控制：满力高频抖动 + 啸叫，且把限幅从 250 压到 60
 *     （看似降 4 倍）毫无作用 —— 因为两者都远超 3 A。改用安培后现象消失。
 *     教训：改执行域时，"限幅/增益/帧 ID"三处必须同一次改完，并写下单位。
 *
 * 安全约定（考核硬性）：
 *   - 未 enable 时所有电机输入恒为 0；
 *   - target 会被限幅，任何输入都不会产生越界指令。
 */
class Chassis {
 public:
  struct Params {
    float max_linear_speed = 2.0f;    ///< 平移速度限幅，m/s（与 Robot 侧一致）
    float max_angular_speed = 10.0f;  ///< 角速度限幅，rad/s（仅兜底）
    float max_wheel_vel = 120.0f;     ///< 轮电机输出端角速度限幅，rad/s
    /* 【单位·2026-11-18 现场踩坑】GM6020 / M3508 在本工程的 input_type = kCurr，
     * 而 Motor::setInput() 在 kCurr 下的入参单位是**安培**，不是报文 raw：
     *     setInput(x) -> curr = Bound(x, ±curr_input_lim) -> raw = curr / curr_rat
     *     GM6020: curr_input_lim=3.0 A,  curr_rat=3/16384 -> 16384 raw = 3 A
     *     M3508 : curr_input_lim=20.0 A, curr_rat=20/16384
     * 下面所有 steer_* 输出量的单位因此是 A / (A/rad) / (A/(rad/s)) / (A/s)。
     * 踩过的坑：把它们当 raw 用（steer_raw_limit=250、kp=150）等于每拍要求 150~250 A，
     * 被钳到 3 A = 满电流开关控制 -> 四台 GM6020 满力高频抖动 + 啸叫，
     * 而"限幅压到 250→60"看起来降了 4 倍、实际仍然是满电流，所以怎么改都没用。 */
    /* 【保守档 2026-11-18 现场联调】为了让任何异常都不至于变成"乱转"：
     *   舵电流上限 0.8 A（满量程 3 A 的 27%，扭矩约 0.6 N·m，轮子悬空足够转）
     *   舵变化率 6 A/s（从 0 到 0.8 A 要 133 ms，不会突然满力）
     *   轮 raw 上限 1500（≈1.8 A，满量程 20 A 的 9%）
     * 等联调确认行为正确后，再把这三项按需要往上放。 */
    float steer_curr_limit = 0.8f;   ///< 舵电流限幅，**安培**（保守档）
    /* ⚠ 注意：轮电机在 ins_motor.cpp 里仍然是 kRaw（kWheelInputType = kRaw），
     * 所以轮的下限幅是 **raw**，不是安培！两路电机量纲不同，改的时候看清楚。 */
    float wheel_raw_limit = 1500.0f;  ///< 轮 raw 限幅（保守档，≈1.8 A）
    float steer_curr_slew = 6.0f;     ///< 舵电流变化率限幅，A/s
    /* 【现场修复 2026-11-18】舵角环改用手写 PD，增益放在这里。
     * 为什么不用 Pid 类：Pid 的 D 项对"测量角度"做差分，而舵角在 ±180 会回绕，
     * 一次回绕就是 358 度/帧的假角速度，D 项瞬间打满，于是在边界上永久振荡
     * （现场实测：LB/RF 恰好停在 ±180 附近，一直来回绕圈，永远收不住）。
     * 改成直接用电机反馈的角速度（连续、不回绕）做阻尼项。 */
    float steer_kp = 2.0f;    ///< A per rad（误差 0.4 rad -> 满 0.8 A）
    /* kd 必须足够大：舵轮带轮子、转动惯量不小，而电流上限只有 0.8 A。
     * kd 太小（0.4）时阻尼项在 30 rad/s 也只有 12 A 的需求，被限幅后只剩 0.8 A
     * —— 刹不住，轮子越转越快、绕着 ±180° 打转（2026-11-18 实测就是这个问题）。
     * kd = 2.0 A/(rad/s)：转速到 0.4 rad/s（23°/s）阻尼项就打满，等于给舵轮限速。 */
    float steer_kd = 2.00f;   ///< A per (rad/s)
    /* 硬刹车阈值：角速度超过它就**不看目标**，直接反向满电流刹。
     * 这是最后一道保险：即使增益/反馈都不对，也不会出现"越转越快"。 */
    float steer_vel_hard_limit = 3.0f;   ///< rad/s
    float steer_deadband = 0.01f;     ///< 舵角误差死区，rad
    /** 是否按 δ 把遥控的 (vx, vy) 从云台系旋到车体系。
     *  考核 3.2「前进正方向始终是云台朝向」→ true（默认）。
     *  若分离模式（kDepart）本意是「前进方向沿车头」，把它改成 false 即可，
     *  这是一项现场待确认项，不要两边都改。 */
    bool rotate_velocity_by_delta = true;
  };

  void init(void);

  void registerIkSolver(hello_world::chassis_ik_solver::ChassisIkSolver *ik) { ik_ = ik; }

  /** @brief 云台坐标系相对车头坐标系的夹角 δ（考核第 6 项：前进方向 = 云台朝向）。
   *  由 Robot 每拍从「云台绝对 yaw − 车头 yaw」算出并下发；不顺时针为正。 */
  void setFrameDelta(float delta) { frame_delta_ = delta; }
  float frameDelta(void) const { return frame_delta_; }

  /** @brief 是否按 δ 把遥控的 (vx, vy) 从**云台坐标系**旋到车体系。
   *  考核第 6 项的实现点：跟随 / 小陀螺 = true（车朝云台指向前进）；
   *  分离 = false（纯车头坐标系，转云台不影响行走方向）。
   *  由 Robot 每拍按当前模式下发，所以"分离/跟随"的差别不只是 w 的来源，
   *  连平移方向的参考系都不一样 —— 这才是这三种模式的本质区别。 */
  void setRotateByDelta(bool on) { params_.rotate_velocity_by_delta = on; }

  void setTarget(const ChassisMotion &tar) { target_ = tar; }
  const ChassisMotion &target(void) const { return target_; }

  void setEnabled(bool en);
  bool enabled(void) const { return enabled_; }

  /** @brief 所有电机输入置 0（急停 / 未就绪时调用）。 */
  void setOutputZero(void);

  /** @brief 【调试】反解得到的 4 组舵角/轮速，只给调试快照用；
   *         不影响控制，删掉也不影响功能。 */
  const hello_world::chassis_ik_solver::ChassisIkSolver::WheelState *wheelState(void) const {
    return wheel_state_;
  }

  void update(void);

 private:
  Params params_{};
  hello_world::chassis_ik_solver::ChassisIkSolver *ik_ = nullptr;
  ChassisMotion target_{};
  hello_world::chassis_ik_solver::ChassisIkSolver::WheelState wheel_state_[4]{};
  float steer_hold_[4] = {0.0f, 0.0f, 0.0f, 0.0f};  ///< 死区内保持的舵输出
  float steer_out_[4] = {0.0f, 0.0f, 0.0f, 0.0f};   ///< 经过变化率限幅后的实际舵输出（现场修复 2026-11-17）
  float frame_delta_ = 0.0f;                        ///< 云台相对车头的夹角 δ
  bool enabled_ = false;
  bool inited_ = false;
  /* 【安全】反馈丢失检测（见 chassis.cpp 里那段说明）：
   * 两路电机反馈计数之和超过 kFeedbackTimeoutMs 没变化 -> 立刻零输出。 */
  static constexpr uint32_t kFeedbackTimeoutMs = 50u;
  uint32_t last_rx_sum_ = 0xFFFFFFFFu;
  uint32_t last_rx_ms_ = 0;
  /* 【调试区 · 一次性舵角符号测试】状态（排障完连同 chassis.cpp 里那段一起删） */
  bool bench_started_ = false;
  bool bench_force_run_ = false;   /* 台架闭环测试窗口内强制运行 */
  uint32_t bench_start_ms_ = 0;
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */
