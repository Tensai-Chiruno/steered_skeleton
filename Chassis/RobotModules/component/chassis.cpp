/**
 *******************************************************************************
 * @file      :chassis.cpp
 * @brief     :底盘运动模块实现
 * @category : 功能 | 考核第 6 项（底盘闭环）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *  V1.1.0      2026-11-09      Tensai-Chiruno  1. 按第二轮复查报告修 P0/P1
 *  V1.2.0      2026-11-09      Tensai-Chiruno  1. 舵角就近解 + 前进方向变换接入
 *  V1.3.0      2026-11-16      Tensai-Chiruno  1. 逐句注释
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis.hpp"

// 逆解器单例（几何参数在 ins_chassis_iksolver.cpp 里配）
#include "ins_chassis_iksolver.hpp"
// 电机单例（8 台底盘电机的 ID/方向/偏移都在 ins_motor.cpp）
#include "ins_motor.hpp"
// PID 实例（4 个舵角环 + 4 个轮速环）
#include "ins_pid.hpp"
// fmod / fabs：角度回绕与就近解判定
#include <cmath>

// 引入完整 Pid 定义（ins_pid.hpp 里只有前置声明，调成员函数必须看到完整类型）
#include "pid.hpp"

/* Private macro -------------------------------------------------------------*/
namespace {
// 控制周期写死 1 ms，与 TIM6 的中断频率一致。PID 的 dt 必须与之一致，
// 否则微分/积分项的等效增益会整体偏一个倍数。
constexpr float kDt = 1e-3f;
constexpr float kPi = 3.14159265358979f;

/**
 * @brief 限幅，且顺手把 NaN 折成 0。
 *
 * 为什么专门处理 NaN：上游是遥控器归一化值 × 上限、以及 PID 输出，
 * 一旦某个环节出现 inf/NaN，下面的 setInput() 会把它转成整数（UB），
 * 在 Cortex-M4 上通常变成 0 或极大值——后者等于给电机一脚满输出。
 * 用 `!(v == v)` 而不是 `isnan()`：不依赖 <cmath> 的浮点环境，且不会被 -ffast-math 优化掉。
 */
float Clamp(float v, float lo, float hi) {
  if (!(v == v)) return 0.0f;  /* NaN 直接折 0，绝不让它进 float->int */
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
}  // namespace

namespace robot {

/**
 * @brief 一次性初始化：取单例、清 PID 状态、建立失效安全初值。
 *
 * 调用时机：MainTaskInit() 里，**主循环之前**、且此时 TIM6 中断还没开
 * （main.c 里 Start_IT 在 MainTaskInit 之后），所以这里不需要考虑并发。
 *
 * 初值方向必须是"安全"的：target_ 清零、输出清零、PID 状态清零，
 * 这样即使后面状态机一直没跑起来，车也不会动。
 */
void Chassis::init(void) {
  // 逆解器走懒加载单例；已经注册过（测试时注入）就不覆盖。
  if (!ik_) {
    ik_ = GetChassisIkSolver();
  }
  // 8 个 PID 的积分/微分状态全部清零：避免上一次运行的残留积分在使能瞬间变成一脚输出。
  for (int i = 0; i < 4; ++i) {
    auto *ps = GetPidMotorSteer(i);
    auto *pw = GetPidMotorWheel(i);
    if (ps) ps->reset();
    if (pw) pw->reset();
    steer_hold_[i] = 0.0f;  // 死区保持值也清掉
  }
  target_.reset();   // 目标速度清零
  inited_ = true;    // 允许 update() 工作（否则 update 第一句就 return）
  setOutputZero();   // 立刻把 8 台电机的输入写成 0
}

/**
 * @brief 使能/失能底盘。
 *
 * 去抖：只有状态真的变化时才动作。否则状态机每拍都调 setEnabled(false)，
 * 会把 setOutputZero() 里的 PID reset 每毫秒执行一次，积分永远长不起来。
 */
void Chassis::setEnabled(bool en) {
  if (enabled_ == en) return;  // 状态没变，直接返回（避免重复清 PID）
  enabled_ = en;
  if (!en) setOutputZero();    // 失能瞬间必须把输出打成 0，不能等下一拍
}

/**
 * @brief 把 8 台电机的输入全部写 0，并清掉 PID 与死区保持值。
 *
 * 这是安全红线（考核 1.3）的落地点：急停档 / 遥控离线 / 未进 Working 时，
 * 状态机都会走到这里。所以这里的实现必须"无论如何都把 0 写下去"，
 * 不能因为某个 getter 返回 nullptr 就跳过整轮循环。
 */
void Chassis::setOutputZero(void) {
  for (int i = 0; i < 4; ++i) {
    auto *steer = GetMotorSteer(i);
    auto *wheel = GetMotorWheel(i);
    if (steer) steer->setInput(0.0f);  // 舵电机 raw = 0（电压/电流域的中位）
    if (wheel) wheel->setInput(0.0f);  // 驱动电机 raw = 0
    auto *ps = GetPidMotorSteer(i);
    auto *pw = GetPidMotorWheel(i);
    if (ps) ps->reset();  // 清积分：否则下次使能会带着旧积分突跳
    if (pw) pw->reset();
    steer_hold_[i] = 0.0f;
  }
}

/**
 * @brief 每个控制周期（1 ms）调用一次：限幅 → δ 变换 → 逆解 → 8 个闭环 → 下发。
 *
 * 数据流：
 *   遥控器 → Robot 生成 ChassisMotion（云台系）→ setTarget()
 *        → 本函数限幅
 *        → gimbalToChassis() 旋到车体系
 *        → ChassisIkSolver::solve() 得到每轮的「舵角 + 轮速」
 *        → 就近解选择（+π 且轮速取反）
 *        → 舵角环 PID / 轮速环 PID
 *        → Motor::setInput(raw)
 */
void Chassis::update(void) {
  // 还没初始化完（或逆解器缺失）：直接返回。注意这里**不写输出**——
  // 此时电机对象可能都还没建立，写也写不进去；而且 init() 里已经置过 0。
  if (!inited_ || ik_ == nullptr) {
    return;
  }
  // 未使能：每拍都主动把输出压成 0（而不是"保持上次值"）。
  // 这样即使有人误改了 target_，也不会在未使能时被下发。
  if (!enabled_) {
    setOutputZero();
    return;
  }

  /* 1) 目标限幅：任何遥控器输入都不能变成越界指令 */
  // 注意 t 是**局部的**：限幅只影响本拍解算，不会把 target_ 改掉。
  // 这样外部仍然能看到原始目标（调试时有用），也不会出现"越改越小"的累积。
  ChassisMotion t;
  t.vx = Clamp(target_.vx, -params_.max_linear_speed, params_.max_linear_speed);
  t.vy = Clamp(target_.vy, -params_.max_linear_speed, params_.max_linear_speed);
  t.w = Clamp(target_.w, -params_.max_angular_speed, params_.max_angular_speed);

  /* 2) 前进方向变换（考核第 6 项）：遥控给的 (vx, vy) 定义在**云台坐标系**，
   *    而逆解吃的是车头坐标系，所以先转到车体系再解算。
   *    δ 由 Robot 每拍下发；δ == 0 时是恒等变换，单板台架无需对端也能跑。 */
  // 先给出"不旋转"的默认值：δ=0 或开关关掉时直接用它，省一次三角函数。
  float vx_c = t.vx;
  float vy_c = t.vy;
  // rotate_velocity_by_delta 是现场可配的开关（分离模式是否也按云台朝向走）。
  // frame_delta_ == 0 时跳过，既省算力，也保证单板调试时行为完全可预测。
  if (params_.rotate_velocity_by_delta && frame_delta_ != 0.0f) {
    hello_world::chassis_ik_solver::ChassisIkSolver::gimbalToChassis(
        t.vx, t.vy, frame_delta_, &vx_c, &vy_c);
  }

  /* 3) 逆解：速度 -> 每个舵的目标转角 + 每个轮输出端角速度（rad/s） */
  // 解算结果直接写进成员 wheel_state_，方便调试快照/测试读取。
  ik_->solve(vx_c, vy_c, t.w, wheel_state_);

  // 4 个轮组逐个闭环。索引 i 的语义在 ChassisIkSolver::WheelIndex 里定义：
  // 0=左前(LF) 1=左后(LB) 2=右后(RB) 3=右前(RF)，与 ins_motor.cpp 的数组顺序一致。
  for (int i = 0; i < 4; ++i) {
    auto *steer = GetMotorSteer(i);  // GM6020，舵向角
    auto *wheel = GetMotorWheel(i);  // M3508，驱动轮
    if (steer == nullptr || wheel == nullptr) {
      continue;  // 单例没建立（理论上 init 后不会发生）——跳过而不是崩
    }
    // 读一次实测舵角，后面就近解与角度环都用它。
    // 注意：这个值已经被 ins_motor 里的 angle_offset 折算成"机械零位为 0"，
    // 与逆解输出的目标角同一个基准，所以可以直接相减。
    const float cur_angle = steer->angle();

    /* ---- 舵角就近解（IK 不播种，舵角本来就多解）----
     * 舵向轮有两个等价解：角度 +π 且轮速取反。
     * 逆解每拍独立算，若直接采用它给的角度，上电时可能让舵轮跨 149° 去一个
     * 「数学上等价、机械上很远」的位置。这里以当前实测角为基准挑近的那个。 */
    float target_angle = wheel_state_[i].angle;
    float target_speed = wheel_state_[i].speed;
    // 候选一：直接采用逆解角。err_fwd 是它相对当前角的**回绕后**误差，
    // 范围 [-π, π]。用 fmod 实现回绕而不是 while 循环：耗时确定，适合 1 kHz。
    const float err_fwd = std::fmod(target_angle - cur_angle + 3.14159265f, 6.28318531f) - 3.14159265f;
    // 候选二：角度 +π（舵轮反向安装），此时轮速也必须取反，否则车会往反方向走。
    // 表达式里多出来的那个 +π 是为了让"目标角 +π"也参与同一个回绕公式。
    const float err_rev = std::fmod(target_angle + 3.14159265f - cur_angle + 3.14159265f,
                                    6.28318531f) - 3.14159265f;
    // 谁的角度误差绝对值更小就选谁：机械行程更短，不会出现"上电就大角度回摆"。
    if (std::fabs(err_rev) < std::fabs(err_fwd)) {
      target_angle += 3.14159265f;  // 舵角转 180°
      target_speed = -target_speed; // 轮速取反，保证行走方向不变
    }
    // 把目标角统一回绕到 [-π, π]，避免把 3π 这种值喂给 PID。
    target_angle = std::fmod(target_angle + 3.14159265f, 6.28318531f) - 3.14159265f;
    // 写回成员：让调试快照/测试看到的是"就近解之后"的最终目标，而不是逆解原始输出。
    wheel_state_[i].angle = target_angle;
    wheel_state_[i].speed = target_speed;

    /* ---- 舵向角度环 ---- */
    // 每拍取一次限幅值（成员参数可能被现场改），避免在循环里反复解引用。
    const float steer_raw_limit = params_.steer_raw_limit;
    // 再算一次回绕误差，供死区判断用（与上面 err_fwd 同源，但这里是就近解之后的最终值）。
    const float err = std::fmod(target_angle - cur_angle + 3.14159265f, 6.28318531f) - 3.14159265f;

    auto *pid = GetPidMotorSteer(i);
    float steer_raw = 0.0f;
    if (pid != nullptr) {
      /* 无条件调用，保证 PID 的 dt 契约（P2-14 / G-2：跳拍会把 D 项放大 N 倍） */
      // target/measured 都已被 angle_offset 折算到同一基准；angle_wrap 已在 ins_pid 里打开，
      // 所以 PID 内部还会对误差和"测量差"各做一次回绕，防止过 ±π 时 D 项尖峰。
      steer_raw = pid->update(target_angle, cur_angle, kDt);
    }
    if (err < params_.steer_deadband && err > -params_.steer_deadband) {
      /* 死区内不切断输出，而是保持上一拍的值：
       * 舵轮带 caster 力，切 0 会让它漂回零位、再被拉回，形成极限环抖动。 */
      // 注意：保持的是上一拍的**输出**，不是上一拍的目标——舵机需要一点静态保持量。
      steer_raw = steer_hold_[i];
    } else {
      // 出了死区就更新保持值，供下次进死区时使用。
      steer_hold_[i] = steer_raw;
    }
    // 最后一道限幅：raw 域（GM6020 电压帧满量程 25000，这里默认 20000）。
    // 台架整定阶段建议先降到 3000~5000，防止一脚打飞。
    steer->setInput(Clamp(steer_raw, -steer_raw_limit, steer_raw_limit));

    /* ---- 轮速环 ----
     * 逆解已经把轮心线速度按 params_.wheel_radius 折算成「输出端 rad/s」，
     * 与 wheel->vel() 同域。这里**绝不能**再除一次轮半径
     * （旧代码这样写导致整定当天实际车速为指令的约 15 倍）。 */
    // 速度限幅：max_wheel_vel 默认 120 rad/s，约等于 7.9 m/s 的轮面线速度，
    // 是兜底值而不是设计值；正常工况下不应该顶到这里。
    const float wheel_vel_target =
        Clamp(target_speed, -params_.max_wheel_vel, params_.max_wheel_vel);
    auto *pidw = GetPidMotorWheel(i);
    float wheel_raw = 0.0f;
    if (pidw != nullptr) {
      // 反馈是 wheel->vel()：M3508 输出端角速度（已除过 14 的减速比），
      // 与目标同域，所以这里同样是"直接相减"。
      wheel_raw = pidw->update(wheel_vel_target, wheel->vel(), kDt);
    }
    // raw 域限幅：M3508 控制帧满量程 16384，这里默认 16000。
    wheel->setInput(Clamp(wheel_raw, -params_.wheel_raw_limit, params_.wheel_raw_limit));
  }
}

}  // namespace robot
