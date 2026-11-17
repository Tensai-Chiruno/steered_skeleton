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
#include "tick.hpp"

/* Private macro -------------------------------------------------------------*/

/* 【诊断·临时】舵角环三个关键量的"探针"：直接暴露成全局符号，用 SWD 读。
 * 为什么要单独开变量而不塞进 g_dbg：g_dbg 的布局是 96 字节定长，加字段要同步改
 * 上位机解析；而排障只需要 12 个 float，单独读符号更快、改完就删。
 *   g_dbg_steer_cmd = 真正下发进 CAN 帧的 raw（经过死区保持 + 限幅 + 变化率限幅）
 *   g_dbg_steer_pd  = 限幅前的 PD 控制量（看增益是不是打满）
 *   g_dbg_steer_vel = 电机反馈角速度（看 D 项是不是在被噪声放大）
 */
volatile float g_dbg_steer_cmd[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float g_dbg_steer_pd[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float g_dbg_steer_vel[4] = {0.0f, 0.0f, 0.0f, 0.0f};
/* 台架窗口的触发状态：[0]=armed 标志, [1]=MotorSteerRxCount() 读到的值。
 * 用来分辨"窗口没 arm"到底是计数为 0（RX 没进来）还是逻辑没走到。 */
volatile uint32_t g_dbg_bench[2] = {0u, 0u};
/* 【诊断·临时】轮电机：PID 原始输出、限幅后真正下发的电流（A）。排障完删。
 * 现场症状：遥控推杆后逆解目标到 20 rad/s，但四个轮子悬空也不转、没声音，
 * 说明电流没到电机。这两个探针用来分清"PID 没算"还是"算了但没下发"。 */
volatile float g_dbg_wheel_pd[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float g_dbg_wheel_cmd[4] = {0.0f, 0.0f, 0.0f, 0.0f};

namespace {
/* 【调试区 · 一次性舵角符号测试】参数（排障完连同那段代码一起删） */
constexpr float kBenchRaw = 0.5f;     /* A（电流域！不是 raw。原来写 150 = 150A 已满电流） */
constexpr uint32_t kBenchMs = 600u;   /* 持续 300 ms 后永久归零 */
constexpr uint32_t kBenchStartMs = 2000u;   /* 兜底触发：开机满 2 s 也可（等总线/电机就绪） */
constexpr bool kBenchEnable = false;   /* 一次性开环测试：已验证完，关掉 */
constexpr uint32_t kBenchLoopMs = 5000u;  /* 台架闭环测试窗口长度：由“舵反馈第一次到达”起算 */
constexpr float kBenchStepRad = 0.5235988f;  /* 台架阶跃：+30°（测阶跃响应用） */
/* 台架闭环窗口总开关：2026-11-18 已验证通过（四个舵轮 +30° 阶跃、稳态电流 0.04~0.07 A、
 * 无振荡、CAN 错误 0），**收工必须置 false**，否则每次开机都会给舵角加 30° 阶跃。 */
constexpr bool kBenchLoopEnable = false;
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
    steer_hold_[i] = 0.0f;
    steer_out_[i] = 0.0f;  // 死区保持值也清掉
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
  /* 【调试·台架窗口】窗口内输出由 update() 自己管线负责，任何外部"归零请求"都跳过。
   * 为什么必须加：状态机在 Dead 状态下每拍都会调用本函数，
   * 而它会把 steer_out_ 清零 —— 于是舵角环每个周期都从 0 开始爬变化率限幅，
   * 永远只能输出"一拍的变化量"（旧版 = 2 raw，新版 = 0.006 A）。
   * 现场表现极具误导性：PD 算得好好的、探针里的指令却永远约等于 0，
   * 让人以为增益/限幅不对，其实是输出被人从后面擦掉了。
   * 只影响 5 s 台架窗口（最大 1.5 A），窗口一过立刻恢复"无论如何写 0"。 */
  if (bench_force_run_) {
    return;
  }
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
    steer_out_[i] = 0.0f;
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

  /* ===== 【安全】反馈丢失保护（2026-11-18 现场事故后新增）=====
   *
   * 为什么必须有这一条：GM6020 是**直驱**，没有内部位置环。
   * 舵角环每拍算 (目标角 − 实测角) 再出力。一旦反馈停了（CAN1 掉线、
   * 云台板断电导致总线失去 ACK、总线错误……），实测角就**冻结**，
   * 误差永远不为零，环路把电流一直顶在限幅上 —— 直驱电机于是越转越快、
   * 一直转到断电为止。现场两次"舵轮乱转 / 啸叫"就是这个链路：
   *   反馈冻结 -> 误差恒定 -> 满限幅电流 -> 持续旋转。
   * 而 Dead 状态只是"不再更新指令"，电机里已经锁住的电流不会自己消失。
   *
   * 判据：两路电机的反馈计数在 50 ms 内都没变化 = 两条总线全哑了，
   * 直接按"失效即归零"处理（考核 1.3）。单条总线哑掉也归零，
   * 因为舵角环依赖 CAN1、轮速环依赖 CAN2，任一失效都不允许继续出力。
   */
  {
    const uint32_t now = hello_world::tick::GetTickMs();
    const uint32_t rx_sum = MotorSteerRxCount() + MotorWheelRxCount();
    if (rx_sum != last_rx_sum_) {
      last_rx_sum_ = rx_sum;
      last_rx_ms_ = now;
    }
    if ((now - last_rx_ms_) > kFeedbackTimeoutMs) {
      setOutputZero();
      return;   /* 反馈死了就不许出力，也不许"保持上一帧" */
    }
  }

  /* ===== 【调试区 · 一次性舵角符号测试】开始 =====
   * 目的：确定"正的 raw 输出"让舵角变大还是变小（正反馈就是疯转的根因）。
   * 做法：开机后 kBenchStartMs ~ kBenchStartMs+kBenchMs 之间，**只给 LF 舵电机**
   *       kBenchRaw（150 raw，满量程 0.9%），其它三个舵电机和四个轮电机全 0；
   *       窗口结束立刻全部归零、并且之后不再执行。
   * 特点：不经过 PID（不存在正反馈）、不依赖状态机/遥控（只要有电就准时执行一次），
   *       所以可以在"先烧录、后通电"的安全顺序下做。
   * 排障完把本段连同 chassis.hpp 里的 bench_* 成员一起删掉（grep【调试区 · 一次性】）。
   */
  {
    const uint32_t now_ms = hello_world::tick::GetTickMs();
    if (kBenchEnable && now_ms >= kBenchStartMs && now_ms < (kBenchStartMs + kBenchMs)) {
      for (int i = 0; i < 4; ++i) {
        auto *st = GetMotorSteer(i);
        auto *wh = GetMotorWheel(i);
        if (st != nullptr) {
          st->setInput((i == 0) ? kBenchRaw : 0.0f);   /* 只有 LF 出力 */
        }
        if (wh != nullptr) {
          wh->setInput(0.0f);                          /* 轮子一帧都不出力 */
        }
      }
      return;   /* 测试窗口内不跑正常控制 */
    }
  }
  /* ===== 【调试区 · 一次性舵角符号测试】结束 ===== */

  // 未使能：每拍都主动把输出压成 0（而不是"保持上次值"）。
  // 这样即使有人误改了 target_，也不会在未使能时被下发。
  /* 【调试区 · 舵角环台架闭环测试】窗口由**舵电机反馈第一次到达**触发。
   *
   * 为什么不用开机计时：现场的安全顺序必须是「先断电 → 烧录/复位 → 再上电」，
   * 上电时刻比复位晚好几秒，任何"开机 N 秒后开始"的窗口都赶不上，
   * 要么窗口在没电时白跑完，要么逼着人带电复位（2026-11-17 就是这么把 CAN
   * 打坏、把四台 GM6020 逼成高频啸叫的）。
   * 改成监听 CAN1 上 GM6020 的反馈计数：反馈一到 = 电机已上电 + 总线已活，
   * 从那一刻起开 kBenchLoopMs 的窗口。
   * 窗口外行为完全不变（未使能就压 0）。排障完删掉本段。 */
  {
    static bool armed = false;
    static uint32_t start_ms = 0;
    const uint32_t now_ms = hello_world::tick::GetTickMs();
    /* 两条触发并联：舵反馈计数到了（电机已上电 + 总线已活）**或**开机满 2 s。
     * 为什么并联：2026-11-18 现场出现过"只用反馈计数"却始终不 arm 的情况
     * （g_steer_rx_count 明明在涨），为了不再猜，把 armed 标志和计数值也做成探针
     * g_dbg_bench[] 一起读出来；同时保留开机计时作为兜底，保证一次复位就能拿到数据。
     * （整片 AIRCR 复位已验证可在电机通电时安全使用：实测 TEC 全程 0。） */
    if (!armed && (MotorSteerRxCount() > 0u || now_ms >= kBenchStartMs)) {
      armed = true;
      start_ms = now_ms;
    }
    bench_force_run_ = kBenchLoopEnable && armed && ((now_ms - start_ms) < kBenchLoopMs);
    g_dbg_bench[0] = armed ? 1u : 0u;
    g_dbg_bench[1] = MotorSteerRxCount();
  }
  if (!enabled_ && !bench_force_run_) {
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
    /* 【调试·台架】窗口内给所有舵角加一个固定阶跃，用来测阶跃响应
     * （上升时间、超调、稳态误差、有没有振荡）。不加阶跃就只能"保持原位"，
     * 什么都测不出来。窗口外为 0，不影响正常控制。 */
    if (bench_force_run_) {
      target_angle += kBenchStepRad;
    }
    /* ---- 舵向轮等价解：确定性选解（2026-11-18 现场重写）----
     * 舵向轮有两个**物理等价**的解：(θ, v) 与 (θ+π, −v)（把舵轮转 180°、轮速取反）。
     *
     * 旧写法：按"离当前实测角更近"来选（为了上电时不跨 149° 回摆）。
     * 现场实测两个问题（lv=+99 纯前进时的原始数据）：
     *   1) 同一条前进指令，四轮分别停在 0° 和 ±180° 两种姿态，看起来"舵角不一致"；
     *   2) 实测角接近 ±90° 时两个解距离相等，目标会在 0°/180° 之间**每拍翻转**
     *      （实测 8 Hz 抖动）—— 机械上就是舵机 180° 来回抽，正是考核禁止的回摆。
     * 现在改成确定性规则：**一律取 |θ| ≤ 90° 的那个解**（必要时 +π 并把轮速取反）。
     *   - 与实测角无关 -> 目标不可能跳变；
     *   - 同一指令永远给出同一个机械姿态 -> 四轮姿态一致、看着对；
     *   - 代价：最坏要多转 180°，由 steer_curr_slew 的变化率限幅平滑掉。 */
    if (std::fabs(target_angle) > 1.5707963f) {
      target_angle += (target_angle > 0.0f) ? -3.14159265f : 3.14159265f;
      target_speed = -target_speed;   // 舵角转 180°，轮速必须取反才等价
    }
    // 把目标角统一回绕到 [-π, π]，避免把 3π 这种值喂给 PID。
    target_angle = std::fmod(target_angle + 3.14159265f, 6.28318531f) - 3.14159265f;
    // 写回成员：让调试快照/测试看到的是"就近解之后"的最终目标，而不是逆解原始输出。
    wheel_state_[i].angle = target_angle;
    wheel_state_[i].speed = target_speed;

    /* ---- 舵向角度环 ---- */
    // 每拍取一次限幅值（成员参数可能被现场改），避免在循环里反复解引用。
    // 单位是**安培**（见 chassis.hpp 里那段说明），不是报文 raw。
    const float steer_curr_limit = params_.steer_curr_limit;
    // 再算一次回绕误差，供死区判断用（与上面 err_fwd 同源，但这里是就近解之后的最终值）。
    const float err = std::fmod(target_angle - cur_angle + 3.14159265f, 6.28318531f) - 3.14159265f;

    auto *pid = GetPidMotorSteer(i);
    float steer_raw = 0.0f;
    float vel_meas = 0.0f;
    if (pid != nullptr) {
      /* 【现场修复 2026-11-18】舵角环改用手写 PD，D 项直接用电机反馈的角速度。
       *
       * 原来的写法是 pid->update(target, measured, dt)，它的 D 项对测量角度做差分；
       * 而舵角在 ±180 处会回绕，一次回绕就产生 358 度/帧的假角速度，
       * D 项瞬间打满 -> 反向加速 -> 又在边界另一侧回绕 -> 永久振荡。
       * 现场实测：LB/RF 恰好停在 ±180 附近，无论怎么调 kp/kd 都收不住。
       *
       * 现在：err 回绕（安全）+ 阻尼项用 steer->vel()（连续、不回绕），
       *      再过死区保持 / 限幅 / 变化率限幅。
       * PID 对象仍然保留给轮速环用。 */
      (void)pid;   /* 保留对象实例，避免影响其他地方对 PID 的访问 */
      vel_meas = (steer != nullptr) ? steer->vel() : 0.0f;
      steer_raw = params_.steer_kp * err - params_.steer_kd * vel_meas;
      /* 【安全】硬刹车：角速度超过阈值就不看目标，直接反向满电流。
       * 为什么需要：电流上限只有 0.8 A（27% 满量程），而阻尼项在高速时需要的
       * 电流远超上限，纯 PD 在"舵轮空载飞转"这种工况下刹不住 ——
       * 现场实测会出现越转越快、绕着 ±180° 转圈（2026-11-18）。
       * 这一条与增益无关，所以即使参数配错也不会失控。 */
      if (std::fabs(vel_meas) > params_.steer_vel_hard_limit) {
        steer_raw = (vel_meas > 0.0f) ? -params_.steer_curr_limit : params_.steer_curr_limit;
      }
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
    // 最后一道限幅：安培域（GM6020 满量程 3 A，默认只给 1.5 A）。
    // 注意这里之前写的是 "raw 域满量程 25000"，那就是满电流抖动的源头：
    // setInput() 在 kCurr 下把 25000 当 25000 A，Bound 到 3 A 后 raw 直接打满。
    const float steer_cmd = Clamp(steer_raw, -steer_curr_limit, steer_curr_limit);

    /* 【现场修复 2026-11-17】舵角输出**变化率限幅**（现在单位是 A/s）。
     *
     * 为什么必须加：上电时舵轮可能停在 ±180° 附近，目标 0° 的误差≈π，
     * 正是"就近转弯"的歧义边界，任何增益都会立刻要求满电流，电机瞬间满力冲出。
     *
     * 现在每拍最多改变 params_.steer_curr_slew * dt：
     *   6 A/s × 1 ms = 0.006 A/拍 -> 从 0 爬到 1.5 A 要 250 ms，
     * 于是"无论多大误差、多坏的初始姿态"，输出都只能缓慢爬升，冲不起来。
     */
    const float dmax = params_.steer_curr_slew * kDt;
    const float d = steer_cmd - steer_out_[i];
    if (d > dmax) {
      steer_out_[i] += dmax;
    } else if (d < -dmax) {
      steer_out_[i] -= dmax;
    } else {
      steer_out_[i] = steer_cmd;
    }
    steer->setInput(steer_out_[i]);

    /* 【诊断·临时】探针：实际命令 / 限幅前 PD / 反馈角速度。排障完删。 */
    g_dbg_steer_cmd[i] = steer_out_[i];
    g_dbg_steer_pd[i] = steer_raw;
    g_dbg_steer_vel[i] = vel_meas;

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
      /* 目标为 0（松杆）时先清积分：加速阶段积满的 I 会在松杆后继续推，
       * 实测松杆后 LF 仍有 0.7~1.9 rad/s 的爬行（指令 +155 raw 全是积分项）。
       * 清掉后这一拍只剩 P 项，立刻变成刹车。正常调速时不清，保留消静差能力。 */
      if (std::fabs(wheel_vel_target) < 0.1f) {
        pidw->reset();
      }
      wheel_raw = pidw->update(wheel_vel_target, wheel->vel(), kDt);
    }
    // raw 域限幅（轮电机是 kRaw！不是安培；M3508 满量程 16384，默认 3000 ≈ 3.7 A）。
    const float wheel_cmd = Clamp(wheel_raw, -params_.wheel_raw_limit, params_.wheel_raw_limit);
    wheel->setInput(wheel_cmd);
    /* 【诊断·临时】轮电流探针。排障完删。 */
    g_dbg_wheel_pd[i] = wheel_raw;
    g_dbg_wheel_cmd[i] = wheel_cmd;
  }
}

}  // namespace robot
