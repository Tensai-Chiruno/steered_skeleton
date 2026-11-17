/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     :底盘板顶层状态机实现
 * @category : 功能 | 考核第 9 项（状态机；含 1 处调试翻号开关）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *  V1.1.0      2026-11-09      Tensai-Chiruno  1. 按第二轮复查报告修 P0/P1
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot.hpp"

#include <cmath>

#include "HW_can.hpp"
#include "ins_comm_state.hpp"
#include "rc_relay.hpp"
#include "tick.hpp"

namespace {
constexpr float kDt = 1e-3f;
}  // namespace

namespace robot {

/// 摇杆死区：|v| 小于 db 就当作 0。
/// 目的不是"手感"，而是防止静止时摇杆中位漂移（±0.01 量级）让车缓慢爬行。
float Robot::Deadband(float v, float db) {
  return (v > -db && v < db) ? 0.0f : v;
}

/* 【诊断·临时】最近一次"不安全"的原因与累计进 Dead 次数。用 J-Link 直接读符号。
 * 0 = 遥控离线, 1 = 急停(左拨杆), 2 = IMU 未就绪, 0xFF = 从未发生。排障完删。 */
/* 【诊断·临时】急停来源探针：一次读清"到底是谁把 estop 拉起来的"。
 * [0] UseRelay()  [1] LeftSwitch 原始编码  [2] RightSwitch 原始编码  [3] tx_fail_streak_
 * [4] g_tx_fail_count 是否在涨(1/0)  [5] estop_ 最终值  排障完删。 */
volatile uint32_t g_dbg_rc[6] = {0u, 0u, 0u, 0u, 0u, 0u};

volatile uint8_t g_dbg_dead_reason = 0xFFu;
volatile uint32_t g_dbg_dead_count = 0u;

/* 摇杆原始值域实测可能是 ±1.55（P1-11），在这里统一夹到 ±1，
 * 保证「满舵 = 设计车速」这件事与 DT7 的实际量程无关。 */
float Robot::ClampNorm(float v) {
  if (!(v == v)) return 0.0f;   // NaN → 0：绝不让它乘上速度上限再传下去
  if (v > 1.0f) return 1.0f;    // 满舵夹到 +1
  if (v < -1.0f) return -1.0f;  // 满舵夹到 -1
  return v;
}


void Robot::init(void) {
  // 全部状态置成"最安全"的初值：上电后即使 update() 立刻被调用，
  // 也一定是 Dead + 急停 + 输出归零，车不会动。
  life_state_ = LifeState::kDead;
  mode_ = ChassisMode::kDepart;
  ctrl_source_ = CtrlSource::kNone;
  motion_.reset();
  yaw_error_ = 0.0f;
  rc_online_ = false;
  estop_ = true;        // 上电即视为急停：必须先看到"拨杆在 Up/Mid"才解除
  peer_offline_ = true; // 上电时对端肯定不在线，等第一帧双板帧到达
  imu_ready_ = false;   // IMU 未标定完成前不许进 Working
  inited_ = true;       // 置位后 update() 才开始真正工作
  if (chassis_ != nullptr) {
    chassis_->setTarget(motion_);
    chassis_->setEnabled(false);
  }
}

void Robot::updateRemote(void) {
  /* 【现场修复 2026-11-17】接收机的信号线只接到云台板，底盘板 DBUS 输入是空的
   * （实测：底盘 PC11 800 次采样零跳变、USART3 零字节）。
   * 硬件不能改线，所以云台板把遥控数据用 0x212 转发过来（见 rc_relay.hpp）。
   * 控制源优先级：本地 DBUS > 转发的遥控；两者都没有才算离线急停。 */
  const bool local_online = IsRemoteOnline();
  const bool relay_online = robot::RcRelayFresh() && (robot::g_rc_relay.online != 0u);
  rc_from_relay_ = (!local_online && relay_online);
  rc_online_ = local_online || relay_online;

  // 遥控离线 = 收到过合法帧 且 最近有更新（见 ins_comm_state.cpp）。
  if (!rc_online_ || (rc_ == nullptr && !rc_from_relay_)) {
    // 离线即急停：这是考核 1.3「失效即归零」的第一层。
    // 注意这里 return 之前没有清 peer_offline_，那是有意的：
    // 遥控掉了就够判急停了，不必再依赖对端状态。
    ctrl_source_ = CtrlSource::kNone;
    estop_ = true;
    return;
  }
  ctrl_source_ = CtrlSource::kRemote;   // 这一拍的运动指令来源于遥控器

  /* 急停判据用白名单（P2-22 / N-4）：
   * 只有明确落在 Up / Mid 才不算急停；未知编码（含 0）一律按急停处理。
   * 若左拨杆实际是两档开关，只产生 Up/Down，这条依然正确。 */
  const auto sw = LeftSwitch();
  estop_ = !(sw == hello_world::remote_control::kSwitchStateUp ||
             sw == hello_world::remote_control::kSwitchStateMid);

  /* 双板链路（P1-4）：对端离线 = 数据不可信 = 急停。 */
  peer_offline_ = (comm_ == nullptr) ? true : comm_->isOffline();
  if (peer_offline_) {
    estop_ = true;
  }

  /* P1-8①：CAN 发送持续失败 = 对端在、但发不出去（bus-off / 邮箱被重传占满）。
   * 只在对端"在线"时才判：单板台架没有 ACK，发送失败是常态，不能因此刹车。 */
  uint32_t growing = 0u;   /* 【诊断·临时】本拍发送失败计数是否在涨 */
  {
    // 用"计数是否在涨"来判断，而不是直接看单次返回值：
    // 单次失败可能只是邮箱短暂满，连续 20 ms 都在涨才说明总线真出问题。
    const uint32_t fail_now = CAN_GetTxFailCount();
    growing = (fail_now != last_tx_fail_) ? 1u : 0u;
    if (fail_now != last_tx_fail_) {
      last_tx_fail_ = fail_now;
      ++tx_fail_streak_;                 // 这一拍又失败了一次
    } else if (tx_fail_streak_ > 0u) {
      --tx_fail_streak_;                 // 没再失败就慢慢衰减，避免误判偶发
    }
    // 只有"对端在线"时才据此刹车：单板台架没有 ACK，发送失败是常态。
    if (!peer_offline_ && tx_fail_streak_ >= 20u) {
      estop_ = true;
    }
  }

  /* IMU 就绪（P1-2）：未注册、标定未完成、SPI 读失败都不许进 Working。 */
  imu_ready_ = (imu_ != nullptr) && imu_->isReady() && imu_->isOk();

  /* 【诊断·临时】把急停的四个来源一起暴露出来。 */
  g_dbg_rc[0] = UseRelay() ? 1u : 0u;
  g_dbg_rc[1] = static_cast<uint32_t>(LeftSwitch());
  g_dbg_rc[2] = static_cast<uint32_t>(RightSwitch());
  g_dbg_rc[3] = tx_fail_streak_;
  g_dbg_rc[4] = growing;
  g_dbg_rc[5] = estop_ ? 1u : 0u;

  /* 第 6 项：δ = 云台相对**车体**的夹角。
   *
   * ★ 参考系（P0-5，别改回去）：
   *   - 云台板发布的 `remote().yaw` 是 gimbal_->current().yaw，
   *     即「云台相对车体」的编码器角（DM 编码器 + kYawOffset 标定），
   *     天生与车体朝向无关 —— 这正是 gimbalToChassis() 需要的 δ。
   *   - 本板 `imu_->yaw()` 是「车体相对**本板上电瞬间**」的积分量，
   *     而 BMI088 无磁参考、只靠加速度计+陀螺，yaw 会自由漂移。
   *   两者不能相减：相减等于往 δ 里混进一个不属于同一参考系的 ψ，
   *   表现为「车体转过多少，前进方向就偏多少，而且方向相反」；
   *   更糟的是车体不动时 ψ≈0，台架静态测试完全正常，查不出来。
   *   而且 IWDG 复位是自动发生、车不会停的 —— 只要有一块板复位过，
   *   或两块板不是同时上电，δ 就凭空多/少一个未知常数，车往没预料的方向跑且不报错。
   *
   *   若将来确实想用 IMU，必须让两个 yaw 共享同一零点
   *   （底盘把自己的 yaw 发给云台、由云台算相对角），
   *   绝不能拿两块板各自的开机零点相减。
   * 对端离线时 δ 不可信，保持 0（此时也已急停，车不会动）。 */
  // 默认 δ = 0（等价于"云台与车头同向"），对端离线时保持这个安全缺省。
  float delta = 0.0f;
  if (!peer_offline_ && comm_ != nullptr) {
    // 直接取云台发布的相对角，**不减去本板 IMU 的 yaw**（见上方参考系说明）。
    delta = comm_->remote().yaw;
    // 回绕到 [-π, π)：进三角函数前必须归一化，否则 359°→1° 会反向大转。
    delta = std::fmod(delta + 3.14159265f, 6.28318531f) - 3.14159265f;
  }
  /* 【已删除：δ 符号"拨轮翻号"调试开关】
   * 原来这里写 `if (rc_wheel() < -0.7f) delta = -delta;`，用拨轮临时给 δ 翻号。
   * 2026-11-18 起拨轮被正式分配给「分离模式自转」，不能再当调试开关；
   * 而且交接文档要求提交前把符号写死。
   * δ 的符号**目前无法用实车验证**（云台发布的 yaw 恒为 0，见下），
   * 所以先写死为「不取反」；等 yaw 通了，按 交接文档 的方法验证一次：
   *   车头上电、云台指向车身左侧、推前进 -> 车应向左横移；
   * 若反了，把 delta 的赋值改成 -comm_->remote().yaw 并记录在现场调试记录里。 */

  yaw_error_ = delta;                       /* 跟随模式用 */
  if (chassis_ != nullptr) {
    chassis_->setFrameDelta(delta);          /* 前进方向变换用 */
  }
}

void Robot::enterDead(void) {
  // 进入顺序有讲究：先改状态 → 清运动指令 → 再让底盘失能并清零。
  // 若先清零再改状态，中间这一拍 chassis_->update() 可能用旧 target 输出一次。
  if (life_state_ != LifeState::kDead) {
    ++g_dbg_dead_count;   /* 【诊断·临时】统计"真的从别的状态掉进 Dead"的次数 */
  }
  life_state_ = LifeState::kDead;
  motion_.reset();
  if (chassis_ != nullptr) {
    chassis_->setTarget(motion_);
    chassis_->setEnabled(false);
    chassis_->setOutputZero();
  }
}

void Robot::enterResurrection(uint32_t now) {
  // 自检期的三个动作：
  //   1) 记下起点时间（后面用它算 hold 是否到期）
  //   2) 运动指令清零（自检期不接受运动指令）
  //   3) 使能底盘（舵角要能回中、IMU 要能收敛，但目标速度是 0）
  life_state_ = LifeState::kResurrection;
  resurrection_start_ms_ = now;
  motion_.reset();
  if (chassis_ != nullptr) {
    chassis_->setTarget(motion_);
    chassis_->setEnabled(true);
  }
}

void Robot::enterWorking(void) {
  life_state_ = LifeState::kWorking;
}

void Robot::updateLifeState(uint32_t now) {
  // 三态迁移。每个分支都先处理"退出条件"，再处理"进入条件"。
  switch (life_state_) {
    case LifeState::kDead:
      // 每拍都重新执行 enterDead()：让 Dead 成为"持续维持"的状态，
      // 而不是"只在进入时清零一次"。这样任何外部误写输出都会被下一拍压回 0。
      enterDead();
      // 进入自检的门槛：遥控在线 且 没有急停（拨杆不在 Down）。
      if (rc_online_ && !estop_) enterResurrection(now);
      break;

    case LifeState::kResurrection:
      // 自检期间任何一道安全门破了，立刻退回 Dead（不是留在自检里）。
      if (unsafe_confirmed_) { enterDead(); break; }
      /* 两道条件同时满足才准进 Working：计时到了 且 IMU 真的就绪 */
      // 为什么还要计时：电机使能后舵角回中需要时间，IMU 也需要静置才能标零漂。
      // 为什么还要 imu_ready_：计时只是"等够久"，不代表标定真的成功（可能超时降级）。
      if ((now - resurrection_start_ms_) >= params_.resurrection_hold_ms && imu_ready_) {
        enterWorking();
      }
      break;

    case LifeState::kWorking:
      // Working 是最危险的状态，所以退出条件最多：遥控掉、急停、IMU 失效，
      // 任意一条**持续**成立（kUnsafeConfirmMs）就退回 Dead 并把输出清零。
      // 注意：输出的清零不看这个去抖 —— 见 update() 里 setEnabled 用的是 unsafe_active_。
      if (unsafe_confirmed_) enterDead();
      break;

    default:
      enterDead();
      break;
  }
}

void Robot::updateMode(void) {
  // 右拨杆三档映射到三种底盘模式；没有遥控器时给最保守的"分离"。
  if (rc_ == nullptr && !rc_from_relay_) { mode_ = ChassisMode::kDepart; return; }
  switch (RightSwitch()) {
    case hello_world::remote_control::kSwitchStateUp:   mode_ = ChassisMode::kDepart; break;
    case hello_world::remote_control::kSwitchStateMid:  mode_ = ChassisMode::kFollow; break;
    case hello_world::remote_control::kSwitchStateDown: mode_ = ChassisMode::kGyro; break;
    default: mode_ = ChassisMode::kDepart; break;
  }
}

void Robot::updateMotion(void) {
  motion_.reset();
  if (life_state_ != LifeState::kWorking) return;
  if (rc_ == nullptr && !rc_from_relay_) return;
  if (ctrl_source_ != CtrlSource::kRemote) return;

  /* 平移：**通道分配 2026-11-18 现场重排，一根杆只管一件事**。
   * 起因：原来底盘用 lv 当"前后"、rh 当"自转"，而云台板正在用 lv 当 pitch、
   * rh 当 yaw —— 同一根杆被两处抢用（用户现场指出）。
   * 最终分配（已确认）：
   *   云台 pitch <- 左杆竖直 lv（Gimbal/robot.cpp，底盘不碰）
   *   云台 yaw   <- 左杆水平 lh（云台板，待其固件更新）
   *   底盘 vx    <- 右杆竖直 rv（推前为正）
   *   底盘 vy    <- 右杆水平 rh（推右为正）
   *   底盘 w     <- 拨轮（仅分离模式用，不占摇杆）
   * 于是：左手一根杆 = 云台，右手一根杆 = 车，互不干扰。
   * 注意 (vx, vy) 的坐标系由 Chassis 按模式决定是否乘 δ（分离=车体，跟随/小陀螺=云台）。
   * 三步处理：先夹到 ±1（吸收 DT7 实际量程差异）→ 过死区 → 乘上限。 */
  motion_.vx = Deadband(ClampNorm(StickRv()), params_.remote_deadband) * params_.max_vx;
  motion_.vy = Deadband(ClampNorm(StickRh()), params_.remote_deadband) * params_.max_vy;

  switch (mode_) {
    case ChassisMode::kDepart:
      /* 分离模式：原地自转由**拨轮**控制（不占摇杆通道）。
       * 拨轮还没进 0x212 转发帧（8 字节已满），因此现在恒为 0；
       * 云台板更新后 StepWheel() 会给出真实值，这一行不用再改。 */
      motion_.w = Deadband(ClampNorm(StickWheel()), params_.remote_deadband) * params_.max_w_manual;
      break;
    case ChassisMode::kFollow:
      /* δ 由双板帧喂入；未接入前 δ 恒 0 → w 恒 0（安全缺省②）。 */
      motion_.w = params_.follow_kp * yaw_error_;
      break;
    case ChassisMode::kGyro:
      motion_.w = params_.gyro_w;
      break;
    default:
      motion_.w = 0.0f;
      break;
  }
}

void Robot::update(void) {
  // 未初始化完就直接返回：此时各模块指针可能还是空的。
  if (!inited_) return;
  /* 姿态链必须每拍真的更新一次：只查 isReady() 而不调 update() 的话，
   * 零偏标定永远不推进，状态机永远进不了 Working（这也是 nm 查出来的死代码）。 */
  if (imu_ != nullptr) {
    imu_->update();
  }

  const uint32_t now = hello_world::tick::GetTickMs();

  // ★ 调用顺序不能换，每一步都依赖上一步的结论：
  //   updateRemote    → 决定 rc_online_ / estop_ / peer_offline_ / imu_ready_ / δ
  //   updateLifeState → 用上面这些标志做状态迁移（决定这一拍能不能动）
  //   updateMode      → 读右拨杆，决定 w 的来源
  //   updateMotion    → 依赖 life_state（非 Working 直接给 0）与 mode_
  updateRemote();

  /* "不安全"三条件的确认去抖（见 robot.hpp 里的说明）。
   * 一帧噪声不该把 Working 打回 Dead 再走一遍自检；真实故障会持续存在，100 ms 内必然被确认。 */
  {
    const bool unsafe_raw = (!rc_online_) || estop_ || (!imu_ready_);
    if (unsafe_raw) {
      if (!unsafe_active_) {
        unsafe_active_ = true;
        unsafe_since_ms_ = now;
        /* 记录"第一次出现不安全"时到底是哪一条（0=遥控离线 1=急停 2=IMU未就绪），
         * 供现场用 J-Link 直接读，省掉反复猜。 */
        g_dbg_dead_reason = static_cast<uint8_t>((!rc_online_) ? 0u : (estop_ ? 1u : 2u));
      }
      unsafe_confirmed_ = (now - unsafe_since_ms_) >= kUnsafeConfirmMs;
    } else {
      unsafe_active_ = false;
      unsafe_confirmed_ = false;
    }
  }

  updateLifeState(now);
  updateMode();
  updateMotion();

  if (chassis_ != nullptr) {
    // 先下发目标，再设使能，最后让底盘算这一拍：
    //   setTarget  → 这一拍的速度指令
    //   setEnabled → 只有 Working 才允许输出（false 会立刻清 0）
    //   update()   → 限幅 → δ 变换 → 逆解 → 8 个 PID → setInput
    chassis_->setTarget(motion_);
    /* 平移方向的参考系按模式走（考核第 6 项）：分离=车头系，跟随/小陀螺=云台系。
     * 以前这个是常量 true（三种模式都在云台系），分离模式形同虚设。 */
    chassis_->setRotateByDelta(mode_ != ChassisMode::kDepart);
    /* 使能条件比"状态机进 Working"更严一层：**任何一帧**不安全（遥控掉/急停/IMU 失）都立刻不使能，
     * 输出当拍归零 —— 安全动作零延迟。而"掉回 Dead、重走一遍自检"要看持续 100 ms，
     * 避免一帧抖动就让车停一秒（见 updateRemote 后面的去抖）。 */
    chassis_->setEnabled(life_state_ == LifeState::kWorking && !unsafe_active_);
    chassis_->update();
  }

  /* 双板帧语义字段的唯一写者（P1-3）。底盘板的 local 帧只承载状态，
   * 角度由云台板填，所以这里不写 yaw/pitch。 */
  if (comm_ != nullptr) {
    // 这四个字段**只有这里能写**（旧代码在 comm_task 里每拍覆盖，导致别人改了也没用）。
    auto &f = comm_->local();
    // online：只要不是 Dead 就自报在线（Dead 时对端应当据此进入保护）。
    f.online = (life_state_ != LifeState::kDead) ? 1u : 0u;
    f.ready = imu_ready_ ? 1u : 0u;
    f.work_mode = ToByte(mode_);
    f.ctrl_mode = ToByte(life_state_);
  }
}

/* ---- 遥控来源访问器（本地 DBUS / 云台转发） -------------------------------
 * 【2026-11-18 现场修复】原来的判据是"本地离线才用转发"（rc_from_relay_），
 * 结果出现一个致命组合：本板 DBUS 物理上是断开的，却报告"在线"
 * （本地对象被别的路径写活了/超时判据没触发），于是：
 *   左拨杆去读**本地的假数据** -> 拨杆值 0 -> 不在白名单 -> 急停恒为 1
 *   现象：遥控在线=1、对端在线=1、IMU ready=1、拨杆实际在合法档位，
 *         但状态机永远停在 Dead，且 dead_reason=1（急停）。
 * 现在改成：**只要转发数据新鲜（300 ms 内 + 校验通过 + online=1）就优先用转发**，
 * 本地 DBUS 只在没有转发时才用。这台车的接收机只接在云台板，所以这条更符合事实，
 * 而且转发帧自带 XOR 校验与超时，失效时自然回落到本地/急停，安全性不降低。 */
bool Robot::UseRelay(void) const {
  return robot::RcRelayFresh() && (robot::g_rc_relay.online != 0u);
}

hello_world::remote_control::SwitchState Robot::LeftSwitch(void) const {
  if (UseRelay()) {
    return static_cast<hello_world::remote_control::SwitchState>(robot::g_rc_relay.s1);
  }
  if (rc_ != nullptr) {
    return rc_->rc_l_switch();
  }
  return hello_world::remote_control::kSwitchStateDown;   /* 未知一律按急停 */
}

hello_world::remote_control::SwitchState Robot::RightSwitch(void) const {
  if (UseRelay()) {
    return static_cast<hello_world::remote_control::SwitchState>(robot::g_rc_relay.s2);
  }
  if (rc_ != nullptr) {
    return rc_->rc_r_switch();
  }
  return hello_world::remote_control::kSwitchStateUp;
}

float Robot::StickLv(void) const {
  if (UseRelay()) return robot::DequantizeStick(robot::g_rc_relay.lv);
  return (rc_ != nullptr) ? rc_->rc_lv() : 0.0f;
}

float Robot::StickLh(void) const {
  if (UseRelay()) return robot::DequantizeStick(robot::g_rc_relay.lh);
  return (rc_ != nullptr) ? rc_->rc_lh() : 0.0f;
}

float Robot::StickRh(void) const {
  if (UseRelay()) return robot::DequantizeStick(robot::g_rc_relay.rh);
  return (rc_ != nullptr) ? rc_->rc_rh() : 0.0f;
}

float Robot::StickRv(void) const {
  if (UseRelay()) return robot::DequantizeStick(robot::g_rc_relay.rv);
  return (rc_ != nullptr) ? rc_->rc_rv() : 0.0f;
}

float Robot::StickWheel(void) const {
  /* 拨轮（分离模式的原地自转）。
   * 现状：0x212 转发帧 8 字节已经排满（头/online+seq/四摇杆/两拨杆/校验），
   * 拨轮塞不进去，所以**经转发路径时恒为 0** —— 即分离模式暂时没有手动自转，
   * 小陀螺模式的自转不受影响。等云台板固件更新（缩 s1/s2 到低位、拨轮占 4 bit，
   * 或新增一帧 0x213）后，把这里改成
   *   return robot::DequantizeStick(robot::g_rc_relay.wheel);
   * 即可，控制逻辑不用动。本地 DBUS（若以后接收机接到底盘）直接可用。 */
  if (UseRelay()) return 0.0f;
  return (rc_ != nullptr) ? rc_->rc_wheel() : 0.0f;
}

}  // namespace robot
