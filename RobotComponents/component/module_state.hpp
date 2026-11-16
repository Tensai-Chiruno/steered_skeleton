/**
 *******************************************************************************
 * @file      :module_state.hpp
 * @brief     :两板共享的状态枚举与运动结构（考核第 5 项）
 * @category : 功能 | 考核第 5 项（两板共享类型）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *  V1.1.0      2026-11-16      Tensai-Chiruno  1. 逐句注释
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifndef ROBOT_COMPONENTS_MODULE_STATE_HPP_
#define ROBOT_COMPONENTS_MODULE_STATE_HPP_

/* Includes ------------------------------------------------------------------*/
// 只用到定长整型（uint8_t）。刻意不引任何 HAL / 电机 / CAN 头文件：
// 本文件被底盘板和云台板**同时**编译，一旦引入硬件依赖，另一块板就编不过。
#include <cstdint>

/* ============================================================================
 * 本文件是整个工程「共享词汇表」：两板要说同一套话，就必须共用这一份定义。
 *
 * 为什么单独立一个文件（考核第 5 项要求的正是这个）：
 *   1. 底盘板与云台板是两份**独立固件**，唯一的信息通道是 CAN 帧；
 *      如果两边各自定义一套枚举，取值一漂移就会出现「对面其实在自检、
 *      我却当成正常工作」这类静默错误。
 *   2. 所以：枚举 → 字节的映射只写在 ToByte() / FromByte() 里，全工程唯一。
 *
 * 允许放什么：enum class、POD 结构体、constexpr、inline 小函数。
 * 禁止放什么：任何 HAL 类型、任何全局可变状态（本文件里那个 volatile 标志除外，
 *              它是跨层共享的"准入开关"，见下面说明）。
 * ==========================================================================*/

namespace robot {

/* ----------------------------------------------------------------------------
 * 生命状态：上电未就绪 → 恢复自检 → 正常工作（考核 3.5）
 *
 * 为什么是这三态而不是「开机/运行」两态：
 *   上电瞬间 IMU 还没收敛、舵角还在任意位置、电机刚使能可能有残留指令。
 *   如果直接进「运行」，一旦操作手在这几百毫秒里推了摇杆，车就会带着
 *   未标定的姿态和随机的舵角动起来。所以中间必须插一个"自检期"：
 *   期间电机已使能（用于回中/收敛），但不接受运动指令。
 *
 * 谁会读它：Robot 的迁移逻辑、双板帧的 ctrl_mode 字段、PC8 状态灯、调试快照。
 * --------------------------------------------------------------------------*/
enum class LifeState : uint8_t {
  kDead = 0,          ///< 上电未就绪 / 已急停 / 遥控离线：**所有电机输出归零**
  kResurrection = 1,  ///< 恢复自检：电机已使能，等 IMU 零漂收敛 + 计时；不接受运动指令
  kWorking = 2,       ///< 正常工作：只有这个状态才允许按遥控器输出
};

/**
 * @brief 是否允许使用「位置保持类」电机接口（如 DaMiao::setMitInput）。
 *
 * 只有状态机处于 Working 时才置 true。原因（G-1）：
 * 达妙的 setInput() 会强制 kp = kd = 0，所以「急停 = 零力矩 ≈ 松掉」成立；
 * 但一旦有人改用 setMitInput()（kp != 0 做位置保持），电机就会死守最后的目标位，
 * 急停立刻变成危险态。把这个权限做成全局唯一标志，motor 层可以直接断言。
 *
 * 为什么用 volatile：它在云台板的中断上下文里被写、在电机接口里被读，
 * 编译器不能把它优化进寄存器。为什么是 inline 变量：C++17 起 inline 变量
 * 在头文件里只有一份实体，两板各自编译也不会重复定义。
 */
inline volatile bool g_mit_input_allowed = false;

/* ----------------------------------------------------------------------------
 * 控制源：这一拍的运动指令从哪来。
 *
 * 现在只有遥控器一种，但保留枚举是因为考核 1.2 提到"手动控制源"这一层抽象：
 * 以后接键鼠/上位机时，状态机只需要多一个来源，不必改动运动生成逻辑。
 * --------------------------------------------------------------------------*/
enum class CtrlSource : uint8_t {
  kNone = 0,    ///< 没有任何有效输入源（遥控离线等）→ 输出必须归零
  kRemote = 1,  ///< 遥控器 DT7（唯一实现了的入口）
};

/* ----------------------------------------------------------------------------
 * 底盘工作模式（考核 3.1，验收会逐个切换验证）
 *
 * 三者的区别在「w（自转角速度）从哪来」，平移 (vx, vy) 都来自摇杆：
 *   分离   w 也是摇杆直接给的，底盘不自动回正、不自旋（操作手完全掌控）
 *   跟随   w 由「云台朝向 − 车头朝向」的闭环算出，车头自动对准云台
 *   小陀螺 w 是限幅后的常值，车原地自旋；平移方向仍以云台为基准
 * --------------------------------------------------------------------------*/
enum class ChassisMode : uint8_t {
  kDepart = 0,  ///< 分离：底盘速度全部来自遥控器直接映射，不自动回正
  kFollow = 1,  ///< 跟随：车头自动对准云台（w 由 δ 闭环给出）
  kGyro = 2,    ///< 小陀螺：w 为限幅后的常值，平移方向仍以云台为基准
};

/* ----------------------------------------------------------------------------
 * 云台工作模式：本次只做关节空间控制（给定 yaw/pitch 目标角）。
 * 保留枚举是为了和底盘侧对称，也方便以后加"云台跟随底盘"等模式。
 * --------------------------------------------------------------------------*/
enum class GimbalMode : uint8_t {
  kJoint = 0,  ///< 关节空间：直接按目标关节角做位置环
};

/* ----------------------------------------------------------------------------
 * 底盘运动状态。
 *
 * ★ 单位与坐标系（考核 3.2 的硬性约定，写错这一条整台车方向都会错）：
 *   - vx：前向速度，m/s，**x 轴 = 云台朝向（枪管方向）**，不是车头朝向；
 *   - vy：左向速度，m/s，y 轴 = 云台左方（右手系，z 向上）；
 *   - w ：绕 z 轴角速度，rad/s，**逆时针为正**（从车顶往下看）。
 *
 *   也就是说：操作手推"前"，车永远朝云台指的方向走，与车头转到哪里无关。
 *   底盘解算前必须先用 δ（云台相对车头角）把它旋到车体坐标系，
 *   这一步在 robot::Chassis::update() 里调 gimbalToChassis() 完成。
 *
 * 为什么要运算符重载（考核第 5 项明确要求"给出加减、数乘"）：
 *   运动指令在代码里是"分量叠加"的关系——比如跟随模式下平移来自摇杆、
 *   自转来自 δ 闭环，两者要合成一个 ChassisMotion 再下发。
 *   有了运算符，生成逻辑可以写成 `motion = stick + followTerm;`，
 *   而不是三次手工赋值（手工赋值最容易漏掉某一分量）。
 * --------------------------------------------------------------------------*/
struct ChassisMotion {
  float vx = 0.0f;  ///< 前向，m/s（x = 云台朝向）
  float vy = 0.0f;  ///< 左向，m/s（y = 云台左方）
  float w = 0.0f;   ///< 绕 z 角速度，rad/s（逆时针为正）

  /// 分量相加：把两组运动指令叠加（如"摇杆平移 + 跟随自转"）。
  ChassisMotion operator+(const ChassisMotion &o) const {
    return ChassisMotion{vx + o.vx, vy + o.vy, w + o.w};
  }
  /// 分量相减：用于求"目标 − 实测"的偏差，或做增量式更新。
  ChassisMotion operator-(const ChassisMotion &o) const {
    return ChassisMotion{vx - o.vx, vy - o.vy, w - o.w};
  }
  /// 数乘：按比例缩放（如遥控器摇杆归一化值 × 上限 = 实际速度）。
  ChassisMotion operator*(float k) const {
    return ChassisMotion{vx * k, vy * k, w * k};
  }
  /// 原地累加：避免写出三次 `motion.vx += ...` 这种易漏的散装代码。
  ChassisMotion &operator+=(const ChassisMotion &o) {
    vx += o.vx; vy += o.vy; w += o.w; return *this;
  }
  /// 清零：进入 Dead / 非 Working 时必须调用，保证下发的是 0 而不是残留值。
  void reset(void) { vx = 0.0f; vy = 0.0f; w = 0.0f; }
};

/* ----------------------------------------------------------------------------
 * 云台关节状态（yaw / pitch），单位 rad。
 *
 * yaw  ：允许 360° 连续旋转，做误差计算时必须先归一化到 [-π, π]，
 *        否则 359° → 1° 会被当成反向 −358° 的大幅回摆。
 * pitch：有限角度，必须加软件限位（见 Gimbal::Params::pitch_min/max），
 *        越过机械限位会堵转、发烫、甚至扯坏线。
 * --------------------------------------------------------------------------*/
struct GimbalMotion {
  float yaw = 0.0f;    ///< 偏航角，rad（连续旋转，误差需回绕）
  float pitch = 0.0f;  ///< 俯仰角，rad（有限角度，需软件限位）

  /// 两关节目标叠加（例如"当前角 + 摇杆增量"的增量式控制）。
  GimbalMotion operator+(const GimbalMotion &o) const {
    return GimbalMotion{yaw + o.yaw, pitch + o.pitch};
  }
  /// 两关节求差（目标 − 实测），喂给位置环。
  GimbalMotion operator-(const GimbalMotion &o) const {
    return GimbalMotion{yaw - o.yaw, pitch - o.pitch};
  }
  /// 数乘（摇杆速率 × 周期 = 本拍角度增量）。
  GimbalMotion operator*(float k) const {
    return GimbalMotion{yaw * k, pitch * k};
  }
  /// 清零（进入 Dead 时用）。
  void reset(void) { yaw = 0.0f; pitch = 0.0f; }
};

/* ----------------------------------------------------------------------------
 * 枚举 ↔ 帧字节的唯一映射。
 *
 * 双板协议里模式字段只占 4 bit（见 gimbal_chassis_comm_codec），
 * 收发两端都必须通过这两个函数换算——只要有一端手写数字，就会出现
 * "两边都觉得自己对、但对面理解成别的模式"的静默错误。
 * --------------------------------------------------------------------------*/

/// 枚举 → 字节（发帧侧用）。
inline uint8_t ToByte(LifeState s) { return static_cast<uint8_t>(s); }
inline uint8_t ToByte(CtrlSource s) { return static_cast<uint8_t>(s); }
inline uint8_t ToByte(ChassisMode s) { return static_cast<uint8_t>(s); }
inline uint8_t ToByte(GimbalMode s) { return static_cast<uint8_t>(s); }

/**
 * 字节 → 枚举（收帧侧用）。
 *
 * 关键：**未知取值一律折回最安全的状态**，而不是强转。
 *   生命状态收到 3 以上 → 当 Dead（宁可当对面没就绪，也不能当成 Working）；
 *   底盘模式收到 3 以上 → 当 kDepart（分离最保守，不会自作主张自旋）。
 * 这就是"失效安全方向"：协议出错时，车应该停下来，而不是动起来。
 */
inline LifeState LifeStateFromByte(uint8_t v) {
  return (v <= 2u) ? static_cast<LifeState>(v) : LifeState::kDead;
}
inline ChassisMode ChassisModeFromByte(uint8_t v) {
  return (v <= 2u) ? static_cast<ChassisMode>(v) : ChassisMode::kDepart;
}

}  // namespace robot

#endif /* ROBOT_COMPONENTS_MODULE_STATE_HPP_ */
