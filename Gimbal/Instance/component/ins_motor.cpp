/**
 *******************************************************************************
 * @file      :ins_motor.cpp
 * @brief     :云台板电机实例（yaw / pitch DM-J4310）+ CAN 帧分发/打包
 * @category : 功能 | 考核第 1 项（10 台电机配置 + CAN 分发）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. yaw/pitch 实例化，接收分发与发送
 *******************************************************************************
 * @attention :
 *  【DM 电机的 CAN ID 方向，以设备类为准】
 *    motor_DaMiao.hpp 里写得很明确：
 *      GetTxId(id) = 0x00 + id   → 主控发给电机的控制帧
 *      GetRxId(id) = 0x10 + id   → 电机回给主控的反馈帧
 *    所以 yaw(id=1)：控制 0x01 / 反馈 0x11；pitch(id=2)：控制 0x02 / 反馈 0x12。
 *
 *    注意：考核说明正文里那张「电机配置」表把反馈与控制两列写反了
 *    （表里写「反馈 0x01 / 控制 0x11」）。实现以设备类代码为准，
 *    接线上电后可用 Ozone 抓帧复核一次，确认反馈确实来自 0x11/0x12。
 *
 *  【DM 电机上电处于失能状态】
 *    失能时不下发控制帧也不会有反馈。DM_J4310 构造时 auto_enable = true，
 *    encode() 内部会在需要时自动插入使能指令——所以必须让 CommTask 周期性
 *    调用 encode，电机才会真正被使能。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_motor.hpp"

#include "HW_can.hpp"
#include "assert.hpp"
#include "can.h"
#include "module_state.hpp"  /* robot::g_mit_input_allowed：非 Working 时不发达妙帧 */
#include "tick.hpp"          /* hello_world::tick::GetTickMs()：使能命令限频 */

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace {

constexpr uint8_t kYawId = 1;    ///< yaw 关节 DM-J4310 的 CAN id
constexpr uint8_t kPitchId = 2;  ///< pitch 关节 DM-J4310 的 CAN id

/* 云台两轴的零点偏移与方向，取自考核说明 6.2（已按实机补齐的固定值）。
 * 注意两轴方向不同：yaw 正向、pitch 反向。写反了的表现是
 * 「摇杆往右云台往左」，而闭环本身还是收敛的，所以很容易被误判成摇杆映射问题。 */
constexpr float kYawOffset = 1.60879f;
constexpr float kPitchOffset = 2.91772f;

/** yaw / pitch 所在总线。按实车拓扑图：CAN2 是主干总线，
 *  Yaw(0x01) 与 Pitch(0x02) 都挂 CAN2，与双板通讯同一条总线。
 *  两者 ID 段不冲突（电机反馈 0x11/0x12，双板 0x210/0x211）。 */
CAN_HandleTypeDef *const kMotorBus = &hcan2;

}  // namespace

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

namespace {

hello_world::motor::DM_J4310 yaw_;
hello_world::motor::DM_J4310 pitch_;
bool inited_ = false;

/* 使能命令限频（见 MotorSendAllCommands 里的说明）。
 * 之所以要自己做，是因为驱动里的 auto_enable 是"每帧发现未使能就发一次"，
 * 反馈一不稳就变成 1 kHz 反复使能 -> 电机抽搐。 */
constexpr uint32_t kEnableRetryMs = 500u;
bool g_enable_dirty = true;
uint32_t g_last_enable_ms = 0;

void EnsureInit(void) {
  if (inited_) {
    return;
  }

  /* yaw：连续旋转，角度域用 [-π, π)，过零处理交给上层 WrapPi */
  hello_world::motor::OptionalParams opt_yaw;
  opt_yaw.input_type = hello_world::motor::InputType::kTorq;
  opt_yaw.dir = hello_world::motor::kDirFwd;
  opt_yaw.ex_redu_rat = 1.0f;
  opt_yaw.remove_build_in_reducer = false;
  opt_yaw.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
  opt_yaw.angle_offset = kYawOffset;
  yaw_.init(kYawId, opt_yaw, false, false);  /* auto_enable=false, enable_mode_switch=false：改由 MotorSendAllCommands 限频显式使能 */

  /* pitch：只用关节角的一小段，限位由上层用反馈角度做，不在这里卡死 */
  hello_world::motor::OptionalParams opt_pitch;
  opt_pitch.input_type = hello_world::motor::InputType::kTorq;
  opt_pitch.dir = hello_world::motor::kDirRev;  // 考核说明 6.2：pitch 反向
  opt_pitch.ex_redu_rat = 1.0f;
  opt_pitch.remove_build_in_reducer = false;
  opt_pitch.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
  opt_pitch.angle_offset = kPitchOffset;
  pitch_.init(kPitchId, opt_pitch, false, false);  /* 同上 */

  /* 把期望的 CAN ID 钉死：DM 的反馈恒为 id + 0x10，控制恒为 id。
   * 这条同时是对「考核说明表格写反了」的固化——真实情况以设备类为准，
   * 若上板后反馈来自 0x01/0x02 而不是 0x11/0x12，说明电机上位机里的
   * CAN ID / Master ID 配置反了，需要重新配置电机而不是改这里。 */
  HW_ASSERT(hello_world::motor::DM_J4310::GetTxId(kYawId) == 0x01, "yaw tx id");
  HW_ASSERT(hello_world::motor::DM_J4310::GetRxId(kYawId) == 0x11, "yaw rx id");
  HW_ASSERT(hello_world::motor::DM_J4310::GetTxId(kPitchId) == 0x02, "pitch tx id");
  HW_ASSERT(hello_world::motor::DM_J4310::GetRxId(kPitchId) == 0x12, "pitch rx id");

  inited_ = true;
}

}  // namespace

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

namespace {

hello_world::motor::Motor *LocateMotor(CAN_HandleTypeDef *hcan, uint32_t id) {
  if (hcan != kMotorBus) {
    return nullptr;
  }
  if (hello_world::motor::DM_J4310::GetRxId(kYawId) == id) {
    return &yaw_;
  }
  if (hello_world::motor::DM_J4310::GetRxId(kPitchId) == id) {
    return &pitch_;
  }
  return nullptr;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *GetMotorYaw() {
  EnsureInit();
  return &yaw_;
}

hello_world::motor::Motor *GetMotorPitch() {
  EnsureInit();
  return &pitch_;
}

void MotorDispatchCanFrame(CAN_HandleTypeDef *hcan, uint32_t std_id,
                           const uint8_t data[8]) {
  EnsureInit();

  hello_world::motor::Motor *m = LocateMotor(hcan, std_id);
  if (m != nullptr) {
    m->decode(data);
    return;
  }

  /* 未命中：双板通讯帧（0x210 / 0x211）在阶段 4 接入这里。 */
}

void MotorSendAllCommands(void) {
  EnsureInit();

  /* 【安全】非 Working 期间，**一帧都不发给达妙电机**。
   *
   * 为什么必须这样（2026-11-17 现场实测踩到的）：
   *   DaMiao::encode() 里有 auto_enable 逻辑：只要反馈显示电机"未使能"，就发一帧
   *   使能命令（0xFF×8）。而一旦接收链路有问题（收不到反馈帧），is_enabled 永远是
   *   false → **每 1 ms 发一次使能命令**，电机被反复使能，现场现象就是云台顶部
   *   电机"反复抽搐"。这种抽搐是真实力矩输出，既危险，也不是我们要的行为。
   *
   * 现在的策略：
   *   - Dead / Resurrection：不发任何电机帧（电机保持原状，或按其自身超时松力）
   *   - Working：正常发 MIT 帧
   * 与"急停 = 零力矩 ≈ 松掉"的设计一致，也避免"没反馈就狂发使能"这条隐患。 */
  if (!robot::g_mit_input_allowed) {
    g_enable_dirty = true;   /* 下次进 Working 要重新使能 */
    return;
  }

  /* 【安全】使能命令限频。
   *
   * 为什么：DaMiao 驱动里的 auto_enable 只要反馈显示"未使能"就每帧发一次使能命令，
   * 而一旦使能状态/反馈有任何不稳，就会变成"每 1 ms 反复使能"→ 电机反复抽搐
   * （2026-11-17 现场两次踩到；第一次在 Dead、第二次在 Working）。
   *
   * 现在把 auto_enable / enable_mode_switch 都关掉（见 EnsureInit），改由这里显式控制：
   *   - 进入 Working 时补发一次使能；
   *   - 之后每 500 ms 最多补一次"仍未使能"的使能命令；
   *   - 绝不在同一控制周期里重复发。
   * 频率从 1000 Hz 降到 2 Hz，且语义清晰：我们只在"想让它使能"的时候才发。 */
  const uint32_t now_ms = hello_world::tick::GetTickMs();
  if (g_enable_dirty || (now_ms - g_last_enable_ms) >= kEnableRetryMs) {
    g_enable_dirty = false;
    g_last_enable_ms = now_ms;
    if (!yaw_.is_enabled()) {
      yaw_.enable();
    }
    if (!pitch_.is_enabled()) {
      pitch_.enable();
    }
  }

  uint8_t buf[8];

  /* 【极其重要】encode() 返回 false 时**绝不能发送**。
   *
   * 2026-11-17 现场："云台一下子猛地砸到最猛的地方"。
   * 根因就在这里：DaMiao::encode() 在 `current_ctrl_mode` 还未知（刚上电、反馈没到）
   * 时会走 `default: return false;` —— **一个字节都不写**。
   * 而 MIT 帧的力矩字段是"无符号 12 位居中表示"：**全 0 = 最大负力矩**。
   * 于是"缓冲清零 + 发送"等于给电机下了一记满力矩反向指令 -> 猛地砸到极限。
   *
   * 现在：只有 encode() 真的生成了帧才发；没生成就一个字都不发。
   * 这一条同时也是"绝不对未知状态电机瞎发指令"的通用纪律。 */
  for (int i = 0; i < 8; ++i) {
    buf[i] = 0;
  }
  if (yaw_.encode(buf)) {
    CAN_Send_Msg(kMotorBus, buf, yaw_.tx_id(), 8);
  }

  for (int i = 0; i < 8; ++i) {
    buf[i] = 0;
  }
  if (pitch_.encode(buf)) {
    CAN_Send_Msg(kMotorBus, buf, pitch_.tx_id(), 8);
  }
}

/* Private function definitions ----------------------------------------------*/
