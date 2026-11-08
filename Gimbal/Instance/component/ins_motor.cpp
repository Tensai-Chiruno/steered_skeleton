/**
 *******************************************************************************
 * @file      :ins_motor.cpp
 * @brief     :云台板电机实例（yaw / pitch DM-J4310）+ CAN 帧分发/打包
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
  yaw_.init(kYawId, opt_yaw, true, false);

  /* pitch：只用关节角的一小段，限位由上层用反馈角度做，不在这里卡死 */
  hello_world::motor::OptionalParams opt_pitch;
  opt_pitch.input_type = hello_world::motor::InputType::kTorq;
  opt_pitch.dir = hello_world::motor::kDirRev;  // 考核说明 6.2：pitch 反向
  opt_pitch.ex_redu_rat = 1.0f;
  opt_pitch.remove_build_in_reducer = false;
  opt_pitch.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
  opt_pitch.angle_offset = kPitchOffset;
  pitch_.init(kPitchId, opt_pitch, true, false);

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

  uint8_t buf[8];

  /* DM 是「一台电机一帧」，不像 GM6020/M3508 那样 4 台共用，
   * 所以不需要清零整块缓冲，但 encode 是否写满 8 字节由设备类保证。
   * 这里仍然清零，避免设备类只填部分字节时把栈上垃圾发出去。 */
  for (int i = 0; i < 8; ++i) {
    buf[i] = 0;
  }
  yaw_.encode(buf);
  CAN_Send_Msg(kMotorBus, buf, yaw_.tx_id(), 8);

  for (int i = 0; i < 8; ++i) {
    buf[i] = 0;
  }
  pitch_.encode(buf);
  CAN_Send_Msg(kMotorBus, buf, pitch_.tx_id(), 8);
}

/* Private function definitions ----------------------------------------------*/
