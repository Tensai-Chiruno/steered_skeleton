/**
 *******************************************************************************
 * @file      :ins_motor.cpp
 * @brief     :底盘板电机实例 + CAN 帧分发/打包
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 4xM3508 + 4xGM6020 实例化，
 *                                                 接收分发与共享帧打包
 *******************************************************************************
 * @attention :
 *  【本题踩的两个坑，都写在这里免得以后忘】
 *
 *  1) 共享发送帧：M3508 的 4 台共用一个 0x200、GM6020 的 4 台共用一个 0x1FF。
 *     encode() 只写自己那 2 个字节（data[2*((id-1)%4)] 与 +1），不碰其余 6 字节。
 *     所以必须：清零 → 4 台依次 encode 进同一缓冲 → 发一次。
 *     若逐台各发一帧，后面的会覆盖前面的，现象是「只有一台电机有反应」。
 *
 *  2) 电机 ID 与总线：id 决定它在共享帧里的字节段，不是接线顺序。
 *     轮电机 id 1~4 → 0x200 的第 0/2/4/6 字节；舵电机 id 1~4 → 0x1FF 同样。
 *     实机上 id 由电调拨码/上位机设定，必须与这里一致，否则轮子会拿到别人的指令。
 *
 *  【总线归属是当前唯一的假设】
 *     考核说明给了 CAN1(Fifo0) 与 CAN2(Fifo1) 两条总线，但没写哪个电机挂哪条。
 *     这里默认「全部电机在 CAN1」，双板通讯帧留给 CAN2（阶段 4 接入）。
 *     实机若不同，只改下面的 kWheelBus / kSteerBus 两个常量即可，别的地方不用动。
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

constexpr int kWheelNum = 4;  ///< 轮电机（M3508）数量
constexpr int kSteerNum = 4;  ///< 舵电机（GM6020）数量

/** 各电机组所在的总线。**按实车拓扑图**（2026-11-08 用户提供）：
 *    CAN2 = 主干：轮电机 M3508、Yaw、摩擦轮、拨弹、以及双板通讯
 *    CAN1 = 只在底盘侧，仅挂 4 台舵电机 GM6020
 *  所以轮电机在 CAN2、舵电机在 CAN1。两者不同总线，LocateMotor 用总线区分是成立的。
 *  实机若与此不符，只改这两行。 */
CAN_HandleTypeDef *const kWheelBus = &hcan2;
CAN_HandleTypeDef *const kSteerBus = &hcan1;

/* 轮电机：id 1~4，外接减速比 14，且不自带减速器 */
constexpr uint8_t kWheelId[kWheelNum] = {1, 2, 3, 4};
constexpr hello_world::motor::Dir kWheelDir[kWheelNum] = {
    hello_world::motor::kDirFwd, hello_world::motor::kDirFwd,
    hello_world::motor::kDirFwd, hello_world::motor::kDirFwd};

/* 舵电机：id 1~4，保留内置减速器（ex_redu_rat = 1），方向全部反向。
 * 舵向零点偏移来自考核说明 6.2「已按实机配置统一补齐」的固定值，必须照抄：
 *   缺了它，舵角反馈就不是车体坐标系下的舵角，逆解算出来的目标角会整体偏掉，
 *   现象是「四轮都在转但车身斜着走」。 */
constexpr uint8_t kSteerId[kSteerNum] = {1, 2, 3, 4};
constexpr hello_world::motor::Dir kSteerDir[kSteerNum] = {
    hello_world::motor::kDirRev, hello_world::motor::kDirRev,
    hello_world::motor::kDirRev, hello_world::motor::kDirRev};
constexpr float kSteerOffset[kSteerNum] = {2.61346f, -2.06844f, 0.00729f,
                                           1.58518f};  // LF / LB / RB / RF

}  // namespace

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

namespace {

hello_world::motor::M3508 wheel_[kWheelNum];
hello_world::motor::GM6020 steer_[kSteerNum];
bool inited_ = false;

/** 一次性构造全部电机对象。首次访问任何 GetMotorXxx 时执行。 */
void EnsureInit(void) {
  if (inited_) {
    return;
  }
  for (int i = 0; i < kWheelNum; ++i) {
    hello_world::motor::OptionalParams opt;
    opt.input_type = hello_world::motor::InputType::kCurr;
    opt.dir = static_cast<hello_world::motor::Dir>(kWheelDir[i]);
    opt.ex_redu_rat = 14.0f;              // M3508 外接减速比
    opt.remove_build_in_reducer = true;   // M3508 内置减速器已计入 14，不再重复
    opt.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf;
    wheel_[i].init(kWheelId[i], opt);
  }
  for (int i = 0; i < kSteerNum; ++i) {
    hello_world::motor::OptionalParams opt;
    opt.input_type = hello_world::motor::InputType::kCurr;
    opt.dir = static_cast<hello_world::motor::Dir>(kSteerDir[i]);
    opt.ex_redu_rat = 1.0f;               // GM6020 保留内置减速器
    opt.remove_build_in_reducer = false;
    opt.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
    steer_[i].init(kSteerId[i], opt);
  }

  /* 把期望的 CAN ID 钉死在这里。
   * 这四行不产生运行开销（Debug 下断言，Release 下通常被编译掉），
   * 但它把「协议表」写成了可执行的检查：以后谁把 input_type 从 kCurr 改成
   * kRaw、或把 id 改成 5~8，上电第一帧就会在这里炸掉，而不是在实验室里
   * 表现为「指令给了电机不动」或「动得很怪」。 */
  HW_ASSERT(hello_world::motor::M3508::GetTxId(1) == 0x200, "wheel tx id");
  HW_ASSERT(hello_world::motor::M3508::GetRxId(1) == 0x201, "wheel rx id");
  HW_ASSERT(hello_world::motor::GM6020::GetTxId(
                kSteerId[0], hello_world::motor::InputType::kRaw) == 0x1FF,
            "steer tx id (kRaw -> 0x1FF)");
  HW_ASSERT(hello_world::motor::GM6020::GetRxId(1) == 0x205, "steer rx id");

  inited_ = true;
}

}  // namespace

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

namespace {

/** 在一条总线上按反馈 ID 找电机；找不到返回 nullptr。 */
hello_world::motor::Motor *LocateMotor(CAN_HandleTypeDef *hcan, uint32_t id) {
  if (hcan == kWheelBus) {
    for (int i = 0; i < kWheelNum; ++i) {
      if (hello_world::motor::M3508::GetRxId(kWheelId[i]) == id) {
        return &wheel_[i];
      }
    }
  }
  if (hcan == kSteerBus) {
    for (int i = 0; i < kSteerNum; ++i) {
      if (hello_world::motor::GM6020::GetRxId(kSteerId[i]) == id) {
        return &steer_[i];
      }
    }
  }
  return nullptr;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *GetMotorWheel(int idx) {
  HW_ASSERT(idx >= 0 && idx < kWheelNum, "Error wheel idx: %d", idx);
  EnsureInit();
  return &wheel_[idx];
}

hello_world::motor::Motor *GetMotorSteer(int idx) {
  HW_ASSERT(idx >= 0 && idx < kSteerNum, "Error steer idx: %d", idx);
  EnsureInit();
  return &steer_[idx];
}

void MotorDispatchCanFrame(CAN_HandleTypeDef *hcan, uint32_t std_id,
                           const uint8_t data[8]) {
  EnsureInit();

  hello_world::motor::Motor *m = LocateMotor(hcan, std_id);
  if (m != nullptr) {
    m->decode(data);
    return;
  }

  /* 未命中：双板通讯帧（0x101 / 0x102）在阶段 4 接入这里。
   * 现阶段直接丢弃——静默丢弃比误判成电机更安全。 */
}

void MotorSendAllCommands(void) {
  EnsureInit();

  uint8_t buf[8];

  /* ---- 轮电机：4 台共用 0x200，各占 2 字节 ---- */
  for (int i = 0; i < 8; ++i) {
    buf[i] = 0;
  }
  for (int i = 0; i < kWheelNum; ++i) {
    wheel_[i].encode(buf);
  }
  CAN_Send_Msg(kWheelBus, buf, hello_world::motor::M3508::GetTxId(kWheelId[0]), 8);

  /* ---- 舵电机：4 台共用一帧，各占 2 字节 ---- */
  for (int i = 0; i < 8; ++i) {
    buf[i] = 0;
  }
  for (int i = 0; i < kSteerNum; ++i) {
    steer_[i].encode(buf);
  }
  /* GM6020 用「帧 ID」区分这个 int16 是电压还是电流：
   *   0x1FF = kRawTx1_4_（电压域）、0x1FE = kCurrTx1_4_（电流域）。
   * 本车按考核说明 2.3 用默认的电压控制 kRaw，所以走 0x1FF。
   * 若将来在上位机打开电流环，必须同时把 input_type 改成 kCurr，
   * 只改帧 ID 不改量纲会让指令被差一个比例。 */
  CAN_Send_Msg(kSteerBus, buf,
               hello_world::motor::GM6020::GetTxId(
                   kSteerId[0], hello_world::motor::InputType::kRaw),
               8);
}

/* Private function definitions ----------------------------------------------*/
