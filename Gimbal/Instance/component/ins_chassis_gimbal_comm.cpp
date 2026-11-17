/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.cpp
 * @brief     :本板双板通讯对象实现（Gimbal）
 * @category : 功能 | 考核第 8 项（双板实例与 CAN ID）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_gimbal_comm.hpp"

#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "HW_can.hpp"
#include "ins_comm.hpp"
#include "ins_comm_state.hpp"
#include "rc_relay.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace {

/* 云台板：发送 0x211（实际角反馈），接收 0x210（期望角） */
constexpr uint32_t kC2gId = 0x210;  // 考核说明 3.4 建议值
constexpr uint32_t kG2cId = 0x211;  // 必须避开 0x200/0x201~0x208/0x01~0x1F

/* 双板通讯总线 —— **2026-11-17 现场实测改成 CAN1**（原先是 CAN2，文档也是这么写的，但实车不是）。
 *
 * 现场怎么测出来的（tools/diag_can_sniff.py 把过滤器临时设成全收，把总线上的 ID 列出来）：
 *   云台板 **CAN1** 上同时看到 0x200（轮电机控制帧）、0x202/0x203/0x204（M3508 反馈）、
 *   **以及 0x210（底盘 → 云台的板间帧）**；
 *   而云台板 **CAN2** 上只有达妙电机的应答。
 *   ⇒ 实车拓扑是：**云台 CAN1 ↔ 底盘 CAN2（主干）；云台 CAN2 ↔ 两台达妙电机**。
 *
 * 之前配在 CAN2 的后果：底盘发的 0x210 落在云的 CAN1 上，
 * BoardCommDispatch 判 `hcan != kBoardBus` 直接 return → 永远"对端离线"
 * → 而 robot.cpp 有"对端离线强制急停"的互保 → 状态机永远进不了 Working。
 * 现象就是"明明都通电、遥控也正常，车就是不动"。 */
CAN_HandleTypeDef *const kBoardBus = &hcan1;

robot::GimbalChassisComm g_comm(robot::GimbalChassisComm::CodePart::Gimbal, kG2cId, kC2gId);

}  // namespace

/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm *GetGimbalChassisComm() { return &g_comm; }

/* 【现场修复 2026-11-17】把"板间帧该走哪条总线"暴露出去。
 * 之前 comm_task.cpp 里把发送写死成 &hcan2，而 kBoardBus 已经改成 hcan1 ——
 * 结果"收在 CAN1、发却在 CAN2"，底盘永远收不到云台的 0x211（现场：对端离线=1）。 */
CAN_HandleTypeDef *GetBoardCommBus(void) { return kBoardBus; }

/* 遥控转发：把云台板收到的遥控数据打包成 8 字节发到底盘（见 rc_relay.hpp）。
 * 接收机的信号线只接到云台板，而底盘要动必须有遥控 —— 硬件不能改线，只能这样转发。 */
void SendRcRelay(void) {
  static uint8_t seq = 0;
  const auto *rc = GetRemoteControl();
  if (rc == nullptr) {
    return;
  }
  uint8_t buf[8];
  const bool online = IsRemoteOnline();
  robot::PackRcRelay(online ? 1u : 0u, rc->rc_lv(), rc->rc_lh(), rc->rc_rv(), rc->rc_rh(),
                     static_cast<uint8_t>(rc->rc_l_switch()),
                     static_cast<uint8_t>(rc->rc_r_switch()), seq, buf);
  CAN_Send_Msg(kBoardBus, buf, robot::kRcRelayTxId, 8);
  ++seq;
}

void BoardCommDispatch(CAN_HandleTypeDef *hcan, uint32_t std_id,
                       const uint8_t data[8]) {
  if (hcan != kBoardBus || std_id != g_comm.rx_id()) {
    return;
  }
  g_comm.decode(data);
}

/* Private function definitions ----------------------------------------------*/
