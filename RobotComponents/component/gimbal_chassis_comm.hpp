/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.hpp
 * @brief     :底盘↔云台双板 CAN 通讯（考核第 9 项）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 帧布局 + 编解码 + 离线判断
 *******************************************************************************
 * @attention :
 *  【分工】
 *    字节布局与缩放 → gimbal_chassis_comm_codec.{hpp,cpp}（纯函数，可主机端单测）
 *    本类            → 方向（code_part）、收发 ID、离线计时
 *    这样拆分的原因见 codec 头文件的注释：布局容易错、且能离线证明；
 *    而 tick / 在线判断只能上板验。
 *
 *  【方向约定】
 *    底盘板实例化：CodePart::Chassis，tx = 0x101，rx = 0x102
 *    云台板实例化：CodePart::Gimbal， tx = 0x102，rx = 0x101
 *    我方要发的那份放在「属于本板」的缓存里：
 *      Chassis 板 → chassis_data() 是本地（期望云台角），gimbal_data() 是收到的反馈
 *      Gimbal  板 → gimbal_data() 是本地（实际角反馈），chassis_data() 是收到的期望
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_CHASSIS_COMM_HPP_
#define GIMBAL_CHASSIS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "gimbal_chassis_comm_codec.hpp"
#include "tick.hpp"

/* Exported macro ------------------------------------------------------------*/
namespace robot {
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* 底盘↔云台双板 CAN 通讯协议类。底盘板与云台板各实例化一个，用 code_part 区分方向。 */
class GimbalChassisComm {
 public:
  /* 标记本实例运行在哪块板，决定 encode / decode 的方向。 */
  enum class CodePart : uint8_t { Chassis = 0, Gimbal = 1 };

  /* 两个方向的语义字段相同（在线/就绪/模式/yaw/pitch），只是含义不同，
   * 因此统一用 Frame，保留 ChassisData / GimbalData 两个别名以维持原有接口。 */
  using Frame = hello_world::comm::Frame;
  using ChassisData = Frame;
  using GimbalData = Frame;

  /** 超过这个时长没收到对方帧即判离线（1 ms 周期下约 100 帧） */
  static constexpr uint32_t kOfflineTimeoutMs = 100;

  GimbalChassisComm(CodePart code_part, uint32_t tx_id, uint32_t rx_id);

  /* 编码己方帧：把本地 ChassisData / GimbalData 打包进 8 字节。 */
  void encode(uint8_t tx_data[8]);
  /* 解码对方帧：把 8 字节还原成对方语义并更新本地缓存。校验失败时丢弃。 */
  void decode(const uint8_t rx_data[8]);

  /* 判断对方是否离线（超过阈值时长未收到新帧）。 */
  bool isOffline() const;

  /* 数据访问（供 RobotModules / Task 读写）。 */
  ChassisData &chassis_data() { return chassis_data_; }
  GimbalData &gimbal_data() { return gimbal_data_; }
  const ChassisData &chassis_data() const { return chassis_data_; }
  const GimbalData &gimbal_data() const { return gimbal_data_; }

  /** 本方要发出去的那份缓存（按 code_part 选择）。 */
  Frame &local() {
    return (code_part_ == CodePart::Chassis) ? chassis_data_ : gimbal_data_;
  }
  /** 收到的对方缓存。 */
  const Frame &remote() const {
    return (code_part_ == CodePart::Chassis) ? gimbal_data_ : chassis_data_;
  }

  uint32_t tx_id() const { return tx_id_; }
  uint32_t rx_id() const { return rx_id_; }

  /** 发送序列号（每次 encode 自增，供对面发现「卡住不发新帧」） */
  uint8_t sequence() const { return seq_; }
  /** 校验失败的累计帧数（调试用；正常应为 0 或极少） */
  uint32_t bad_frame_count() const { return bad_frame_count_; }

 private:
  CodePart code_part_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  uint32_t last_rx_tick_;  ///< 上次收到对方帧的 GetTickMs() 时间戳
  uint8_t seq_ = 0;
  uint32_t bad_frame_count_ = 0;
  ChassisData chassis_data_;
  GimbalData gimbal_data_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot

#endif /* GIMBAL_CHASSIS_COMM_HPP_ */
