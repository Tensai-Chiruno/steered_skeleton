/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm_codec.hpp
 * @brief     :双板通讯帧的纯字节编解码（无 tick / 无 HAL 依赖）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 8 字节帧布局 + 校验
 *******************************************************************************
 * @attention :
 *  【为什么把编解码单独拆出来】
 *    GimbalChassisComm 需要 tick 来做离线判断，而 tick 依赖 HAL，于是整个类没法
 *    在电脑上编译。但真正容易写错的是**字节布局**（偏移、字节序、缩放、正负号），
 *    时序反而是最简单的部分。
 *    所以：布局与缩放在本文件（纯函数，可主机端单测），
 *          tick/在线判断留在 GimbalChassisComm（只能上板验）。
 *    这样 pc_test 就能证明「同一结构 encode 后 decode 能原值还原」，而那正是
 *    双板联调最常见的 bug 来源。
 *
 *  【帧布局（8 字节，两个方向共用）】
 *    [0]     flags   ：bit0 = 在线，bit1 = 就绪
 *    [1]     modes   ：高 4 位 = 工作模式，低 4 位 = 控制模式
 *    [2..3]  yaw     ：int16，大端，单位 1e-4 rad
 *    [4..5]  pitch   ：int16，大端，单位 1e-4 rad
 *    [6]     序列号  ：每发一帧加一，用来发现「对面卡住不发新帧」
 *    [7]     校验    ：前 7 字节逐字节异或
 *
 *  为什么用 1e-4 rad 而不是直接塞 float：
 *    float 占 4 字节，两个角度就吃满 8 字节，没有余量放模式/校验；
 *    而 1e-4 rad ≈ 0.0057°，对云台关节分辨率（DM4310 编码器远粗于此）足够。
 *    代价是两端缩放系数必须一致——这也是要单测的原因：一端写成 1000 就会整体
 *    差 10 倍，实车表现为「云台转得比指令快十倍」，很难当场归因。
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#ifndef ROBOT_COMPONENTS_GIMBAL_CHASSIS_COMM_CODEC_HPP_
#define ROBOT_COMPONENTS_GIMBAL_CHASSIS_COMM_CODEC_HPP_

#include <cstdint>

namespace hello_world {
namespace comm {

/** 角度缩放：1 个 int16 单位 = 1e-4 rad */
constexpr float kAngleScale = 10000.0f;

/** 8 字节帧承载的语义。两个方向字段相同，含义由 CodePart 决定。 */
struct Frame {
  uint8_t online = 0;     ///< 本方是否在线（1 = 在线）
  uint8_t ready = 0;      ///< 本方是否就绪（零漂完成 / 模式已稳定）
  uint8_t ctrl_mode = 0;  ///< 控制模式：0 = Dead，1 = Working
  uint8_t work_mode = 0;  ///< 工作模式：0 = 分离，1 = 随动，2 = 小陀螺
  float yaw = 0.0f;       ///< rad。底盘方向表示「期望云台角」，云台方向表示「实际角」
  float pitch = 0.0f;     ///< rad
};

/**
 * @brief  把 Frame 打包进 8 字节
 * @param  frame 待打包语义（会读取 yaw/pitch）
 * @param  out   8 字节输出缓冲区
 * @param  seq   序列号，由调用方维护并在每次发送后自增
 * @retval None
 * @note   yaw/pitch 会被夹到 int16 可表示范围（±3.2767 rad），不会溢出回绕。
 */
void PackFrame(const Frame &frame, uint8_t seq, uint8_t out[8]);

/**
 * @brief  把 8 字节还原成 Frame
 * @param  in  收到的 8 字节
 * @param  out 解出的语义（仅在返回 true 时被写入）
 * @retval true  校验通过，out 已填充
 * @retval false 校验失败，out 保持不变（调用方应丢弃本帧并计一次丢包）
 * @note   先解到临时量再整体赋值，避免校验失败的帧把缓存写坏一半。
 */
bool UnpackFrame(const uint8_t in[8], Frame *out);

}  // namespace comm
}  // namespace hello_world

#endif /* ROBOT_COMPONENTS_GIMBAL_CHASSIS_COMM_CODEC_HPP_ */
