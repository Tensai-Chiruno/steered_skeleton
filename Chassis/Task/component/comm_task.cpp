/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :通讯层初始化与周期发送（底盘板）
 * @category : 功能 | 考核第 2/10 项（CAN+DBUS+喂狗）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"
#include "main_task.hpp"
#include "ins_comm_state.hpp"

#include "HW_can.hpp"
#include "rc_frame_gate.hpp"
#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_motor.hpp"
#include "usart.h"
#include "iwdg.h"
#include "iwdg_policy.hpp"

/* Private variables ---------------------------------------------------------*/
namespace {
constexpr uint16_t kDbusFrameLen = 18u;  ///< DBUS 固定 18 字节
/* 接收缓冲区开两帧（36 字节）：对齐漂移时也能保证里面含一整帧（同云台板）。 */
constexpr uint16_t kDbusBufLen = 36u;    ///< 接收缓冲区（2 帧）
uint8_t g_dbus_buf[kDbusBufLen];
/* 上一次成功的对齐偏移（对齐正确后稳定不变） */
uint8_t g_align_hint = 0u;
/* 先解到临时对象，跳变检查通过才采用 —— 这样"保持上一帧"是天然的：
 * 不通过时 GetRemoteControl() 里的 rc_*_ 一个字节都没被改过。 */
hello_world::remote_control::DT7 g_rc_scratch;
hello_world::comm::RcFrameGate g_rc_gate;
}  // namespace

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void) {
  /* 1) 两条总线都配置全收滤波器 */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);
  MotorDiagSweepEnable();  /* 诊断·临时：扫描两条总线上有哪些 ID 在回话 */

  /* 2) 启动外设并打开接收中断。
   *    CanFilter_Init 只配滤波器、不启动外设；漏掉 HAL_CAN_Start 的现象是
   *    「滤波器配好、中断也使能，但一帧都收不到」。 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* 3) DBUS：考核说明任务 2 明确「两板」都要做。两板的 USART3 都是
   *    100k / 9bit / 偶校验 / 仅接收，DMA1_Stream1 循环模式，所以启动一次即可。
   *    关掉半传输中断：DBUS 每 14 ms 才来一帧，半满中断只增加负载。 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf, sizeof(g_dbus_buf));
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

void CommTask(void) {
  /* 1) 4 轮 M3508 + 4 舵 GM6020 指令打包发出（CAN1） */
  MotorSendAllCommands();

  /* 2) 双板帧：把本板状态编码后发到 CAN2 */
  robot::GimbalChassisComm *bc = GetGimbalChassisComm();
  /* P1-3：online / ready / work_mode / ctrl_mode 由数据拥有者（Robot）每拍写入，
   * 这里**不能**再无条件覆盖——旧代码每 1 ms 写一次 online=1 / ready=0，
   * 表现为「online 永远自报在线（反诊断）」且「别人写的 ready 下一拍就被清掉」。 */
  uint8_t buf[8];
  bc->encode(buf);
  CAN_Send_Msg(&hcan2, buf, bc->tx_id(), 8);

#if IWDG_FEED_IN_COMM_TASK
  /* 正常形态：CommTask 每 1ms 刷新。验收演示复位时把
   * iwdg_policy.hpp 的 IWDG_FEED_IN_COMM_TASK 改成 0。 */
  /* P1-8②：喂狗条件里加一条「主上下文还活着」。
   * 只证明 TIM6 在跳是不够的：Error_Handler() 这类主上下文死循环发生时，
   * TIM6 照样每 1 ms 抢占并喂狗，看门狗等于关闭。 */
  if (MainLoopIdleMs() < 200u) {
    HAL_IWDG_Refresh(&hiwdg);
  }
#endif
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief UART 空闲/DMA 接收完成回调（底盘板）
 * @note  必须 extern "C"，否则覆盖不到 HAL 的弱符号，现象是「收到了但回调不进」。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size) {
  if (huart != &huart3) {
    return;
  }
  if (Size >= kDbusFrameLen) {
    /* 【现场修复 2026-11-17，与云台板同一套】对齐自愈。
     *
     * 根因：DBUS 是 18 字节定长帧但没有帧头，靠"空闲+DMA"接收时缓冲区起点会漂移，
     * 长度仍是 18，`Size >= 18` 查不出来；18 字节缓冲里可能整块都是"半帧混合物"，
     * 于是解码出拨杆 0（非法）→ 被判急停 → 状态机在 Working/Dead 之间反复跳。
     *
     * 修法：
     *   ① 缓冲区开到 36 字节（两帧），保证里面一定含一整帧；
     *   ② 滑窗找第一段满足"拨杆 1..3 + ch0/ch1 在 200..1900"的 18 字节；
     *   ③ 优先沿用上一次成功的偏移，找到就停（对齐正确后不再变动）。
     * 注意：本机实测 ch2/ch3 恒为 0 附近，不能拿 4 个通道一起当判据。 */
    auto frame_ok = [](const uint8_t *d) -> bool {
      const uint16_t c0 = static_cast<uint16_t>((d[0] | (d[1] << 8)) & 0x7FF);
      const uint16_t c1 = static_cast<uint16_t>(((d[1] >> 3) | (d[2] << 5)) & 0x7FF);
      const uint8_t s1 = static_cast<uint8_t>((d[5] >> 6) & 0x03);
      const uint8_t s2 = static_cast<uint8_t>((d[5] >> 4) & 0x03);
      if (s1 < 1u || s1 > 3u || s2 < 1u || s2 > 3u) return false;
      const uint16_t lo = 200u, hi = 1900u;
      return c0 >= lo && c0 <= hi && c1 >= lo && c1 <= hi;
    };

    bool aligned = false;
    const uint16_t begin = (g_align_hint < kDbusBufLen) ? g_align_hint : 0u;
    for (uint16_t n = 0; n < kDbusBufLen; ++n) {
      const uint16_t r = static_cast<uint16_t>((begin + n) % kDbusBufLen);
      uint8_t tmp[kDbusFrameLen];
      for (uint16_t k = 0; k < kDbusFrameLen; ++k) {
        tmp[k] = g_dbus_buf[(r + k) % kDbusBufLen];
      }
      if (!frame_ok(tmp)) {
        continue;
      }
      g_rc_scratch.decode(tmp);
      const auto s1 = g_rc_scratch.rc_l_switch();
      const auto s2 = g_rc_scratch.rc_r_switch();
      if (s1 >= 1u && s1 <= 3u && s2 >= 1u && s2 <= 3u) {
        aligned = true;
        g_align_hint = static_cast<uint8_t>(r);
        break;
      }
    }

    if (aligned) {
      hello_world::comm::RcSticks now;
      now.lv = g_rc_scratch.rc_lv();
      now.lh = g_rc_scratch.rc_lh();
      now.rv = g_rc_scratch.rc_rv();
      now.rh = g_rc_scratch.rc_rh();
      if (g_rc_gate.accept(now)) {
        *GetRemoteControl() = g_rc_scratch;
        NotifyRemoteFrame();
      } else if (g_rc_gate.shouldDropOnline()) {
        InvalidateRemote();   /* 连续可疑：立即判离线，车停住 */
      }
    }
  }
  if (huart3.RxState != HAL_UART_STATE_BUSY_RX) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf, sizeof(g_dbus_buf));
  }
}

/**
 * @brief UART 错误回调（P1-7）
 * @note  PE/FE/NE/ORE 任一发生时 HAL 会中止 DMA 接收并且**不再**进 RxEventCallback，
 *        所以「在 RxEventCallback 里检查 RxState 重新挂载」这条兜底永远等不到机会 ——
 *        遥控器会因为一次噪声错误而永久失联，直到重启。这里必须重新挂载。
 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart != &huart3) {
    return;
  }
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);
  HAL_UART_DMAStop(huart);
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf, sizeof(g_dbus_buf));
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}
