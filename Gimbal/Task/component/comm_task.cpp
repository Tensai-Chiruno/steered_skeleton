/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :通讯层初始化与周期发送（云台板，含 DBUS）
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
/* DBUS 一帧固定 18 字节。缓冲区必须是静态的：DMA 直写内存，
 * 放栈上函数一返回就被覆盖。 */
constexpr uint16_t kDbusFrameLen = 18u;  ///< DBUS 固定 18 字节
uint8_t g_dbus_buf[kDbusFrameLen];
/* 先解到临时对象，跳变检查通过才采用 —— 这样"保持上一帧"是天然的：
 * 不通过时 GetRemoteControl() 里的 rc_*_ 一个字节都没被改过。 */
hello_world::remote_control::DT7 g_rc_scratch;
hello_world::comm::RcFrameGate g_rc_gate;
/* 【诊断】UART 原始字节（54 字节 ≈ 3 帧）。只在 CommTaskInit 里抓一次，
 * 用 SWD 直接读内存看，不参与任何控制逻辑；排障完可以整段删掉。 */
uint8_t g_diag_uart[54];
/* 诊断：最近一次 DBUS 对齐偏移（正常应恒为 0） */
volatile uint8_t diag_db_align = 0xFF;
}  // namespace

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void) {
  /* 1) 两条总线都配置全收滤波器 */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  /* 2) 启动外设并打开接收中断。
   *    CanFilter_Init 只配滤波器、不启动外设；漏掉 HAL_CAN_Start 的现象是
   *    「滤波器配好、中断也使能，但一帧都收不到」。 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* 3) DBUS：USART3 已由 CubeMX 配成 100k / 9bit / 偶校验 / 仅接收，
   *    且 DMA1_Stream1 是【循环模式】（见 usart.c），所以只需要启动一次，
   *    不需要每帧重启接收。
   *    关掉半传输中断：DBUS 每 14 ms 才来一帧，半满中断只会制造无谓负载。 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf, sizeof(g_dbus_buf));
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

  /* 4) 【诊断·一次性】把 UART 原始字节抓下来，不做任何解码，直接用 SWD 读。
   *    目的：排除"我们的解码/缓冲区对齐错了"这一可能——把字节流原样摊开看：
   *      有没有 18 字节周期、有没有 0x0F 之类的 SBUS 头、
   *      通道字节是 0x6C/0x01（油门最低）还是 0x00/0x04（居中）。
   *    只在启动时做一次，抓完立刻恢复正常的"空闲+DMA"接收。
   *    阻塞时间 ≈ 3 帧 ≈ 40 ms，远小于 IWDG 的 200 ms。 */
  HAL_UART_AbortReceive(&huart3);
  (void)HAL_UART_Receive(&huart3, g_diag_uart, sizeof(g_diag_uart), 60u);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_buf, sizeof(g_dbus_buf));
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

void CommTask(void) {
  /* 1) yaw / pitch 指令打包发出（CAN1；DM 使能指令由 encode 自动插入） */
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
 * @brief UART 空闲/DMA 接收完成回调
 * @note  必须 extern "C"：HAL 的弱符号是 C 链接，不加就覆盖不到它，
 *        现象是「UART 收到了但回调从不进」。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size) {
  if (huart != &huart3) {
    return;
  }
  if (Size >= kDbusFrameLen) {
    /* 【关键修复 2026-11-17】对齐自愈。
     *
     * 现场现象：遥控数据时好时坏——同样的固件，有时解出 s1=s2=3（合法中位）、
     * 目标值与姿态都正常；有时解出 s1=s2=0（非法值）→ 被状态机判成"急停"→
     * 云台在 Working / Dead 之间反复跳。
     *
     * 根因：DBUS 是 18 字节定长帧但**没有帧头**，靠"空闲中断 + 循环 DMA"接收时，
     * 只要某次少收/多收一个字节，缓冲区起点就会永久错位——长度仍是 18，
     * 用 `Size >= 18` 根本发现不了，于是整帧被当成另一个"合法的错帧"。
     *
     * 修法：逐字节旋转试探，取第一个"两个拨杆值都在 1..3 之间"的解。
     * 拨杆只有 1/2/3 三种取值，错位帧几乎不可能同时凑出两个合法值，
     * 这个判据的区分度远高于"摇杆跳变"。
     */
    bool aligned = false;
    for (uint16_t r = 0; r < kDbusFrameLen; ++r) {
      uint8_t tmp[kDbusFrameLen];
      for (uint16_t k = 0; k < kDbusFrameLen; ++k) {
        tmp[k] = g_dbus_buf[(r + k) % kDbusFrameLen];
      }
      g_rc_scratch.decode(tmp);
      const auto s1 = g_rc_scratch.rc_l_switch();
      const auto s2 = g_rc_scratch.rc_r_switch();
      if (s1 >= 1u && s1 <= 3u && s2 >= 1u && s2 <= 3u) {
        aligned = true;
        diag_db_align = r;   /* 诊断：记录本次对齐偏移（正常应恒为 0） */
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
  /* 循环 DMA 下不需要重启；只有出错导致 RxState 掉出 BUSY 时才补一次 */
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
