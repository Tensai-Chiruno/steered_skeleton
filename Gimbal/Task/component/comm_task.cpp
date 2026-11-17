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
/* 【现场修复 2026-11-17】接收缓冲区故意开**两帧 36 字节**。
 *
 * 为什么：空闲+DMA 的缓冲区起点会漂移（实测偏 2 字节）。只留 18 字节时，
 * 缓冲区整块就是"上一帧尾巴 + 这一帧开头"的混合物，**任何旋转都拼不出一整帧**
 * （现场症状：加严校验后 align 一直是 255、一帧都认不出来）。
 * 开 36 字节后，无论 DMA 从哪里开始写，里面**至少含一整帧**，
 * 回调里滑窗一定能把它找出来。代价只有 18 字节 RAM。 */
constexpr uint16_t kDbusBufLen = 36u;    ///< 接收缓冲区（2 帧）
uint8_t g_dbus_buf[kDbusBufLen];
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
  MotorDiagSweepEnable();  /* 诊断·临时：扫描电机总线上有哪些 ID 在回话 */

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
  /* 用 kBoardBus（实车是 CAN1）；写死 hcan2 会发到电机总线上去，底盘永远收不到 */
  CAN_Send_Msg(GetBoardCommBus(), buf, bc->tx_id(), 8);

  /* 3) 遥控转发：接收机只接在云台板，底盘要动必须有遥控 -> 0x212 转发帧 */
  SendRcRelay();

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
    /* 【加严 2026-11-17 第二次】只查"拨杆在 1..3"还是太松：
     * 错位帧偶尔会凑出 `2,2`（= 最下 = 急停），结果**对齐自愈自己把车停了**
     * （现场现象：rc 在线、peer 在线、tx_fail 不涨，但 estop 自己变 1，
     *  于是 Working -> Dead -> 云台在重力下掉下去，再恢复又抬起来 = "猛砸"）。
     *
     * 现在要求同时满足 6 个条件，并且**优先沿用上一次成功的偏移**：
     *   ① s1、s2 都在 1..3
     *   ② 4 个通道的原始 11 位值都在 200..1900（合法范围是 364..1684，留余量）
     * 错位帧几乎不可能同时凑齐这 6 个条件；一旦对齐正确就稳定不变。
     */
    auto frame_ok = [](const uint8_t *d) -> bool {
      const uint16_t c0 = static_cast<uint16_t>((d[0] | (d[1] << 8)) & 0x7FF);
      const uint16_t c1 = static_cast<uint16_t>(((d[1] >> 3) | (d[2] << 5)) & 0x7FF);
      const uint8_t s1 = static_cast<uint8_t>((d[5] >> 6) & 0x03);
      const uint8_t s2 = static_cast<uint8_t>((d[5] >> 4) & 0x03);
      if (s1 < 1u || s1 > 3u || s2 < 1u || s2 > 3u) return false;
      /* 只校验 ch0 / ch1：这台 DR16 实际发出的帧里 **ch2/ch3 恒为 0 附近**
       * （实测原始帧 `00 04 20 00 01 F8 00 ... 00 04` -> ch2=0、ch3=128），
       * 一开始拿 4 个通道一起卡，反而把真帧全部拒了（align 一直 255、永远 Dead）。 */
      const uint16_t lo = 200u, hi = 1900u;
      return c0 >= lo && c0 <= hi && c1 >= lo && c1 <= hi;
    };

    bool aligned = false;
    const uint16_t begin = (diag_db_align < kDbusBufLen) ? diag_db_align : 0u;
    for (uint16_t n = 0; n < kDbusBufLen; ++n) {
      const uint16_t r = static_cast<uint16_t>((begin + n) % kDbusBufLen);
      uint8_t tmp[kDbusFrameLen];
      for (uint16_t k = 0; k < kDbusFrameLen; ++k) {
        tmp[k] = g_dbus_buf[(r + k) % kDbusBufLen];
      }
      if (!frame_ok(tmp)) {
        continue;
      }
      /* 通过校验才真正解码（decode 会写 g_rc_scratch 的摇杆值） */
      g_rc_scratch.decode(tmp);
      const auto s1 = g_rc_scratch.rc_l_switch();
      const auto s2 = g_rc_scratch.rc_r_switch();
      if (s1 >= 1u && s1 <= 3u && s2 >= 1u && s2 <= 3u) {
        aligned = true;
        diag_db_align = static_cast<uint8_t>(r);   /* 诊断：对齐偏移（正常应稳定不变） */
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
