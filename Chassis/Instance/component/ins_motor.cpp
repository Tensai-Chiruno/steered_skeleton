/**
 *******************************************************************************
 * @file      :ins_motor.cpp
 * @brief     :底盘板电机实例 + CAN 帧分发/打包
 * @category : 功能 | 考核第 1 项（10 台电机配置 + CAN 分发）
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
/* ---- 量纲的唯一来源（P0-1）：配置、断言、发送帧 ID 三处都用这两个常量 ---- */
constexpr hello_world::motor::InputType kWheelInputType = hello_world::motor::InputType::kRaw;
/* 【现场实测 2026-11-17】GM6020 改用**电流模式**（0x1FE）。
 *
 * 证据链：CAN1 上 TEC=0、tx_fail 冻结 —— 说明我们发往 0x1FF 的帧**都被 ACK 了**，
 * 也就是 GM6020 确实收到了；但 0x205~0x208 的反馈帧一帧都没有、舵角实测 15 秒纹丝不动。
 * CAN 的 ACK 与"是否采纳该帧"无关：电机在电流模式时会 ACK 电压帧却既不执行也不回反馈，
 * 现象与本机完全一致。文档写"默认电压控制"，但今天文档已在拓扑、yaw ID 上两次出错。 */
constexpr hello_world::motor::InputType kSteerInputType = hello_world::motor::InputType::kCurr;


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
    hello_world::motor::kDirFwd, hello_world::motor::kDirFwd};  // RF 改回 Fwd，见下：
/* 【2026-11-18 闭环实测】RF 用 kDirRev 时闭环符号是反的：
 *   遥控前推 -> 三台 tgt=+12.42 / vel=+6.91,+8.67,+8.30，而 RF vel=-7.57、指令也为负，
 *   即该轮的"逻辑正"与其它三轮相反，整车会原地打转。
 * 11-17 那次"前推车身四轮符号一致"是**开环手推**测的：手推时轮子是发电机，
 * 编码器符号只反映机械安装方向；闭环要的是"逻辑正=车前进"，两者基准不同。
 * 改成 kDirFwd 后逻辑正与其它三轮一致。*/
/* 现场实测 2026-11-17（J-Link 读 robot::g_dbg，前推车身时四轮轮速符号）：
 *   前推 → LF +1.53  LB +1.69  RB +1.62  RF -1.14
 *   回拉 → LF -3.10  LB -2.86  RB -2.89  RF +2.69   （三段来回，符号完全可复现）
 * 三个轮「前推为正」= kDirFwd 正确；RF 相反，且它的舵零位同时差 180°（见 kSteerOffset[3]），
 * 两者是同一个物理事实的两面：RF 的舵轮装配方向与另外三个差 π。 */

/* 舵电机：id 1~4，保留内置减速器（ex_redu_rat = 1），方向全部反向。
 * 舵向零点偏移来自考核说明 6.2「已按实机配置统一补齐」的固定值，必须照抄：
 *   缺了它，舵角反馈就不是车体坐标系下的舵角，逆解算出来的目标角会整体偏掉，
 *   现象是「四轮都在转但车身斜着走」。 */
constexpr uint8_t kSteerId[kSteerNum] = {1, 2, 3, 4};
constexpr hello_world::motor::Dir kSteerDir[kSteerNum] = {
    hello_world::motor::kDirRev, hello_world::motor::kDirRev,
    hello_world::motor::kDirRev, hello_world::motor::kDirRev};
constexpr float kSteerOffset[kSteerNum] = {2.61346f, -2.06844f, 0.00729f,
                                           4.72677f};  // LF / LB / RB / RF
/* RF 的 1.58518 → +π = 4.72677：现场实测，把四个舵轮手掰到「轮面朝正前方」的机械零位时，
 *   LF -2.11°  LB -2.88°  RB +2.44°（都在 ±3°，说明考核说明给的偏移是对的），
 *   而 RF 读到 +179.87° —— 差整整 π。判据 |θ|<0.2 rad 下这是唯一不通过的一项。
 * 手掰左转 90° 复验：四台读数分别 +95.1° / +86.9° / +88.0° / +85.9°，方向全部正确，
 * 所以只补 π，不动 kSteerDir。 */

}  // namespace

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* 舵电机（GM6020，CAN1）反馈帧累计计数。
 * 用途：给"台架闭环测试窗口"当触发条件 —— 反馈一到就说明电机已上电、总线已活，
 * 比"开机 N 秒后"可靠得多（现场顺序必须是先断电、复位、再上电）。
 * 必须放在匿名 namespace 之外，才能被 ins_motor.hpp 里导出的访问函数拿到。 */
volatile uint32_t g_steer_rx_count = 0;
/* 轮电机（M3508，CAN2）反馈帧累计计数，用途同上（区分"没指令"和"没反馈"）。 */
volatile uint32_t g_wheel_rx_count = 0;

namespace {

hello_world::motor::M3508 wheel_[kWheelNum];
hello_world::motor::GM6020 steer_[kSteerNum];
bool inited_ = false;

/* 【诊断·临时】两条总线上"收到过哪些 ID"的位掩码（bit n = ID n 出现过）。
 * 用途：找 yaw 电机到底挂在哪条总线、用什么 ID 回帧
 * （云台板扫描后只发现 0x12 一台，说明 yaw 不在云台那侧）。 */
volatile uint32_t g_diag_mask_c1 = 0;   /* CAN1（舵总线） */
volatile uint32_t g_diag_mask_c2 = 0;   /* CAN2（轮总线 / 主干） */

/** 一次性构造全部电机对象。首次访问任何 GetMotorXxx 时执行。 */
void EnsureInit(void) {
  if (inited_) {
    return;
  }
  for (int i = 0; i < kWheelNum; ++i) {
    hello_world::motor::OptionalParams opt;
    opt.input_type = kWheelInputType;
    opt.dir = static_cast<hello_world::motor::Dir>(kWheelDir[i]);
    opt.ex_redu_rat = 14.0f;              // M3508 外接减速比
    opt.remove_build_in_reducer = true;   // M3508 内置减速器已计入 14，不再重复
    opt.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf;
    wheel_[i].init(kWheelId[i], opt);
  }
  for (int i = 0; i < kSteerNum; ++i) {
    hello_world::motor::OptionalParams opt;
    opt.input_type = kSteerInputType;
    opt.dir = static_cast<hello_world::motor::Dir>(kSteerDir[i]);
    opt.ex_redu_rat = 1.0f;               // GM6020 保留内置减速器
    opt.remove_build_in_reducer = false;
    opt.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
    /* P0-4：这条之前漏了！kSteerOffset[] 只被定义、从未使用，全文件 angle_offset
     * 出现 0 次。后果是四个舵的反馈零位分别偏 149.7° / -118.5° / 0.42° / 90.8°，
     * 而逆解给的目标角是以「机械零位」为基准的，两边不同源 → 舵角环从一开始就是错的。
     * GCC 对 C++ 的 constexpr 数组默认不报 -Wunused-const-variable，所以编译器不吭声。 */
    opt.angle_offset = kSteerOffset[i];
    steer_[i].init(kSteerId[i], opt);
  }

  /* 把期望的 CAN ID 钉死在这里。
   * 这四行不产生运行开销（Debug 下断言，Release 下通常被编译掉），
   * 但它把「协议表」写成了可执行的检查：以后谁把 input_type 从 kCurr 改成
   * kRaw、或把 id 改成 5~8，上电第一帧就会在这里炸掉，而不是在实验室里
   * 表现为「指令给了电机不动」或「动得很怪」。 */
  HW_ASSERT(hello_world::motor::M3508::GetTxId(1) == 0x200, "wheel tx id");
  HW_ASSERT(hello_world::motor::M3508::GetRxId(1) == 0x201, "wheel rx id");
  /* 这条断言必须用「真正配置电机用的那个常量」，否则它恒为真、等于没写
   * （P0-1：旧写法传的是字面量 kRaw，而 opt.input_type 是 kCurr，两者不一致却没被发现）。 */
  HW_ASSERT(hello_world::motor::GM6020::GetTxId(kSteerId[0], kSteerInputType) == 0x1FE,
            "steer tx id must be 0x1FE: 现场实测 GM6020 在电流模式，0x1FF 电压帧只被 ACK 不执行");
  HW_ASSERT(hello_world::motor::M3508::GetTxId(kWheelId[0]) == 0x200,
            "wheel tx id must be 0x200");
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

/* 舵电机反馈累计计数（只增不减）。0 表示"这一版固件还没收到过任何 GM6020 反馈"，
 * 即电机未上电或 CAN1 不通。台架测试窗口用它当"电机已上电"的信号。 */
uint32_t MotorSteerRxCount(void) {
  return g_steer_rx_count;
}

uint32_t MotorWheelRxCount(void) {
  return g_wheel_rx_count;
}

void MotorDispatchCanFrame(CAN_HandleTypeDef *hcan, uint32_t std_id,
                           const uint8_t data[8]) {
  EnsureInit();

  /* 【诊断·临时】记录两条总线上出现过的 CAN ID（bit n = ID n 出现过）。
   * 用途：找 yaw 电机到底挂在哪条总线/用哪个 ID（云台板扫描后只发现 0x12 一台）。 */
  if (std_id < 32u) {
    if (hcan == kWheelBus) {
      g_diag_mask_c2 |= (1UL << std_id);
    } else if (hcan == kSteerBus) {
      g_diag_mask_c1 |= (1UL << std_id);
    }
  }

  hello_world::motor::Motor *m = LocateMotor(hcan, std_id);
  if (m != nullptr) {
    m->decode(data);
    /* 只有"确实命中舵电机反馈 ID"的帧才计数（CAN1 上还有板间帧 0x211，不能算）。 */
    if (hcan == kSteerBus) {
      ++g_steer_rx_count;
    } else if (hcan == kWheelBus) {
      ++g_wheel_rx_count;
    }
    return;
  }

  /* 未命中：双板通讯帧（0x210 / 0x211）在阶段 4 接入这里。
   * 现阶段直接丢弃——静默丢弃比误判成电机更安全。 */
}

/* 【诊断·临时】向两条总线各 ID 1..8 发一帧达妙"使能"命令，扫描有哪些电机存在。
 * 只发使能（0xFF×7 + 0xFC），不给位置/速度/力矩，电机不会动。
 * 发完看 g_diag_mask_c1 / g_diag_mask_c2 就知道谁在回话。 */
void MotorDiagSweepEnable(void) {
  EnsureInit();
  uint8_t buf[8];
  CAN_HandleTypeDef *const buses[2] = {&hcan1, &hcan2};
  for (uint8_t b = 0; b < 2u; ++b) {
    for (uint8_t id = 1; id <= 8u; ++id) {
      for (uint8_t i = 0; i < 7u; ++i) {
        buf[i] = 0xFFu;
      }
      buf[7] = 0xFCu;
      CAN_Send_Msg(buses[b], buf, id, 8);
      for (volatile uint32_t d = 0; d < 20000u; ++d) {
      }
    }
  }
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
               hello_world::motor::GM6020::GetTxId(kSteerId[0], kSteerInputType),
               8);
}

/* Private function definitions ----------------------------------------------*/
