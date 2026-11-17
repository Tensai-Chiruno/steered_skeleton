# -*- coding: utf-8 -*-
"""诊断：某条 CAN 总线上到底有没有活着的节点（不碰电机、不改固件）。

原理（为什么这个探针是安全的）
    1) bxCAN 的 ACK 是**帧级**行为：任何收到帧且校验无误的节点都必须拉低 ACK 位，
       **与它自己的接收过滤器、与帧 ID 是不是给它的，完全无关**。
    2) 所以我们发一帧 ID = 0x7FF 的标准帧（全工程没有任何电机/板间协议用这个 ID，
       电机的验收过滤不会让这帧进控制逻辑 → **电机不会因为它做任何动作**），
       然后看发送结果：
         TXOK=1 / TEC=0  → 总线上**有别的节点**（有人给 ACK）
         TERR=1 / TEC 涨 → 总线上**只有我们自己**（没人应答）
    3) 全程只写 CAN 控制器的发送邮箱，不发任何运动指令。

用法
    python tools\\diag_can_txprobe.py            # 默认探 CAN1 和 CAN2
    python tools\\diag_can_txprobe.py --bus CAN1
"""
import argparse
import sys
import time

CAN1 = 0x40006400
CAN2 = 0x40006800

MCR, MSR, TSR, ESR = 0x00, 0x04, 0x08, 0x18
TIR, TDTR, TDLR, TDHR = 0x180, 0x184, 0x188, 0x18C

PROBE_ID = 0x7FF  # 谁都不用的 ID，保证不会触发任何电机的控制逻辑


def rd(j, a):
    return j.memory_read32(a, 1)[0]


def wr(j, a, v):
    j.memory_write32(a, [v & 0xFFFFFFFF])


def probe(j, base, name):
    msr = rd(j, base + MSR)
    tsr = rd(j, base + TSR)
    tec0 = rd(j, base + ESR) & 0xFF
    print("  %s 初始：MSR=0x%08X INAK=%d | TSR=0x%08X TME0=%d | TEC=%d"
          % (name, msr, msr & 1, tsr, (tsr >> 26) & 1, tec0))
    if msr & 1:
        print("    [SKIP] 控制器在初始化态（INAK=1）")
        return None
    # 找一个空闲发送邮箱：0 → 1 → 2（固件占着 0 时用 1/2）
    mb = None
    for i in range(3):
        if (tsr >> (26 + i)) & 1:
            mb = i
            break
    if mb is None:
        print("    [SKIP] 三个发送邮箱都被固件占着")
        return None
    tir, tdtr, tdlr, tdhr = (0x180 + 0x10 * mb, 0x184 + 0x10 * mb,
                             0x188 + 0x10 * mb, 0x18C + 0x10 * mb)
    print("    使用发送邮箱 %d" % mb)

    # 组一帧：标准帧、ID=0x7FF、DLC=1、数据 0x00
    wr(j, base + tdtr, 1 & 0xF)                       # DLC=1
    wr(j, base + tdlr, 0x00000000)
    wr(j, base + tdhr, 0x00000000)
    wr(j, base + tir, (PROBE_ID << 21) | (1 << 0))    # STDID=PROBE_ID + TXRQ

    ok = False
    for _ in range(200):
        tsr = rd(j, base + TSR)
        if (tsr >> mb) & 1:          # RQCPn：该邮箱请求完成
            ok = bool((tsr >> (mb + 1)) & 1)   # TXOKn
            break
        time.sleep(0.002)

    esr = rd(j, base + ESR)
    tec = esr & 0xFF
    if ok:
        print("    → TXOK=1，TEC %d → %d ：**总线上有别的节点在应答** ✅" % (tec0, tec))
        return True
    print("    → 发送失败（TERR），TEC %d → %d ：**总线上只有自己，没有节点应答** ❌" % (tec0, tec))
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", choices=["CAN1", "CAN2", "both"], default="both")
    args = ap.parse_args()

    import pylink
    j = pylink.JLink()
    j.open()
    try:
        j.set_tif(pylink.enums.JLinkInterfaces.SWD)
        j.set_speed(4000)
        j.connect("STM32F407IG", verbose=False)
        print("=== CAN 总线上有没有活节点（探针帧 ID=0x7FF，不会让电机动作）===")
        res = {}
        if args.bus in ("CAN1", "both"):
            res["CAN1"] = probe(j, CAN1, "CAN1")
        if args.bus in ("CAN2", "both"):
            res["CAN2"] = probe(j, CAN2, "CAN2")
        print("\n汇总：", {k: ("有节点" if v else ("无节点" if v is False else "跳过")) for k, v in res.items()})
    finally:
        try:
            j.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
