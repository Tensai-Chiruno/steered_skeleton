# -*- coding: utf-8 -*-
"""诊断：CAN 控制器到底通不通（不改固件、不发电机指令）。

背景
   现场现象：两条总线 TEC=85、mailbox0 TERR=1、ESR.LEC=3（ACK error）。
   ACK error 的意思是"帧发出去了，但总线上没有任何节点应答"。
   可能原因有三类：① 电调没电 ② CAN 线/接口没接对 ③ 板载收发器或 MCU 侧坏了。

怎么做判定（bxCAN 的 loopback 自环）
   BTR.LBKM（bit30）= 1 时，控制器把发出的帧**内部回环**给自己，
   ACK 由自己产生 → 不依赖外部节点。于是：
     - loopback 下发送成功（TERR=0、TEC 掉到 0、ESR.LEC=0）
       ⇒ MCU 的 CAN 外设 + 我们的发送路径是好的，问题在板外（供电/接线/收发器）
     - loopback 下仍然 TERR
       ⇒ MCU 侧就有问题（BTR 配错、时钟不对、外设没使能）
   关键：LBKM 时 TX 引脚**不驱动总线**，所以这个测试不可能让电机转。

用法（需要固件已在跑、J-Link 已连）
    python tools\\diag_can_loopback.py

注意
   本脚本会临时改 CAN1/CAN2 的 MCR/BTR，结束时**原样写回**。
   写完必须读回核对；核对失败会打印警告（那就重烧一次固件恢复）。
"""
import sys
import time

CAN1 = 0x40006400
CAN2 = 0x40006800

MCR = 0x00
MSR = 0x04
TSR = 0x08
ESR = 0x18
BTR = 0x1C

INRQ = 1 << 0
INAK = 1 << 0
LBKM = 1 << 30


def rd(j, a):
    return j.memory_read32(a, 1)[0]


def wr(j, a, v):
    j.memory_write32(a, [v & 0xFFFFFFFF])


def set_init(j, base, on, label=""):
    mcr = rd(j, base + MCR)
    wr(j, base + MCR, (mcr | INRQ) if on else (mcr & ~INRQ))
    for _ in range(400):
        if bool(rd(j, base + MSR) & INAK) == on:
            return True
        time.sleep(0.005)
    print("    [WARN] %s 等 INAK 超时（想要 %s）" % (label, on))
    return False


def decode(esr, tsr):
    return {
        "TEC": esr & 0xFF,
        "REC": (esr >> 8) & 0xFF,
        "LEC": (esr >> 16) & 0x7,
        "TXOK": (esr >> 19) & 1,
        "ABRQ": (esr >> 21) & 1,
        "TERR0": (tsr >> 3) & 1,
        "TXOK0": (tsr >> 1) & 1,
    }


LEC_TXT = {0: "无错", 1: "位填充错", 2: "格式错", 3: "ACK 错（没有节点应答）", 4: "位错", 5: "CRC 错", 6: "软件设置"}


def probe(j, base, name):
    esr, tsr = rd(j, base + ESR), rd(j, base + TSR)
    d = decode(esr, tsr)
    print("  %s  ESR=0x%08X TSR=0x%08X  TEC=%-3d REC=%-3d LEC=%d(%s) TERR0=%d TXOK0=%d"
          % (name, esr, tsr, d["TEC"], d["REC"], d["LEC"], LEC_TXT.get(d["LEC"], "?"), d["TERR0"], d["TXOK0"]))
    return d


def loopback(j, base, name):
    print("\n=== %s loopback 自环测试 ===" % name)
    print("  [正常模式] 基线：")
    base_d = probe(j, base, name)

    btr_before = rd(j, base + BTR)
    set_init(j, base, True, name)
    wr(j, base + BTR, btr_before | LBKM)
    btr_lb = rd(j, base + BTR)
    if not (btr_lb & LBKM):
        print("    [FAIL] LBKM 置位失败（BTR 读回 0x%08X）" % btr_lb)
        set_init(j, base, False, name)
        return False
    set_init(j, base, False, name)

    time.sleep(0.4)  # 让固件正常发几十帧，看它们会不会被自己 ACK
    print("  [loopback]  固件在自环下发了 0.4 s：")
    lb_d = probe(j, base, name)

    # 还原
    set_init(j, base, True, name)
    wr(j, base + BTR, btr_before)
    set_init(j, base, False, name)
    btr_after = rd(j, base + BTR)
    print("  [还原] BTR 0x%08X -> 0x%08X  %s"
          % (btr_lb, btr_after, "OK" if btr_after == btr_before else "!! 与原始值不一致，建议重烧固件"))

    good = (lb_d["TERR0"] == 0) and (lb_d["LEC"] == 0) and (lb_d["TEC"] <= 2)
    print("  结论：%s" % ("MCU 侧发送路径正常（问题在板外）" if good else "MCU 侧也有问题，先查 CAN 外设/时钟"))
    return good


def main():
    try:
        import pylink
    except Exception as e:
        print("[FAIL] 需要 pylink：pip install pylink-square（%r）" % (e,))
        return 2
    j = pylink.JLink()
    j.open()
    try:
        j.set_tif(pylink.enums.JLinkInterfaces.SWD)
        j.set_speed(4000)
        j.connect("STM32F407IG", verbose=False)
        ok1 = loopback(j, CAN1, "CAN1")
        ok2 = loopback(j, CAN2, "CAN2")
        print("\n汇总：CAN1 %s / CAN2 %s" % ("MCU 侧 OK" if ok1 else "MCU 侧异常",
                                             "MCU 侧 OK" if ok2 else "MCU 侧异常"))
    finally:
        try:
            j.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
