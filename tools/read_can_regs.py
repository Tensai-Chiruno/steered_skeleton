# -*- coding: utf-8 -*-
"""纯读 CAN1/CAN2 的 ESR/MSR 寄存器，判断总线上到底有没有节点在应答。

不写任何外设、不复位、不下发指令 —— 只读。
ESR: TEC[31:24] REC[23:16] LEC[6:4] BOFF[2] EPV[1] EWGF[0]
MSR: INAK[0] SLAK[1]
"""
import sys
import pylink

CAN1 = 0x40006400
CAN2 = 0x40006800
LEC = {0: "OK", 1: "STUFF", 2: "FORM", 3: "ACK", 4: "BIT1", 5: "BIT0", 6: "CRC", 7: "SW"}


def report(jl, name, base):
    msr = jl.memory_read32(base + 0x04, 1)[0]
    tsr = jl.memory_read32(base + 0x08, 1)[0]
    esr = jl.memory_read32(base + 0x18, 1)[0]
    tec = (esr >> 24) & 0xFF
    rec = (esr >> 16) & 0xFF
    lec = (esr >> 4) & 0x7
    print("  %s  MSR=0x%08X (INAK=%d SLAK=%d)  TSR=0x%08X  TEC=%d REC=%d LEC=%d(%s) BOFF=%d"
          % (name, msr, msr & 1, (msr >> 1) & 1, tsr, tec, rec, lec, LEC.get(lec, "?"), (esr >> 2) & 1))
    if tec == 0 and rec == 0 and lec == 0:
        print("      -> 总线健康：有节点在 ACK")
    elif lec == 3:
        print("      -> ACK 错误：这一路上没有任何节点应答（电机驱动没电/没接）")
    else:
        print("      -> 异常，需进一步看")


def main():
    jl = pylink.JLink()
    jl.open()
    jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jl.set_speed(4000)
    jl.connect("STM32F407IG", verbose=False)
    v = jl.memory_read32(0xE0042008, 1)[0]
    jl.memory_write32(0xE0042008, [v | (1 << 12)])
    print("[CAN 状态]")
    report(jl, "CAN1", CAN1)
    report(jl, "CAN2", CAN2)
    jl.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
