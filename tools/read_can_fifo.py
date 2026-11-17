# -*- coding: utf-8 -*-
"""纯读 CAN 的接收 FIFO 状态：RF0R/RF1R 的 FMP（填充数）与 FOVR（溢出标志）。

为什么：现场出现"TEC=0 没有任何错误，但四个舵角 90 秒不再更新"——
发送正常、接收静默。bxCAN 在 FIFO 溢出（FOVR=1）后，只要不清标志，
新报文就不再装入 FIFO，表现为"接收中断再也不来"。
本脚本只读，不写外设、不下发任何指令。
"""
import sys
import pylink

CAN1 = 0x40006400
CAN2 = 0x40006800


def report(jl, name, base):
    rf0 = jl.memory_read32(base + 0x0C, 1)[0]
    rf1 = jl.memory_read32(base + 0x10, 1)[0]
    esr = jl.memory_read32(base + 0x18, 1)[0]
    ier = jl.memory_read32(base + 0x14, 1)[0]
    for tag, r, fovr_bit, full_bit in (("RF0R", rf0, 0, 1), ("RF1R", rf1, 0, 1)):
        fmp = r & 0x3
        full = (r >> full_bit) & 1
        fovr = (r >> 2) & 1
        print("  %s %s=0x%08X  FMP=%d  FULL=%d  FOVR=%d" % (name, tag, r, fmp, full, fovr))
    print("      %s IER=0x%08X  FMPIE0=%d FOVIE0=%d FMPIE1=%d FOVIE1=%d  TEC=%d LEC=%d"
          % (name, ier, (ier >> 1) & 1, (ier >> 3) & 1, (ier >> 4) & 1, (ier >> 5) & 1,
             (esr >> 24) & 0xFF, (esr >> 4) & 7))


def main():
    jl = pylink.JLink()
    jl.open()
    jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jl.set_speed(4000)
    jl.connect("STM32F407IG", verbose=False)
    v = jl.memory_read32(0xE0042008, 1)[0]
    jl.memory_write32(0xE0042008, [v | (1 << 12)])
    print("[CAN 接收 FIFO 状态]")
    report(jl, "CAN1", CAN1)
    report(jl, "CAN2", CAN2)
    jl.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
