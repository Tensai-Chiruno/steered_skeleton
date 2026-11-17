# -*- coding: utf-8 -*-
"""临时诊断：把云台板 CAN2（电机总线）上的 MIT 帧解出来，看我们到底给电机发了什么。

回答两个问题：
  1. 我们有没有在发 MIT 帧（还是卡在使能/模式切换）
  2. kp / kd / torq 字段到底是多少（尤其 torq 是不是 2048 附近 = 零力矩）
"""
import time

CAN2 = 0x40006800
FMR, FM1R, FS1R, FFA1R, FA1R = 0x200, 0x204, 0x20C, 0x214, 0x21C
RF0R, RI0R, RDT0R, RDL0R, RDH0R = 0x0C, 0x1B0, 0x1B4, 0x1B8, 0x1BC
BANK = 27


def rd(j, off):
    return j.memory_read32(CAN2 + off, 1)[0]


def wr(j, off, v):
    j.memory_write32(CAN2 + off, [v & 0xFFFFFFFF])


def main():
    import pylink
    j = pylink.JLink()
    j.open()
    try:
        j.set_tif(pylink.enums.JLinkInterfaces.SWD)
        j.set_speed(4000)
        j.connect("STM32F407IG", verbose=False)

        fr = 0x240 + BANK * 8
        fmr = rd(j, FMR)
        wr(j, FMR, fmr | 1)
        wr(j, fr + 0, 0)
        wr(j, fr + 4, 0)
        wr(j, FM1R, rd(j, FM1R) & ~(1 << BANK))
        wr(j, FS1R, rd(j, FS1R) & ~(1 << BANK))
        wr(j, FFA1R, rd(j, FFA1R) & ~(1 << BANK))
        wr(j, FA1R, rd(j, FA1R) | (1 << BANK))
        wr(j, FMR, fmr & ~1)

        seen = {}
        t0 = time.time()
        while time.time() - t0 < 3.0:
            r = rd(j, RF0R)
            fmp = r & 3
            if fmp == 0:
                time.sleep(0.002)
                continue
            for _ in range(fmp):
                rir = rd(j, RI0R)
                sid = (rir >> 21) & 0x7FF
                lo, hi = rd(j, RDL0R), rd(j, RDH0R)
                d = [(lo >> (8 * k)) & 0xFF for k in range(4)] + \
                    [(hi >> (8 * k)) & 0xFF for k in range(4)]
                pos = (d[0] << 8) | d[1]
                vel = (d[2] << 4) | (d[3] >> 4)
                kp = ((d[3] & 0xF) << 8) | d[4]
                kd = (d[5] << 4) | (d[6] >> 4)
                torq = ((d[6] & 0xF) << 8) | d[7]
                seen.setdefault(sid, []).append((pos, vel, kp, kd, torq,
                                                 " ".join("%02X" % x for x in d)))
            wr(j, RF0R, 0x20)

        print("3 秒内 CAN2 上出现的 ID：")
        for sid, lst in sorted(seen.items()):
            print("  ID=0x%03X  收到 %d 帧，最后一帧：" % (sid, len(lst)))
            pos, vel, kp, kd, torq, raw = lst[-1]
            print("    raw: %s" % raw)
            print("    MIT 解出: pos=%d vel=%d kp=%d kd=%d torq=%d  (torq=2048 约等于零力矩)" % (pos, vel, kp, kd, torq))

        fmr = rd(j, FMR)
        wr(j, FMR, fmr | 1)
        wr(j, FA1R, rd(j, FA1R) & ~(1 << BANK))
        wr(j, FMR, fmr & ~1)
    finally:
        try:
            j.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
