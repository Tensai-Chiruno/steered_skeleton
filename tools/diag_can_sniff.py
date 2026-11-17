# -*- coding: utf-8 -*-
"""诊断：把某条 CAN 设成"收全部帧"，把总线上的 ID 列出来（不碰电机、不发帧）。

为什么要这个
    我们的固件只把「认识的 ID」收进 FIFO（过滤器精确匹配），所以
    「总线上有没有 0x210 / 电机反馈 0x11/0x12」从固件层面看不出来。
    这里临时给一个空闲过滤器组设置 "ID=0，mask=0"（= 全收），
    把帧 ID 捞出来看一眼，看完**恢复原状**（把过滤器组关掉）。

安全性
    - 只读总线 + 改过滤器配置，**不发送任何帧**，电机不会因它动作
    - 只在 CAN1（云台板当前完全没用到这条总线）或单独指定的总线上做

用法
    python tools\\diag_can_sniff.py --bus CAN1
    python tools\\diag_can_sniff.py --bus CAN2 --frames 12
"""
import argparse
import sys
import time

BASE = {"CAN1": 0x40006400, "CAN2": 0x40006800}
FMR, FM1R, FS1R, FFA1R, FA1R = 0x200, 0x204, 0x20C, 0x214, 0x21C
RF0R = 0x0C
RI0R, RDT0R, RDL0R, RDH0R = 0x1B0, 0x1B4, 0x1B8, 0x1BC


def rd(j, base, off):
    return j.memory_read32(base + off, 1)[0]


def wr(j, base, off, v):
    j.memory_write32(base + off, [v & 0xFFFFFFFF])


def sniff(j, bus, nframes):
    base = BASE[bus]
    # CAN1 用过滤器组 0..13，CAN2 用 14..27（SlaveStartFilterBank = 14）
    bank = 0 if bus == "CAN1" else 27
    fr = 0x240 + bank * 8
    print("=== %s 嗅探（临时把过滤器组 %d 设为「全收」）===" % (bus, bank))

    fmr = rd(j, base, FMR)
    wr(j, base, FMR, fmr | 1)                      # FINIT = 1 进入配置
    wr(j, base, fr + 0, 0x00000000)                # FR1: ID = 0
    wr(j, base, fr + 4, 0x00000000)                # FR2: mask = 0 → 全收
    fm1r = rd(j, base, FM1R); wr(j, base, FM1R, fm1r & ~(1 << bank))   # 掩码模式
    fs1r = rd(j, base, FS1R); wr(j, base, FS1R, fs1r & ~(1 << bank))   # 进 FIFO0
    ffa1r = rd(j, base, FFA1R); wr(j, base, FFA1R, ffa1r & ~(1 << bank))
    fa1r = rd(j, base, FA1R); wr(j, base, FA1R, fa1r | (1 << bank))    # 激活
    wr(j, base, FMR, fmr & ~1)                     # FINIT = 0 生效

    seen = {}
    total = 0
    t0 = time.time()
    while total < nframes and time.time() - t0 < 2.0:
        r = rd(j, base, RF0R)
        fmp = r & 0x3
        if fmp == 0:
            time.sleep(0.002)
            continue
        for _ in range(fmp):
            rir = rd(j, base, RI0R)
            std_id = (rir >> 21) & 0x7FF
            ide = (rir >> 2) & 1
            dlc = rd(j, base, RDT0R) & 0xF
            data = []
            lo = rd(j, base, RDL0R)
            hi = rd(j, base, RDH0R)
            for k in range(min(dlc, 8)):
                if k < 4:
                    data.append((lo >> (8 * k)) & 0xFF)
                else:
                    data.append((hi >> (8 * (k - 4))) & 0xFF)
            seen[std_id] = seen.get(std_id, 0) + 1
            if total < nframes:
                print("  ID=0x%03X %s DLC=%d  %s" % (std_id, "(扩展)" if ide else "",
                                                     dlc, " ".join("%02X" % d for d in data)))
            total += 1
        wr(j, base, RF0R, 0x20)                    # RFOM：释放 FIFO0

    # 恢复：关掉这个过滤器组
    fmr = rd(j, base, FMR)
    wr(j, base, FMR, fmr | 1)
    fa1r = rd(j, base, FA1R); wr(j, base, FA1R, fa1r & ~(1 << bank))
    wr(j, base, FMR, fmr & ~1)
    print("  收帧 %d 个，出现的 ID：%s" % (total, ", ".join("0x%03X×%d" % (k, v)
                                                   for k, v in sorted(seen.items())) or "（无）"))
    return seen


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", choices=["CAN1", "CAN2"], default="CAN1")
    ap.add_argument("--frames", type=int, default=8)
    args = ap.parse_args()
    import pylink
    j = pylink.JLink()
    j.open()
    try:
        j.set_tif(pylink.enums.JLinkInterfaces.SWD)
        j.set_speed(4000)
        j.connect("STM32F407IG", verbose=False)
        sniff(j, args.bus, args.frames)
    finally:
        try:
            j.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
