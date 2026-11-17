# -*- coding: utf-8 -*-
"""连续采样 g_dbg 若干秒，抓"手转/推车"这类瞬时动作。

为什么要它：g_dbg 里的 wheel_vel 是**瞬时**值，人推完车手一停就回 0，
单次读只能看到 0。所以这里连续采一段时间，事后统计每个量的 min/max，
再判断"往车头方向滚时符号是正还是负"。

用法：
    python tools\\capture_motion.py -Board Chassis --seconds 25
输出：每 0.5 s 一行的进度 + 最后的分轮统计 + tools\\_cap.csv
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_vars as rv  # noqa: E402

NAMES = ["LF", "LB", "RB", "RF"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-Board", "--board", default="Chassis", choices=["Chassis", "Gimbal"])
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--rate", type=float, default=50.0, help="采样频率 Hz")
    args = ap.parse_args()

    import pylink

    nm = rv.find_nm()
    elf = rv.elf_path(args.board)
    addr, _ = rv.symbol_addr(nm, elf)

    j = pylink.JLink()
    j.open()
    try:
        j.set_tif(pylink.enums.JLinkInterfaces.SWD)
        j.set_speed(4000)
        j.connect("STM32F407IG", verbose=False)
        try:
            v = j.memory_read32(0xE0042008, 1)[0]
            j.memory_write32(0xE0042008, [v | (1 << 12)])
        except Exception:
            pass

        rows = []
        t0 = time.time()
        period = 1.0 / args.rate
        next_t = t0
        next_print = t0 + 1.0
        prev = None
        wrap = [0.0] * 4   # 舵角累计变化（处理 ±180 回绕）
        while time.time() - t0 < args.seconds:
            now = time.time()
            if now < next_t:
                time.sleep(next_t - now)
            next_t += period
            try:
                buf = bytes(bytearray(j.memory_read8(addr, rv.SIZE)))
            except Exception as e:
                print("[WARN] 读失败：%r" % (e,))
                time.sleep(0.05)
                continue
            d = rv.decode(buf)
            if d["magic"] != 0x5A5ADB91:
                print("[WARN] magic 不对，跳过")
                continue
            t = time.time() - t0
            rows.append((t, d["tick"], d["steer"], d["wheel_v"]))
            if now >= next_print:
                next_print = now + 1.0
                sv = " ".join("%+6.1f" % (x * 57.29578) for x in d["steer"])
                wv = " ".join("%+7.2f" % x for x in d["wheel_v"])
                print("  t=%5.1fs 舵[%s] 轮速[%s]" % (t, sv, wv))
    finally:
        try:
            j.close()
        except Exception:
            pass

    if not rows:
        print("[FAIL] 一个样都没采到")
        return 2

    print("\n================ 统计（%d 个样，%.0f Hz，%.0f s）================"
          % (len(rows), args.rate, args.seconds))
    for i, nmx in enumerate(NAMES):
        wv = [r[3][i] for r in rows]
        nz = sum(1 for x in wv if abs(x) > 0.05)
        print("  轮 %-3s 轮速 min %+7.2f  max %+7.2f  非零样本 %d/%d"
              % (nmx, min(wv), max(wv), nz, len(wv)))
    print("  舵角首末（度）：")
    for i, nmx in enumerate(NAMES):
        a, b = rows[0][2][i], rows[-1][2][i]
        print("    %-3s %+7.2f -> %+7.2f  (Δ %+7.2f)" % (nmx, a * 57.29578, b * 57.29578,
                                                          (b - a) * 57.29578))

    csv = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_cap.csv")
    with open(csv, "w", encoding="ascii") as f:
        f.write("t,tick,steer0,steer1,steer2,steer3,w0,w1,w2,w3\n")
        for t, tick, s, w in rows:
            f.write("%.4f,%d,%s,%s\n" % (t, tick, ",".join("%.5f" % x for x in s),
                                         ",".join("%.5f" % x for x in w)))
    print("  CSV: %s" % csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
