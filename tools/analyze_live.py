# -*- coding: utf-8 -*-
"""在 capture_bench 的 CSV 里定位"控制环真正在跑"的区间（按 |pd|>0 判读）。"""
import csv
import math
import os
import sys

NAMES = ["LF", "LB", "RB", "RF"]


def f(x):
    try:
        return float(x)
    except Exception:
        return 0.0


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.environ["TEMP"], "bench4.csv")
    rows = []
    with open(path, encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            rows.append({
                "tick": int(r["tick_ms"]),
                "meas": [f(r["meas_" + n]) for n in NAMES],
                "tgt": [f(r["tgt_" + n]) for n in NAMES],
                "cmd": [f(r["cmd_" + n]) for n in NAMES],
                "pd": [f(r["pd_" + n]) for n in NAMES],
                "vel": [f(r["vel_" + n]) for n in NAMES],
                "tec1": int(r["tec1"]), "lec1": int(r["lec1"]),
                "tec2": int(r["tec2"]), "lec2": int(r["lec2"]),
            })
    # 控制环活跃 = 任意一路 pd 或 cmd 非零
    live = [k for k, r in enumerate(rows) if any(abs(x) > 1e-6 for x in r["pd"] + r["cmd"])]
    if not live:
        print("[i] 全程没有任何控制输出（窗口没在采样期间发生）。")
        print("    tick 范围 %d..%d，样本 %d" % (rows[0]["tick"], rows[-1]["tick"], len(rows)))
        return
    a, b = live[0], live[-1]
    print("[i] 控制环活跃区间：tick %d .. %d ms（%d 条样本，约 %.1f s）"
          % (rows[a]["tick"], rows[b]["tick"], b - a + 1,
             (rows[b]["tick"] - rows[a]["tick"]) / 1000.0))
    seg = rows[a:b + 1]
    print("\n时间(ms)   " + "  ".join("%-30s" % n for n in NAMES))
    step = max(1, len(seg) // 22)
    for r in seg[::step]:
        print("  %6d   " % r["tick"] + "  ".join(
            "角%+7.2f 目标%+7.2f 电流%+6.2f" % (r["meas"][i], r["tgt"][i], r["cmd"][i])
            for i in range(4)))
    print("\n----- 逐轮判读 -----")
    for i, n in enumerate(NAMES):
        meas = [r["meas"][i] for r in seg]
        cmds = [r["cmd"][i] for r in seg]
        vels = [r["vel"][i] for r in seg]
        jumps = [abs(meas[k] - meas[k - 1]) for k in range(1, len(meas))]
        big = sum(1 for j in jumps if j > 0.5)
        print("  %-3s 角 %+7.2f .. %+7.2f（跨度 %5.2f） 末角 %+7.2f  |电流|max %5.2f A  |角速度|max %5.2f"
              "  |Δ角|>0.5° 占 %4.1f%%" %
              (n, min(meas), max(meas), max(meas) - min(meas), meas[-1],
               max(abs(c) for c in cmds), max(abs(v) for v in vels),
               100.0 * big / max(1, len(jumps))))
    # 误差趋势：把区间等分成 5 段，看末尾误差是否在减小
    print("\n误差趋势（每轮按区间 5 等分，看 |目标-实测| 均值是否下降）：")
    for i, n in enumerate(NAMES):
        marks = []
        chunk = max(1, len(seg) // 5)
        for c in range(5):
            part = seg[c * chunk:(c + 1) * chunk] or seg[-1:]
            errs = [abs(r["tgt"][i] - r["meas"][i]) for r in part]
            marks.append(sum(errs) / len(errs))
        arrow = "下降" if marks[-1] < marks[0] * 0.8 else ("持平" if marks[-1] < marks[0] * 1.2 else "恶化")
        print("  %-3s %s  -> %s" % (n, "  ".join("%6.2f" % m for m in marks), arrow))
    tec = [(r["tec1"], r["lec1"], r["tec2"], r["lec2"]) for r in seg]
    bad = [x for x in tec if any(x)]
    print("\nCAN 错误：活跃区间内 %d/%d 条样本有错%s" %
          (len(bad), len(tec), "" if not bad else "，首条 %r" % (bad[0],)))


if __name__ == "__main__":
    main()
