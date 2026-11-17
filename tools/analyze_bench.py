# -*- coding: utf-8 -*-
"""分析 capture_bench.py 产出的 CSV：找抖动频率、反馈冻结时刻、轮速反应。"""
import csv
import math
import sys
import os

NAMES = ["LF", "LB", "RB", "RF"]


def f(x):
    try:
        return float(x)
    except Exception:
        return 0.0


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.environ["TEMP"], "bench2.csv")
    rows = []
    with open(path, encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            rows.append({
                "t": f(r["t_s"]),
                "tick": int(r["tick_ms"]),
                "meas": [f(r["meas_" + n]) for n in NAMES],
                "tgt": [f(r["tgt_" + n]) for n in NAMES],
            })
    print("样本 %d 条，tick %d..%d" % (len(rows), rows[0]["tick"], rows[-1]["tick"]))

    # 1) 找"冻结"起点：某轮连续 50 个样本完全不变
    for i, n in enumerate(NAMES):
        frozen_at = None
        run = 1
        for k in range(1, len(rows)):
            if abs(rows[k]["meas"][i] - rows[k - 1]["meas"][i]) < 1e-6:
                run += 1
                if run == 50 and frozen_at is None:
                    frozen_at = rows[k]["tick"]
            else:
                run = 1
        print("  %-3s 反馈最后一次变化 @tick %s" % (n, frozen_at))

    # 2) 找电机通电/反馈开始的第一个样本
    arm = None
    for r in rows:
        if r["tick"] > 200 and any(abs(x) > 0.3 for x in r["meas"]):
            arm = r["tick"]
            break
    if arm is None:
        print("[FAIL] 没有非零反馈")
        return
    win = [r for r in rows if arm <= r["tick"] < arm + 5000]
    print("\n窗口 %d..%d ms，%d 条样本" % (arm, arm + 5000, len(win)))

    # 3) 每 250 ms 打一行，看清"抖 -> 冻结"的过程
    print("\n时间(ms)  " + "  ".join("%9s" % n for n in NAMES))
    step = max(1, len(win) // 20)
    for r in win[::step]:
        print("  %6d  " % r["tick"] + "  ".join("%+9.2f" % x for x in r["meas"]))

    # 4) 抖动频率估计：用相邻样本的符号翻转率，估等效频率
    print("\n各轮抖动估计（只在反馈还活着的段内算）：")
    for i, n in enumerate(NAMES):
        # 用"最后一次变化"作为活着的截止
        alive = []
        for k in range(1, len(win)):
            if abs(win[k]["meas"][i] - win[k - 1]["meas"][i]) > 1e-6:
                alive.append(k)
        if not alive:
            print("  %-3s 全程无变化" % n)
            continue
        last = alive[-1]
        seg = win[: last + 1]
        diffs = [seg[k]["meas"][i] - seg[k - 1]["meas"][i] for k in range(1, len(seg))]
        nz = [1 if d > 0 else -1 for d in diffs if abs(d) > 1e-6]
        flips = sum(1 for k in range(1, len(nz)) if nz[k] != nz[k - 1])
        dt = (seg[-1]["t"] - seg[0]["t"])
        dur = seg[-1]["tick"] - seg[0]["tick"]
        amp = max(x["meas"][i] for x in seg) - min(x["meas"][i] for x in seg)
        print("  %-3s 有效段 %d ms，峰峰 %.2f°，方向翻转 %d 次 => 约 %.0f Hz，平均采样 %.0f Hz"
              % (n, dur, amp, flips, flips / 2.0 / max(1e-6, dur / 1000.0), len(rows) / (rows[-1]["t"] - rows[0]["t"])))


if __name__ == "__main__":
    main()
