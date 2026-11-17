# -*- coding: utf-8 -*-
"""复位/直接采样底盘调试快照 + 舵角环探针 + CAN 错误计数器。

为什么单独写：
  read_vars_jlink.py 只读 g_dbg（舵角/轮速/状态机），读不到"到底给电机下了多少 raw"，
  也读不到 CAN 错误计数器 —— 而现场两次"高频啸叫"的因果链恰好需要这两样：
  是 CAN 先坏（帧被破坏 -> 垃圾电流），还是控制先振（PD 打满 -> 真抖动）？
  所以本脚本每拍同时读：
    robot::g_dbg           96 B   舵角实测/目标等
    g_dbg_steer_cmd        16 B   真正进 CAN 帧的 raw（死区保持+限幅+变化率限幅之后）
    g_dbg_steer_pd         16 B   限幅前的 PD 控制量
    g_dbg_steer_vel        16 B   电机反馈角速度
    CAN1/CAN2->ESR          4 B   TEC / REC / LEC

复位用 AIRCR.SYSRESETREQ（整片复位）。
  为什么不用 jl.reset()：它默认只复位内核，外设不动；固件会对着还在跑的 CAN 再跑
  一遍 HAL_CAN_Init，导致两路 CAN 时序错乱（实测 TEC 72/59、LEC=5 BIT0、BOFF=1），
  电机把时序错乱的帧当垃圾电流指令 -> 四台 GM6020 高频啸叫。

用法：
    python tools\\capture_bench.py --ms 90000          # 复位后采 90 s，等你上电
    python tools\\capture_bench.py --no-reset --ms 90000
"""
import argparse
import csv
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_vars as rv  # noqa: E402

DBGMCU_APB1_FZ = 0xE0042008
DBG_IWDG_STOP = 1 << 12
CAN1_ESR = 0x40006418
CAN2_ESR = 0x40006818

NAMES = ["LF", "LB", "RB", "RF"]
PROBES = ["g_dbg_steer_cmd", "g_dbg_steer_pd", "g_dbg_steer_vel"]
BENCH_SYM = "g_dbg_bench"   # 2×uint32: [0]=armed [1]=MotorSteerRxCount()
LEC = {0: "OK", 1: "STUFF", 2: "FORM", 3: "ACK", 4: "BIT1", 5: "BIT0", 6: "CRC", 7: "SW"}


def deg(x):
    return x * 180.0 / math.pi


def resolve(nm, elf, name):
    a, s = rv.symbol_addr(nm, elf, name)
    return a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-Board", "--board", default="Chassis", choices=["Chassis", "Gimbal"])
    ap.add_argument("--ms", type=int, default=90000,
                    help="采样时长（毫秒）；窗口由舵反馈首次到达触发，所以要盖住人工上电的时间")
    ap.add_argument("--device", default="STM32F407IG")
    ap.add_argument("--speed", type=int, default=4000)
    ap.add_argument("--out", default="")
    ap.add_argument("--no-reset", action="store_true")
    args = ap.parse_args()

    import pylink

    nm = rv.find_nm()
    elf = rv.elf_path(args.board)
    if not (nm and os.path.exists(elf)):
        print("[FAIL] 找不到 nm 或 ELF")
        return 2
    addr, size = rv.symbol_addr(nm, elf)
    if addr is None:
        print("[FAIL] ELF 里没有 robot::g_dbg")
        return 2
    probes = {}
    for p in PROBES:
        a = resolve(nm, elf, p)
        if a is None:
            print("[WARN] 找不到探针符号 %s（固件没编进去？）" % p)
            continue
        probes[p] = a
    print("[i] g_dbg@0x%08X 探针=%s  ELF=%s"
          % (addr, {k: hex(v) for k, v in probes.items()},
             time.strftime("%H:%M:%S", time.localtime(os.path.getmtime(elf)))))
    bench_addr = resolve(nm, elf, BENCH_SYM)
    print("[i] bench 探针 %s" % (hex(bench_addr) if bench_addr else "缺失"))

    jl = pylink.JLink()
    jl.open()
    jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jl.set_speed(args.speed)
    jl.connect(args.device, verbose=False)

    def arm_wdg():
        v = jl.memory_read32(DBGMCU_APB1_FZ, 1)[0]
        jl.memory_write32(DBGMCU_APB1_FZ, [v | DBG_IWDG_STOP])

    arm_wdg()

    if not args.no_reset:
        print("[i] 系统复位：AIRCR.SYSRESETREQ")
        jl.memory_write32(0xE000ED0C, [0x05FA0004])
        time.sleep(0.30)
        arm_wdg()
        print("[i] halted=%s（应为 False）" % jl.halted())

    t0 = time.time()
    rows = []
    ex = []
    try:
        while (time.time() - t0) * 1000.0 < args.ms:
            try:
                d = rv.decode(bytes(bytearray(jl.memory_read8(addr, rv.SIZE))))
                x = {}
                for p, a in probes.items():
                    x[p] = list(struct.unpack("<4f", bytes(bytearray(jl.memory_read8(a, 16)))))
                if bench_addr:
                    b = struct.unpack("<2I", bytes(bytearray(jl.memory_read8(bench_addr, 8))))
                    x["armed"] = [float(b[0])] * 4
                    x["rx"] = [float(b[1])] * 4
                e1 = jl.memory_read32(CAN1_ESR, 1)[0]
                e2 = jl.memory_read32(CAN2_ESR, 1)[0]
            except Exception as e:
                print("[FAIL] 读内存失败 @%.0f ms：%r" % ((time.time() - t0) * 1000.0, e))
                break
            t = time.time() - t0
            rows.append((t, d))
            ex.append((x, (e1 >> 24) & 0xFF, (e1 >> 4) & 7, (e2 >> 24) & 0xFF, (e2 >> 4) & 7))
    finally:
        try:
            jl.close()
        except Exception:
            pass

    if not rows:
        print("[FAIL] 没采到样本")
        return 2

    out = args.out or os.path.join(os.environ.get("TEMP", "."), "bench_capture.csv")
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        head = ["t_s", "tick_ms", "magic"]
        for n in NAMES:
            head += ["meas_" + n, "tgt_" + n, "cmd_" + n, "pd_" + n, "vel_" + n]
        head += ["armed", "rx_count", "tec1", "lec1", "tec2", "lec2"]
        w.writerow(head)
        for (t, d), (x, tec1, lec1, tec2, lec2) in zip(rows, ex):
            line = ["%.4f" % t, d["tick"], "%08X" % d["magic"]]
            for i in range(4):
                line += ["%.2f" % deg(d["steer"][i]), "%.2f" % deg(d["steer_t"][i]),
                         "%.3f" % x.get("g_dbg_steer_cmd", [0] * 4)[i],
                         "%.3f" % x.get("g_dbg_steer_pd", [0] * 4)[i],
                         "%.2f" % x.get("g_dbg_steer_vel", [0] * 4)[i]]
            line += ["%d" % int(x.get("armed", [0])[0]), "%d" % int(x.get("rx", [0])[0]),
                     tec1, lec1, tec2, lec2]
            w.writerow(line)
    dt = rows[-1][0] - rows[0][0]
    print("[i] 样本 %d 条，平均 %.0f Hz，CSV=%s" % (len(rows), len(rows) / max(1e-6, dt), out))

    ok = [k for k in range(len(rows)) if rows[k][1]["magic"] == 0x5A5ADB91]
    if not ok:
        print("[FAIL] magic 全不对：固件不是这一版，或复位后没跑到 DbgPublish")
        return 2
    print("[i] magic 正常 %d 条，tick %d..%d"
          % (len(ok), rows[ok[0]][1]["tick"], rows[ok[-1]][1]["tick"]))

    arm = None
    for k in ok:
        if rows[k][1]["tick"] > 200 and any(abs(deg(rows[k][1]["steer"][i])) > 0.3 for i in range(4)):
            arm = k
            break
    if arm is None:
        print("\n[FAIL] 舵角全程 0.00 —— GM6020 反馈一次都没到（电机没电 / CAN1 不通）。")
        print("       窗口没有被触发，本次未下发任何指令。")
        return 3

    t_arm = rows[arm][1]["tick"]
    win = [k for k in ok if t_arm <= rows[k][1]["tick"] < t_arm + 5000]
    print("\n===== 窗口起点 tick=%d ms，取其后 5000 ms：%d 条 =====" % (t_arm, len(win)))
    print("\n时间(ms)   " + "  ".join("%-26s" % n for n in NAMES))
    print("            " + "  ".join("%-26s" % "角/命令/PD/角速度" for _ in NAMES))
    step = max(1, len(win) // 18)
    for k in win[::step]:
        d = rows[k][1]
        x = ex[k][0]
        seg = []
        for i in range(4):
            seg.append("%+7.2f %+6.0f %+7.0f %+6.2f" % (
                deg(d["steer"][i]),
                x.get("g_dbg_steer_cmd", [0] * 4)[i],
                x.get("g_dbg_steer_pd", [0] * 4)[i],
                x.get("g_dbg_steer_vel", [0] * 4)[i]))
        print("  %6d   " % d["tick"] + "  ".join(seg))

    # CAN 错误首次出现时刻（以 tick 表示，方��和窗口对齐）
    first_err = None
    for k in win:
        _, tec1, lec1, tec2, lec2 = ex[k]
        if tec1 or lec1 or tec2 or lec2:
            first_err = (rows[k][1]["tick"], tec1, lec1, tec2, lec2)
            break
    print("\n----- CAN 错误 -----")
    if first_err is None:
        print("  窗口内两路 CAN 的 TEC/LEC 全程为 0：总线没坏，抖动是控制/机械层面的。")
    else:
        print("  首次出现错误 @tick %d ms：CAN1 TEC=%d LEC=%d(%s)，CAN2 TEC=%d LEC=%d(%s)"
              % (first_err[0], first_err[1], first_err[2], LEC.get(first_err[2], "?"),
                 first_err[3], first_err[4], LEC.get(first_err[4], "?")))

    print("\n----- 逐轮 -----")
    for i, n in enumerate(NAMES):
        seg = win
        meas = [deg(rows[k][1]["steer"][i]) for k in seg]
        cmds = [ex[k][0].get("g_dbg_steer_cmd", [0] * 4)[i] for k in seg]
        pds = [ex[k][0].get("g_dbg_steer_pd", [0] * 4)[i] for k in seg]
        vels = [ex[k][0].get("g_dbg_steer_vel", [0] * 4)[i] for k in seg]
        jumps = [abs(meas[k] - meas[k - 1]) for k in range(1, len(meas))]
        big = sum(1 for j in jumps if j > 0.5)
        print("  %-3s 角 %+7.2f..%+7.2f  |cmd|max %5.0f  |pd|max %6.0f  |vel|max %6.2f"
              "  |Δ角|>0.5° 占 %4.1f%%  末尾角 %+7.2f"
              % (n, min(meas), max(meas), max(abs(c) for c in cmds),
                 max(abs(p) for p in pds), max(abs(v) for v in vels),
                 100.0 * big / max(1, len(jumps)), meas[-1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
