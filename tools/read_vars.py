#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
read_vars.py —— 用调试器直接读固件里的调试快照 robot::g_dbg（方案②）

为什么这么做：现场最缺的是「读数值」。这个脚本不依赖串口、不用改代码，
只要 ST-Link 插着，就能一次读到舵角、轮速、IMU、状态机、双板链路全部关键量。

用法：
    python tools/read_vars.py                 # 读底盘板一次
    python tools/read_vars.py -Board Gimbal    # 读云台板一次
    python tools/read_vars.py --watch 0.5      # 每 0.5 秒刷新（现场拨遥控器时用）

依赖：pyocd（本机已装）、arm-none-eabi-nm（在 C:/tools/gcc-arm-none-eabi/bin/）
"""
import argparse
import math
import os
import re
import struct
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

NM_CANDIDATES = [
    r"C:/tools/gcc-arm-none-eabi/bin/arm-none-eabi-nm.exe",
    "arm-none-eabi-nm",
]

# 与 RobotComponents/component/dbg_snapshot.hpp 严格对应，改结构必须同步改这里
LAYOUT = "<II4f4f4f4ffffI8B"
SIZE = struct.calcsize(LAYOUT)   # 应为 96

LIFE = {0: "Dead", 1: "Resurrection", 2: "Working"}
MODE = {0: "分离", 1: "跟随", 2: "小陀螺"}
CTRL = {0: "None", 1: "Remote"}


def find_nm():
    for c in NM_CANDIDATES:
        try:
            subprocess.run([c, "--version"], capture_output=True, check=True)
            return c
        except Exception:
            continue
    return None


def elf_path(board):
    return os.path.join(REPO, board, "build", "steered_%s.elf" % board.lower())


def symbol_addr(nm, elf, name="robot::g_dbg"):
    out = subprocess.run([nm, "-S", "-C", elf], capture_output=True)
    for line in out.stdout.decode("utf-8", "ignore").splitlines():
        if name in line:
            parts = line.split()
            # 形如: 20000c6c 00000060 V robot::g_dbg
            try:
                addr = int(parts[0], 16)
                size = int(parts[1], 16)
                return addr, size
            except (ValueError, IndexError):
                continue
    return None, None


def decode(buf):
    v = struct.unpack(LAYOUT, buf)
    i = 0
    d = {}
    d["magic"] = v[i]; i += 1
    d["tick"] = v[i]; i += 1
    d["steer"] = list(v[i:i+4]); i += 4
    d["steer_t"] = list(v[i:i+4]); i += 4
    d["wheel_t"] = list(v[i:i+4]); i += 4
    d["wheel_v"] = list(v[i:i+4]); i += 4
    d["imu_yaw"] = v[i]; i += 1
    d["imu_pitch"] = v[i]; i += 1
    d["frame_delta"] = v[i]; i += 1
    d["tx_fail"] = v[i]; i += 1
    d["life"] = v[i]; i += 1
    d["mode"] = v[i]; i += 1
    d["ctrl"] = v[i]; i += 1
    d["imu_ready"] = v[i]; i += 1
    d["peer_off"] = v[i]; i += 1
    d["estop"] = v[i]; i += 1
    d["rc_online"] = v[i]; i += 1
    d["reserved"] = v[i]
    return d


def deg(x):
    return x * 180.0 / math.pi


def show(d, board, raw_addr):
    print("=" * 72)
    print(" %s   g_dbg @ 0x%08X   读取时刻 tick=%d ms" % (board, raw_addr, d["tick"]))
    print("=" * 72)

    if d["magic"] != 0x5A5ADB91:
        print("  !! magic = 0x%08X，不是 0x5A5ADB91" % d["magic"])
        print("     原因：固件不是这一版，或还没跑到 DbgPublish()（未上电/刚复位）")
        return

    if board == "Chassis":
        print("  舵角（实测 / 目标，度）：")
        for i, name in enumerate(["LF", "LB", "RB", "RF"]):
            print("    %-3s  实测 %+8.2f   目标 %+8.2f   误差 %+7.2f" % (
                name, deg(d["steer"][i]), deg(d["steer_t"][i]),
                deg(d["steer_t"][i] - d["steer"][i])))
        print("  轮速（目标线速度 m/s / 实测输出端 rad/s）：")
        for i, name in enumerate(["LF", "LB", "RB", "RF"]):
            print("    %-3s  目标 %+7.3f  实测 %+8.3f" % (name, d["wheel_t"][i], d["wheel_v"][i]))
    else:
        print("  云台关节（实测，度）：")
        print("    yaw   %+8.2f" % deg(d["steer"][0]))
        print("    pitch %+8.2f" % deg(d["steer"][1]))

    print("  IMU： yaw %+8.2f 度   pitch %+8.2f 度   ready=%d"
          % (deg(d["imu_yaw"]), deg(d["imu_pitch"]), d["imu_ready"]))
    print("  δ（云台相对车头）：%+8.2f 度" % deg(d["frame_delta"]))
    print("  状态机： %-12s  模式 %-8s  控制源 %-6s"
          % (LIFE.get(d["life"], d["life"]), MODE.get(d["mode"], d["mode"]), CTRL.get(d["ctrl"], d["ctrl"])))
    print("  标志位： 急停=%d  遥控在线=%d  对端离线=%d  CAN发送失败=%d"
          % (d["estop"], d["rc_online"], d["peer_off"], d["tx_fail"]))


def main():
    ap = argparse.ArgumentParser(description="用调试器读固件里的调试快照 robot::g_dbg")
    ap.add_argument("-Board", "--board", default="Chassis", choices=["Chassis", "Gimbal"])
    ap.add_argument("--watch", type=float, default=0.0, help="秒；>0 则循环刷新")
    ap.add_argument("--target", default="stm32f407vg", help="pyocd 目标名")
    args = ap.parse_args()

    nm = find_nm()
    if not nm:
        print("[FAIL] 找不到 arm-none-eabi-nm，请确认 C:/tools/gcc-arm-none-eabi/bin 存在")
        return 2

    elf = elf_path(args.board)
    if not os.path.exists(elf):
        print("[FAIL] 找不到固件：%s（先编译一次）" % elf)
        return 2

    addr, size = symbol_addr(nm, elf)
    if addr is None:
        print("[FAIL] 在 %s 里没找到 robot::g_dbg 符号" % os.path.basename(elf))
        return 2
    if size != SIZE:
        print("[WARN] ELF 里 g_dbg 大小 %d，脚本按 %d 解析 —— 结构改过就要同步改 LAYOUT" % (size, SIZE))

    try:
        from pyocd.core.helpers import ConnectHelper
    except Exception as e:
        print("[FAIL] 导入 pyocd 失败：%r" % (e,))
        print("        pip install pyocd")
        return 2

    try:
        session = ConnectHelper.session_with_chosen_probe(
            options={"target_override": args.target}, blocking=False, return_first=True)
    except TypeError:
        session = ConnectHelper.session_with_chosen_probe(
            target_override=args.target)
    if session is None:
        print("[FAIL] 没有找到调试器。检查：ST-Link 插好？只接 SWD？驱动装好？")
        print("       列出调试器： pyocd list")
        return 2

    try:
        with session:
            target = session.target
            first = True
            while True:
                try:
                    buf = bytes(target.read_memory_block8(addr, SIZE))
                except Exception as e:
                    print("[FAIL] 读内存失败：%r" % (e,))
                    print("       常见原因：目标未运行（被 halt）或调试器连接不稳定")
                    return 2
                show(decode(buf), args.board, addr)
                first = False
                if args.watch <= 0:
                    break
                time.sleep(args.watch)
                print()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
