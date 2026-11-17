# -*- coding: utf-8 -*-
"""用"模拟遥控 + 云台 yaw 目标 + 速度注入"在固件内部跑完整验证，不需要人操作。

流程（每一步都读回真实反馈）：
  1) 打开模拟遥控（左拨杆=中/合法档、右拨杆=跟随），等状态机进 Working
  2) 把云台 yaw 目标从当前值**分小步**爬到 target，读 δ（frame_delta，解算器约定）
  3) 注入前进 vx，读四个舵角目标与轮速目标 —— 判据：舵角应 ≈ +target
  4) 收尾：清注入、云台回 0、关模拟遥控

用法：
    python tools\\sim_test.py --yaw-deg 90
    python tools\\sim_test.py --yaw-deg 90 --vx 0.6 --keep
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_vars as rv  # noqa: E402

NAMES = ["LF", "LB", "RB", "RF"]


class Link:
    def __init__(self):
        import pylink
        nm = rv.find_nm()
        elf = rv.elf_path("Chassis")
        self.gdbg, _ = rv.symbol_addr(nm, elf)
        self.sym = {}
        # 只要求脚本真正会用到的符号。注意 g_dbg_sim_lv/lh 会被 -gc-sections 回收
        # （底盘不用左杆，StickLv/StickLh 是死代码），所以不列进来。
        for n in ("robot::g_dbg_rc_sim", "robot::g_dbg_sim_s1", "robot::g_dbg_sim_s2",
                  "robot::g_dbg_sim_rv", "robot::g_dbg_sim_rh",
                  "robot::g_dbg_drive_inject", "robot::g_dbg_inject_vx",
                  "robot::g_dbg_inject_vy", "robot::g_dbg_inject_w",
                  "g_dbg_yaw_hold_enable", "g_dbg_yaw_target_delta"):
            a, _sz = rv.symbol_addr(nm, elf, n)
            if a is None:
                raise SystemExit("[FAIL] 符号缺失：%s（固件不是这一版？）" % n)
            self.sym[n] = a
        self.jl = pylink.JLink()
        self.jl.open()
        self.jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
        self.jl.set_speed(4000)
        self.jl.connect("STM32F407IG", verbose=False)
        v = self.jl.memory_read32(0xE0042008, 1)[0]
        self.jl.memory_write32(0xE0042008, [v | (1 << 12)])

    def u32(self, name, val=None):
        a = self.sym[name]
        if val is None:
            return self.jl.memory_read32(a, 1)[0]
        self.jl.memory_write32(a, [int(val) & 0xFFFFFFFF])

    def f32(self, name, val=None):
        import struct
        a = self.sym[name]
        if val is None:
            b = bytes(bytearray(self.jl.memory_read8(a, 4)))
            return struct.unpack("<f", b)[0]
        self.jl.memory_write32(a, [struct.unpack("<I", struct.pack("<f", float(val)))[0]])

    def state(self):
        return rv.decode(bytes(bytearray(self.jl.memory_read8(self.gdbg, rv.SIZE))))

    def close(self):
        try:
            self.jl.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaw-deg", type=float, default=90.0, help="云台目标 δ（度）")
    ap.add_argument("--vx", type=float, default=0.6, help="注入前进速度 m/s（0 = 不注入）")
    ap.add_argument("--mode", type=int, default=2, help="右拨杆：1=上(分离) 3=中(跟随) 2=下(小陀螺)，这里填开关编码")
    ap.add_argument("--keep", action="store_true", help="结束后不清理（便于继续观察）")
    args = ap.parse_args()

    L = Link()
    try:
        # ---- 1) 打开模拟遥控 ----
        L.u32("robot::g_dbg_sim_s1", 3)      # 左拨杆：中（合法档，不触发急停）
        L.u32("robot::g_dbg_sim_s2", args.mode)
        L.f32("robot::g_dbg_sim_rv", 0.0)
        L.f32("robot::g_dbg_sim_rh", 0.0)
        L.u32("robot::g_dbg_drive_inject", 0)
        L.f32("robot::g_dbg_inject_vx", 0.0)
        L.f32("robot::g_dbg_inject_vy", 0.0)
        L.f32("robot::g_dbg_inject_w", 0.0)
        L.u32("robot::g_dbg_rc_sim", 1)
        print("[i] 模拟遥控已打开（s1=3 左中，s2=%d），等状态机进 Working ..." % args.mode)
        t0 = time.time()
        while time.time() - t0 < 6.0:
            d = L.state()
            if d["life"] == 2:
                break
            time.sleep(0.1)
        d = L.state()
        print("    状态: life=%d(%s) mode=%d estop=%d rcon=%d peer_off=%d imu=%d"
              % (d["life"], rv.LIFE.get(d["life"], "?"), d["mode"], d["estop"],
                 d["rc_online"], d["peer_off"], d["imu_ready"]))
        if d["life"] != 2:
            print("[FAIL] 进不了 Working（peer_off或imu不满足），后续动作会没有输出。")

        # ---- 2) 云台转到目标 δ（分小步，避免一次大阶跃）----
        cur = rv.deg(d["frame_delta"])
        print("[i] 当前 δ=%+.1f°，目标 %+.1f°，分步爬升 ..." % (cur, args.yaw_deg))
        L.u32("g_dbg_yaw_hold_enable", 0)     # 先保持软掉，写入目标后再使能
        L.f32("g_dbg_yaw_target_delta", math.radians(cur))
        L.u32("g_dbg_yaw_hold_enable", 1)
        steps = max(1, int(abs(args.yaw_deg - cur) / 10.0))
        for k in range(1, steps + 1):
            tgt = cur + (args.yaw_deg - cur) * k / steps
            L.f32("g_dbg_yaw_target_delta", math.radians(tgt))
            time.sleep(0.25)
        # 云台是力控位置环（kp=0.6 N·m/rad），需要时间收敛 + 克服摩擦，多等一会儿
        for _ in range(10):
            time.sleep(0.3)
            d = L.state()
            if abs(rv.deg(d["frame_delta"]) + args.yaw_deg) < 3.0:
                break
        time.sleep(0.5)
        d = L.state()
        got = rv.deg(d["frame_delta"])
        print("    云台到位：δ(解算器约定)=%+.1f°  （云台实际 ψ=%+.1f°）" % (got, -got))

        # ---- 3) 注入前进，看逆解 ----
        if abs(args.vx) > 1e-6:
            print("[i] 注入前进 vx=%.2f m/s（跟随模式：应有 δ 参与的旋转变换）" % args.vx)
            L.f32("robot::g_dbg_inject_vx", args.vx)
            L.u32("robot::g_dbg_drive_inject", 1)
            time.sleep(1.5)
            d = L.state()
            print("    舵角目标: %s  （判据：应 ≈ %+.0f°）"
                  % (" ".join("%+6.1f" % rv.deg(x) for x in d["steer_t"]), args.yaw_deg))
            print("    轮速目标: %s  rad/s" % (" ".join("%+6.2f" % x for x in d["wheel_t"])))
            time.sleep(1.0)
            d = L.state()
            print("    实测轮速: %s  rad/s" % (" ".join("%+6.2f" % x for x in d["wheel_v"])))
            L.u32("robot::g_dbg_drive_inject", 0)
            L.f32("robot::g_dbg_inject_vx", 0.0)
        else:
            print("[i] 未注入速度（--vx 0）")

        if not args.keep:
            print("[i] 收尾：云台回 0、关模拟遥控")
            for k in range(9):
                L.f32("g_dbg_yaw_target_delta", math.radians(-got * (1 - k / 8.0)))
                time.sleep(0.2)
            L.u32("g_dbg_yaw_hold_enable", 0)
            L.u32("robot::g_dbg_rc_sim", 0)
        else:
            print("[i] --keep：云台保持在位、模拟遥控仍然打开（方便你继续看）")
    finally:
        L.close()


if __name__ == "__main__":
    main()
