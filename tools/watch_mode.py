# -*- coding: utf-8 -*-
"""只打印数字：左/右拨杆原始编码、由它算出的急停与模式、状态机。

为什么不用中文输出：现场 PowerShell 控制台是 GBK，中文会变成乱码，
"分离"和"跟随"看不出区别（2026-11-18 就这么误判过一次）。
数字约定：
  life : 0=Dead 1=Resurrection 2=Working
  mode : 0=Depart(分离) 1=Follow(跟随) 2=Gyro(小陀螺)
  s1/s2: 拨杆原始编码（DR16: 1=Up 2=Down 3=Mid，0=非法）
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_vars as rv  # noqa: E402


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    import pylink
    nm = rv.find_nm()
    addr, _ = rv.symbol_addr(nm, rv.elf_path("Chassis"))
    relay, _ = rv.symbol_addr(nm, rv.elf_path("Chassis"), "robot::g_rc_relay")
    jl = pylink.JLink()
    jl.open()
    jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jl.set_speed(4000)
    jl.connect("STM32F407IG", verbose=False)
    v = jl.memory_read32(0xE0042008, 1)[0]
    jl.memory_write32(0xE0042008, [v | (1 << 12)])
    t0 = time.time()
    last = None
    print("tick     s1 s2 | life mode estop rcon |  ddelta_deg")
    while time.time() - t0 < secs:
        d = rv.decode(bytes(bytearray(jl.memory_read8(addr, rv.SIZE))))
        rb = bytes(bytearray(jl.memory_read8(relay, 16)))
        s1, s2 = rb[5], rb[6]
        cur = (s1, s2, d["life"], d["mode"], d["estop"])
        if cur != last:
            print("0x%06X %2d %2d |  %d    %d    %d     %d   | %+8.2f"
                  % (d["tick"], s1, s2, d["life"], d["mode"], d["estop"],
                     d["rc_online"], rv.deg(d["frame_delta"])))
            last = cur
        time.sleep(0.04)
    jl.close()


if __name__ == "__main__":
    main()
