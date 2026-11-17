# -*- coding: utf-8 -*-
"""盯舵轮：角度 / 角速度 / 实际电流。用于确认"是否收敛、是否飞转"。纯数字输出。"""
import os
import sys
import struct
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_vars as rv  # noqa: E402


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    import pylink
    nm = rv.find_nm()
    elf = rv.elf_path("Chassis")
    a, _ = rv.symbol_addr(nm, elf)
    a_cmd, _ = rv.symbol_addr(nm, elf, "g_dbg_steer_cmd")
    a_vel, _ = rv.symbol_addr(nm, elf, "g_dbg_steer_vel")
    jl = pylink.JLink()
    jl.open()
    jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jl.set_speed(4000)
    jl.connect("STM32F407IG", verbose=False)
    v = jl.memory_read32(0xE0042008, 1)[0]
    jl.memory_write32(0xE0042008, [v | (1 << 12)])
    t0 = time.time()
    print("tick   life | ang(deg) LF/LB/RB/RF              | vel(rad/s)                | cmd(A)")
    while time.time() - t0 < secs:
        d = rv.decode(bytes(bytearray(jl.memory_read8(a, rv.SIZE))))
        cm = struct.unpack("<4f", bytes(bytearray(jl.memory_read8(a_cmd, 16))))
        vl = struct.unpack("<4f", bytes(bytearray(jl.memory_read8(a_vel, 16))))
        print("%6d   %d  | %+7.1f %+7.1f %+7.1f %+7.1f | %+6.2f %+6.2f %+6.2f %+6.2f | %+5.2f %+5.2f %+5.2f %+5.2f"
              % ((d["tick"], d["life"])
                 + tuple(rv.deg(x) for x in d["steer"]) + tuple(vl) + tuple(cm)))
        time.sleep(0.4)
    jl.close()


if __name__ == "__main__":
    main()
