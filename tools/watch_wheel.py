# -*- coding: utf-8 -*-
"""监视轮电机：逆解目标 / PID 原始输出 / 实际下发电流 / 实测速度。"""
import os
import sys
import struct
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_vars as rv  # noqa: E402

NAMES = ["LF", "LB", "RB", "RF"]


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    import pylink
    nm = rv.find_nm()
    elf = rv.elf_path("Chassis")
    addr, _ = rv.symbol_addr(nm, elf)

    def sym(n):
        return rv.symbol_addr(nm, elf, n)[0]

    a_pd, a_cmd = sym("g_dbg_wheel_pd"), sym("g_dbg_wheel_cmd")
    jl = pylink.JLink()
    jl.open()
    jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jl.set_speed(4000)
    jl.connect("STM32F407IG", verbose=False)
    v = jl.memory_read32(0xE0042008, 1)[0]
    jl.memory_write32(0xE0042008, [v | (1 << 12)])
    t0 = time.time()
    print("tick    wtgt    vel   | wheel: pd(A) / cmd(A)")
    try:
        while time.time() - t0 < secs:
            d = rv.decode(bytes(bytearray(jl.memory_read8(addr, rv.SIZE))))
            pd = struct.unpack("<4f", bytes(bytearray(jl.memory_read8(a_pd, 16))))
            cm = struct.unpack("<4f", bytes(bytearray(jl.memory_read8(a_cmd, 16))))
            print("%6d  tgt=%+6.2f | %s" % (
                d["tick"], d["wheel_t"][0],
                "  ".join("%s cmd%+6.2f pd%+8.1f" % (NAMES[i], cm[i], pd[i]) for i in range(4))))
            print("         vel=%s" % ("  ".join("%+6.2f" % d["wheel_v"][i] for i in range(4))))
            time.sleep(0.25)
    finally:
        jl.close()


if __name__ == "__main__":
    main()
