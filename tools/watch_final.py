# -*- coding: utf-8 -*-
"""最终验收：摇杆四路原始值 + 舵角目标 + 轮速目标 + 模式。纯数字。"""
import os
import sys
import struct
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_vars as rv  # noqa: E402


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
    import pylink
    nm = rv.find_nm()
    elf = rv.elf_path("Chassis")
    a, _ = rv.symbol_addr(nm, elf)
    rl, _ = rv.symbol_addr(nm, elf, "robot::g_rc_relay")
    jl = pylink.JLink()
    jl.open()
    jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jl.set_speed(4000)
    jl.connect("STM32F407IG", verbose=False)
    v = jl.memory_read32(0xE0042008, 1)[0]
    jl.memory_write32(0xE0042008, [v | (1 << 12)])
    t0 = time.time()
    print("tick   mode life | lv  lh  rv  rh | steer_t(deg)            | wheel_t(rad/s)")
    while time.time() - t0 < secs:
        d = rv.decode(bytes(bytearray(jl.memory_read8(a, rv.SIZE))))
        rb = bytes(bytearray(jl.memory_read8(rl, 16)))
        lv, lh, rvt, rh = (struct.unpack("b", rb[k:k + 1])[0] for k in (1, 2, 3, 4))
        print("%6d   %d    %d   | %3d %3d %3d %3d | %+6.0f %+6.0f %+6.0f %+6.0f | %+6.1f %+6.1f %+6.1f %+6.1f"
              % ((d["tick"], d["mode"], d["life"], lv, lh, rvt, rh)
                 + tuple(rv.deg(x) for x in d["steer_t"]) + tuple(d["wheel_t"])))
        time.sleep(0.3)
    jl.close()


if __name__ == "__main__":
    main()
