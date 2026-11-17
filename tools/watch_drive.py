# -*- coding: utf-8 -*-
"""实时监视底盘：舵角实测/目标 + 轮速目标/实测 + 状态位。用于"推杆看响应"。"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_vars as rv  # noqa: E402

NAMES = ["LF", "LB", "RB", "RF"]


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    import pylink
    nm = rv.find_nm()
    elf = rv.elf_path("Chassis")
    addr, _ = rv.symbol_addr(nm, elf)
    jl = pylink.JLink()
    jl.open()
    jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jl.set_speed(4000)
    jl.connect("STM32F407IG", verbose=False)
    v = jl.memory_read32(0xE0042008, 1)[0]
    jl.memory_write32(0xE0042008, [v | (1 << 12)])
    t0 = time.time()
    print("tick   " + " ".join("%-21s" % n for n in NAMES) + "  wheel_tgt/vel  state")
    try:
        while time.time() - t0 < secs:
            d = rv.decode(bytes(bytearray(jl.memory_read8(addr, rv.SIZE))))
            seg = " ".join("%+7.2f/%+7.2f/%+6.2f" % (rv.deg(d["steer"][i]), rv.deg(d["steer_t"][i]), d["wheel_v"][i])
                           for i in range(4))
            print("%6d %s  wtgt=%+.2f rcon=%d life=%s estop=%d"
                  % (d["tick"], seg, d["wheel_t"][0], d["rc_online"], rv.LIFE.get(d["life"], "?"), d["estop"]))
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        jl.close()


if __name__ == "__main__":
    main()
