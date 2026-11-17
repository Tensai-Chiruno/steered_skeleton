# -*- coding: utf-8 -*-
"""一次跑完验收：状态机 / 模式 / 舵角目标 / 轮速目标。纯数字输出。"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_vars as rv  # noqa: E402


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    import pylink
    nm = rv.find_nm()
    a, _ = rv.symbol_addr(nm, rv.elf_path("Chassis"))
    jl = pylink.JLink()
    jl.open()
    jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jl.set_speed(4000)
    jl.connect("STM32F407IG", verbose=False)
    v = jl.memory_read32(0xE0042008, 1)[0]
    jl.memory_write32(0xE0042008, [v | (1 << 12)])
    t0 = time.time()
    print("tick   life mode estop | steer_t(deg)              | wheel_t(rad/s)")
    last = None
    while time.time() - t0 < secs:
        d = rv.decode(bytes(bytearray(jl.memory_read8(a, rv.SIZE))))
        key = (d["life"], d["mode"], d["estop"],
               tuple(round(rv.deg(x)) for x in d["steer_t"]),
               tuple(round(x) for x in d["wheel_t"]))
        if key != last:      # 只在目标真正变化时打印，日志才看得清
            print("%6d   %d    %d    %d    | %+7.1f %+7.1f %+7.1f %+7.1f | %+6.1f %+6.1f %+6.1f %+6.1f"
                  % ((d["tick"], d["life"], d["mode"], d["estop"])
                     + tuple(rv.deg(x) for x in d["steer_t"]) + tuple(d["wheel_t"])))
            last = key
        time.sleep(0.05)
    jl.close()


if __name__ == "__main__":
    main()
