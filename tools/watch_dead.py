# -*- coding: utf-8 -*-
"""验证"状态机不再因为一帧抖动退回 Dead"：读 life / mode / estop / 进 Dead 次数与原因。
纯数字输出，避免控制台乱码。"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_vars as rv  # noqa: E402

REASON = {0: "rc_offline", 1: "estop", 2: "imu_not_ready", 3: "none"}


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    import pylink
    nm = rv.find_nm()
    elf = rv.elf_path("Chassis")

    def sym(n):
        return rv.symbol_addr(nm, elf, n)[0]

    a_dbg = rv.symbol_addr(nm, elf)[0]
    a_cnt, a_rsn = sym("robot::g_dbg_dead_count"), sym("robot::g_dbg_dead_reason")
    jl = pylink.JLink()
    jl.open()
    jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jl.set_speed(4000)
    jl.connect("STM32F407IG", verbose=False)
    v = jl.memory_read32(0xE0042008, 1)[0]
    jl.memory_write32(0xE0042008, [v | (1 << 12)])
    t0 = time.time()
    last = None
    print("tick      life mode estop rcon imu | dead_count dead_reason")
    while time.time() - t0 < secs:
        d = rv.decode(bytes(bytearray(jl.memory_read8(a_dbg, rv.SIZE))))
        cnt = jl.memory_read32(a_cnt, 1)[0]
        rsn = jl.memory_read8(a_rsn, 1)[0]
        cur = (d["life"], d["mode"], d["estop"], d["rc_online"], d["imu_ready"], cnt)
        if cur != last:
            print("0x%06X   %d    %d    %d     %d    %d  | %5d      %s"
                  % (d["tick"], d["life"], d["mode"], d["estop"], d["rc_online"],
                     d["imu_ready"], cnt, REASON.get(rsn, str(rsn))))
            last = cur
        time.sleep(0.03)
    jl.close()


if __name__ == "__main__":
    main()
