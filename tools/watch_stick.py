# -*- coding: utf-8 -*-
"""测"摇杆 -> 车"的映射与符号：同时打印四个摇杆原始量、四轮舵角目标、四轮轮速目标。

判读方法（不需要车动，看逆解输出即可）：
  左杆前推  -> 四轮舵角目标应 ≈ 0°，轮速目标应为正（前进）
  左杆后拉  -> 舵角目标 ≈ ±180°（等价）或轮速为负，取其一，二者必须自洽
  左杆右推  -> vy 对应的横移：四个舵角应同时偏 ±45° 方向
  右杆左/右 -> 分离模式下 w：四个轮速目标应"对角同向、邻角反向"（原地旋转）

输出全部为数字，避免 PowerShell 控制台 GBK 把中文变乱码。
"""
import os
import sys
import struct
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_vars as rv  # noqa: E402


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    import pylink
    nm = rv.find_nm()
    elf = rv.elf_path("Chassis")
    a_dbg, _ = rv.symbol_addr(nm, elf)
    a_relay, _ = rv.symbol_addr(nm, elf, "robot::g_rc_relay")
    jl = pylink.JLink()
    jl.open()
    jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jl.set_speed(4000)
    jl.connect("STM32F407IG", verbose=False)
    v = jl.memory_read32(0xE0042008, 1)[0]
    jl.memory_write32(0xE0042008, [v | (1 << 12)])
    t0 = time.time()
    print("tick    lv  lh  rv  rh | steer_t(deg) LF/LB/RB/RF | wheel_t LF/LB/RB/RF")
    while time.time() - t0 < secs:
        d = rv.decode(bytes(bytearray(jl.memory_read8(a_dbg, rv.SIZE))))
        rb = bytes(bytearray(jl.memory_read8(a_relay, 16)))
        lv, lh, rv_, rh = (struct.unpack("b", rb[k:k + 1])[0] for k in (1, 2, 3, 4))
        if d["life"] == 2 and (abs(lv) + abs(lh) + abs(rv_) + abs(rh)) > 8:
            print("0x%06X %4d %4d %4d %4d | %+7.1f %+7.1f %+7.1f %+7.1f | %+7.2f %+7.2f %+7.2f %+7.2f"
                  % ((d["tick"], lv, lh, rv_, rh)
                     + tuple(rv.deg(d["steer_t"][i]) for i in range(4))
                     + tuple(d["wheel_t"][i] for i in range(4))))
        time.sleep(0.12)
    jl.close()


if __name__ == "__main__":
    main()
