# -*- coding: utf-8 -*-
import time
import pylink

jl = pylink.JLink()
jl.open()
jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
jl.set_speed(4000)
jl.connect("STM32F407IG", verbose=False)
v = jl.memory_read32(0xE0042008, 1)[0]
jl.memory_write32(0xE0042008, [v | (1 << 12)])

print("before: halted=%s" % jl.halted())
try:
    jl.reset()
    print("reset() ok")
except Exception as e:
    print("reset() failed: %r" % (e,))
print("after reset: halted=%s" % jl.halted())
try:
    jl.restart(0)
    print("restart(0) ok")
except Exception as e:
    print("restart(0) failed: %r" % (e,))
print("after restart: halted=%s" % jl.halted())

# 看 tick 是否在增长（说明真的在跑）
addr = 0x20000CC0
import struct
LAYOUT = "<II4f4f4f4ffffI8B"
for k in range(5):
    raw = bytes(bytearray(jl.memory_read8(addr, 96)))
    v2 = struct.unpack(LAYOUT, raw)
    print("  t=%d magic=0x%08X tick=%d" % (k, v2[0], v2[1]))
    time.sleep(0.2)
jl.close()
