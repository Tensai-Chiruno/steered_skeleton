# -*- coding: utf-8 -*-
"""用 SEGGER J-Link（pylink）读固件里的调试快照 robot::g_dbg。

为什么单独写一个：
  pyocd 0.45 的内置目标表里没有 stm32f407vg（要联网装 Keil DFP 包），
  而本机插的是 J-Link —— 所以走 pylink 这条路，不需要装包、不需要跑 JLink.exe 子进程。

与 read_vars.py 的区别：read_vars.py 用 pyocd；本脚本复用它的 LAYOUT/decode/show，
只把"怎么连、怎么读"换成 J-Link，所以两边的解析永远一致。

额外做一件事：读之前把 DBGMCU->APB1_FZ 的 DBG_IWDG_STOP 置位。
  固件里 IWDG 是 ~200 ms，一旦 J-Link 把 CPU 停住超过 200 ms 芯片就会复位，
  读数会莫名其妙清零。置这一位后"停核时看门狗也停"，读数稳定。
  这个位只影响调试器停核时的行为，**不改变固件正常运行时的看门狗**。

用法：
    python tools\\read_vars_jlink.py -Board Chassis
    python tools\\read_vars_jlink.py -Board Chassis --watch 0.3
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import read_vars as rv  # noqa: E402  复用 LAYOUT / decode / show / 符号查找

DBGMCU_APB1_FZ = 0xE0042008
DBG_IWDG_STOP = 1 << 12


def main():
    ap = argparse.ArgumentParser(description="用 J-Link 读 robot::g_dbg 快照")
    ap.add_argument("-Board", "--board", default="Chassis", choices=["Chassis", "Gimbal"])
    ap.add_argument("--watch", type=float, default=0.0, help="秒；>0 则循环刷新")
    ap.add_argument("--speed", type=int, default=4000, help="SWD 时钟 kHz")
    ap.add_argument("--device", default="STM32F407IG", help="J-Link 器件名")
    ap.add_argument("--keep-wdg", action="store_true", help="不要动 DBG_IWDG_STOP")
    args = ap.parse_args()

    try:
        import pylink
    except Exception as e:  # pragma: no cover
        print("[FAIL] 导入 pylink 失败：%r" % (e,))
        print("       pip install pylink-square")
        return 2

    nm = rv.find_nm()
    if not nm:
        print("[FAIL] 找不到 arm-none-eabi-nm")
        return 2
    elf = rv.elf_path(args.board)
    if not os.path.exists(elf):
        print("[FAIL] 找不到固件：%s（先编译一次）" % elf)
        return 2
    addr, size = rv.symbol_addr(nm, elf)
    if addr is None:
        print("[FAIL] %s 里没有 robot::g_dbg 符号" % os.path.basename(elf))
        return 2
    if size != rv.SIZE:
        print("[WARN] ELF 里 g_dbg 大小 %d，脚本按 %d 解析" % (size, rv.SIZE))

    jl = pylink.JLink()
    try:
        jl.open()
    except Exception as e:
        print("[FAIL] 打不开 J-Link：%r" % (e,))
        return 2

    rc = 0
    try:
        try:
            jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
        except Exception:
            pass
        jl.set_speed(args.speed)
        try:
            jl.connect(args.device, verbose=False)
        except TypeError:
            jl.connect(args.device)

        if not args.keep_wdg:
            # 不 halt：DBGMCU 是调试寄存器，核在跑也能通过 AHB-AP 读写。
            # （pylink 1.7 的 Python API 没有 resume()，halt 了就没法恢复运行，所以干脆不停核。）
            try:
                v = jl.memory_read32(DBGMCU_APB1_FZ, 1)[0]
                jl.memory_write32(DBGMCU_APB1_FZ, [v | DBG_IWDG_STOP])
                v2 = jl.memory_read32(DBGMCU_APB1_FZ, 1)[0]
                if not (v2 & DBG_IWDG_STOP):
                    print("[WARN] DBG_IWDG_STOP 写入后读回不一致：0x%08X" % v2)
            except Exception as e:
                print("[WARN] 设置 DBG_IWDG_STOP 失败（%r），读数可能被复位打断" % (e,))

        while True:
            buf = bytes(bytearray(jl.memory_read8(addr, rv.SIZE)))
            rv.show(rv.decode(buf), args.board, addr)
            if args.watch <= 0:
                break
            time.sleep(args.watch)
            print()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("[FAIL] 读内存失败：%r" % (e,))
        print("       常见原因：目标没上电 / SWD 没接对 / 调试器被别的程序占用")
        rc = 2
    finally:
        try:
            jl.close()
        except Exception:
            pass
    return rc


if __name__ == "__main__":
    sys.exit(main())
