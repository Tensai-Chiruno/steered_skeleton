# 给接手 agent 的交接说明（电控辅助任务）

> 生成时间：2026-11-16。当前提交 `b781c77`（分支 main，已推送，与远端一致）。
> 你接手的是**浙大 Hello World 战队电控组 RoboMaster 舵轮步兵考核**的辅助工作。
> 请完整读一遍再动手；第 3 节是硬性红线，违反会造成设备损坏或考核不通过。

---

## 1. 你的角色与最终目标

学生（下称"用户"）正在做**舵向步兵（舵轮）新人考核**，考核形式是**现场功能演示 + GitHub 代码验收**。

你的定位是**辅助工程师**，不是代做作业：
- 用户负责：上电/断电、接线、看真实车、量电压、报读数、按遥控器
- 你负责：写代码、编译、烧录、读调试数据、判断"哪个假设错了"、改参数、记录

**硬性截止：2026-11-18**（群主原话"最后几天人会很多，截止时间 11.18 尽量早点来"）。

---

## 2. 环境与路径

| 项 | 值 |
|---|---|
| 工程根目录 | `C:\Users\13588\Desktop\learning\steered_skeleton` |
| 远端 | `https://github.com/Tensai-Chiruno/steered_skeleton.git` |
| 两块板子工程 | `Chassis\`（底盘板）、`Gimbal\`（云台板），各自独立编译 |
| 共享组件 | `RobotComponents\component\`（两板同时编译） |
| 主机测试 | `pc_test\`（双击 `build_and_run.bat`，共 64 项） |
| 工具链 | `cmake --preset Debug` / `cmake --build --preset Debug`；arm-none-eabi-gcc 13.3.1 |
| 烧录工具 | `C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe`（v2.23.0）；备用 `python -m pyocd` |
| 我的错误日志 | `C:\Users\13588\Desktop\Running\错误日志.md`（**每次动手前先读**） |

---

## 3. 硬性安全红线（考核一票否决，不可违反）

1. **禁止电机失控持续转动**：不允许上电自转、开环常转、"空载测一下"。
2. **必须由遥控器驱动**：只有收到有效 DBUS 且档位正确才允许输出。
3. **失效即归零**：急停档 / 遥控离线 / 未进 Working → 所有电机 `setInput(0)`。
4. 上电调试时**车轮必须离地**；首次整机测试前先单板单电机验证。
5. 小陀螺必须有明确进出档与限速；禁止把无约束自转写死在初始化里。
6. **你绝不碰电源**（上电/断电只在人手上）；也绝不自己决定"给电机发指令试试"。

---

## 4. 当前状态

### 已完成（代码层面）

- 考核十项全部实现：CAN 分发+10 台电机配置、DBUS（两板）、PID 类、IMU 封装、
  `module_state` 共享类型、舵轮逆解+前进方向变换、云台两轴位置环、双板协议、
  状态机+任务调度、IWDG。
- 两板编译通过：Chassis `text=68000 data=1652 bss=8656`；Gimbal `text=59684 data=676 bss=6016`。
- `pc_test` 64 项全过（12 PID + 18 逆解 + 11 双板协议 + 23 遥控错帧闸门）。
- 三轮代码复查的 P0/P1 已修（详见上面 `上板验证清单.md` 的第二、三轮修订节）。
- 新增现场套件：`tools/flash.ps1`（编译+烧录+校验）、`tools/read_vars.py`（调试器直读）、
  `现场参数总表.md`、`今晚基地执行单.md`、`代码分区说明.md`。
- 新增只读调试快照 `robot::g_dbg`（96 字节，两板固定符号名）。

### 未完成（**必须靠现场数据**，不能靠读代码确定）

| 项 | 状态 | 判据 |
|---|---|---|
| 总线归属（M3508/GM6020 各挂 CAN1 还是 CAN2） | 未知 | 看实车接线。文档 2.3 与拓扑图相反，**以实车为准** |
| 8 个电机方向 | 未知 | 手转法：舵轮往左转看 `steer[i]` 变大/变小；驱动轮前滚看 `wheel_vel[i]` 正负 |
| 4 个舵角零位 | 未知 | 摆到机械零位，`steer[i]` 应 ≈ 0（<0.2 rad） |
| IMU 两片 CS 引脚 | 推定 PA4/PB0 | `imuInit()` 是否报错位图 |
| 云台零位（yaw/pitch） | 未知 | 摆到物理零位读数应 ≈ 0；若 pitch ≈ +0.4477 rad 则是 S-4 疑点 |
| δ 符号 | 未定 | 转云台 90° 看 `frame_delta` ≈ +1.5708；抱着车转 30° 它**不该变**；推前进应左移 |
| PID 增益 | 全 0（刻意） | 台架整定 |

### 代码里刻意留的"安全缺省"（不要当成 bug）

- 所有 PID 增益 = 0（输出恒 0，不会乱动）
- `gyro_w = 0`、`follow_kp = 0`（小陀螺/跟随不生效）
- `rotate_velocity_by_delta = true`（现场可确认；若分离模式应沿车头则改 false）
- IMU 标定 3 s 超时后降级为 `isCalibTimeout()`，此时姿态只能短期可信

---

## 5. 必备硬件事实

**双板架构**：底盘板 + 云台板，各一片 STM32F407IGH6，只能通过 CAN 帧通信（`0x210` C2G / `0x211` G2C）。

**10 台电机**：
- 底盘：4 × M3508（驱动，C620 电调，控制帧 `0x200`，反馈 `0x200+id`，减速比 14）
- 底盘：4 × GM6020（转向，电压帧 `0x1FF`，反馈 `0x204+id`）
- 云台：2 × 达妙 DM-J4310（MIT 模式，发送 ID = 电机 ID，反馈 `id+0x10`）

**几何**：R = 0.066 m，半轴距 = 半轮距 = 0.167 m，
`LF(0.167,0.167) LB(-0.167,0.167) RB(-0.167,-0.167) RF(0.167,-0.167)`

**舵零位偏移**（rad）：`LF 2.61346  LB -2.06844  RB 0.00729  RF 1.58518`

**云台零位偏移**：yaw `1.60879`、pitch `2.91772`

**DBUS**：USART3，100 kbps，9 bit 含偶校验，18 字节，空闲+DMA 循环接收

**IWDG**：128 / 50 → 约 200 ms

**SWD 接线**：`3V3 / GND / SWDIO→PA13 / SWCLK→PA14`。**不要接 5V、不要接 RST**。
供电只能一路：ST-Link 3V3 供电，或者板子自己上电（此时拔掉 ST-Link 的 3V3，只留 GND+SWDIO+SWCLK）。

---

## 6. 工具用法

```powershell
cd C:\Users\13588\Desktop\learning\steered_skeleton

# 看工具/调试器/固件时间戳
.\tools\flash.ps1 -List

# 只读探测（不擦不写）：读芯片 ID、Flash 容量、RDP 保护等级
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD mode=UR

# 编译 + 烧录 + 校验
.\tools\flash.ps1 -Board Chassis
.\tools\flash.ps1 -Board Both
.\tools\flash.ps1 -Board Chassis -NoBuild     # 改完参数快速重烧

# 读固件里的实时数值（需要 ST-Link 插着）
python tools\read_vars.py -Board Chassis
python tools\read_vars.py -Board Chassis --watch 0.5

# 主机测试
cd pc_test; .\build_and_run.bat
```

`read_vars.py` 输出：4 舵实测/目标/误差角、4 轮目标线速度/实测角速度、IMU yaw/pitch、
δ、CAN 发送失败数、生命状态/模式/控制源/急停/离线/在线标志。

**底盘板 PC8 状态灯**：常亮=全绿；1=IMU未ready（**未接 IMU 时正常**）；
2=标定超时；3=CAN发送失败在涨（单板台架无 ACK 时正常）；4=主循环心跳停（**不正常**）；
5=对端离线（单板时正常）；6=对端非Working；7=舵零位异常。

---

## 7. 工作纪律（每条都来自实际踩过的坑）

1. **改代码前先读** `C:\Users\13588\Desktop\Running\错误日志.md` 的最近几条。
2. **一次只改一个参数** → 重烧 → 复验 → 记录。同时改两个，出错分不清是谁的锅。
3. **"编译通过"不等于"代码会被执行"**：`--gc-sections` 会删掉没人调用的函数。
   判断"是否真的接上"用 `arm-none-eabi-nm -C --defined-only <elf> | Select-String <符号>`。
4. **判断新文件是否进构建**：不要 `Test-Path` 猜路径。工程名是 `steered_chassis`/`steered_gimbal`
   （不是文件夹名），产物是 `build\steered_chassis.elf`。查 `build.ninja` 或 `Get-ChildItem -Recurse *.elf`。
5. **写文件必须沿用原行尾**：骨架文件多为 CRLF，用 `newline="\n"` 整体重写会让 diff 膨胀上万行。
   提交后**必看** `git show --shortstat`，行数与预期量级不符就是有东西被整体重写了。
6. **全局文本替换必须限定范围**：排除 `third_party/`、`Drivers/`、`build/`。
   曾经把 `0x101→0x210` 替换到 CMSIS-DSP 的 4.9 MB 数据表里。
7. **自动替换必须断言次数**：正则失配返回 0 时必须中止，不能"打印了就算知道了"。
   曾经因此导致 `Robot::update()` 根本没被调用、整条控制链被编译器回收。
8. **.ps1 必须存 UTF-8 带 BOM**（PowerShell 5.1 否则按 GBK 读）；`.bat` 保持纯 ASCII。
9. 每次提交用**文件 + `git commit -F`**（PowerShell 里 `\"` 不是转义）。推送后读回远端哈希确认。
10. 改注释也要**每改一个文件编译一次**；额外用**固件体积不变**交叉验证没误改代码。

---

## 8. 代码分区（要能讲清"哪些是考核功能"）

每个文件头有 `@category` 一行：
- `功能 | 考核第 N 项（…）`：考核要求，删了不通过
- `调试 | 只读，不参与控制，可整块删除`：调试区
- `功能+骨架 | 提供件但被我们补过`

调试代码有围栏，搜 `【调试区` 定位。**共 5 处**：`dbg_snapshot.hpp`、两板 `main_task.cpp` 的调试区、
`chassis.hpp` 的 `wheelState()`、`HW_can.hpp` 的 `CAN_LastTxOk()`。

**唯一影响控制行为的调试代码**：`Chassis/RobotModules/component/robot.cpp` 里搜
`【调试】现场翻号开关`（`rc_wheel() < -0.7f` 时 δ 取反）。δ 符号定下来后必须删除、把符号写死。

---

## 9. 验收标准（用户会被这样判分）

| 等级 | 要求 |
|---|---|
| 通过 | 十项全完成；三模式可切换且行为正确；转云台后推杆，车朝云台指向前进；8 台电机方向正确、舵角无疯狂回摆；云台两轴稳定；急停归零、离线归零；**屏蔽喂狗后板子在 ~200 ms 内复位** |
| 良好 | 通过 + 模块封装清晰、代码规范、**Git 提交历史合理**、模式切换无突跳、各 PID 稳快无超调 |
| 优秀 | 完成任一加分项并讲清取舍（回摆抑制、跟随变增益、小陀螺角速度前馈、双板互保、开机自检指示、PID 整定记录） |
| 一票否决 | 电机失控持续转、不受遥控控制、急停/离线不归零、任务卡死看门狗不复位（谎称已实现）、抄袭 |

---

## 10. 接手后立即要做的事（按顺序）

1. `cd` 到工程、`git log --oneline -3` 确认在 `b781c77` 或之后；`.\tools\flash.ps1 -List` 确认工具在。
2. 等用户报告怎么接的线（哪块板、4 针还是 20 针、谁供电），然后跑只读探测确认芯片 → 报告
   `STM32F407IGHx` / `1024 KB` / `RDP: Level 0`。
3. 烧底盘板 → 读灯（1 闪正常）→ `read_vars.py` 确认 `tick` 在涨、`magic` 正确。
4. 按 `今晚基地执行单.md` 的"手转法"采集 8 个方向 + 4 个零位 + 云台零位 + 总线归属。
5. 按用户报的读数改 `ins_motor.cpp` 的偏移/方向、`ins_chassis_iksolver.cpp` 的几何，
   重烧复验（一次一个）。
6. 整定顺序：轮速环 → 舵角环 → 云台 pitch → 云台 yaw；每轮先把 raw 限幅降到 3000~5000。
7. 排练演示脚本：三模式切换 → 转云台推杆 → 急停 → 拔遥控器 → 屏蔽喂狗看复位。

---

## 11. 明确不要做的事

- 不要动 `.ioc` / 用 CubeMX 重新生成（会覆盖 `Core/Src`）
- 不要改 `chassis_ik_solver.cpp` 的旋转矩阵（矩阵和 `pc_test` 期望必须一起改，否则测试等于没测）
- 不要动 `auto_enable`（现在靠"零力矩≈松掉"，`setMitInput` 有断言守卫）
- 不要打开 `BMI088::gyroSelfTest()`（内部 `while(1)` 无上限）
- 不要在演示版固件里留任何"上电自转/自动测试"逻辑
- 不要为了赶时间跳过"编译 + nm + pc_test + 体积对比"这套验证
