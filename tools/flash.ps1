<#
.SYNOPSIS
    一键编译 + 烧录 + 校验（底盘板 / 云台板）。

.DESCRIPTION
    优先使用 STM32CubeProgrammer 的 CLI（路线 A），没装则回退到 pyocd（路线 B）。
    两个都没有时给出明确的安装指引，不会假装成功。

.EXAMPLE
    .\flash.ps1 -Board Chassis
    .\flash.ps1 -Board Both -NoBuild
    .\flash.ps1 -List

.NOTES
    上电/断电必须由人手动完成；本脚本只做「编译 → 烧录 → 校验」。
#>
[CmdletBinding()]
param(
    [ValidateSet("Chassis", "Gimbal", "Both")]
    [string]$Board = "Both",

    # 跳过编译，直接烧现有 elf（改完参数想快速重烧时用）
    [switch]$NoBuild,

    # SWD 连接方式：under-reset 能在固件复位循环/低功耗时也连上，建议默认
    [ValidateSet("normal", "under-reset")]
    [string]$Connect = "under-reset",

    # 烧完做一次校验（读回比对）
    [switch]$NoVerify,

    # 只列出当前接的调试器，不做任何操作
    [switch]$List
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$nm = "C:/tools/gcc-arm-none-eabi/bin/arm-none-eabi-nm.exe"

# ---------------------------------------------------------------- 工具探测
function Find-StCli {
    $cands = @(
        "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        "C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        "$env:LOCALAPPDATA\Programs\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
    )
    foreach ($c in $cands) { if (Test-Path $c) { return $c } }
    $w = Get-Command STM32_Programmer_CLI.exe -ErrorAction SilentlyContinue
    if ($w) { return $w.Source }
    return $null
}

function Test-PyOcdModule {
    # pyocd 通常是 python 模块，不在 PATH 上；用 python -m pyocd 调用
    try {
        & python -c "import pyocd" 2>$null | Out-Null
        return ($LASTEXITCODE -eq 0)
    } catch { return $false }
}

$stcli = Find-StCli
$pyocd = Test-PyOcdModule

# ---------------------------------------------------------------- -List
if ($List) {
    Write-Host "=== 烧录工具探测 ===" -ForegroundColor Cyan
    if ($stcli) { Write-Host "  [A] STM32CubeProgrammer CLI : $stcli" -ForegroundColor Green }
    else        { Write-Host "  [A] STM32CubeProgrammer CLI : 未安装" -ForegroundColor Yellow }

    if ($pyocd) { Write-Host "  [B] pyocd                   : python -m pyocd" -ForegroundColor Green }
    else        { Write-Host "  [B] pyocd                   : 未安装" -ForegroundColor Yellow }

    Write-Host ""
    Write-Host "=== 当前连接的调试器 ===" -ForegroundColor Cyan
    if ($pyocd) { & python -m pyocd list } else { Write-Host "  （需要 pyocd 才能列出）" }

    Write-Host ""
    Write-Host "=== 已编译的固件 ===" -ForegroundColor Cyan
    foreach ($b in @("Chassis", "Gimbal")) {
        $elf = Join-Path $repo "$b\build\steered_$($b.ToLower()).elf"
        if (Test-Path $elf) {
            $t = (Get-Item $elf).LastWriteTime
            Write-Host ("  {0,-8} {1}  ({2:yyyy-MM-dd HH:mm})" -f $b, $elf, $t)
        } else {
            Write-Host ("  {0,-8} 还没编译过" -f $b) -ForegroundColor Yellow
        }
    }
    return
}

if (-not $stcli -and -not $pyocd) {
    Write-Host "找不到任何烧录工具。" -ForegroundColor Red
    Write-Host ""
    Write-Host "路线 A（推荐，官方）：装 STM32CubeProgrammer" -ForegroundColor Yellow
    Write-Host "  1. 打开 https://www.st.com/en/development-tools/stm32cubeprog.html"
    Write-Host "  2. 下载 Windows 版（可能需要注册 ST 账号），双击安装，一路默认"
    Write-Host "  3. 装完重新打开 PowerShell，运行  .\flash.ps1 -List  确认能找到"
    Write-Host ""
    Write-Host "路线 B（已装，但可能需要换 ST-Link 驱动）：pyocd" -ForegroundColor Yellow
    Write-Host "  pip install pyocd  （本机已有）"
    exit 1
}

# ---------------------------------------------------------------- 烧录
function Get-Elf([string]$b) {
    $p = Join-Path $repo "$b\build\steered_$($b.ToLower()).elf"
    if (-not (Test-Path $p)) { throw "找不到固件：$p（先编译一次）" }
    return $p
}

function Show-Size([string]$elf) {
    if (Test-Path $nm) {
        $out = & $nm -S -C $elf 2>$null
        $dbg = $out | Select-String "robot::g_dbg"
        if ($dbg) { Write-Host "      g_dbg: $($dbg.Line.Trim())" -ForegroundColor DarkGray }
    }
}

$targets = if ($Board -eq "Both") { @("Chassis", "Gimbal") } else { @($Board) }
$failed = 0

foreach ($b in $targets) {
    Write-Host ""
    Write-Host "================ $b ================" -ForegroundColor Cyan

    if (-not $NoBuild) {
        Write-Host "[1/3] 编译 ..." -ForegroundColor Cyan
        Push-Location (Join-Path $repo $b)
        try {
            $out = & cmake --build --preset Debug 2>&1
            if ($LASTEXITCODE -ne 0) {
                Write-Host "  编译失败：" -ForegroundColor Red
                $out | Select-String "error:" | Select-Object -First 8 | ForEach-Object { Write-Host "    $($_.Line.Trim())" -ForegroundColor Red }
                $failed++; continue
            }
            $flashLine = ($out | Select-String "FLASH:").Line
            Write-Host "  OK  $($flashLine.Trim())" -ForegroundColor Green
        } finally { Pop-Location }
    }

    $elf = Get-Elf $b
    Show-Size $elf

    Write-Host "[2/3] 烧录 ($elf) ..." -ForegroundColor Cyan

    if ($stcli) {
        # 路线 A：官方 CLI
        $mode = if ($Connect -eq "under-reset") { "UR" } else { "NORMAL" }
        $args = @("-c", "port=SWD", "mode=$mode", "-w", $elf, "-rst")
        if (-not $NoVerify) { $args += "-v" }
        & $stcli @args
        if ($LASTEXITCODE -ne 0) {
            Write-Host "  烧录失败（退出码 $LASTEXITCODE）" -ForegroundColor Red
            Write-Host "  排查：① 只接 SWD（PA13/PA14+GND+3V3）② 试 -Connect under-reset ③ 拉高 BOOT0 全片擦除" -ForegroundColor Yellow
            $failed++; continue
        }
        Write-Host "  烧录成功" -ForegroundColor Green
    }
    else {
        # 路线 B：pyocd
        $tgt = "stm32f407vg"
        & python -m pyocd flash -t $tgt --format elf $elf
        if ($LASTEXITCODE -ne 0) {
            Write-Host "  pyocd 烧录失败" -ForegroundColor Red
            Write-Host "  常见原因：ST-Link 驱动不是 WinUSB（需 Zadig 替换）→ 建议改用路线 A" -ForegroundColor Yellow
            $failed++; continue
        }
        Write-Host "  烧录成功" -ForegroundColor Green
    }

    Write-Host "[3/3] 完成。上电后看 PC8 状态灯（底盘板）：常亮=全绿，1~7 闪=对应告警" -ForegroundColor DarkGray
}

Write-Host ""
if ($failed -eq 0) {
    Write-Host "全部成功。" -ForegroundColor Green
    Write-Host "读数： python tools\read_vars.py -Board Chassis" -ForegroundColor Cyan
    exit 0
} else {
    Write-Host "有 $failed 块板失败。" -ForegroundColor Red
    exit 1
}
