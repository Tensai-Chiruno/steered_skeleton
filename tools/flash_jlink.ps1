<#
.SYNOPSIS
    J-Link 路线的「编译 + 烧录 + 校验」（底盘板 / 云台板）。

.DESCRIPTION
    为什么需要这个脚本：本机插的是 SEGGER J-Link，而
      - STM32CubeProgrammer CLI 只认 ST-Link（报 "No debug probe detected"）
      - pyocd 0.45 的内置目标表里没有 stm32f407vg（要联网装 Keil DFP 包才能用）
    所以走 JLink.exe + objcopy 生成的 hex。三条路线互不依赖，这脚本只管 J-Link。

    编译用 ninja 直调 build/<Preset>（不用 `cmake --build --preset`：多一层包装，
    而且被中断时留下 .ninja_lock 会让下一次构建静默挂在等锁上——本脚本会先清掉它）。

.EXAMPLE
    .\tools\flash_jlink.ps1 -Board Chassis
    .\tools\flash_jlink.ps1 -Board Chassis -NoBuild     # 只重新烧现有 elf
    .\tools\flash_jlink.ps1 -Board Both -Preset Release

.NOTES
    上电/断电必须由人手动完成；本脚本只做「编译 → 烧录 → 校验」。
#>
[CmdletBinding()]
param(
    [ValidateSet("Chassis", "Gimbal", "Both")]
    [string]$Board = "Chassis",

    [ValidateSet("Debug", "RelWithDebInfo", "Release", "MinSizeRel")]
    [string]$Preset = "Debug",

    [switch]$NoBuild,
    [switch]$NoFlash
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$jlink = "C:\Program Files\SEGGER\JLink_V980\JLink.exe"
$objcopy = "C:/tools/gcc-arm-none-eabi/bin/arm-none-eabi-objcopy.exe"
$size = "C:/tools/gcc-arm-none-eabi/bin/arm-none-eabi-size.exe"

if (-not (Test-Path $jlink)) { throw "找不到 JLink.exe：$jlink" }
if (-not (Test-Path $objcopy)) { throw "找不到 objcopy：$objcopy" }

$targets = if ($Board -eq "Both") { @("Chassis", "Gimbal") } else { @($Board) }
$failed = 0

foreach ($b in $targets) {
    $proj = "steered_$($b.ToLower())"
    $buildDir = Join-Path $repo "$b\build\$Preset"
    $elfOut = Join-Path $repo "$b\build\$proj.elf"
    $hexOut = Join-Path $repo "$b\build\$proj.hex"

    Write-Host ""
    Write-Host "================ $b ================" -ForegroundColor Cyan

    if (-not $NoBuild) {
        Write-Host "[1/3] 编译（ninja 直调）..." -ForegroundColor Cyan
        $lock = Join-Path $buildDir ".ninja_lock"
        if (Test-Path $lock) {
            Remove-Item $lock -Force -ErrorAction SilentlyContinue
            Write-Host "      （清掉了上次中断留下的 .ninja_lock）" -ForegroundColor DarkGray
        }
        Push-Location $buildDir
        try {
            $out = & ninja 2>&1
            $rc = $LASTEXITCODE
        } finally {
            Pop-Location
        }
        if ($rc -ne 0) {
            Write-Host "  编译失败：" -ForegroundColor Red
            $out | Select-String "error:|FAILED:" | Select-Object -First 8 |
                ForEach-Object { Write-Host "    $($_.Line.Trim())" -ForegroundColor Red }
            $failed++; continue
        }
        $flashLine = ($out | Select-String "FLASH:").Line
        if ($flashLine) { Write-Host "  OK  $($flashLine.Trim())" -ForegroundColor Green }
        else { Write-Host "  OK  (无输出：已是最新)" -ForegroundColor Green }
    }

    if (-not (Test-Path $elfOut)) { Write-Host "  找不到 $elfOut" -ForegroundColor Red; $failed++; continue }

    Write-Host "[2/3] 生成 hex ..." -ForegroundColor Cyan
    & $objcopy -O ihex $elfOut $hexOut
    if ($LASTEXITCODE -ne 0) { Write-Host "  objcopy 失败" -ForegroundColor Red; $failed++; continue }

    if (Test-Path $size) {
        & $size $elfOut | ForEach-Object { Write-Host "      $_" -ForegroundColor DarkGray }
    }

    if ($NoFlash) { Write-Host "[3/3] -NoFlash：跳过烧录" -ForegroundColor Yellow; continue }

    Write-Host "[3/3] 烧录 + 校验 ..." -ForegroundColor Cyan
    $script = Join-Path $env:TEMP "flash_$proj.jlink"
    $lines = @(
        "si SWD",
        "speed 4000",
        "connect",
        "h",
        "loadfile $hexOut",
        "r",
        "g",
        "q"
    )
    [System.IO.File]::WriteAllLines($script, $lines, (New-Object System.Text.UTF8Encoding($false)))

    $log = & $jlink -device STM32F407IG -if SWD -speed 4000 -autoconnect 1 -ExitOnError 1 -CommanderScript $script 2>&1
    $log | Select-String -Pattern "Downloading|Flash download|O\.K\.|Error|Can not attach" |
        ForEach-Object { Write-Host "      $($_.Line.Trim())" -ForegroundColor DarkGray }
    if (-not ($log | Select-String -Pattern "^O\.K\.")) {
        Write-Host "  烧录失败（没看到 O.K.）" -ForegroundColor Red
        Write-Host "  排查：① SWD 四线 VTref/GND/PA13/PA14 ② 板子自己上电 ③ 别接 RST/5V" -ForegroundColor Yellow
        $failed++; continue
    }
    Write-Host "  烧录成功（含校验）" -ForegroundColor Green
}

Write-Host ""
if ($failed -eq 0) {
    Write-Host "全部成功。读数： python tools\read_vars_jlink.py -Board Chassis" -ForegroundColor Green
    exit 0
} else {
    Write-Host "有 $failed 块板失败。" -ForegroundColor Red
    exit 1
}
