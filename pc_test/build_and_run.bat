@echo off
rem ===========================================================================
rem  Host-side verification - build and run all tests
rem  Usage: double-click this file, or run  pc_test\build_and_run.bat
rem  Requires: MSVC from Visual Studio / Build Tools
rem  Note: this script is NOT part of any board CMake build.
rem ===========================================================================
setlocal enabledelayedexpansion
cd /d "%~dp0"

echo [1/4] Locating MSVC environment...
set "VCVARS="
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if exist "%VSWHERE%" (
    for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
        if exist "%%i\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=%%i\VC\Auxiliary\Build\vcvars64.bat"
    )
)
if not defined VCVARS (
    set "VCVARS=%ProgramFiles(x86)%\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
)
if not exist "!VCVARS!" (
    echo   [FAIL] vcvars64.bat not found.
    echo          Install the "Desktop development with C++" workload.
    exit /b 1
)
call "!VCVARS!" >nul 2>&1
if errorlevel 1 (
    echo   [FAIL] vcvars64.bat failed.
    exit /b 1
)

set "FAILED=0"

echo [2/4] Compiling test_pid.cpp ...
cl /nologo /EHsc /std:c++17 /utf-8 /O2 /W4 test_pid.cpp /Fe:test_pid.exe
if errorlevel 1 (
    echo   [FAIL] compile failed: test_pid
    set "FAILED=1"
)

echo [3/4] Compiling test_chassis_ik.cpp ...
cl /nologo /EHsc /std:c++17 /utf-8 /O2 /W4 test_chassis_ik.cpp ..\Chassis\RobotModules\component\chassis_ik_solver.cpp /Fe:test_chassis_ik.exe
if errorlevel 1 (
    echo   [FAIL] compile failed: test_chassis_ik
    set "FAILED=1"
)

if "!FAILED!"=="1" (
    echo.
    echo RESULT: BUILD FAILED
    exit /b 1
)

echo [4/4] Running ...

echo.
test_pid.exe
if errorlevel 1 set "FAILED=1"
echo.
test_chassis_ik.exe
if errorlevel 1 set "FAILED=1"

echo.
if "!FAILED!"=="0" (
    echo RESULT: ALL PASSED
    exit /b 0
) else (
    echo RESULT: FAILED
    exit /b 1
)
