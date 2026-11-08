@echo off
rem ===========================================================================
rem  Host-side PID verification - build and run
rem  Usage: double-click this file, or run  pc_test\build_and_run.bat
rem  Requires: MSVC from Visual Studio / Build Tools
rem  Note: this script is NOT part of any board CMake build.
rem ===========================================================================
setlocal enabledelayedexpansion
cd /d "%~dp0"

echo [1/3] Locating MSVC environment...
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
echo       !VCVARS!
call "!VCVARS!" >nul 2>&1
if errorlevel 1 (
    echo   [FAIL] vcvars64.bat failed.
    exit /b 1
)

echo [2/3] Compiling test_pid.cpp ...
cl /nologo /EHsc /std:c++17 /utf-8 /O2 /W4 test_pid.cpp /Fe:test_pid.exe /Fo:test_pid.obj
if errorlevel 1 (
    echo   [FAIL] compile failed.
    exit /b 1
)

echo [3/3] Running ...
echo.
test_pid.exe
set "RC=%ERRORLEVEL%"
echo.
if "%RC%"=="0" (
    echo RESULT: ALL PASSED
) else (
    echo RESULT: FAILED  exit=%RC%
)
exit /b %RC%
