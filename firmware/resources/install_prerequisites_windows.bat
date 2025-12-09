REM SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
REM
REM SPDX-License-Identifier: MIT

@echo off
setlocal enabledelayedexpansion

:: -------------------------------
:: Configuration
:: -------------------------------
set "MSYS2_PATH=C:\msys64"
set "UCRT64_BIN=C:\msys64\ucrt64\bin"

:: -------------------------------
:: Check winget
:: -------------------------------
where winget >nul 2>&1
if errorlevel 1 (
    echo ERROR: winget is not available on this system.
    exit /b 1
)

:: -------------------------------
:: Install MSYS2 if needed
:: -------------------------------
if not exist "%MSYS2_PATH%\usr\bin\bash.exe" (
    echo Installing MSYS2 via winget...
    winget install MSYS2.MSYS2
)

:: -------------------------------
:: Update MSYS2 and install packages
:: -------------------------------
echo Updating MSYS2 and installing toolchain...
"%MSYS2_PATH%\usr\bin\bash.exe" -lc "pacman -Syu --noconfirm"
"%MSYS2_PATH%\usr\bin\bash.exe" -lc ^
  "pacman -S --noconfirm mingw-w64-ucrt-x86_64-arm-none-eabi-gcc mingw-w64-ucrt-x86_64-cmake"

:: -------------------------------
:: Read user PATH from registry
:: -------------------------------
set "USER_PATH="

for /f "tokens=2,*" %%A in (
  'reg query HKCU\Environment /v Path 2^>nul ^| find "Path"'
) do set "USER_PATH=%%B"

:: -------------------------------
:: Create PATH if missing
:: -------------------------------
if not defined USER_PATH (
    echo Creating user PATH...
    reg add HKCU\Environment /v Path /t REG_EXPAND_SZ /d "%UCRT64_BIN%" /f
    goto notify
)

:: -------------------------------
:: Append if missing
:: -------------------------------
echo %USER_PATH% | find /I "%UCRT64_BIN%" >nul
if errorlevel 1 (
    echo Adding MSYS2 UCRT64 to user PATH...
    reg add HKCU\Environment /v Path /t REG_EXPAND_SZ /d "%USER_PATH%;%UCRT64_BIN%" /f
) else (
    echo MSYS2 UCRT64 already present in PATH.
)

:notify
:: -------------------------------
:: Notify Windows (new shells only)
:: -------------------------------
powershell -NoProfile -Command ^
  "[Environment]::SetEnvironmentVariable('Path', (Get-ItemProperty 'HKCU:\Environment').Path, 'User')"

echo Installation complete.
exit /b 0
