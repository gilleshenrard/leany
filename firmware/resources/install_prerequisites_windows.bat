@echo off
setlocal enabledelayedexpansion

:: Check if ARM GCC is installed
arm-none-eabi-gcc --version >nul 2>&1
set ARM_GCC_EXISTS=%ERRORLEVEL%

:: Check if CMake is installed
cmake --version >nul 2>&1
set CMAKE_EXISTS=%ERRORLEVEL%

:: If both are installed, nothing to do
if !ARM_GCC_EXISTS! equ 0 if !CMAKE_EXISTS! equ 0 (
    echo Prerequisites are already present. Nothing to do.
    exit /b 0
)

:: Install the missing tools
echo Installing missing tools...

:: Define MSYS2 installation path
set MSYS2_PATH=C:\msys64

:: Install and update UCRT64
echo Install and update UCRT64
winget install MSYS2.MSYS2
"%MSYS2_PATH%\usr\bin\bash.exe" -lc "pacman -Syu --noconfirm"

:: Install ARM GCC if missing
if !ARM_GCC_EXISTS! neq 0 (
    echo Installing ARM GNU Toolchain...
    "%MSYS2_PATH%\usr\bin\bash.exe" -lc "pacman -S --noconfirm mingw-w64-ucrt-x86_64-arm-none-eabi-gcc"
)

:: Install CMake if missing
if !CMAKE_EXISTS! neq 0 (
    echo Installing CMake...
    "%MSYS2_PATH%\usr\bin\bash.exe" -lc "pacman -S --noconfirm mingw-w64-ucrt-x86_64-cmake"
)

:: Update the PATH
echo Updating the user's PATH...
setx path "%path%;%MSYS2_PATH%\ucrt64\bin\;"

echo Done with success
exit /b 0
