@echo off
REM Build script for Battery-Emulator on Windows
REM This avoids the MSys/Git Bash issues

setlocal

REM Add toolchain to PATH
set PATH=%USERPROFILE%\.platformio\packages\toolchain-xtensa-esp-elf\bin;%PATH%
set PATH=%USERPROFILE%\.platformio\packages\xtensa-esp-elf\bin;%PATH%
set PATH=%USERPROFILE%\.platformio\penv\Scripts;%PATH%

REM Clear MSYS environment variables that cause issues
set MSYSTEM=
set MINGW_PREFIX=

REM Run PlatformIO build
echo Building for environment: %1
if "%1"=="" (
    echo Usage: build.bat [environment]
    echo Available: 6ch_relay_s3, lilygo_2CAN_330, BECom_330, etc.
    exit /b 1
)

python -m platformio run -e %1

endlocal
