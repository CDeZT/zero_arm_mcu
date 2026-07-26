@echo off
cd /d "%~dp0"
"C:\Tools\OpenOCD-20260121-0.12.0\bin\openocd.exe" -s "C:\Tools\OpenOCD-20260121-0.12.0\share\openocd\scripts" -f "openocd_flash_verify.cfg"
if %errorlevel% neq 0 (
    echo.
    echo Flash or verify failed.
    pause
    exit /b %errorlevel%
)
echo.
echo Flash and verify completed.
pause
