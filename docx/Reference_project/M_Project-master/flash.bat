@echo off
"C:\Tools\OpenOCD-20260121-0.12.0\bin\openocd.exe" -c "adapter driver cmsis-dap; cmsis-dap backend usb_bulk; transport select swd; adapter speed 100" -f "C:\Tools\OpenOCD-20260121-0.12.0\share\openocd\scripts\target\stm32g4x.cfg" -c "init; reset halt; program build/Debug/M_Project.elf verify reset exit"
