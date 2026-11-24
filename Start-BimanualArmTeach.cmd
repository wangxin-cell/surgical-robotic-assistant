@echo off
setlocal
choice /c LR /n /m "Select arm: [L]eft or [R]ight: "
if errorlevel 3 exit /b 2
if errorlevel 2 (set "ARM=right") else if errorlevel 1 (set "ARM=left") else (exit /b 2)
powershell.exe -NoProfile -STA -ExecutionPolicy Bypass -File "%~dp0BimanualArmTeach.ps1" -Arm %ARM%
set "rc=%errorlevel%"
if not "%rc%"=="0" pause
exit /b %rc%
