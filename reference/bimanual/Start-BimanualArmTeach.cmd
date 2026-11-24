@echo off
setlocal
cd /d "%~dp0"
echo ???????L=???R=??
set /p ARM=?? L ? R: 
if /I "%ARM%"=="R" set ARM=right
if /I "%ARM%"=="L" set ARM=left
if not "%ARM%"=="left" if not "%ARM%"=="right" (echo ?????& pause & exit /b 2)
powershell.exe -NoProfile -STA -ExecutionPolicy Bypass -File "%~dp0BimanualArmTeach.ps1" -Arm %ARM%
if errorlevel 1 pause