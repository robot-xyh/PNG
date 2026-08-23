@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\install_windows.ps1"
set "RC=%ERRORLEVEL%"
if "%RC%"=="3010" (
  echo.
  echo Windows 10 enabled WSL1 and requested a reboot.
  echo Reboot Windows, then run install_windows.bat again.
)
exit /b %RC%
