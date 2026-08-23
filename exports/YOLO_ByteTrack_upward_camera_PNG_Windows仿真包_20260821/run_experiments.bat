@echo off
setlocal EnableDelayedExpansion
cd /d "%~dp0"
set "PRESET=%~1"
set "TIER=%~2"
if "%PRESET%"=="" set "PRESET=standard"
if "%TIER%"=="" set "TIER=all"
if not exist ".venv\Scripts\python.exe" (
  echo Python environment not found. Run install_windows.bat first.
  exit /b 2
)
set "AIRSIM_RPC_HOST=127.0.0.2"
set "PYTHONUTF8=1"
shift
shift
set "EXTRA_ARGS="
:collect_args
if "%~1"=="" goto run
set EXTRA_ARGS=!EXTRA_ARGS! "%~1"
shift
goto collect_args
:run
".venv\Scripts\python.exe" "tools\windows_experiments.py" --preset "%PRESET%" --tier "%TIER%" !EXTRA_ARGS!
exit /b %ERRORLEVEL%
