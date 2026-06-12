@echo off
setlocal EnableExtensions
title Mini4AI Firmware Writer
cd /d "%~dp0"

echo ============================================================
echo Mini4AI Firmware Writer for Windows
echo ============================================================
echo.
echo Before flashing: turn OFF the Mini 4WD side power, then connect USB.
echo.

set "PS1=%~dp0flash_windows.ps1"
set "ROOT=%~dp0..\..\.."
set "SKETCH=%ROOT%\firmware\mini4ai_v358\mini4ai_v358.ino"
set "LOG=%ROOT%\Mini4AI_flash_log.txt"

if not exist "%PS1%" (
  echo [ERROR] flash_windows.ps1 was not found.
  echo Extract the ZIP file first, then run this BAT file again.
  echo.
  pause
  exit /b 1
)

if not exist "%SKETCH%" (
  echo [ERROR] Firmware sketch was not found.
  echo Expected: %SKETCH%
  echo.
  echo You may be running this file from inside the ZIP viewer.
  echo Right-click the ZIP and choose "Extract All", then run it again.
  echo.
  pause
  exit /b 1
)

where powershell.exe >nul 2>nul
if errorlevel 1 (
  echo [ERROR] powershell.exe was not found.
  echo This tool requires Windows PowerShell.
  echo.
  pause
  exit /b 1
)

echo Starting PowerShell writer...
echo Log file: %LOG%
echo.

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%PS1%" -LogPath "%LOG%"
set "RESULT=%ERRORLEVEL%"

echo.
if not "%RESULT%"=="0" (
  echo [Mini4AI] Flashing failed.
  echo Please check Mini4AI_flash_log.txt and docs\recovery.md.
  echo.
  pause
  exit /b %RESULT%
)

echo [Mini4AI] Done.
echo.
pause
