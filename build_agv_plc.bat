@echo off
setlocal

set "ROOT=%~dp0"
cd /d "%ROOT%"

echo [agv_plc] Building release binary...
cargo build --release -p agv_plc
if errorlevel 1 (
  echo [agv_plc] Build failed.
  exit /b 1
)

set "DIST_DIR=%ROOT%dist\agv_plc"
if not exist "%DIST_DIR%" mkdir "%DIST_DIR%"

copy /Y "%ROOT%target\release\agv_plc.exe" "%DIST_DIR%\agv_plc.exe" >nul
if errorlevel 1 (
  echo [agv_plc] Copy failed: target\release\agv_plc.exe
  exit /b 1
)

copy /Y "%ROOT%agv_plc\config.toml" "%DIST_DIR%\config.toml" >nul
if errorlevel 1 (
  echo [agv_plc] Copy failed: agv_plc\config.toml
  exit /b 1
)

if exist "%ROOT%agv_plc\maps" (
  robocopy "%ROOT%agv_plc\maps" "%DIST_DIR%\maps" /E /R:1 /W:1 /NFL /NDL /NJH /NJS /NC /NS >nul
  if errorlevel 8 (
    echo [agv_plc] Copy failed: agv_plc\maps
    exit /b 1
  )
) else (
  echo [agv_plc] Warning: maps folder not found, skipped.
)

echo [agv_plc] Done. Output folder: "%DIST_DIR%"
exit /b 0
