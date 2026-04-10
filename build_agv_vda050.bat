@echo off
setlocal

set "ROOT=%~dp0"
cd /d "%ROOT%"

echo [agv_vda050] Building release binary...
cargo build --release -p agv_vda050
if errorlevel 1 (
  echo [agv_vda050] Build failed.
  exit /b 1
)

set "DIST_DIR=%ROOT%dist\agv_vda050"
if not exist "%DIST_DIR%" mkdir "%DIST_DIR%"

copy /Y "%ROOT%target\release\agv_vda050.exe" "%DIST_DIR%\agv_vda050.exe" >nul
if errorlevel 1 (
  echo [agv_vda050] Copy failed: target\release\agv_vda050.exe
  exit /b 1
)

copy /Y "%ROOT%agv_vda050\config.toml" "%DIST_DIR%\config.toml" >nul
if errorlevel 1 (
  echo [agv_vda050] Copy failed: agv_vda050\config.toml
  exit /b 1
)

if exist "%ROOT%agv_vda050\maps" (
  robocopy "%ROOT%agv_vda050\maps" "%DIST_DIR%\maps" /E /R:1 /W:1 /NFL /NDL /NJH /NJS /NC /NS >nul
  if errorlevel 8 (
    echo [agv_vda050] Copy failed: agv_vda050\maps
    exit /b 1
  )
) else (
  echo [agv_vda050] Warning: maps folder not found, skipped.
)

echo [agv_vda050] Done. Output folder: "%DIST_DIR%"
exit /b 0
