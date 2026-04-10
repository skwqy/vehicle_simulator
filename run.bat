@echo off
setlocal

set "ROOT=%~dp0"
set "MODE=%~1"

if "%MODE%"=="" set "MODE=all"

if /I "%MODE%"=="agv" goto run_agv
if /I "%MODE%"=="vda" goto run_vda
if /I "%MODE%"=="all" goto run_all

echo Usage: run.bat [agv^|vda^|all]
echo Default: all
exit /b 1

:run_agv
cd /d "%ROOT%agv_plc"
echo Starting AGV simulator...
cargo run --release
exit /b %errorlevel%

:run_vda
cd /d "%ROOT%agv_vda050"
echo Starting VDA5050 simulator...
cargo run --release
exit /b %errorlevel%

:run_all
echo Starting AGV and VDA5050 simulators in separate windows...
start "AGV Simulator" cmd /k "cd /d \"%ROOT%agv_plc\" && cargo run --release"
start "VDA5050 Simulator" cmd /k "cd /d \"%ROOT%agv_vda050\" && cargo run --release"
exit /b 0
