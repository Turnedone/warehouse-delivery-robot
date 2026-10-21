@echo off
rem One-click demo for Windows: needs Docker Desktop running (no ROS install).
rem Builds the image (first time ~10-15 min, then cached), runs the robot headless on Nav2's
rem loopback simulator, delivers three orders and prints the metrics report.
rem Output goes to demo-output\ next to this file.
cd /d "%~dp0"
if not exist demo-output mkdir demo-output

docker version >nul 2>&1
if errorlevel 1 (
  echo Docker is not running. Install Docker Desktop, start it, then run this again.
  pause
  exit /b 1
)

echo == building the image (cached after the first run)
docker build -f docker/Dockerfile -t warehouse-delivery-robot:demo . || goto :failed

echo == running the demo (about 5 minutes)
docker run --rm -v "%cd%\demo-output:/out" warehouse-delivery-robot:demo bash /ws/src/wdr_bringup/scripts/demo_headless.sh /out || goto :failed

echo.
echo Done. Report: demo-output\report.txt   Bag: demo-output\bag   Log: demo-output\launch.log
pause
exit /b 0

:failed
echo The demo failed; see the messages above and demo-output\launch.log.
pause
exit /b 1
