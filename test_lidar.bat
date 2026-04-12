@echo off
echo ==============================================
echo Running lidar test...
echo 1) Make sure the car board is connected via USB
echo 2) Please ensure Thonny/serial tools are CLOSED
echo ==============================================
pause

echo Running APP\test_lidar.py ...
python -m mpremote run APP\test_lidar.py

pause
