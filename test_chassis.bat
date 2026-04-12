@echo off
echo ==============================================
echo Running chassis test...
echo [WARNING] Please keep the car wheels off the ground!
echo ==============================================
pause
echo Running APP\test_chassis.py ...
python -m mpremote run APP\test_chassis.py
pause
