@echo off
echo ==============================================
echo Running chassis test...
echo [WARNING] Please keep the car wheels off the ground!
echo ==============================================
pause
echo Running tests\hardware\manual_chassis.py ...
python -m mpremote run tests\hardware\manual_chassis.py
pause
