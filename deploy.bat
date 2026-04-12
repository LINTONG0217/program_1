@echo off
echo ==============================================
echo Ready to deploy code to the smart car...
echo Please ensure:
echo 1. The car is connected to the PC via USB
echo 2. Thonny is COMPLETELY CLOSED
echo ==============================================
pause

echo [0/4] Launching advanced deployment script (deploy.ps1)...
powershell -ExecutionPolicy Bypass -File "%~dp0deploy.ps1"

if %ERRORLEVEL% NEQ 0 (
    echo ==============================================
    echo [ERROR] Deployment failed! Please check the error messages above.
    echo ==============================================
) else (
    echo ==============================================
    echo Deployment Finished Successfully!
    echo ==============================================
)
pause
