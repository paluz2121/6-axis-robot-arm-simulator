@echo off
title ROBOKIN 6D - Robot Kinematigi Simulatoru
echo ========================================================
echo   ROBOKIN 6D - 6-Eksenli Robot Kinematigi Simulatoru
echo ========================================================
echo Simulatör başlatılıyor...
start http://localhost:8080
python -m http.server 8080
pause
