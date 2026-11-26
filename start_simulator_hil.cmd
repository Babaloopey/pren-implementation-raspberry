@echo off
rem Hardware-in-the-loop (Variante B): Roboter-Nodes auf dem Raspberry Pi, Simulator auf dem Laptop.
rem Aufruf: start_simulator_hil.cmd 192.168.1.50   (IP des Pi; auf dem Pi zuerst tools/pi_hil.sh starten)
cd /d "%~dp0"
set "PI_HOST=%~1"
if "%PI_HOST%"=="" set /p "PI_HOST=IP-Adresse des Raspberry Pi: "
if "%PI_HOST%"=="" exit /b 1
echo Teste Verbindung zum Pi (%PI_HOST%:7447) ...
powershell -NoProfile -Command "if (-not (Test-NetConnection %PI_HOST% -Port 7447 -InformationLevel Quiet -WarningAction SilentlyContinue)) { Write-Host Pi nicht erreichbar: laeuft tools/pi_hil.sh? Gleiches Netz? (Uni-WLAN blockiert oft Geraete untereinander); exit 1 }"
if errorlevel 1 ( pause & exit /b 1 )
docker compose -f docker-compose.yml -f docker-compose.hil.yml up -d --build
if errorlevel 1 ( echo Docker antwortet nicht. Laeuft Docker Desktop? & pause & exit /b 1 )
echo Warte auf http://localhost:8080 ...
:wait
curl.exe -s -o NUL http://localhost:8080/api/state || (timeout /t 2 /nobreak >nul & goto wait)
start "" http://localhost:8080
echo Simulator (HIL mit %PI_HOST%) laeuft: http://localhost:8080
echo Zurueck zum reinen Laptop-Simulator: start_simulator.cmd
