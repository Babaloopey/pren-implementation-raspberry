@echo off
rem PREN Simulator: startet den Container, die Kamera/Mikrofon-Bruecke und den Browser.
rem Beenden: Fenster "PREN Bruecke" schliessen und "docker compose down".
cd /d "%~dp0"
echo Starte den Simulator-Container (erster Start baut das Image, ca. 10 min) ...
docker compose up -d --build
if errorlevel 1 (
  echo.
  echo Docker antwortet nicht. Laeuft Docker Desktop?
  pause
  exit /b 1
)
echo Warte auf http://localhost:8080 ...
:wait
curl.exe -s -o NUL http://localhost:8080/api/state || (timeout /t 2 /nobreak >nul & goto wait)
rem Bruecke in eigenem Fenster: Kamera und Mikrofon sind nur an, solange ein Node auf "Echt" steht
start "PREN Bruecke" cmd /k tools\pc_bridge.cmd
start "" http://localhost:8080
echo Simulator laeuft: http://localhost:8080
