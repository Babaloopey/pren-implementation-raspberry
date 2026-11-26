# Starts the webcam/microphone bridge to the simulator container.
# First run creates tools\.venv-bridge and installs opencv-python, sounddevice and numpy there
# (your system Python stays untouched). Extra arguments are passed on, e.g.:
#   tools\pc_bridge.cmd --camera 1 --preview     (the .cmd also works when scripts are blocked)
#   tools\pc_bridge.cmd --list
$ErrorActionPreference = 'Stop'
$venv = Join-Path $PSScriptRoot '.venv-bridge'
$py = Join-Path $venv 'Scripts\python.exe'
if (-not (Test-Path $py)) {
    Write-Host 'Erstelle Python-Umgebung fuer die Bruecke (einmalig) ...'
    python -m venv $venv
    & $py -m pip install --quiet --upgrade pip
    & $py -m pip install --quiet opencv-python sounddevice numpy
}
& $py (Join-Path $PSScriptRoot 'pc_bridge_client.py') @args
