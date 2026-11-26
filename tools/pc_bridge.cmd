@echo off
rem Webcam/Mikrofon-Bruecke zum Simulator. Argumente werden weitergegeben, z.B. pc_bridge.cmd --list
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0pc_bridge.ps1" %*
