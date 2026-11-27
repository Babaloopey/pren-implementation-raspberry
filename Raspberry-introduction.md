


# Spickliste PREN-Roboter

**Verbinden**
```bash
ssh pren12@<ip-adresse>
```

Gleiches Netzwerk wie Raspbi dann ssh pren12@raspberrygruppe12

Für neuen Zugang: SSH-Key nach Raspberry rüberkopieren

## Bauen (immer in `~/ros2_ws`, nie im Home-Ordner)
**Nach `git pull` mit neuen Dateien, Configs oder Nachrichten:**
```bash
cd ~/ros2_ws && git pull && colcon build --symlink-install && source install/setup.bash
```

**Komplett neu bauen, wenn etwas komisch ist:**
```bash
cd ~/ros2_ws && rm -rf build install log && colcon build --symlink-install && source install/setup.bash
```

Bei reinen Python-Änderungen reicht ein Neustart.

## Starten
**Alles:**
```bash
ros2 launch robot_core robot.launch.py
```

**Mit Log-Datei:**
```bash
ros2 launch robot_core robot.launch.py 2>&1 | tee ~/pren_logs/launch_$(date +%Y%m%d_%H%M).txt
```

**Teile weglassen:** `gpio:=false` `gestures:=false` `voice:=false` `bones:=false` `camera:=false`

**Weitere Optionen:** `mic:=MV7` `debug:=true`

Beenden: Strg+C. Tracebacks mit `KeyboardInterrupt` sind dabei harmlos.

## Prüfen (zweites Terminal)
```bash
ros2 node list
```
```bash
ros2 topic echo /robot_state
```
```bash
ros2 topic hz /camera/image_raw
```
```bash
ros2 topic hz /audio/raw
```
```bash
vcgencmd measure_temp; vcgencmd get_throttled
```

**Abschalten im Zustand SITZEN:**
```bash
ros2 topic pub --once /system/shutdown std_msgs/msg/Empty
```

## Logs und Performance
```bash
grep -h "perf:" ~/pren_logs/launch_*.txt | tail -15
```
```bash
cat ~/pren_logs/states_*.log
```
```bash
cat ~/pren_logs/commands_*.log
```
```bash
python3 ~/ros2_ws/tools/benchmark.py
```

**Alle Logs einpacken:**
```bash
tar czf ~/pren_test_$(date +%Y%m%d).tgz -C ~ pren_logs
```

**Richtwerte:**

| Wert | Gut |
|---|---|
| Kamera | ~30 Hz |
| Audio | ~12.5 Hz |
| Gesten | ≥ 10 Hz processed |
| Sprache | 0 dropped |
| `get_throttled` | `throttled=0x0` |

## Laptop-Simulator (Windows, Docker Desktop muss laufen)

| | |
|---|---|
| `start_simulator.cmd` | Simulator + Brücke + Browser |
| `tools\pc_bridge.cmd` | Laptop-Webcam und -Mikrofon von Hand |
| `docker compose down` | Beenden |
| http://localhost:8080 | GUI |

## Hardware-in-the-Loop (Branch `feature/simulator-hil`)
**Auf dem Pi:**
```bash
bash ~/ros2_ws/tools/pi_hil.sh
```

**Auf dem Laptop:** `start_simulator_hil.cmd 10.175.211.138`
