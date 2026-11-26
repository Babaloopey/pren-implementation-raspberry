# PREN Hunde-Roboter – ROS 2 (Raspberry Pi)

ROS 2 Jazzy auf Ubuntu 24.04, Raspberry Pi.

| Paket | Inhalt |
|---|---|
| `robot_vision` | `camera_driver`, `bone_detector`, `gesture_detector`, `frame_saver` |
| `robot_audio` | `microphone_driver`, `voice_command_detector` |
| `robot_gpio` | `search_button_driver`, `bone_button_driver`, `led_driver`, `speaker_driver` |
| `robot_core` | `state_coordinator` (Zustandsautomat), `command_coordinator`, `topic_debugger` |
| `robot_communication` | (später `serial_uart`) |
| `robot_interfaces` | eigene Nachrichtentypen |

## Datenfluss

```
camera_driver ──/camera/image_raw──┬─> gesture_detector ──/perception/gestures───────┐
 (Pi Camera Module 3)              └─> bone_detector ─────/perception/bone_position──┤
microphone_driver ──/audio/raw──────> voice_command_detector ──/perception/voice_cmds─┤
 (Shure MV7+)                                                                         ├─> state_coordinator
search_button_driver ──/perception/search_contact─────────────────────────────────────┤        │ /robot_command
bone_button_driver ────/perception/bone_contact───────────────────────────────────────┘        ▼   ▲ /robot_command_status, /odom
                                                                                       command_coordinator
                                                     led_driver <──/actuation/led_cmds──────┤
                                                 speaker_driver <──/actuation/speaker_cmds──┤
                                       (später serial_uart, jetzt Log-Datei) <── Motorbefehle ┘
```

Es gibt **keinen `image_processor` und keinen `voice_processor`**: Die Treiber liefern bereits das
Endformat (Kamera: 1280×720 BGR; Mikrofon: 16 kHz mono in 80-ms-Blöcken). Die modellspezifische
Vorverarbeitung (YOLO: Letterbox 640×640, MediaPipe: RGB 640 px) passiert in den Detektoren, weil
jedes Modell eine andere braucht; ein Zwischen-Node würde jedes Bild nur ein zweites Mal kopieren.

## Topics und Nachrichten

| Topic | Typ | Von → An | Inhalt |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | `camera_driver` → Detektoren | 1280×720 `bgr8`, ~30 fps |
| `/audio/raw` | `robot_interfaces/AudioChunk` | `microphone_driver` → `voice_command_detector` | 16 kHz mono int16, 1280 Samples (80 ms) pro Nachricht |
| `/perception/gestures` | `robot_interfaces/Gesture` | `gesture_detector` | `command` (`SITZ`, `PLATZ`, `AUF`, `TRICK`), `confidence` |
| `/perception/voice_cmds` | `robot_interfaces/VoiceCommand` | `voice_command_detector` | `command` (`SUCH`, `HIER`, `BRAV`, `SITZ`), `confidence` |
| `/perception/bone_position` | `robot_interfaces/BoneDetectionArray` | `bone_detector` | Knochen: `bearing`, `score`, `bbox`, optional `position` |
| `/perception/pylon_position` | `robot_interfaces/BoneDetectionArray` | `bone_detector` | Pylonen, gleiches Format |
| `/perception/search_contact` | `std_msgs/Bool` | `search_button_driver` | `true` = gedrückt, `false` = losgelassen |
| `/perception/bone_contact` | `std_msgs/Bool` | `bone_button_driver` | `true` = gedrückt, `false` = losgelassen |
| `/actuation/led_cmds` | `std_msgs/String` | → `led_driver` | `on`, `off`, `blink` |
| `/actuation/speaker_cmds` | `std_msgs/String` | → `speaker_driver` | `on`, `off`, `beep`, `double_beep` |
| `/robot_command` | `robot_interfaces/RobotCommand` | `state_coordinator` → `command_coordinator` | `MOVE`, `TURN`, `VELOCITY`, `STOP`, `POSTURE`, `HEAD`, `BEHAVIOR`, `SIGNAL` mit `id` |
| `/robot_command_status` | `robot_interfaces/CommandStatus` | `command_coordinator` → `state_coordinator` | Befehl `id` erledigt (`SUCCEEDED`) oder ersetzt (`ABORTED`) |
| `/odom` | `nav_msgs/Odometry` | `command_coordinator` | Pose relativ zum Start; **vorerst aus den Befehlen berechnet, nicht gemessen** |
| `/robot_state` | `std_msgs/String` (latched) | `state_coordinator` | aktueller Zustand, z. B. `SUCHE_ANFAHREN` |
| `/system/shutdown` | `std_msgs/Empty` | → `state_coordinator` | «System ausschalten» (nur im Zustand `SITZEN`) |

Nachrichten ansehen: `ros2 interface show robot_interfaces/msg/BoneDetection`. In Python:

```python
from robot_interfaces.msg import Gesture, VoiceCommand, BoneDetectionArray

def on_gesture(msg):
    if msg.command == Gesture.TRICK: ...
```

* **Gesten** werden **einmal** gesendet, sobald sie in 6 der letzten 10 Frames erkannt wurden;
  erneut erst, nachdem die Geste weg war. `confidence` = Anteil zustimmender Frames.
* **Sprache**: ein Befehl pro erkanntem Wort, `confidence` = openWakeWord-Score.
  Für «Kunststück» gibt es noch kein Sprachmodell (als Geste: Daumen hoch).
* **Knochen**: bei **jedem** ausgewerteten Bild (leere Liste = nichts gesehen).
  `bearing` in **Radiant**, positiv = links, negativ = rechts (ROS-Konvention REP 103).
  Mit der geradeaus blickenden Kamera (Neigung 0, Linse 90° zum Boden) hängt der Winkel nur von der
  Pixelspalte und dem Sichtfeld ab, nicht von der Kamerahöhe. Pi Camera Module 3: 75° diagonal ⇒
  ca. 67.5° horizontal bei 16:9, also ±0.59 rad (±33.8°) am Bildrand.
  `position` ist nur gültig, wenn `has_position` true ist; das ist erst der Fall, wenn
  `camera_height` in `robot_vision/config/vision.yaml` gesetzt ist (Bodenprojektion in `camera_link`).
  Mit Neigung 0 geht das nur für Knochen unterhalb der Bildmitte und wird mit der Distanz schnell ungenau.

## Gesten

| Befehl | Geste | Woher die Entscheidung kommt |
|---|---|---|
| SITZ | offene Hand vor der Brust | MediaPipe-Klasse `Open_Palm` **und** Hand in der Oberkörper-Box (Pose) |
| TRICK (Kunststück) | Daumen hoch | MediaPipe-Klasse `Thumb_Up` |
| AUF | Zeigefinger (oder «L») nach oben | MediaPipe-Klasse `Pointing_Up`, sonst Geometrie der Handpunkte |
| PLATZ | Zeigefinger zum Boden | Geometrie der Handpunkte (+ Arm aus der Pose). MediaPipe hat **keine** Klasse «Zeigefinger nach unten» |

Reihenfolge pro Hand: SITZ > TRICK > AUF > PLATZ.

## Verhalten (`robot_core`)

`state_coordinator` setzt die drei Zustandsdiagramme um (Hunde-Roboter, Grundpositionen,
Such-Vorgang). Der Automat selbst liegt ohne ROS in `robot_core/dog_fsm.py`; jeder Zustand und jeder
Übergang ist dort mit dem Diagramm-Namen kommentiert.

| Auslöser | von | nach |
|---|---|---|
| Einschalten, sobald `command_coordinator` läuft | – | Startzustand (Ton + LED, Stehposition) → **Stehen** |
| Geste/Sprache SITZ | Stehen, Liegen | Sitzposition einnehmen → **Sitzen** |
| Geste PLATZ | Stehen, Sitzen | Liegeposition einnehmen → **Liegen** |
| Geste AUF | Sitzen, Liegen | Stehposition einnehmen → **Stehen** |
| Sprache HIER | jede Grundposition | 30 cm vor, 180°, 30 cm vor, 180° → **Stehen** |
| Sprache SUCH / Such-Taster | jede Grundposition | Such-Vorgang → **Sitzen** (oder nach 2 min) |
| Sprache BRAV | Stehen | Brav machen → **Stehen** (oder nach 1 min) |
| Geste Daumen hoch (TRICK) | Liegen | Kunststück machen → **Stehen** (oder nach 1 min) |
| `/system/shutdown` | Sitzen | Ausschalten → Ende |

Befehle in anderen Zuständen werden ignoriert (steht in der Log-Datei). Jedes «entry / Ton + LED»
ist ein `SIGNAL ACK` (Buzzer `beep`, LED an), «Bellen» ein `SIGNAL BARK` (`double_beep`, LED blinkt).

**Such-Vorgang:** 180° drehen → 2 s stillstehen und scannen (Knochen in ≥ 3 Bildern = gefunden,
sonst 1 m vor und neu scannen) → auf den Knochen drehen, bis `bearing` < 4° → darauf zufahren und
nachlenken, bis der Knochen «nahe» ist → Kopf senken → langsam vor, bis der Knochen-Taster auslöst →
bellen → zum Start drehen und zurückfahren (aus `/odom`) → Sitzposition, beim Verlassen bellen → Sitzen.

«Knochen ist nahe» ohne bekannte Distanz: Die Unterkante der Box liegt in den unteren 15 % des
Bildes (`near_bottom_frac`). Bei gesetzter Kamerahöhe gilt stattdessen die Distanz `near_distance`.

**Was die Diagramme offen lassen** (umgesetzt, im Code mit «added» markiert, bitte prüfen):
* HIER und SUCH aus Sitzen/Liegen nehmen zuerst die Stehposition ein (Fahren im Sitzen?).
* Knochen beim Ausrichten/Anfahren 2 s nicht mehr gesehen → zurück zu Scannen.
* Nach Kopf senken 40 cm ohne Kontakt → Kopf hoch, zurück zu Scannen.
* Beim Bellen nach dem Kontakt geht der Kopf wieder hoch.
* Keine 1-m-Schritte mehr über 5 m vom Start hinaus (Ende des Segments), dann wird bis zum
  2-min-Timeout weiter gescannt.
* Beim 2-min-Timeout setzt sich der Hund **dort, wo er ist** (wie im Diagramm: direkt → Sitzen),
  er fährt nicht zum Start zurück.
* «Befehl für Kunststück» ist die Geste Daumen hoch; ein Sprachmodell für «Kunststück» fehlt noch.
* «System ausschalten» kommt über das Topic `/system/shutdown` (z. B. von einem Taster oder per
  `ros2 topic pub --once /system/shutdown std_msgs/msg/Empty`). Mit `poweroff_command` fährt der Pi herunter.

**`command_coordinator` ohne Mikrocontroller:** schreibt alle Motorbefehle in die Log-Datei, steuert
Buzzer und LED, und meldet Befehle nach der berechneten Zeit als erledigt (MOVE: Distanz / 0.2 m/s,
TURN: Winkel / 1 rad/s, Haltung 2 s, Kopf 1 s, Brav 5 s, Kunststück 8 s; `config/core.yaml`).
Wenn `serial_uart` steht, kommen Erledigt-Meldung und `/odom` vom Mikrocontroller.

**Log-Dateien** in `~/pren_logs/`:

```
states_20261127_101500.log      10:15:03.120  state STEHEN -> SUCHE_DREHEN
                                10:15:09.402  bone found (4 hits)
                                10:15:12.880  gesture SITZ ignored in SUCHE_ANFAHREN
commands_20261127_101500.log    10:15:03.121  RECV  #14 TURN 3.142
                                10:15:03.121  MOTOR v=+0.000 w=+1.000
                                10:15:06.272  MOTOR v=+0.000 w=+0.000
                                10:15:06.272  DONE  #14 TURN +3.142 (3.15 s)
```

## GPIO-Belegung (BCM)

| Node | Standard-Pin | Parameter |
|---|---|---|
| `bone_button_driver` | 17 | `gpio_pin` |
| `search_button_driver` | 23 | `gpio_pin` |
| `speaker_driver` (Buzzer) | 22 | `gpio_pin` |
| `led_driver` | 27 | `gpio_pin` |

Taster verbinden den Pin mit GND (interner Pull-up). **Pins an die tatsächliche Verdrahtung anpassen**,
z. B. `ros2 run robot_gpio search_button_driver --ros-args -p gpio_pin:=24`.

## Was wird wo verwendet

| Was | Wofür | Wo (Code) | Datei / Installation |
|---|---|---|---|
| **YOLO26n**, eigenes Training (Knochen, Pylon), ONNX 640×640 | Knochen- und Pylonen-Boxen | `robot_vision/bone_detector.py` | `robot_vision/models/bone_recognition_640.onnx` |
| **onnxruntime** (CPU) | führt YOLO ohne PyTorch/Ultralytics aus | `robot_vision/yolo_onnx.py` | pip `onnxruntime` |
| Kameramodell (75° diagonal) | Pixel → Winkel; optional Bodenprojektion | `GroundProjector` in `bone_detector.py` | `robot_vision/config/vision.yaml` |
| **MediaPipe Gesture Recognizer** | Handpunkte + `Open_Palm` / `Pointing_Up` / `Thumb_Up` | `robot_vision/gesture_detector.py` | `robot_vision/models/gesture_recognizer.task`, pip `mediapipe` |
| **MediaPipe Pose Landmarker (lite)** | Oberkörper-Box, Armrichtung | `gesture_detector.py`, `mediapipe_persons()` in `gesture_logic.py` | `robot_vision/models/pose_landmarker_lite.task` |
| Gesten-Logik aus `poc.py` | Entscheidung, Hysterese, Mehrheitsentscheid | `robot_vision/gesture_logic.py` | – |
| GStreamer + libcamera | Kamerabild | `robot_vision/camera_driver.py` | apt (OpenCV mit GStreamer) |
| **sounddevice** + SciPy (FIR-Tiefpass) | Mikrofon lesen (48 kHz), auf 16 kHz mono umrechnen | `robot_audio/microphone_driver.py` | pip `sounddevice`, apt `libportaudio2`, `python3-scipy` |
| **openWakeWord** (ONNX) + Silero-VAD | ein Modell pro Sprachbefehl, alle in einem `Model` | `robot_audio/voice_command_detector.py` | `robot_audio/models/{such,hier,brav,sitz}_de.onnx`, pip `openwakeword` |
| **gpiozero** + lgpio | Taster, LED, Buzzer | `robot_gpio/*.py` | apt `python3-gpiozero`, `python3-lgpio` |
| Eigene Nachrichten | Typen der Erkennungs-Topics und Audio | `robot_interfaces/msg/*.msg` | `rosidl_default_generators` |

## Installation auf dem Pi

```bash
sudo apt install ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-rosidl-default-generators \
                 python3-opencv python3-numpy python3-scipy libportaudio2 \
                 python3-gpiozero python3-lgpio
# pip-Pakete (Ubuntu 24.04 schützt das System-Python -> --break-system-packages
# oder ein venv mit --system-site-packages, aus dem auch colcon/ros2 gestartet wird)
pip install --break-system-packages onnxruntime mediapipe sounddevice
# openwakeword hängt von tflite-runtime ab, das es für Python 3.12 nicht gibt -> ohne deps:
pip install --break-system-packages --no-deps openwakeword
pip install --break-system-packages scikit-learn tqdm requests
# einmalig: melspectrogram-, embedding- und VAD-Modell herunterladen
python3 -c "import openwakeword.utils as u; u.download_models()"
```

Hinweise:
* `numpy` muss < 2 bleiben, wenn `cv_bridge` aus apt kommt. Falls pip numpy 2 installiert:
  `pip install --break-system-packages "numpy<2"`.
* gpiozero braucht auf dem Pi 5 das Backend `lgpio` (RPi.GPIO funktioniert dort nicht).
  Der Benutzer muss Zugriff auf `/dev/gpiochip*` haben (Gruppe `gpio` oder `dialout`, je nach Image).
* Die pip-Pakete stehen nicht als rosdep-Schlüssel in den `package.xml` (nur als Kommentar);
  `rosdep install` installiert sie also nicht.

## Build und Start

```bash
cd ~/pren-implementation-raspberry
colcon build --symlink-install          # baut robot_interfaces zuerst (Abhängigkeit)
source install/setup.bash

ros2 launch robot_core robot.launch.py                   # ALLES: Erkennung + GPIO + Verhalten
ros2 launch robot_core robot.launch.py gpio:=false gestures:=false   # Teile weglassen
ros2 launch robot_vision perception.launch.py            # nur Kamera + Knochen + Gesten + Mikrofon + Sprache
ros2 launch robot_vision perception.launch.py debug:=true mic:=MV7
ros2 launch robot_audio voice.launch.py scores:=true      # nur Audio, mit Scores zum Einstellen
ros2 run robot_vision bone_detector --ros-args -p camera_height:=0.18

ros2 run robot_gpio search_button_driver
ros2 run robot_gpio bone_button_driver
ros2 run robot_gpio led_driver
ros2 run robot_gpio speaker_driver
ros2 run robot_core topic_debugger                       # zeigt alle Topics in der Konsole
```

Prüfen:

```bash
ros2 topic echo /perception/gestures
ros2 topic echo /perception/voice_cmds
ros2 topic echo /perception/bone_position
ros2 topic echo /perception/voice_scores     # mit scores:=true, Reihenfolge SUCH HIER BRAV SITZ
ros2 topic hz /audio/raw                     # muss ~12.5 Hz sein (80 ms pro Nachricht)
ros2 topic pub --once /actuation/led_cmds std_msgs/String "data: blink"
ros2 run rqt_image_view rqt_image_view /perception/gesture_debug   # mit debug:=true
ros2 topic echo /robot_state                 # aktueller Zustand
tail -f ~/pren_logs/states_*.log ~/pren_logs/commands_*.log
```

## Performance prüfen

In dieser Reihenfolge, vom schnellsten zum aufwendigsten Test:

**1. Benchmark ohne ROS und ohne Kamera (~1 min):** misst die reine Rechenzeit jedes Modells.

```bash
python3 tools/benchmark.py                       # nimmt das neueste Bild aus ~/saved_frames
python3 tools/benchmark.py --threads 2 --only yolo
```

Richtwerte: openWakeWord muss klar unter 80 ms pro Block liegen (sonst verpasst die Sprache Wörter).
Gesten brauchen ≥ 10 fps für die 6-von-10-Abstimmung. Für die Knochen reichen 2–5 Hz.
Auf dem PC: YOLO 640 ≈ 67 ms, Gesten ≈ 32 ms pro Bild, Sprache ≈ 11 ms.

**2. Alles zusammen laufen lassen** (`ros2 launch robot_core robot.launch.py`). Jeder
Erkennungs-Node schreibt alle 10 s eine Zeile:

```
[bone_detector]          perf: 4.9 Hz processed, 21 frames skipped | infer 182/240 ms | latency 230/301 ms
[gesture_detector]       perf: 9.8 Hz processed, 190 frames skipped | total 95/140 ms | hands 60/90 ms | pose 70/95 ms | latency 120/180 ms
[voice_command_detector] perf: 12.5 chunks/s (realtime 12.5), predict 25/40 ms of 80 ms, 0 dropped, backlog 0
```

* **Hz processed** unter `max_rate` → die CPU schafft es nicht.
* **latency** = Zeit von der Bildaufnahme bis zum Ergebnis (auch `ros2 topic delay /perception/bone_position`).
* Bei der Sprache müssen **chunks/s ≈ 12.5** und **dropped = 0** sein.

**3. System beobachten:** `htop` (CPU pro Prozess), `vcgencmd measure_temp` und `vcgencmd get_throttled`.
`throttled=0x0` ist gut, alles andere heisst Überhitzung oder zu schwaches Netzteil (Pi 5: 5 A).

**Stellschrauben, falls es nicht reicht:** `max_rate` der Detektoren senken; `process_width: 480`
und `pose_every_n: 3` bei den Gesten; das 320er-YOLO-Modell (`-p model_path:=...bone_recognition_320.onnx`,
liegt in `PythonProject/ros-recognition-nodes/models/`); `num_threads` anpassen; den `camera_driver`
auf 15 fps setzen (jedes Bild wird von zwei Nodes gelesen).

## Modelle

| Datei | Herkunft |
|---|---|
| `robot_vision/models/bone_recognition_640.onnx` | `PythonProject/ros-recognition-nodes/models/` (YOLO26n, Klassen Knochen/Pylon) |
| `robot_vision/models/gesture_recognizer.task` | MediaPipe, wie in `poc.py` |
| `robot_vision/models/pose_landmarker_lite.task` | `PythonProject/personenerkennung_abstandserkennung_objekterkennung/mediapipe/` |
| `robot_audio/models/{such,hier,brav,sitz}_de.onnx` | `PythonProject/spracherkennung/openwakeword/models/` |

Ein anderes Modell ohne Neubau: `-p model_path:=/pfad/zum/modell.onnx`.
