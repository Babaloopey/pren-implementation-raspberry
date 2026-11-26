# PREN Hunde-Roboter – ROS 2 (Raspberry Pi)

ROS 2 Jazzy auf Ubuntu 24.04, Raspberry Pi.

| Paket | Inhalt |
|---|---|
| `robot_vision` | `camera_driver`, `bone_detector`, `gesture_detector`, `frame_saver` |
| `robot_audio` | `microphone_driver`, `voice_command_detector` |
| `robot_gpio` | `search_button_driver`, `bone_button_driver`, `led_driver`, `speaker_driver` |
| `robot_core` | `topic_debugger` (später `state_coordinator`, `command_coordinator`) |
| `robot_communication` | (später `serial_uart`) |
| `robot_interfaces` | eigene Nachrichtentypen |

## Datenfluss

```
camera_driver ──/camera/image_raw──┬─> gesture_detector ──/perception/gestures───────> state_coordinator
 (Pi Camera Module 3)              └─> bone_detector ─────/perception/bone_position──> state_coordinator
microphone_driver ──/audio/raw──────> voice_command_detector ──/perception/voice_cmds──> state_coordinator
 (Shure MV7+)
search_button_driver ──/perception/search_contact──> state_coordinator
bone_button_driver ────/perception/bone_contact────> state_coordinator
command_coordinator ──/actuation/led_cmds─────> led_driver
                    ──/actuation/speaker_cmds─> speaker_driver
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

ros2 launch robot_vision perception.launch.py            # Kamera + Knochen + Gesten + Mikrofon + Sprache
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
```

## Modelle

| Datei | Herkunft |
|---|---|
| `robot_vision/models/bone_recognition_640.onnx` | `PythonProject/ros-recognition-nodes/models/` (YOLO26n, Klassen Knochen/Pylon) |
| `robot_vision/models/gesture_recognizer.task` | MediaPipe, wie in `poc.py` |
| `robot_vision/models/pose_landmarker_lite.task` | `PythonProject/personenerkennung_abstandserkennung_objekterkennung/mediapipe/` |
| `robot_audio/models/{such,hier,brav,sitz}_de.onnx` | `PythonProject/spracherkennung/openwakeword/models/` |

Ein anderes Modell ohne Neubau: `-p model_path:=/pfad/zum/modell.onnx`.
