# PREN Hunde-Roboter – ROS 2 (Raspberry Pi)

ROS 2 Jazzy auf Ubuntu 24.04. Paketstruktur gemäss Architekturdiagramm
(`robot_gpio`, `robot_vision`, `robot_audio`, `robot_core`, `robot_communication`, `robot_actuation`)
plus `robot_interfaces` für die eigenen Nachrichtentypen.

## Datenfluss der Erkennung

```
camera_driver ──/camera/image_raw──┬─> gesture_detector ──/perception/gestures──────> state_coordinator
 (Pi Camera Module 3)              └─> bone_detector ─────/perception/bone_position─> state_coordinator
microphone_driver ──/audio/raw──────> voice_command_detector ──/perception/voice_cmds──> state_coordinator
```

Abweichung vom Architekturdiagramm: **kein `image_processor` und kein `voice_processor`**.
Die Treiber liefern bereits das Endformat (Kamera: 1280×720 BGR, Mikrofon: 16 kHz mono in
80-ms-Blöcken). Die modellspezifische Vorverarbeitung (YOLO: Letterbox 640×640, MediaPipe: RGB
640 px) passiert in den Detektoren, weil jedes Modell eine andere braucht. Ein Zwischen-Node
würde nur jedes Bild ein zweites Mal über DDS kopieren.

## Nachrichten (`robot_interfaces`)

| Nachricht | Topic | Inhalt |
|---|---|---|
| `Gesture` | `/perception/gestures` | `header`, `command` (`SITZ`, `PLATZ`, `AUF`, `TRICK` als Konstanten), `confidence` |
| `VoiceCommand` | `/perception/voice_cmds` | `header`, `command` (`SUCH`, `HIER`, `BRAV`, `SITZ`), `confidence` |
| `BoneDetectionArray` | `/perception/bone_position` (Knochen), `/perception/pylon_position` (Pylonen) | `header` (Bild-Zeitstempel, `camera_link`), Liste von `BoneDetection` |
| `BoneDetection` | – | `class_id`, `score`, `bearing`, `bbox`, `has_position`, `position` |
| `AudioChunk` | `/audio/raw` | `header`, `sample_rate`, `data` (int16 mono) |

Ansehen: `ros2 interface show robot_interfaces/msg/BoneDetection`.

In Python:

```python
from robot_interfaces.msg import Gesture, VoiceCommand, BoneDetectionArray

def on_gesture(msg):
    if msg.command == Gesture.TRICK: ...
```

* **Gesten** werden **einmal** gesendet, sobald sie in 6 der letzten 10 Frames erkannt wurden;
  erneut erst, nachdem die Geste weg war. `confidence` = Anteil zustimmender Frames.
* **Sprache**: ein Befehl pro erkanntem Wort, `confidence` = openWakeWord-Score.
  Für «Kunststück» gibt es noch kein Sprachmodell (als Geste: Daumen hoch).
* **Knochen**: wird bei **jedem** ausgewerteten Bild gesendet (leere Liste = nichts gesehen).
  `bearing` in **Radiant**, positiv = links, negativ = rechts (ROS-Konvention REP 103).
  Mit der geradeaus blickenden Kamera (Neigung 0, Linse 90° zum Boden) hängt der Winkel nur von der
  Pixelspalte und dem Sichtfeld ab, nicht von der Kamerahöhe. Pi Camera Module 3: 75° diagonal ⇒
  ca. 67.5° horizontal bei 16:9, also ±0.59 rad (±33.8°) am Bildrand.
  `position` ist nur gültig, wenn `has_position` true ist; das ist erst der Fall, wenn
  `camera_height` in `config/vision.yaml` gesetzt ist (Bodenprojektion in `camera_link`).
  Mit Neigung 0 geht das nur für Knochen unterhalb der Bildmitte und wird mit der Distanz schnell ungenau.

## Gesten

| Befehl | Geste | Woher die Entscheidung kommt |
|---|---|---|
| SITZ | offene Hand vor der Brust | MediaPipe-Klasse `Open_Palm` **und** Hand in der Oberkörper-Box (Pose) |
| TRICK (Kunststück) | Daumen hoch | MediaPipe-Klasse `Thumb_Up` |
| AUF | Zeigefinger (oder «L») nach oben | MediaPipe-Klasse `Pointing_Up`, sonst Geometrie der Handpunkte |
| PLATZ | Zeigefinger zum Boden | Geometrie der Handpunkte (+ Arm aus der Pose). MediaPipe hat **keine** Klasse «Zeigefinger nach unten» |

Reihenfolge pro Hand: SITZ > TRICK > AUF > PLATZ.

## Was wird wo verwendet

| Was | Wofür | Wo (Code) | Datei / Installation |
|---|---|---|---|
| **YOLO26n**, eigenes Training (Knochen, Pylon), ONNX 640×640 | Knochen- und Pylonen-Boxen | `robot_vision/bone_detector.py` | `robot_vision/models/bone_recognition_640.onnx` |
| **onnxruntime** (CPU) | führt YOLO ohne PyTorch/Ultralytics aus | `robot_vision/yolo_onnx.py` | pip `onnxruntime` |
| Kameramodell (75° diagonal) | Pixel → Winkel; optional Bodenprojektion | `GroundProjector` in `bone_detector.py` | `config/vision.yaml` |
| **MediaPipe Gesture Recognizer** | Handpunkte + `Open_Palm` / `Pointing_Up` / `Thumb_Up` | `robot_vision/gesture_detector.py` | `robot_vision/models/gesture_recognizer.task`, pip `mediapipe` |
| **MediaPipe Pose Landmarker (lite)** | Oberkörper-Box, Armrichtung | `gesture_detector.py`, `mediapipe_persons()` in `gesture_logic.py` | `robot_vision/models/pose_landmarker_lite.task` |
| Gesten-Logik aus `poc.py` | Entscheidung, Hysterese, Mehrheitsentscheid | `robot_vision/gesture_logic.py` | – |
| **PyAudio** + SciPy (FIR-Tiefpass) | Mikrofon lesen, falls nötig auf 16 kHz umrechnen | `robot_audio/microphone_driver.py` | apt `python3-pyaudio`, `python3-scipy` |
| **openWakeWord** (ONNX) + Silero-VAD | ein Modell pro Sprachbefehl, alle in einem `Model` | `robot_audio/voice_command_detector.py` | `robot_audio/models/{such,hier,brav,sitz}_de.onnx`, pip `openwakeword` |
| Eigene Nachrichten | Typen aller Erkennungs-Topics und Audio | `robot_interfaces/msg/*.msg` | `rosidl_default_generators` |
| OpenCV, cv_bridge, tf2_ros | Bildverarbeitung, ROS-Bilder, statische TF `base_link → camera_link` | Nodes, `launch/perception.launch.py` | apt |

## Installation auf dem Pi

```bash
sudo apt install ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-rosidl-default-generators \
                 python3-opencv python3-numpy python3-scipy python3-pyaudio portaudio19-dev
# pip-Pakete (Ubuntu 24.04 schützt das System-Python -> --break-system-packages
# oder ein venv mit --system-site-packages, aus dem auch colcon/ros2 gestartet wird)
pip install --break-system-packages onnxruntime mediapipe
# openwakeword hängt von tflite-runtime ab, das es für Python 3.12 nicht gibt -> ohne deps:
pip install --break-system-packages --no-deps openwakeword
pip install --break-system-packages scikit-learn tqdm requests
# einmalig: melspectrogram-, embedding- und VAD-Modell herunterladen
python3 -c "import openwakeword.utils as u; u.download_models()"
```

Hinweis: `numpy` muss < 2 bleiben, wenn `cv_bridge` aus apt kommt. Falls pip numpy 2 installiert:
`pip install --break-system-packages "numpy<2"`.

## Build und Start

```bash
cd ~/pren-implementation-raspberry
colcon build --symlink-install          # baut robot_interfaces zuerst (Abhängigkeit)
source install/setup.bash

ros2 launch robot_vision perception.launch.py            # Kamera + Knochen + Gesten + Mikrofon + Sprache
ros2 launch robot_vision perception.launch.py debug:=true mic:=USB
ros2 launch robot_audio voice.launch.py scores:=true      # nur Audio, mit Scores zum Einstellen
ros2 run robot_vision bone_detector --ros-args -p camera_height:=0.18
```

Prüfen:

```bash
ros2 topic echo /perception/gestures
ros2 topic echo /perception/voice_cmds
ros2 topic echo /perception/bone_position
ros2 topic echo /perception/voice_scores     # mit scores:=true, Reihenfolge SUCH HIER BRAV SITZ
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
