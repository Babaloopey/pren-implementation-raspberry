#!/usr/bin/env python3
"""Measures how fast the recognition models run on this machine. No ROS, no camera needed.

    python3 tools/benchmark.py                       # newest image in ~/saved_frames, else noise
    python3 tools/benchmark.py --image frame.jpg --runs 50 --threads 4
    python3 tools/benchmark.py --only yolo           # yolo | gestures | voice

Prints mean / 95th percentile per model and the resulting maximum rate, plus the CPU
temperature and throttling state on a Raspberry Pi (vcgencmd). Run it once with the Pi idle
and once while `ros2 launch robot_core robot.launch.py` is running to see the difference.
"""
import argparse
import glob
import os
import subprocess
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VISION = os.path.join(ROOT, 'src', 'robot_vision')
VMODELS = os.path.join(VISION, 'models')
AMODELS = os.path.join(ROOT, 'src', 'robot_audio', 'models')
sys.path.insert(0, VISION)


def stats(name, ms, budget_ms=None):
    ms = np.array(ms)
    line = f'{name:<34} mean {ms.mean():7.1f} ms   p95 {np.percentile(ms, 95):7.1f} ms   ' \
           f'-> max {1000 / ms.mean():5.1f} /s'
    if budget_ms:
        line += f'   ({ms.mean() / budget_ms * 100:.0f} % of the {budget_ms:.0f} ms budget)'
    print(line)
    return ms.mean()


def timed(fn, runs, warmup=3):
    for _ in range(warmup):
        fn()
    out = []
    for _ in range(runs):
        t = time.perf_counter()
        fn()
        out.append((time.perf_counter() - t) * 1000)
    return out


def load_image(path):
    import cv2
    if path is None:
        files = sorted(glob.glob(os.path.expanduser('~/saved_frames/*.jpg')), key=os.path.getmtime)
        path = files[-1] if files else None
    if path:
        img = cv2.imread(path)
        if img is not None:
            print(f'image: {path} {img.shape[1]}x{img.shape[0]}')
            return cv2.resize(img, (1280, 720)) if img.shape[:2] != (720, 1280) else img
    print('image: random noise 1280x720 (use --image or frame_saver for a real frame)')
    return np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)


def system_info():
    print(f'python {sys.version.split()[0]}, {os.cpu_count()} CPUs')
    for args in (['vcgencmd', 'measure_temp'], ['vcgencmd', 'get_throttled'],
                 ['vcgencmd', 'measure_clock', 'arm']):
        try:
            print('  ' + subprocess.run(args, capture_output=True, text=True, timeout=2).stdout.strip())
        except (OSError, subprocess.SubprocessError):
            pass


def bench_yolo(img, runs, threads):
    from robot_vision.yolo_onnx import YoloOnnx
    for name in ('bone_recognition_640.onnx', 'bone_recognition_320.onnx'):
        path = os.path.join(VMODELS, name)
        if not os.path.exists(path):
            continue
        m = YoloOnnx(path, num_threads=threads)
        stats(f'YOLO {name} ({threads} thr)', timed(lambda: m(img, 0.5, 0.45), runs))


def bench_gestures(img, runs):
    import cv2
    import mediapipe as mp
    from mediapipe.tasks import python as mpp
    from mediapipe.tasks.python import vision
    small = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA)   # process_width 640
    rgb = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
    mode = vision.RunningMode.VIDEO
    rec = vision.GestureRecognizer.create_from_options(vision.GestureRecognizerOptions(
        base_options=mpp.BaseOptions(model_asset_path=os.path.join(VMODELS, 'gesture_recognizer.task')),
        running_mode=mode, num_hands=2))
    pose = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
        base_options=mpp.BaseOptions(model_asset_path=os.path.join(VMODELS, 'pose_landmarker_lite.task')),
        running_mode=mode, num_poses=2))
    ts = [0]

    def nxt():
        ts[0] += 33
        return ts[0]
    h = stats('MediaPipe gestures (hands, 640px)', timed(lambda: rec.recognize_for_video(rgb, nxt()), runs))
    p = stats('MediaPipe pose lite (640px)', timed(lambda: pose.detect_for_video(rgb, nxt()), runs))
    conv = stats('BGR->RGB + resize 1280->640', timed(
        lambda: cv2.cvtColor(cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA),
                             cv2.COLOR_BGR2RGB), runs))
    per_frame = h + p / 2 + conv                                        # pose_every_n = 2
    print(f'{"  gesture_detector per frame":<34} ~{per_frame:.0f} ms -> max {1000 / per_frame:.1f} fps '
          f'(configured max_rate 15)')
    rec.close()
    pose.close()


def bench_voice(runs):
    from openwakeword.model import Model
    files = [os.path.join(AMODELS, f) for f in ('such_de.onnx', 'hier_de.onnx', 'brav_de.onnx', 'sitz_de.onnx')]
    model = Model(wakeword_models=files, inference_framework='onnx', vad_threshold=0.3)
    chunk = (np.random.randn(1280) * 500).astype(np.int16)
    stats('openWakeWord 4 words + VAD (80 ms)', timed(lambda: model.predict(chunk), runs * 3, warmup=20),
          budget_ms=80)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image')
    ap.add_argument('--runs', type=int, default=30)
    ap.add_argument('--threads', type=int, default=4, help='onnxruntime threads for YOLO (node: num_threads)')
    ap.add_argument('--only', choices=['yolo', 'gestures', 'voice'])
    a = ap.parse_args()

    system_info()
    img = load_image(a.image)
    print()
    for part, fn in (('yolo', lambda: bench_yolo(img, a.runs, a.threads)),
                     ('gestures', lambda: bench_gestures(img, a.runs)),
                     ('voice', lambda: bench_voice(a.runs))):
        if a.only in (None, part):
            try:
                fn()
            except Exception as e:      # missing package / model: report and continue
                print(f'{part}: skipped ({type(e).__name__}: {e})')
    print()
    system_info()


if __name__ == '__main__':
    main()
