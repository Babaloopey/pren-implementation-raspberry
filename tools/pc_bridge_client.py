#!/usr/bin/env python3
"""Sends the laptop's webcam and microphone to the simulator container (robot_sim/pc_bridge).

Runs on Windows (or any OS) with plain Python. Easiest: tools/pc_bridge.ps1, which creates a
small virtual environment and starts this script.

    python tools/pc_bridge_client.py                    # camera 0 + default microphone
    python tools/pc_bridge_client.py --camera 1 --no-audio
    python tools/pc_bridge_client.py --list             # list cameras and microphones

Reconnects automatically, so it can be started before or after the container.
"""
import argparse
import os
import socket
import struct
import sys
import threading
import time

os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')     # no warnings when probing camera indices


def connect(host, port, name):
    said = False
    while True:
        try:
            s = socket.create_connection((host, port), timeout=3)
            s.settimeout(None)
            print(f'[{name}] verbunden mit {host}:{port}')
            return s
        except OSError:
            if not said:      # the simulator only listens while a node is set to "Echt"
                print(f'[{name}] bereit, wartet bis im Simulator ein Node auf "Echt" steht ...')
                said = True
            time.sleep(2)


def video_loop(a):
    import cv2
    backend = cv2.CAP_DSHOW if sys.platform == 'win32' else cv2.CAP_ANY
    period = 1.0 / a.fps
    while True:
        # the simulator only listens while a node is set to "Echt" -> the camera is only on then
        s = connect(a.host, a.video_port, 'video')
        cap = cv2.VideoCapture(a.camera, backend)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
        if not cap.isOpened():
            print(f'[video] Kamera {a.camera} nicht gefunden (--list zeigt die vorhandenen)')
            s.close()
            return
        print(f'[video] Kamera {a.camera} an: {int(cap.get(3))}x{int(cap.get(4))}')
        n_black = 0
        try:
            while True:
                t0 = time.monotonic()
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue
                if a.preview:
                    cv2.imshow('pc_bridge (q = Vorschau schliessen)', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        a.preview = False
                        cv2.destroyAllWindows()
                n_black = n_black + 1 if frame[::8, ::8].mean() < 3 else 0
                if n_black == 20:
                    print('[video] WARNUNG: Kamerabild ist schwarz. Kamera-Schieber offen? Kamera-Sperrtaste (Fn)?')
                ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                s.sendall(struct.pack('>I', len(jpg)) + jpg.tobytes())
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        except OSError as e:
            print(f'[video] Verbindung weg ({e}), Kamera aus, warte ...')
            s.close()
            cap.release()
            if a.preview:
                cv2.destroyAllWindows()


def audio_loop(a):
    import sounddevice as sd
    rate, block = 16000, 1280                    # 80 ms, what openWakeWord expects
    while True:
        s = connect(a.host, a.audio_port, 'audio')
        s.sendall(struct.pack('>I', rate))
        try:
            with sd.InputStream(device=a.mic, samplerate=rate, channels=1, dtype='int16', blocksize=block) as st:
                print(f'[audio] Mikrofon: {sd.query_devices(st.device)["name"]} @ {rate} Hz')
                quiet = 0
                while True:
                    data, _ = st.read(block)
                    quiet = quiet + 1 if abs(data[:, 0]).max() < 8 else 0
                    if quiet == 40:                 # ~3 s of digital silence
                        print('[audio] WARNUNG: Mikrofon liefert Stille. Stummgeschaltet (Mikrofon-Taste / Windows)? '
                              'Anderes Mikrofon: --list und --mic N')
                    raw = data[:, 0].astype('<i2').tobytes()
                    s.sendall(struct.pack('>I', len(raw)) + raw)
        except OSError as e:
            print(f'[audio] Verbindung weg ({e}), neu verbinden ...')
            s.close()
        except Exception as e:                   # e.g. microphone blocked by Windows privacy settings
            print(f'[audio] Mikrofon-Fehler: {e}')
            s.close()
            time.sleep(3)


def list_devices():
    import cv2
    print('Kameras:')
    for i in range(5):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW if sys.platform == 'win32' else cv2.CAP_ANY)
        if cap.isOpened():
            print(f'  --camera {i}: {int(cap.get(3))}x{int(cap.get(4))}')
        cap.release()
    import sounddevice as sd
    print('Mikrofone:')
    for i, d in enumerate(sd.query_devices()):
        if d['max_input_channels'] > 0:
            print(f'  --mic {i}: {d["name"]}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--video-port', type=int, default=5601)
    ap.add_argument('--audio-port', type=int, default=5602)
    ap.add_argument('--camera', type=int, default=0)
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--fps', type=float, default=15.0)
    ap.add_argument('--mic', type=int, default=None, help='microphone index (see --list), default = Windows default')
    ap.add_argument('--no-video', action='store_true')
    ap.add_argument('--no-audio', action='store_true')
    ap.add_argument('--preview', action='store_true', help='show the camera image in a window')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    if a.list:
        return list_devices()
    threads = []
    if not a.no_video:
        threads.append(threading.Thread(target=video_loop, args=(a,), daemon=True))
    if not a.no_audio:
        threads.append(threading.Thread(target=audio_loop, args=(a,), daemon=True))
    for t in threads:
        t.start()
    print('Strg+C beendet.')
    try:
        while any(t.is_alive() for t in threads):
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
