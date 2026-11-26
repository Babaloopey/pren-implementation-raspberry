"""voice_command_detector: spoken commands SUCH / HIER / BRAV / SITZ via openWakeWord.

Real counterpart of the voice part of the simulator's `command_recognizer`.
Technology as in PythonProject/spracherkennung/openwakeword/test_command_wakewords.py:
one openWakeWord model per command (*_de.onnx), ONNX runtime, Silero VAD, a score
threshold that has to hold for N consecutive 80 ms chunks, and a cooldown.

All command models run in ONE openwakeword.Model, so the melspectrogram and the
embedding network are computed once per chunk, not once per word.

Subscribes:
  /audio/raw              robot_interfaces/AudioChunk (16 kHz mono from microphone_driver)
Publishes:
  /perception/voice_cmds  robot_interfaces/VoiceCommand, once per detected word
  /perception/voice_scores std_msgs/Float32MultiArray, scores of all words in `commands`
                          order (only with publish_scores:=true, for tuning thresholds)
"""
import os
import queue
import threading
import time

import numpy as np
import rclpy
import rclpy.executors
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from robot_interfaces.msg import AudioChunk, VoiceCommand
from std_msgs.msg import Float32MultiArray

RATE = 16000
CHUNK = 1280            # 80 ms, what openWakeWord expects per predict()


def default_model_dir():
    try:
        return os.path.join(get_package_share_directory('robot_audio'), 'models')
    except Exception:
        return os.path.join(os.path.dirname(__file__), '..', 'models')


class VoiceCommandDetector(Node):

    def __init__(self):
        super().__init__('voice_command_detector')
        params = self.declare_parameters('', [
            ('model_dir', default_model_dir()),
            # parallel lists: command i is detected by model file i with threshold i
            ('commands', [VoiceCommand.SUCH, VoiceCommand.HIER, VoiceCommand.BRAV, VoiceCommand.SITZ]),
            ('model_files', ['such_de.onnx', 'hier_de.onnx', 'brav_de.onnx', 'sitz_de.onnx']),
            ('thresholds', [0.8, 0.5, 0.5, 0.8]),   # from the model statistics in the experiments
            ('vad_threshold', 0.3),          # 0 disables the Silero VAD gate
            ('required_consecutive', 2),     # chunks above threshold in a row
            ('cooldown_s', 1.5),             # per command
            ('global_cooldown_s', 0.8),      # after any command, so one word can't fire two models
            ('audio_topic', '/audio/raw'),
            ('publish_scores', False),
        ])
        self.p = {prm.name: prm.value for prm in params}
        cmds, files, thr = self.p['commands'], self.p['model_files'], self.p['thresholds']
        if not len(cmds) == len(files) == len(thr):
            raise ValueError('commands, model_files and thresholds must have the same length')

        paths = [os.path.join(self.p['model_dir'], f) for f in files]
        for path in paths:
            if not os.path.exists(path):
                raise FileNotFoundError(f'wakeword model not found: {path}')

        from openwakeword.model import Model
        kwargs = dict(wakeword_models=paths, inference_framework='onnx')
        if self.p['vad_threshold'] > 0:
            kwargs['vad_threshold'] = self.p['vad_threshold']
        try:
            self.model = Model(**kwargs)
        except Exception as e:
            self.get_logger().error(
                f'openWakeWord could not load ({e}). Missing melspectrogram/embedding/VAD models? '
                "Run once: python3 -c 'import openwakeword.utils as u; u.download_models()'")
            raise
        # openWakeWord names each model after its file name without extension
        self.words = {os.path.splitext(f)[0]: (c.upper(), float(t)) for c, f, t in zip(cmds, files, thr)}
        self.hits = {k: 0 for k in self.words}
        self.last_trigger = {k: 0.0 for k in self.words}
        self.last_any = 0.0
        self.buf = np.zeros(0, dtype=np.int16)
        self.warned_rate = False

        self.cmd_pub = self.create_publisher(VoiceCommand, '/perception/voice_cmds', 10)
        self.score_pub = (self.create_publisher(Float32MultiArray, '/perception/voice_scores', 10)
                          if self.p['publish_scores'] else None)

        # Inference runs in its own thread so a slow predict() never blocks the subscription.
        self.stop_event = threading.Event()
        self.chunks = queue.Queue(maxsize=50)
        self.create_subscription(AudioChunk, self.p['audio_topic'], self.on_audio, 50)
        self.worker = threading.Thread(target=self.process_loop, daemon=True)
        self.worker.start()
        words = ', '.join(f'{c} (>{t})' for c, t in self.words.values())
        self.get_logger().info(f'voice_command_detector ready on {self.p["audio_topic"]}, words: {words}')

    def on_audio(self, msg):
        if msg.sample_rate != RATE:
            if not self.warned_rate:
                self.get_logger().error(f'audio must be {RATE} Hz, got {msg.sample_rate} Hz -> ignored')
                self.warned_rate = True
            return
        samples = np.asarray(msg.data, dtype=np.int16)
        try:
            self.chunks.put_nowait(samples)
        except queue.Full:            # inference can't keep up -> drop the oldest audio
            try:
                self.chunks.get_nowait()
                self.chunks.put_nowait(samples)
            except (queue.Empty, queue.Full):
                pass

    def process_loop(self):
        while not self.stop_event.is_set():
            try:
                samples = self.chunks.get(timeout=0.2)
            except queue.Empty:
                continue
            self.buf = np.concatenate([self.buf, samples])
            while len(self.buf) >= CHUNK:
                chunk, self.buf = self.buf[:CHUNK], self.buf[CHUNK:]
                self.detect(self.model.predict(chunk))

    def detect(self, scores):
        now = time.monotonic()
        best = None
        for key, (cmd, thr) in self.words.items():
            s = float(scores.get(key, 0.0))
            self.hits[key] = self.hits[key] + 1 if s > thr else 0
            if (self.hits[key] >= self.p['required_consecutive']
                    and now - self.last_trigger[key] > self.p['cooldown_s']
                    and now - self.last_any > self.p['global_cooldown_s']
                    and (best is None or s > best[2])):
                best = (key, cmd, s)
        if self.score_pub is not None:
            self.score_pub.publish(Float32MultiArray(
                data=[float(scores.get(k, 0.0)) for k in self.words]))
        if best is None:
            return
        key, cmd, s = best
        self.last_trigger[key] = self.last_any = now
        for k in self.hits:
            self.hits[k] = 0
        out = VoiceCommand()
        out.header.stamp = self.get_clock().now().to_msg()
        out.command = cmd
        out.confidence = s
        self.cmd_pub.publish(out)
        self.get_logger().info(f'voice {cmd} ({s:.2f})')

    def destroy_node(self):
        self.stop_event.set()
        self.worker.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceCommandDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
