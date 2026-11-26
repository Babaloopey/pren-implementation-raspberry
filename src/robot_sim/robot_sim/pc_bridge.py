"""pc_bridge: receives the laptop's webcam and microphone from tools/pc_bridge_client.py (Windows)
and publishes them like the robot's hardware drivers do. Replaces camera_driver and
microphone_driver in the simulator; everything downstream is the unchanged robot code.

  TCP 5601  video: repeated [uint32 big-endian length][JPEG]          -> /camera/image_raw (bgr8)
  TCP 5602  audio: [uint32 sample rate] then [uint32 length][int16 LE mono samples] ...
                                                                       -> /audio/raw (AudioChunk, 16 kHz, 80 ms)
  /sim/bridge_status  std_msgs/String JSON {video, fps, audio, chunks_per_s} every second
"""
import json
import socket
import struct
import threading
import time

import cv2
import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from robot_interfaces.msg import AudioChunk
from sensor_msgs.msg import Image
from std_msgs.msg import String

RATE, CHUNK = 16000, 1280


def recv_exact(conn, n):
    buf = b''
    while len(buf) < n:
        part = conn.recv(n - len(buf))
        if not part:
            raise ConnectionError('closed')
        buf += part
    return buf


class PcBridge(Node):

    def __init__(self):
        super().__init__('pc_bridge')
        params = self.declare_parameters('', [
            ('video_port', 5601), ('audio_port', 5602),
            ('width', 1280), ('height', 720),        # the camera model of the robot expects 1280x720
        ])
        self.p = {prm.name: prm.value for prm in params}
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 5)
        self.audio_pub = self.create_publisher(AudioChunk, '/audio/raw', 20)
        self.status_pub = self.create_publisher(String, '/sim/bridge_status', 5)
        self.stats = {'video': False, 'audio': False, 'frames': 0, 'chunks': 0, 't0': time.monotonic()}
        self.brightness = 128.0         # mean pixel value (EMA): ~0 = black image (shutter closed?)
        self.level = 100.0              # audio RMS (EMA): ~0 = muted microphone
        for port, handler in ((self.p['video_port'], self.video), (self.p['audio_port'], self.audio)):
            threading.Thread(target=self.serve, args=(port, handler), daemon=True).start()
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f'pc_bridge waiting: video on :{self.p["video_port"]}, '
                               f'audio on :{self.p["audio_port"]} (start tools/pc_bridge.cmd on Windows)')

    def serve(self, port, handler):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', port))
        srv.listen(1)
        while rclpy.ok():
            conn, addr = srv.accept()
            self.get_logger().info(f'{handler.__name__} client connected from {addr[0]}')
            try:
                handler(conn)
            except (ConnectionError, OSError) as e:
                self.get_logger().info(f'{handler.__name__} client disconnected ({e})')
            finally:
                self.stats[handler.__name__] = False
                conn.close()

    def video(self, conn):
        self.stats['video'] = True
        while True:
            n = struct.unpack('>I', recv_exact(conn, 4))[0]
            frame = cv2.imdecode(np.frombuffer(recv_exact(conn, n), np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            W, H = self.p['width'], self.p['height']
            h, w = frame.shape[:2]
            if (w, h) != (W, H):            # scale to fit and pad: never stretch (hands/torso keep their shape)
                s = min(W / w, H / h)
                frame = cv2.resize(frame, (round(w * s), round(h * s)), interpolation=cv2.INTER_AREA)
                top, left = (H - frame.shape[0]) // 2, (W - frame.shape[1]) // 2
                frame = cv2.copyMakeBorder(frame, top, H - frame.shape[0] - top, left, W - frame.shape[1] - left,
                                           cv2.BORDER_CONSTANT, value=(0, 0, 0))
                if not self.stats.get('warned'):
                    self.get_logger().warn(f'webcam delivers {w}x{h}, padded to {W}x{H} (bearing of bones is off)')
                    self.stats['warned'] = True
            if self.stats['frames'] % 5 == 0:
                self.brightness = 0.7 * self.brightness + 0.3 * float(frame[::8, ::8].mean())
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            msg.height, msg.width = frame.shape[:2]
            msg.encoding, msg.is_bigendian, msg.step = 'bgr8', 0, frame.shape[1] * 3
            msg.data = frame.tobytes()
            self.image_pub.publish(msg)
            self.stats['frames'] += 1

    def audio(self, conn):
        from robot_audio.microphone_driver import StreamResampler
        rate = struct.unpack('>I', recv_exact(conn, 4))[0]
        resample = StreamResampler(rate, RATE)
        self.get_logger().info(f'audio at {rate} Hz' + ('' if rate == RATE else f' -> resampled to {RATE} Hz'))
        self.stats['audio'] = True
        buf = np.zeros(0, dtype=np.int16)
        while True:
            n = struct.unpack('>I', recv_exact(conn, 4))[0]
            x = np.frombuffer(recv_exact(conn, n), dtype='<i2').astype(np.int16)
            if len(x):
                self.level = 0.9 * self.level + 0.1 * float(np.sqrt(np.mean(x.astype(np.float32) ** 2)))
            buf = np.concatenate([buf, resample(x)])
            while len(buf) >= CHUNK:
                msg = AudioChunk()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'microphone'
                msg.sample_rate = RATE
                msg.data = buf[:CHUNK].tolist()
                buf = buf[CHUNK:]
                self.audio_pub.publish(msg)
                self.stats['chunks'] += 1

    def publish_status(self):
        st, now = self.stats, time.monotonic()
        dt = max(1e-3, now - st['t0'])
        self.status_pub.publish(String(data=json.dumps({
            'video': st['video'], 'fps': st['frames'] / dt,
            'audio': st['audio'], 'chunks_per_s': st['chunks'] / dt,
            'brightness': round(self.brightness, 1), 'level': round(self.level, 1),
            'video_black': st['video'] and self.brightness < 3.0,
            'audio_silent': st['audio'] and self.level < 3.0})))
        st.update(frames=0, chunks=0, t0=now)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PcBridge()
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
