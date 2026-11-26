"""bone_detector: YOLO26 (ONNX, 640 px) finds bones and pylons in the camera image.

Real counterpart of the simulator's `bone_detector`. Runs without PyTorch/Ultralytics
(onnxruntime only, see yolo_onnx.py).

For every object the node computes the DIRECTION (bearing): the horizontal angle from the
pixel column and the camera's field of view. It needs neither the camera height nor a
distance; with the camera looking straight ahead (pitch 0) it depends only on the x pixel.

Optional, once the camera height is known (`camera_height` > 0): the detection is also
projected onto the floor (flat ground) -> `position` in camera_link (x forward, y left,
z up; floor at z = -camera_height) and `has_position = true`.

Subscribes:
  image_topic (sensor_msgs/Image, default /camera/image_raw from camera_driver)
Publishes (every processed image, empty list = nothing seen):
  /perception/bone_position   robot_interfaces/BoneDetectionArray  bones
  /perception/pylon_position  robot_interfaces/BoneDetectionArray  pylons
  /perception/bone_debug      sensor_msgs/Image (only with publish_debug_image:=true)

Only the newest frame is processed (rate-limited by `max_rate`), so inference never
builds up a backlog on the Pi.
"""
import math
import os
import time

import cv2
import rclpy
import rclpy.executors
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.msg import BoneDetection, BoneDetectionArray
from sensor_msgs.msg import Image

from .perf import PerfStats, age_ms
from .yolo_onnx import YoloOnnx

# model class index -> class_id on the topic
CLASSES = {0: BoneDetection.BONE, 1: BoneDetection.PYLON}


def default_model(name):
    try:
        return os.path.join(get_package_share_directory('robot_vision'), 'models', name)
    except Exception:          # not installed (e.g. run from the source tree)
        return os.path.join(os.path.dirname(__file__), '..', 'models', name)


class GroundProjector:
    """Pixel -> bearing / point on the floor in camera_link (x forward, y left, z up).

    Without calibrated intrinsics the focal length comes from the DIAGONAL field of view
    (as the data sheets state it: Pi Camera Module 3 = 75 deg, Module 3 Wide = 120 deg) and
    the image diagonal in pixels. Square pixels and a full-sensor (uncropped) image assumed.
    """

    def __init__(self, width, height, dfov_deg, cam_height, pitch_deg, fx=0.0, fy=0.0,
                 cx=0.0, cy=0.0):
        diag = math.hypot(width, height)
        self.fx = fx if fx > 0 else (diag / 2) / math.tan(math.radians(dfov_deg) / 2)
        self.fy = fy if fy > 0 else self.fx
        self.cx = cx if cx > 0 else width / 2
        self.cy = cy if cy > 0 else height / 2
        self.h = cam_height
        self.cp, self.sp = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))

    def ray(self, u, v):
        # ray in body axes of an un-tilted camera: x fwd, y left, z up
        x, y, z = 1.0, -(u - self.cx) / self.fx, -(v - self.cy) / self.fy
        # tilt down by pitch (rotation about +y)
        return self.cp * x + self.sp * z, y, -self.sp * x + self.cp * z

    def bearing(self, u, v):
        """Horizontal angle to the pixel in rad, + = left. Independent of the camera height."""
        xb, y, _ = self.ray(u, v)
        return math.atan2(y, xb)

    def project(self, u, v):
        if self.h <= 0:          # camera height unknown -> no distance
            return None
        xb, y, zb = self.ray(u, v)
        if zb >= -1e-6:          # ray at or above the horizon -> never hits the floor
            return None
        t = -self.h / zb
        return t * xb, t * y, -self.h


class BoneDetector(Node):

    def __init__(self):
        super().__init__('bone_detector')
        params = self.declare_parameters('', [
            ('model_path', default_model('bone_recognition_640.onnx')),
            ('image_topic', '/camera/image_raw'),
            ('conf_threshold', 0.5),
            ('iou_threshold', 0.45),
            ('max_rate', 5.0),               # Hz, inference rate limit (640 px on a Pi: ~3-6 Hz)
            ('num_threads', 4),
            ('flip_horizontal', False),      # experiments used a mirrored webcam image
            # camera model: Pi Camera Module 3, 75 deg diagonal FOV (Module 3 Wide: 120 deg)
            ('dfov_deg', 75.0),
            ('fx', 0.0), ('fy', 0.0), ('cx', 0.0), ('cy', 0.0),   # > 0 overrides dfov
            ('camera_height', 0.0),          # m above the floor; 0 = unknown -> direction only
            ('camera_pitch_deg', 0.0),       # 0 = looking straight ahead (lens 90 deg to the floor)
            ('max_range', 6.0),              # projections farther than this -> has_position false
            ('frame_id', 'camera_link'),
            ('publish_debug_image', False),
            ('stats_interval', 10.0),        # s between perf log lines, 0 = off
        ])
        self.p = {prm.name: prm.value for prm in params}

        path = self.p['model_path']
        if not os.path.exists(path):
            raise FileNotFoundError(f'model not found: {path}')
        self.model = YoloOnnx(path, num_threads=self.p['num_threads'])
        self.bridge = CvBridge()
        self.projector = None
        self.proj_size = None
        self.latest = None
        self.perf = PerfStats(self.get_logger(), self.p['stats_interval'])

        self.bone_pub = self.create_publisher(BoneDetectionArray, '/perception/bone_position', 10)
        self.pylon_pub = self.create_publisher(BoneDetectionArray, '/perception/pylon_position', 10)
        self.debug_pub = (self.create_publisher(Image, '/perception/bone_debug', 1)
                          if self.p['publish_debug_image'] else None)

        self.create_subscription(Image, self.p['image_topic'], self.on_image, qos_profile_sensor_data)
        self.create_timer(1.0 / max(self.p['max_rate'], 0.1), self.step)
        self.get_logger().info(
            f'bone_detector ready: {os.path.basename(path)} '
            f'({self.model.input_w}x{self.model.input_h}, classes {self.model.names}) '
            f'on {self.p["image_topic"]}')

    def on_image(self, msg):
        if self.latest is not None:
            self.perf.skipped()
        self.latest = msg           # keep only the newest frame

    def step(self):
        msg, self.latest = self.latest, None
        if msg is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.p['flip_horizontal']:
            frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]
        if self.proj_size != (w, h):
            self.proj_size = (w, h)
            p = self.p
            self.projector = GroundProjector(w, h, p['dfov_deg'], p['camera_height'],
                                             p['camera_pitch_deg'], p['fx'], p['fy'], p['cx'], p['cy'])

        t0 = time.perf_counter()
        dets = self.model(frame, self.p['conf_threshold'], self.p['iou_threshold'])
        dt = time.perf_counter() - t0

        bones, pylons = BoneDetectionArray(), BoneDetectionArray()
        for arr in (bones, pylons):
            arr.header.stamp = msg.header.stamp     # moment the image was taken
            arr.header.frame_id = self.p['frame_id']

        for d in dets:                              # already sorted by score
            class_id = CLASSES.get(d.cls)
            if class_id is None:
                continue
            u = (d.x1 + d.x2) / 2
            det = BoneDetection()
            det.class_id = class_id
            det.score = float(d.score)
            det.bearing = float(self.projector.bearing(u, (d.y1 + d.y2) / 2))
            det.bbox = [float(d.x1), float(d.y1), float(d.x2), float(d.y2)]
            # bbox centre ~ centre of the flat bone on the floor; pylons stand on it -> bottom edge
            pt = self.projector.project(u, (d.y1 + d.y2) / 2 if class_id == BoneDetection.BONE else d.y2)
            if pt is not None and math.hypot(pt[0], pt[1]) <= self.p['max_range']:
                det.has_position = True
                det.position.x, det.position.y, det.position.z = (float(c) for c in pt)
            (bones if class_id == BoneDetection.BONE else pylons).detections.append(det)

        self.bone_pub.publish(bones)
        self.pylon_pub.publish(pylons)
        self.perf.add(infer=dt * 1000, latency=age_ms(self, msg.header.stamp))
        if bones.detections:
            b = bones.detections[0]
            self.get_logger().info(
                f'bone at {math.degrees(b.bearing):+.1f} deg, score {b.score:.2f} '
                f'({dt * 1000:.0f} ms)', throttle_duration_sec=1.0)
        if self.debug_pub is not None:
            self.publish_debug(frame, dets, msg.header)

    def publish_debug(self, frame, dets, header):
        img = frame.copy()
        for d in dets:
            color = (0, 200, 255) if d.cls == 0 else (255, 120, 0)
            cv2.rectangle(img, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)), color, 2)
            bearing = math.degrees(self.projector.bearing((d.x1 + d.x2) / 2, (d.y1 + d.y2) / 2))
            label = f'{self.model.names.get(d.cls, d.cls)} {d.score:.2f} {bearing:+.0f}deg'
            cv2.putText(img, label, (int(d.x1), max(15, int(d.y1) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header = header
        self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BoneDetector()
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
