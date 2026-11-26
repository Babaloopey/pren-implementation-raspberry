"""gesture_detector: hand gestures SITZ / PLATZ / AUF / TRICK from the camera image.

Real counterpart of the gesture part of the simulator's `command_recognizer`.
Technology as in PythonProject/poc.py, but MediaPipe only:
  * MediaPipe Gesture Recognizer: hand landmarks + "Open_Palm" / "Pointing_Up" / "Thumb_Up"
    (up to 2 hands)
  * MediaPipe Pose Landmarker (lite) instead of YOLO26-pose: torso box and arm direction
  * geometry + hysteresis + majority vote (gesture_logic.py, thresholds from poc.py)

  SITZ = open palm in front of the chest, AUF = index finger up, PLATZ = index finger down,
  TRICK (Kunststueck) = thumbs up

Subscribes:
  image_topic (sensor_msgs/Image, default /camera/image_raw from camera_driver)
Publishes:
  /perception/gestures       robot_interfaces/Gesture, once each time a gesture becomes stable
  /perception/gesture_debug  sensor_msgs/Image (only with publish_debug_image:=true)
"""
import os
import time

import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
import rclpy
import rclpy.executors
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.msg import Gesture
from sensor_msgs.msg import Image

from .bone_detector import default_model
from .gesture_logic import DEFAULTS, NONE, GestureLogic, mediapipe_persons


class GestureDetector(Node):

    def __init__(self):
        super().__init__('gesture_detector')
        params = self.declare_parameters('', [
            ('gesture_model_path', default_model('gesture_recognizer.task')),
            ('pose_model_path', default_model('pose_landmarker_lite.task')),
            ('image_topic', '/camera/image_raw'),
            ('max_rate', 15.0),              # Hz; the vote window (history_length) is in frames
            ('process_width', 640),          # downscale before inference (0 = keep size)
            ('pose_every_n', 2),             # pose only every n-th frame, box reused in between
            ('num_poses', 2),                # the largest person is used
            ('pose_conf', 0.5),
            ('flip_horizontal', False),      # poc.py mirrored the webcam; the robot camera is not mirrored
            ('hand_detection_conf', 0.5),
            ('repeat_after_s', 0.0),         # > 0: re-publish a held gesture every n s
            ('stale_after_s', 1.0),          # no frames for this long -> reset the vote
            ('publish_debug_image', False),
        ] + [(k, v) for k, v in DEFAULTS.items()])
        self.p = {prm.name: prm.value for prm in params}

        for key in ('gesture_model_path', 'pose_model_path'):
            if not os.path.exists(self.p[key]):
                raise FileNotFoundError(f'{key}: {self.p[key]} not found')

        self.recognizer = vision.GestureRecognizer.create_from_options(
            vision.GestureRecognizerOptions(
                base_options=mp_python.BaseOptions(model_asset_path=self.p['gesture_model_path']),
                running_mode=vision.RunningMode.VIDEO,
                num_hands=2,
                min_hand_detection_confidence=self.p['hand_detection_conf'],
                min_hand_presence_confidence=0.5,
                min_tracking_confidence=0.5,
            ))
        self.pose = vision.PoseLandmarker.create_from_options(
            vision.PoseLandmarkerOptions(
                base_options=mp_python.BaseOptions(model_asset_path=self.p['pose_model_path']),
                running_mode=vision.RunningMode.VIDEO,
                num_poses=self.p['num_poses'],
                min_pose_detection_confidence=self.p['pose_conf'],
                min_pose_presence_confidence=self.p['pose_conf'],
                min_tracking_confidence=0.5,
            ))
        self.logic = GestureLogic(**{k: self.p[k] for k in DEFAULTS})
        self.bridge = CvBridge()

        self.latest = None
        self.last_ts_ms = -1
        self.frame_no = 0
        self.last_frame_time = time.monotonic()
        self.published = NONE           # last gesture sent on /perception/gestures
        self.published_at = 0.0

        self.cmd_pub = self.create_publisher(Gesture, '/perception/gestures', 10)
        self.debug_pub = (self.create_publisher(Image, '/perception/gesture_debug', 1)
                          if self.p['publish_debug_image'] else None)

        self.create_subscription(Image, self.p['image_topic'], self.on_image, qos_profile_sensor_data)
        self.create_timer(1.0 / max(self.p['max_rate'], 0.1), self.step)
        self.get_logger().info(f'gesture_detector ready on {self.p["image_topic"]} (SITZ / PLATZ / AUF / TRICK)')

    def on_image(self, msg):
        self.latest = msg

    def step(self):
        msg, self.latest = self.latest, None
        now = time.monotonic()
        if msg is None:
            if now - self.last_frame_time > self.p['stale_after_s'] and self.logic.history:
                self.logic.reset()           # camera gap: don't let old votes fire later
                self.published = NONE
            return
        self.last_frame_time = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        pw = self.p['process_width']
        if pw and frame.shape[1] > pw:
            frame = cv2.resize(frame, (pw, round(frame.shape[0] * pw / frame.shape[1])),
                               interpolation=cv2.INTER_AREA)
        if self.p['flip_horizontal']:
            frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]
        self.frame_no += 1

        stamp_ms = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000
        ts = max(int(stamp_ms) if stamp_ms > 0 else int(now * 1000), self.last_ts_ms + 1)
        self.last_ts_ms = ts       # MediaPipe VIDEO mode needs strictly increasing timestamps
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB,
                            data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if self.frame_no % max(1, self.p['pose_every_n']) == 0 or self.frame_no == 1:
            pose_result = self.pose.detect_for_video(mp_image, ts)
            self.logic.update_pose(mediapipe_persons(pose_result, w, h), w, h)

        result = self.recognizer.recognize_for_video(mp_image, ts)

        current, hands = self.logic.update_hands(result, w, h)
        stable = self.logic.stabilize(current)

        if stable == NONE:
            self.published = NONE
        elif stable != self.published or (
                self.p['repeat_after_s'] > 0 and now - self.published_at > self.p['repeat_after_s']):
            out = Gesture()
            out.header = msg.header          # stamp of the image the gesture became stable in
            out.command = stable             # gesture_logic uses the same strings as Gesture.*
            out.confidence = float(self.logic.vote_share())
            self.cmd_pub.publish(out)
            self.get_logger().info(f'gesture {stable} ({out.confidence:.2f})')
            self.published, self.published_at = stable, now

        if self.debug_pub is not None:
            self.publish_debug(frame, hands, stable, msg.header)

    def publish_debug(self, frame, hands, stable, header):
        img = frame.copy()
        h, w = img.shape[:2]
        t = self.logic.torso
        if t is not None:
            cv2.rectangle(img, (int(t[0]), int(t[1])), (int(t[2]), int(t[3])), (255, 165, 0), 2)
        for sh, wr, _ in self.logic.arms:
            cv2.line(img, (int(sh[0]), int(sh[1])), (int(wr[0]), int(wr[1])), (255, 0, 255), 2)
        for hd in hands:
            for x, y in hd['landmarks']:
                cv2.circle(img, (int(x * w), int(y * h)), 2, (0, 0, 255), -1)
            label = f'{hd["cmd"]} {hd["gesture"]}'
            cv2.putText(img, label, (int(hd['palm'][0]) - 30, int(hd['palm'][1]) - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img, stable, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (50, 220, 50) if stable != NONE else (60, 60, 220), 2, cv2.LINE_AA)
        out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header = header
        self.debug_pub.publish(out)

    def destroy_node(self):
        self.recognizer.close()
        self.pose.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GestureDetector()
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
