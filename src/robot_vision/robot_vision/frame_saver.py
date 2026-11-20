import os
import threading
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FrameSaver(Node):
    def __init__(self):
        super().__init__('frame_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.latest_frame = None
        self.lock = threading.Lock()

        # Zielordner für gespeicherte Bilder anlegen
        self.save_dir = os.path.expanduser('~/saved_frames')
        os.makedirs(self.save_dir, exist_ok=True)

        # Hintergrund-Thread für Tastatureingabe (funktioniert auch über SSH)
        self.input_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.input_thread.start()

        self.get_logger().info(f'frame_saver bereit. Drücke [ENTER] zum Speichern in {self.save_dir}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Fehler bei cv_bridge: {e}')

    def keyboard_listener(self):
        while rclpy.ok():
            input()  # Wartet auf Enter
            self.save_current_frame()

    def save_current_frame(self):
        with self.lock:
            if self.latest_frame is None:
                self.get_logger().warn('Noch kein Frame empfangen!')
                return
            frame_to_save = self.latest_frame.copy()

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        filepath = os.path.join(self.save_dir, f'frame_{timestamp}.jpg')
        cv2.imwrite(filepath, frame_to_save)
        self.get_logger().info(f'Frame gespeichert: {filepath}')


def main(args=None):
    rclpy.init(args=args)
    node = FrameSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()