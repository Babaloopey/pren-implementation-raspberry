import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # GStreamer-Pipeline für Pi Camera Module v3 (libcamera -> BGR für OpenCV)
        gst_pipeline = (
            'libcamerasrc ! '
            'video/x-raw,width=1280,height=720 ! '
            'videoconvert ! '
            'video/x-raw,format=BGR ! '
            'appsink drop=1'
        )

        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error('GStreamer-Pipeline konnte nicht geöffnet werden!')

        # ~30 FPS
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info('camera_driver (GStreamer) gestartet – publisht auf /camera/image_raw')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Kein Frame von der Kamera erhalten.')

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()