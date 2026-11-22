import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import sounddevice as sd
import numpy as np


class MicrophoneDriver(Node):
    def __init__(self):
        super().__init__('microphone_driver')

        # Shure MV7+ natively operates at 48 kHz
        self.declare_parameter('device_keyword', 'MV7')
        self.declare_parameter('sample_rate', 48000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('chunk_duration', 0.1)  # 100 ms per ROS message

        keyword = self.get_parameter('device_keyword').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        chunk_duration = self.get_parameter('chunk_duration').value
        self.blocksize = int(self.sample_rate * chunk_duration)

        self.publisher_ = self.create_publisher(Int16MultiArray, '/audio/raw', 10)

        device_index = self.find_microphone(keyword)
        if device_index is None:
            self.get_logger().warn(
                f'No input device matching "{keyword}" found. Falling back to system default.'
            )

        try:
            self.stream = sd.InputStream(
                device=device_index,
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype='int16',
                blocksize=self.blocksize,
                callback=self.audio_callback
            )
            self.stream.start()
            self.get_logger().info(
                f'microphone_driver started ({self.sample_rate} Hz, {self.channels} ch) '
                f'-> publishing to /audio/raw'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to open audio stream: {e}')
            self.stream = None

    def find_microphone(self, keyword):
        devices = sd.query_devices()
        for idx, dev in enumerate(devices):
            if dev['max_input_channels'] > 0 and keyword.lower() in dev['name'].lower():
                self.get_logger().info(f'Found microphone [{idx}]: {dev["name"]}')
                return idx
        return None

    def audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f'Audio stream status: {status}')

        # Flatten to 1D int16 list and publish
        msg = Int16MultiArray()
        msg.data = indata.flatten().astype(np.int16).tolist()
        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MicrophoneDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()