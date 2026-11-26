"""microphone_driver: reads the microphone and publishes ready-to-use 16 kHz mono audio.

There is no separate voice_processor: the driver already delivers the format the audio
consumers need (openWakeWord: 16 kHz, mono, int16, 80 ms = 1280 samples per chunk).
The Shure MV7+ records natively at 48 kHz; the driver resamples continuously across chunk
borders (stateful low-pass + interpolation, no clicks) and downmixes to mono.

Publishes:
  /audio/raw   robot_interfaces/AudioChunk  `output_rate` Hz (16000), mono,
               `chunk_duration` s per message (0.08 s = 1280 samples, one openWakeWord frame)
"""
import numpy as np
import rclpy
import rclpy.executors
import sounddevice as sd
from rclpy.node import Node
from robot_interfaces.msg import AudioChunk


class StreamResampler:
    """Arbitrary-ratio resampler for a continuous stream of int16 chunks."""

    def __init__(self, src_rate, dst_rate, taps=63):
        self.step = src_rate / dst_rate
        self.active = src_rate != dst_rate
        if self.active:
            from scipy.signal import firwin
            self.b = firwin(taps, 0.95 * (dst_rate / 2), fs=src_rate)   # anti-alias low-pass
            self.zi = np.zeros(taps - 1)
        # global input-sample indices: output sample k lies at input index k * step
        self.next_t = 0.0       # input index of the next output sample
        self.offset = 0         # input index of the first sample of the current chunk
        self.prev = 0.0         # last filtered sample of the previous chunk (index offset - 1)

    def __call__(self, x):
        if not self.active:
            return x
        from scipy.signal import lfilter
        y, self.zi = lfilter(self.b, 1.0, x.astype(np.float64), zi=self.zi)
        ext = np.concatenate([[self.prev], y])          # ext[j] = sample at offset - 1 + j
        last = self.offset + len(y) - 1                 # newest input index available
        t = np.arange(self.next_t, last + 1e-9, self.step)
        out = np.interp(t - (self.offset - 1), np.arange(len(ext)), ext)
        if len(t):
            self.next_t = t[-1] + self.step
        self.offset += len(y)
        self.prev = y[-1]
        return np.clip(np.round(out), -32768, 32767).astype(np.int16)


class MicrophoneDriver(Node):
    def __init__(self):
        super().__init__('microphone_driver')

        # Shure MV7+ natively operates at 48 kHz
        self.declare_parameter('device_keyword', 'MV7')
        self.declare_parameter('sample_rate', 48000)       # rate the device records at
        self.declare_parameter('channels', 1)
        self.declare_parameter('chunk_duration', 0.08)     # 80 ms per ROS message (openWakeWord frame)
        self.declare_parameter('output_rate', 16000)       # rate published on /audio/raw
        self.declare_parameter('frame_id', 'microphone')

        keyword = self.get_parameter('device_keyword').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        chunk_duration = self.get_parameter('chunk_duration').value
        self.output_rate = self.get_parameter('output_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.blocksize = int(round(self.sample_rate * chunk_duration))
        self.out_chunk = int(round(self.output_rate * chunk_duration))

        self.resample = StreamResampler(self.sample_rate, self.output_rate)
        self.buf = np.zeros(0, dtype=np.int16)
        self.publisher_ = self.create_publisher(AudioChunk, '/audio/raw', 10)

        device_index = self.find_microphone(keyword)
        if device_index is None:
            self.get_logger().warn(
                f'No input device matching "{keyword}" found. Falling back to system default.'
            )

        self.stream = None
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
                f'-> publishing {self.output_rate} Hz mono, {self.out_chunk} samples/msg to /audio/raw'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to open audio stream: {e}')

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

        # indata: (frames, channels) int16 -> mono -> output rate
        mono = indata[:, 0] if self.channels == 1 else indata.mean(axis=1)
        self.buf = np.concatenate([self.buf, self.resample(mono.astype(np.int16))])

        # resampling can give one sample more or less per block -> publish fixed-size chunks
        while len(self.buf) >= self.out_chunk:
            msg = AudioChunk()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.sample_rate = self.output_rate
            msg.data = self.buf[:self.out_chunk].tolist()
            self.buf = self.buf[self.out_chunk:]
            self.publisher_.publish(msg)

    def destroy_node(self):
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MicrophoneDriver()
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
