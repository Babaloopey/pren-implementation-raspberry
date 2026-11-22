import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import Buzzer


class SpeakerDriver(Node):
    def __init__(self):
        super().__init__('speaker_driver')
        self.declare_parameter('gpio_pin', 22)  # BCM pin number
        pin = self.get_parameter('gpio_pin').value

        self.buzzer = Buzzer(pin)
        self.subscription = self.create_subscription(
            String,
            '/actuation/speaker_cmds',
            self.cmd_callback,
            10
        )

        self.get_logger().info(f'speaker_driver started on GPIO {pin} <- listening on /actuation/speaker_cmds')

    def cmd_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'on':
            self.buzzer.on()
        elif cmd == 'off':
            self.buzzer.off()
        elif cmd == 'beep':
            self.buzzer.beep(on_time=0.2, off_time=0.1, n=1, background=True)
        elif cmd == 'double_beep':
            self.buzzer.beep(on_time=0.15, off_time=0.15, n=2, background=True)
        else:
            self.get_logger().warn(f'Unknown speaker command: "{cmd}" (use: on, off, beep, double_beep)')
            return
        self.get_logger().info(f'Speaker command executed: {cmd}')

    def destroy_node(self):
        self.buzzer.off()
        self.buzzer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()