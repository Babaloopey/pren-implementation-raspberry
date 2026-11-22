import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import LED


class LedDriver(Node):
    def __init__(self):
        super().__init__('led_driver')
        self.declare_parameter('gpio_pin', 27)  # BCM pin number
        pin = self.get_parameter('gpio_pin').value

        self.led = LED(pin)
        self.subscription = self.create_subscription(
            String,
            '/actuation/led_cmds',
            self.cmd_callback,
            10
        )

        self.get_logger().info(f'led_driver started on GPIO {pin} <- listening on /actuation/led_cmds')

    def cmd_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'on':
            self.led.on()
        elif cmd == 'off':
            self.led.off()
        elif cmd == 'blink':
            self.led.blink(on_time=0.5, off_time=0.5)
        else:
            self.get_logger().warn(f'Unknown LED command: "{cmd}" (use: on, off, blink)')
            return
        self.get_logger().info(f'LED set to: {cmd}')

    def destroy_node(self):
        self.led.off()
        self.led.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LedDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()