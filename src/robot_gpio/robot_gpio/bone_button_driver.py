import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import Button


class BoneButtonDriver(Node):
    def __init__(self):
        super().__init__('bone_button_driver')
        self.declare_parameter('gpio_pin', 17)  # BCM pin number
        pin = self.get_parameter('gpio_pin').value

        self.publisher_ = self.create_publisher(Bool, '/perception/bone_contact', 10)

        # pull_up=True assumes the button connects the GPIO pin to GND
        self.button = Button(pin, pull_up=True, bounce_time=0.05)
        self.button.when_pressed = self.on_press
        self.button.when_released = self.on_release

        self.get_logger().info(f'bone_button_driver started on GPIO {pin} -> publishing to /perception/bone_contact')

    def on_press(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().info('Button pressed -> published True')

    def on_release(self):
        msg = Bool()
        msg.data = False
        self.publisher_.publish(msg)
        self.get_logger().info('Button released -> published False')

    def destroy_node(self):
        self.button.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BoneButtonDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()