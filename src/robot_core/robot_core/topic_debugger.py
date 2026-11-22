import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message


class TopicDebugger(Node):
    def __init__(self):
        super().__init__('topic_debugger')

        # Topics to ignore so the console doesn't get flooded
        self.ignored_topics = {'/rosout', '/parameter_events'}

        # Keep track of topics we already subscribed to
        self.active_subs = {}

        # BEST_EFFORT subscribers can receive from both RELIABLE and BEST_EFFORT publishers
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Scan for new topics every 2 seconds
        self.timer = self.create_timer(2.0, self.discover_topics)
        self.get_logger().info('topic_debugger started — scanning for topics...')

    def discover_topics(self):
        topic_list = self.get_topic_names_and_types()

        for topic_name, msg_types in topic_list:
            if topic_name in self.ignored_topics or topic_name in self.active_subs:
                continue

            if not msg_types:
                continue

            msg_type_str = msg_types[0]
            try:
                msg_class = get_message(msg_type_str)
                sub = self.create_subscription(
                    msg_class,
                    topic_name,
                    lambda msg, t=topic_name, mt=msg_type_str: self.generic_callback(t, mt, msg),
                    self.qos
                )
                self.active_subs[topic_name] = sub
                self.get_logger().info(f'Subscribed to: {topic_name} [{msg_type_str}]')
            except Exception as e:
                self.get_logger().warn(f'Could not subscribe to {topic_name} ({msg_type_str}): {e}')

    def generic_callback(self, topic_name, msg_type_str, msg):
        # Prevent raw camera frames from flooding the terminal
        if msg_type_str == 'sensor_msgs/msg/Image':
            self.get_logger().info(
                f'[{topic_name}] Image {msg.width}x{msg.height} ({msg.encoding})'
            )
        else:
            self.get_logger().info(f'[{topic_name}] {msg}')


def main(args=None):
    rclpy.init(args=args)
    node = TopicDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()