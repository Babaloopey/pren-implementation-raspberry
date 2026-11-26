"""state_coordinator: the dog's behaviour (state diagrams) as a ROS node.

The state machine itself lives in dog_fsm.py (no ROS, testable on any PC). This node feeds it
with the perception topics and turns its requests into RobotCommand messages.

Subscribes:
  /perception/gestures        robot_interfaces/Gesture
  /perception/voice_cmds      robot_interfaces/VoiceCommand
  /perception/search_contact  std_msgs/Bool   search button (rising edge = "Such-Taster betaetigt")
  /perception/bone_contact    std_msgs/Bool   bone button ("Knochen beruehrt")
  /perception/bone_position   robot_interfaces/BoneDetectionArray
  /odom                       nav_msgs/Odometry  pose relative to the start (command_coordinator)
  /robot_command_status       robot_interfaces/CommandStatus
  /system/shutdown            std_msgs/Empty  "System ausschalten" (only accepted in SITZEN)
Publishes:
  /robot_command              robot_interfaces/RobotCommand
  /robot_state                std_msgs/String  current state (latched)
Log file:
  <log_dir>/states_<date>.log  every state change and every ignored command
"""
import math

import rclpy
import rclpy.executors
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_interfaces.msg import (BoneDetection, BoneDetectionArray, CommandStatus, Gesture,
                                  RobotCommand, VoiceCommand)
from std_msgs.msg import Bool, Empty, String

from .dog_fsm import DEFAULTS, DogFSM, RobotPort
from .filelog import file_logger

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RosRobot(RobotPort):
    """RobotPort -> /robot_command messages."""

    def __init__(self, node):
        self.node = node
        self.next_id = 1

    def _send(self, action, value=0.0, linear=0.0, angular=0.0, name=''):
        msg = RobotCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.id = self.next_id
        self.next_id += 1
        msg.action, msg.value, msg.linear, msg.angular, msg.name = (
            action, float(value), float(linear), float(angular), name)
        self.node.cmd_pub.publish(msg)
        return msg.id

    def stop(self):
        return self._send(RobotCommand.STOP)

    def velocity(self, linear, angular):
        return self._send(RobotCommand.VELOCITY, linear=linear, angular=angular)

    def signal(self, name):
        return self._send(RobotCommand.SIGNAL, name=name)

    def move(self, meters):
        return self._send(RobotCommand.MOVE, value=meters)

    def turn(self, radians):
        return self._send(RobotCommand.TURN, value=radians)

    def posture(self, name):
        return self._send(RobotCommand.POSTURE, name=name)

    def head(self, name):
        return self._send(RobotCommand.HEAD, name=name)

    def behavior(self, name):
        return self._send(RobotCommand.BEHAVIOR, name=name)

    def power_off(self):
        self.node.power_off()


class StateCoordinator(Node):

    def __init__(self):
        super().__init__('state_coordinator')
        params = self.declare_parameters('', [
            ('rate', 20.0),
            ('log_dir', '~/pren_logs'),
            ('image_height', 720),            # camera image height in px
            ('near_bottom_frac', 0.95),       # "Knochen ist nahe": bbox bottom below this image fraction
            ('near_distance', 0.35),          # ... or the floor projection is closer than this (m)
            ('min_bone_score', 0.5),
            ('poweroff_command', ''),         # e.g. 'sudo poweroff'; empty = only stop the nodes' work
        ] + [(k, v) for k, v in DEFAULTS.items()])
        self.p = {prm.name: prm.value for prm in params}

        self.flog, path = file_logger('states', self.p['log_dir'], 'states')
        self.cmd_pub = self.create_publisher(RobotCommand, '/robot_command', 10)
        self.state_pub = self.create_publisher(String, '/robot_state', LATCHED)

        self.fsm = DogFSM(RosRobot(self), log=self.log, **{k: self.p[k] for k in DEFAULTS})
        self.fsm.on_state_change = lambda old, new: self.state_pub.publish(String(data=new))
        self.started = False
        self.search_pressed = False
        self.pose = None

        self.create_subscription(Gesture, '/perception/gestures',
                                 lambda m: self.started and self.fsm.on_gesture(m.command, self.now()), 10)
        self.create_subscription(VoiceCommand, '/perception/voice_cmds',
                                 lambda m: self.started and self.fsm.on_voice(m.command, self.now()), 10)
        self.create_subscription(Bool, '/perception/search_contact', self.on_search_button, 10)
        self.create_subscription(Bool, '/perception/bone_contact',
                                 lambda m: self.started and self.fsm.on_bone_contact(m.data, self.now()), 10)
        self.create_subscription(BoneDetectionArray, '/perception/bone_position', self.on_bones, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(CommandStatus, '/robot_command_status', self.on_status, 50)
        self.create_subscription(Empty, '/system/shutdown',
                                 lambda _: self.started and self.fsm.on_shutdown(self.now()), 10)
        self.create_timer(1.0 / self.p['rate'], self.step)
        self.get_logger().info(f'state_coordinator ready, waiting for command_coordinator; log: {path}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def log(self, text):
        self.get_logger().info(text)
        self.flog.info(text)

    # ------------------------------------------------------------------ inputs
    def on_search_button(self, msg):
        if msg.data and not self.search_pressed and self.started:   # rising edge only
            self.fsm.on_search_button(self.now())
        self.search_pressed = msg.data

    def on_bones(self, msg):
        if not self.started:
            return
        bottom = self.p['near_bottom_frac'] * self.p['image_height']
        bones = []
        for d in msg.detections:
            if d.class_id != BoneDetection.BONE or d.score < self.p['min_bone_score']:
                continue
            near = d.bbox[3] >= bottom or (
                d.has_position and math.hypot(d.position.x, d.position.y) <= self.p['near_distance'])
            bones.append({'bearing': d.bearing, 'score': d.score, 'near': near})
        bones.sort(key=lambda b: b['score'], reverse=True)
        self.fsm.on_bones(bones, self.now())

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_status(self, msg):
        if self.started:
            self.fsm.on_done(msg.id, msg.result == CommandStatus.SUCCEEDED, self.now())

    def step(self):
        if not self.started:
            # "System bereit" needs someone who executes the commands and reports back
            # (topic_debugger also subscribes to /robot_command, so look for the status publisher)
            if self.count_publishers('/robot_command_status') == 0 \
                    or self.cmd_pub.get_subscription_count() == 0:
                return
            self.started = True
            self.fsm.start(self.now())
        self.fsm.tick(self.now(), self.pose)

    def power_off(self):
        self.log('system shut down')
        cmd = self.p['poweroff_command']
        if cmd:
            import subprocess
            subprocess.Popen(cmd, shell=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StateCoordinator()
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
