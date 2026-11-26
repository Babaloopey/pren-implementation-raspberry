"""command_coordinator: executes the RobotCommands of state_coordinator.

For now there is no microcontroller connection (serial_uart), so this node
  * writes every motor command to a log file (what serial_uart will later send over UART),
  * drives the buzzer and LED through the existing robot_gpio drivers ("Ton + LED", "Bellen"),
  * estimates when a command is finished (distance / speed, angle / turn rate, fixed times
    for posture, head and behaviours) and reports it on /robot_command_status,
  * integrates the commanded velocities into a pose estimate on /odom (dead reckoning from
    commands, NOT measured). Once the microcontroller sends encoder/IMU data, /odom has to come
    from there and the completion of MOVE/TURN from the measured pose.

Subscribes:
  /robot_command          robot_interfaces/RobotCommand
Publishes:
  /robot_command_status   robot_interfaces/CommandStatus
  /odom                   nav_msgs/Odometry   (frame odom -> base_link, origin = pose at start)
  /actuation/speaker_cmds std_msgs/String     beep / double_beep
  /actuation/led_cmds     std_msgs/String     on / blink / off
Log file:
  <log_dir>/commands_<date>.log
      RECV  #12 MOVE 0.300            command received from state_coordinator
      MOTOR v=+0.200 w=+0.000         motor command (later: UART to the microcontroller)
      MOTOR POSTURE SIT / HEAD DOWN / BEHAVIOR BRAV
      SIGNAL ACK -> speaker beep, led on
      DONE  #12 MOVE (1.50 s)         / ABORT #12 MOVE (replaced by #13)
"""
import math

import rclpy
import rclpy.executors
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_interfaces.msg import CommandStatus, RobotCommand
from std_msgs.msg import String

from .filelog import file_logger

NAMES = {RobotCommand.STOP: 'STOP', RobotCommand.MOVE: 'MOVE', RobotCommand.TURN: 'TURN',
         RobotCommand.VELOCITY: 'VELOCITY', RobotCommand.POSTURE: 'POSTURE',
         RobotCommand.HEAD: 'HEAD', RobotCommand.BEHAVIOR: 'BEHAVIOR', RobotCommand.SIGNAL: 'SIGNAL'}
MOTION = (RobotCommand.STOP, RobotCommand.MOVE, RobotCommand.TURN, RobotCommand.VELOCITY)


class CommandCoordinator(Node):

    def __init__(self):
        super().__init__('command_coordinator')
        params = self.declare_parameters('', [
            ('rate', 20.0),
            ('log_dir', '~/pren_logs'),
            ('max_linear', 0.20),          # m/s for MOVE
            ('max_angular', 1.0),          # rad/s for TURN
            ('posture_duration', 2.0),     # s, "Zielposition erreicht"
            ('head_duration', 1.0),        # s
            ('brav_duration', 5.0),        # s, "Brav beendet"
            ('trick_duration', 8.0),       # s, "Kunststueck beendet"
            ('led_on_duration', 1.5),      # s the LED stays on / blinks for a signal
            ('publish_odom', True),        # dead reckoning from commands (until the MCU sends odometry)
        ])
        self.p = {prm.name: prm.value for prm in params}
        self.flog, path = file_logger('commands', self.p['log_dir'], 'commands')

        self.status_pub = self.create_publisher(CommandStatus, '/robot_command_status', 50)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.speaker_pub = self.create_publisher(String, '/actuation/speaker_cmds', 10)
        self.led_pub = self.create_publisher(String, '/actuation/led_cmds', 10)
        self.create_subscription(RobotCommand, '/robot_command', self.on_command, 50)

        self.v = self.w = 0.0                  # current motor command
        self.motion = None                     # {'id', 'name', 'left', 'kind', 't0'} for MOVE / TURN
        self.jobs = {}                         # action -> {'id', 'name', 'until', 't0'} (POSTURE/HEAD/BEHAVIOR)
        self.led_off_at = None
        self.x = self.y = self.yaw = 0.0
        self.last_tick = self.now()
        self.create_timer(1.0 / self.p['rate'], self.step)
        self.get_logger().info(f'command_coordinator ready (no MCU: motor commands -> {path})')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def log(self, text):
        self.flog.info(text)
        self.get_logger().debug(text)

    # ------------------------------------------------------------------ commands
    def on_command(self, msg):
        a, name = msg.action, NAMES.get(msg.action, str(msg.action))
        arg = {RobotCommand.MOVE: f'{msg.value:.3f}', RobotCommand.TURN: f'{msg.value:.3f}',
               RobotCommand.VELOCITY: f'{msg.linear:+.3f} {msg.angular:+.3f}'}.get(a, msg.name)
        self.log(f'RECV  #{msg.id} {name} {arg}'.rstrip())

        if a in MOTION:
            if self.motion is not None:          # a new motion command replaces the running one
                self.finish(self.motion, CommandStatus.ABORTED, f'replaced by #{msg.id}')
                self.motion = None
            if a == RobotCommand.STOP:
                self.set_motor(0.0, 0.0)
                self.report(msg.id, CommandStatus.SUCCEEDED)
            elif a == RobotCommand.VELOCITY:
                self.set_motor(msg.linear, msg.angular)
                self.report(msg.id, CommandStatus.SUCCEEDED)
            else:
                kind = 'move' if a == RobotCommand.MOVE else 'turn'
                left = abs(msg.value)
                if left < 1e-4:
                    self.set_motor(0.0, 0.0)
                    self.report(msg.id, CommandStatus.SUCCEEDED)
                    return
                sign = math.copysign(1.0, msg.value)
                self.motion = {'id': msg.id, 'name': f'{name} {msg.value:+.3f}', 'kind': kind,
                               'left': left, 't0': self.now()}
                if kind == 'move':
                    self.set_motor(sign * self.p['max_linear'], 0.0)
                else:
                    self.set_motor(0.0, sign * self.p['max_angular'])

        elif a in (RobotCommand.POSTURE, RobotCommand.HEAD, RobotCommand.BEHAVIOR):
            if a in self.jobs:
                self.finish(self.jobs.pop(a), CommandStatus.ABORTED, f'replaced by #{msg.id}')
            duration = {RobotCommand.POSTURE: self.p['posture_duration'],
                        RobotCommand.HEAD: self.p['head_duration'],
                        RobotCommand.BEHAVIOR: self.p['trick_duration'] if msg.name == RobotCommand.TRICK
                        else self.p['brav_duration']}[a]
            self.log(f'MOTOR {name} {msg.name}')
            t = self.now()
            self.jobs[a] = {'id': msg.id, 'name': f'{name} {msg.name}', 'until': t + duration, 't0': t}

        elif a == RobotCommand.SIGNAL:
            if msg.name == RobotCommand.BARK:
                speaker, led = 'double_beep', 'blink'
            else:
                speaker, led = 'beep', 'on'
            self.speaker_pub.publish(String(data=speaker))
            self.led_pub.publish(String(data=led))
            self.led_off_at = self.now() + self.p['led_on_duration']
            self.log(f'SIGNAL {msg.name} -> speaker {speaker}, led {led}')
            self.report(msg.id, CommandStatus.SUCCEEDED)
        else:
            self.get_logger().warn(f'unknown action {a}')

    def set_motor(self, v, w):
        if (round(v, 3), round(w, 3)) != (round(self.v, 3), round(self.w, 3)):
            self.log(f'MOTOR v={v:+.3f} w={w:+.3f}')
        self.v, self.w = v, w

    def finish(self, job, result, why=''):
        word = 'DONE ' if result == CommandStatus.SUCCEEDED else 'ABORT'
        extra = f'({self.now() - job["t0"]:.2f} s)' if result == CommandStatus.SUCCEEDED else f'({why})'
        self.log(f'{word} #{job["id"]} {job["name"]} {extra}')
        self.report(job['id'], result)

    def report(self, cmd_id, result):
        msg = CommandStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id, msg.result = cmd_id, result
        self.status_pub.publish(msg)

    # ------------------------------------------------------------------ execution
    def step(self):
        t = self.now()
        dt, self.last_tick = max(0.0, t - self.last_tick), t

        # dead reckoning from the commanded velocity
        self.yaw += self.w * dt
        self.x += self.v * dt * math.cos(self.yaw)
        self.y += self.v * dt * math.sin(self.yaw)

        if self.motion is not None:
            m = self.motion
            m['left'] -= abs(self.v if m['kind'] == 'move' else self.w) * dt
            if m['left'] <= 0.0:
                self.set_motor(0.0, 0.0)
                self.motion = None
                self.finish(m, CommandStatus.SUCCEEDED)

        for a in [a for a, j in self.jobs.items() if t >= j['until']]:
            self.finish(self.jobs.pop(a), CommandStatus.SUCCEEDED)

        if self.led_off_at is not None and t >= self.led_off_at:
            self.led_pub.publish(String(data='off'))
            self.led_off_at = None

        if self.p['publish_odom']:
            self.publish_odom()

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(msg)

    def destroy_node(self):
        self.set_motor(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CommandCoordinator()
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
