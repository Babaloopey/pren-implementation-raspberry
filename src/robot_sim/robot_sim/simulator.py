"""simulator: the PC stand-in for everything that is hardware on the robot, plus the web GUI.

  * virtual MCU: executes /motor_command (watchdog, acceleration limits) and reports measured
    wheel odometry with errors on /mcu/odom, exactly the interface serial_uart will serve
  * world: field, bones, pylons, person; true robot pose; bone contact
  * replacements per node ("slot"), switchable in the GUI:
        gesture_detector        real (PC webcam)      | buttons
        voice_command_detector  real (PC microphone)  | buttons
        bone_detector           real (PC webcam)      | sim (camera model on the simulated field)
        bone button             sim (contact)         | button
        state_coordinator       real                  | manual (send single RobotCommands)
        command_coordinator     real                  | manual (drive the virtual MCU directly)
    "real" starts the unchanged robot node as its own process (`ros2 run`).
    "pi" (hardware-in-the-loop, profile:=hil): the node runs on the Raspberry Pi; the simulator
    neither starts nor replaces it, it only shows whether it is seen and its log (/rosout).
  * buttons for everything a human does: gestures, words, search button, shutdown
  * web GUI on http://localhost:<http_port>
"""
import collections
import json
import math
import os
import threading
import time

import rclpy
import rclpy.executors
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from robot_interfaces.msg import (BoneDetection, BoneDetectionArray, CommandStatus, Gesture,
                                  MotorCommand, RobotCommand, VoiceCommand)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty, Float32MultiArray, String

from .supervisor import ManagedProcess
from .web import WebServer
from .world import INTERACT_RADIUS, PERSON, PYLONS, SEG_HALF_ANGLE, SEG_R_MAX, SEG_R_MIN, START_BOX, \
    START_YAW, Camera, World, wrap

SLOTS = [
    dict(key='gesture_detector', label='Gestenerkennung', pkg='robot_vision', cfg='vision.yaml',
         modes=['real', 'buttons', 'pi'], default='buttons', bridge=True, params=['publish_debug_image:=true']),
    dict(key='voice_command_detector', label='Spracherkennung', pkg='robot_audio', cfg='voice.yaml',
         modes=['real', 'buttons', 'pi'], default='buttons', bridge=True, params=['publish_scores:=true']),
    dict(key='bone_detector', label='Knochenerkennung', pkg='robot_vision', cfg='vision.yaml',
         modes=['real', 'sim', 'pi'], default='sim', bridge=True, params=['publish_debug_image:=true']),
    dict(key='bone_button', label='Knochen-Taster', node='bone_button_driver', modes=['sim', 'button', 'pi'],
         default='sim'),
    dict(key='state_coordinator', label='state_coordinator', pkg='robot_core', cfg='core.yaml',
         modes=['real', 'manual', 'pi'], default='real', params=['log_dir:={log}']),
    dict(key='command_coordinator', label='command_coordinator', pkg='robot_core', cfg='core.yaml',
         modes=['real', 'manual', 'pi'], default='real', params=['odom_source:=mcu', 'log_dir:={log}']),
]
# hardware-in-the-loop: recognition and behaviour on the Pi; the robot body, its microcontroller
# and the field are simulated, so bones (detection + contact) come from the simulated field
HIL_MODES = {'gesture_detector': 'pi', 'voice_command_detector': 'pi', 'bone_detector': 'sim',
             'bone_button': 'sim', 'state_coordinator': 'pi', 'command_coordinator': 'pi'}
ACTIONS = {'STOP': RobotCommand.STOP, 'MOVE': RobotCommand.MOVE, 'TURN': RobotCommand.TURN,
           'VELOCITY': RobotCommand.VELOCITY, 'POSTURE': RobotCommand.POSTURE, 'HEAD': RobotCommand.HEAD,
           'BEHAVIOR': RobotCommand.BEHAVIOR, 'SIGNAL': RobotCommand.SIGNAL}
ACTION_NAMES = {v: k for k, v in ACTIONS.items()}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def odom_to_field(x, y, yaw):
    """Odometry frame (origin = start pose, x = initial heading) -> field coordinates."""
    c, s = math.cos(START_YAW), math.sin(START_YAW)
    return c * x - s * y, s * x + c * y, wrap(START_YAW + yaw)


class Simulator(Node):

    def __init__(self):
        super().__init__('simulator')
        params = self.declare_parameters('', [
            ('http_port', 8080),
            ('log_dir', '/ws/logs'),
            ('rate', 50.0),
            ('n_bones', 1),
            ('seed', -1),
            ('camera_height', 0.25),      # m, simulated camera mount (bone_detector does not know it)
            ('detection_rate', 5.0),      # Hz of the simulated bone_detector
            ('watchdog', 0.5),            # s without /motor_command -> virtual MCU stops
            ('profile', 'local'),         # local: everything on this PC | hil: robot nodes on the Pi
        ])
        self.p = {prm.name: prm.value for prm in params}
        self.lock = threading.RLock()
        self.world = World(seed=None if self.p['seed'] < 0 else self.p['seed'], n_bones=self.p['n_bones'],
                           camera=Camera(height_m=self.p['camera_height']))
        self.log_dir = os.path.expanduser(self.p['log_dir'])
        os.makedirs(self.log_dir, exist_ok=True)

        # --- virtual MCU
        self.motor = {'linear': 0.0, 'angular': 0.0, 'posture': 'STAND', 'head': 'UP', 'behavior': '',
                      't': 0.0}
        self.manual_motor = {'linear': 0.0, 'angular': 0.0, 'until': 0.0, 'posture': 'STAND', 'head': 'UP'}
        self.odom_pub = self.create_publisher(Odometry, '/mcu/odom', 20)
        self.motor_pub = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.create_subscription(MotorCommand, '/motor_command', self.on_motor, 20)

        # --- replacement publishers
        self.bone_pub = self.create_publisher(BoneDetectionArray, '/perception/bone_position', 10)
        self.pylon_pub = self.create_publisher(BoneDetectionArray, '/perception/pylon_position', 10)
        self.bone_contact_pub = self.create_publisher(Bool, '/perception/bone_contact', 10)
        self.search_pub = self.create_publisher(Bool, '/perception/search_contact', 10)
        self.gesture_pub = self.create_publisher(Gesture, '/perception/gestures', 10)
        self.voice_pub = self.create_publisher(VoiceCommand, '/perception/voice_cmds', 10)
        self.shutdown_pub = self.create_publisher(Empty, '/system/shutdown', 10)
        self.cmd_pub = self.create_publisher(RobotCommand, '/robot_command', 10)
        self.led_pub = self.create_publisher(String, '/actuation/led_cmds', 10)
        self.speaker_pub = self.create_publisher(String, '/actuation/speaker_cmds', 10)

        # --- observed state for the GUI
        self.events = collections.deque(maxlen=300)
        self.event_id = 0
        self.robot_state = '-'
        self.belief = None
        self.belief_trace = []
        self.led = 'off'
        self.buzzer = {'count': 0, 'last': ''}
        self.last_dets = []                 # [{bearing, class_id, t}] from /perception/bone_position
        self.contact = False
        self.manual_id = 100000             # ids of manual RobotCommands (far away from state_coordinator's)
        self.manual_status = {}
        self.images = {}                    # name -> (jpeg bytes, time)
        self.bridge_status = {}
        self.det_due = 0.0
        self.voice_scores = {'values': [], 'peaks': [], 't': 0.0}

        self.create_subscription(Odometry, '/odom', self.on_belief, 20)
        # latched like the publisher in state_coordinator: the current state arrives even when the
        # simulator connects later (e.g. to the Pi)
        self.create_subscription(String, '/robot_state', self.on_robot_state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(RobotCommand, '/robot_command', self.on_robot_command, 50)
        self.create_subscription(CommandStatus, '/robot_command_status', self.on_status, 50)
        self.create_subscription(Gesture, '/perception/gestures',
                                 lambda m: self.event('Geste', f'{m.command} ({m.confidence:.2f})'), 10)
        self.create_subscription(VoiceCommand, '/perception/voice_cmds',
                                 lambda m: self.event('Sprache', f'{m.command} ({m.confidence:.2f})'), 10)
        self.create_subscription(Bool, '/perception/search_contact',
                                 lambda m: self.event('Such-Taster', 'gedrückt' if m.data else 'losgelassen'), 10)
        self.create_subscription(Bool, '/perception/bone_contact',
                                 lambda m: self.event('Knochen-Taster', 'gedrückt' if m.data else 'losgelassen'), 10)
        self.create_subscription(Empty, '/system/shutdown', lambda m: self.event('System', 'ausschalten'), 10)
        self.create_subscription(BoneDetectionArray, '/perception/bone_position', self.on_bones, 10)
        self.create_subscription(String, '/actuation/led_cmds', self.on_led, 10)
        self.create_subscription(String, '/actuation/speaker_cmds', self.on_speaker, 10)
        self.create_subscription(String, '/sim/bridge_status', self.on_bridge, 10)
        self.create_subscription(Float32MultiArray, '/perception/voice_scores', self.on_voice_scores, 20)
        for name in ('gesture', 'bone'):
            self.create_subscription(Image, f'/perception/{name}_debug',
                                     lambda m, n=name: self.on_image(n, m), qos_profile_sensor_data)

        # --- slots / processes
        hil = self.p['profile'] == 'hil'
        self.slots = {s['key']: dict(s, mode=HIL_MODES[s['key']] if hil else s['default'], proc=None)
                      for s in SLOTS}
        self.rosout = collections.defaultdict(lambda: collections.deque(maxlen=300))   # node -> lines
        self.rosout_perf = {}
        self.seen_nodes = set()
        self.create_subscription(Log, '/rosout', self.on_rosout, 100)
        self.create_timer(1.0, self.scan_nodes)
        self.bridge = ManagedProcess('pc_bridge', ['ros2', 'run', 'robot_sim', 'pc_bridge'])
        self.apply_modes()

        self.dt = 1.0 / self.p['rate']
        self.create_timer(self.dt, self.step)
        self.create_timer(0.05, self.publish_manual_motor)
        self.web = WebServer(self, self.p['http_port'])
        self.web.start()
        self.event('Simulator', f'GUI auf http://localhost:{self.p["http_port"]}')
        self.get_logger().info(f'simulator ready: open http://localhost:{self.p["http_port"]}')

    # ================================================================== helpers
    def now(self):
        return time.monotonic()

    def event(self, kind, text):
        with self.lock:
            self.event_id += 1
            self.events.append({'id': self.event_id, 't': time.strftime('%H:%M:%S'), 'kind': kind, 'text': text})

    def on_node_line(self, key, line):
        """Show what the state machine ignores / decides directly in the GUI event list."""
        if key == 'state_coordinator' and any(w in line for w in ('ignored', 'bone found', 'bone lost',
                                                                  'bone touched', 'missed', 'over', 'aborted')):
            self.event('Automat', line.split('] ', 1)[-1])

    def process_for(self, slot):
        cmd = ['ros2', 'run', slot['pkg'], slot['key'], '--ros-args', '--params-file',
               os.path.join(get_package_share_directory(slot['pkg']), 'config', slot['cfg'])]
        for prm in slot.get('params', []):
            cmd += ['-p', prm.format(log=self.log_dir)]
        return ManagedProcess(slot['key'], cmd, on_line=lambda line, k=slot['key']: self.on_node_line(k, line))

    def apply_modes(self):
        """Start/stop processes so that they match the selected modes."""
        order = ['command_coordinator', 'state_coordinator', 'gesture_detector', 'bone_detector',
                 'voice_command_detector']
        needs_bridge = any(s['mode'] == 'real' and s.get('bridge') for s in self.slots.values())
        if needs_bridge:
            self.bridge.start()
        for key in order:
            s = self.slots[key]
            want = s['mode'] == 'real'
            if want and (s['proc'] is None or not s['proc'].running):
                s['proc'] = s['proc'] if s['proc'] is not None else self.process_for(s)
                s['proc'].start()
            elif not want and s['proc'] is not None and s['proc'].running:
                threading.Thread(target=s['proc'].stop, daemon=True).start()
        if not needs_bridge and self.bridge.running:
            threading.Thread(target=self.bridge.stop, daemon=True).start()

    # ================================================================== ROS callbacks
    def on_motor(self, msg):
        with self.lock:
            self.motor.update(linear=msg.linear, angular=msg.angular, posture=msg.posture or 'STAND',
                              head=msg.head or 'UP', behavior=msg.behavior, t=self.now())

    def on_belief(self, msg):
        p = msg.pose.pose
        x, y, yaw = odom_to_field(p.position.x, p.position.y, yaw_of(p.orientation))
        with self.lock:
            self.belief = (x, y, yaw)
            if not self.belief_trace or math.hypot(x - self.belief_trace[-1][0], y - self.belief_trace[-1][1]) > 0.03:
                self.belief_trace.append((round(x, 3), round(y, 3)))
                self.belief_trace = self.belief_trace[-1500:]

    def on_robot_state(self, msg):
        with self.lock:
            self.robot_state = msg.data
        self.event('Zustand', msg.data)

    def on_robot_command(self, msg):
        name = ACTION_NAMES.get(msg.action, str(msg.action))
        arg = {RobotCommand.MOVE: f'{msg.value:+.2f} m', RobotCommand.TURN: f'{math.degrees(msg.value):+.0f}°',
               RobotCommand.VELOCITY: f'v {msg.linear:+.2f} w {msg.angular:+.2f}'}.get(msg.action, msg.name)
        if msg.action != RobotCommand.VELOCITY:          # VELOCITY comes at 5 Hz, too chatty
            self.event('Befehl', f'#{msg.id} {name} {arg}')

    def on_status(self, msg):
        res = 'erledigt' if msg.result == CommandStatus.SUCCEEDED else 'abgebrochen'
        with self.lock:
            if msg.id >= 100000:
                self.manual_status[msg.id] = res
        if msg.id >= 100000:
            self.event('Status', f'#{msg.id} {res}')

    def on_bones(self, msg):
        t = self.now()
        with self.lock:
            self.last_dets = [{'bearing': d.bearing, 'class_id': d.class_id, 'score': d.score, 't': t}
                              for d in msg.detections]

    def on_led(self, msg):
        with self.lock:
            self.led = msg.data.strip().lower()

    def on_speaker(self, msg):
        with self.lock:
            self.buzzer = {'count': self.buzzer['count'] + 1, 'last': msg.data.strip().lower()}
        self.event('Buzzer', msg.data)

    def on_rosout(self, msg):
        name = msg.name.split('.')[-1]
        level = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}.get(msg.level, str(msg.level))
        with self.lock:
            self.rosout[name].append(f'[{level}] {msg.msg}')
            if msg.msg.startswith('perf:'):
                self.rosout_perf[name] = msg.msg
        slot = next((s for s in self.slots.values() if s.get('node', s['key']) == name), None)
        if slot is not None and slot['mode'] == 'pi':
            self.on_node_line(slot['key'], msg.msg)

    def scan_nodes(self):
        names = set(self.get_node_names())
        with self.lock:
            self.seen_nodes = names

    def on_voice_scores(self, msg):
        vals = list(msg.data)
        with self.lock:
            old = self.voice_scores['peaks']
            decay = [p * 0.97 for p in old] if len(old) == len(vals) else [0.0] * len(vals)
            self.voice_scores = {'values': vals, 'peaks': [max(a, b) for a, b in zip(decay, vals)],
                                 't': self.now()}

    def on_bridge(self, msg):
        try:
            with self.lock:
                self.bridge_status = json.loads(msg.data)
        except ValueError:
            pass

    def on_image(self, name, msg):
        last = self.images.get(name)
        if last and time.time() - last[1] < 0.2:          # max 5 Hz is enough for the preview
            return
        try:
            import cv2
            import numpy as np
            img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                img = img[:, :, ::-1]
            ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                self.images[name] = (jpg.tobytes(), time.time())
        except Exception as e:                                 # wrong encoding etc.
            self.get_logger().warn(f'debug image {name}: {e}', throttle_duration_sec=10.0)

    # ================================================================== simulation
    def step(self):
        t = self.now()
        with self.lock:
            m = self.motor
            alive = t - m['t'] < self.p['watchdog']
            v, w = (m['linear'], m['angular']) if alive else (0.0, 0.0)
            self.world.step(self.dt, v, w)
            ox, oy, oyaw = self.world.ox, self.world.oy, self.world.oyaw
            contact = self.world.in_contact()
        # measured wheel odometry, like the MCU will send it
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = ox, oy
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(oyaw / 2), math.cos(oyaw / 2)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = self.world.v, self.world.w
        self.odom_pub.publish(msg)

        if self.slots['bone_button']['mode'] == 'sim' and contact != self.contact:
            self.bone_contact_pub.publish(Bool(data=contact))
        self.contact = contact

        if self.slots['bone_detector']['mode'] == 'sim' and t >= self.det_due:
            self.det_due = t + 1.0 / self.p['detection_rate']
            self.publish_sim_detections()

    def publish_sim_detections(self):
        with self.lock:
            dets = self.world.detections()
        stamp = self.get_clock().now().to_msg()
        arrays = {'bone': BoneDetectionArray(), 'pylon': BoneDetectionArray()}
        for arr in arrays.values():
            arr.header.stamp, arr.header.frame_id = stamp, 'camera_link'
        for d in dets:
            bd = BoneDetection()
            bd.class_id = d['class_id']
            bd.score = float(d['score'])
            bd.bearing = float(d['bearing'])
            bd.bbox = [float(v) for v in d['bbox']]
            bd.has_position = False                       # like the real node without camera_height
            arrays[d['class_id']].detections.append(bd)
        self.bone_pub.publish(arrays['bone'])
        self.pylon_pub.publish(arrays['pylon'])

    def publish_manual_motor(self):
        if self.slots['command_coordinator']['mode'] != 'manual':
            return
        with self.lock:
            mm = dict(self.manual_motor)
        moving = self.now() < mm['until']                  # GUI keeps sending while a key is held
        msg = MotorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.linear = float(mm['linear']) if moving else 0.0
        msg.angular = float(mm['angular']) if moving else 0.0
        msg.posture, msg.head, msg.behavior = mm['posture'], mm['head'], ''
        self.motor_pub.publish(msg)

    # ================================================================== GUI API
    def snapshot(self):
        t = self.now()
        with self.lock:
            wd = self.world
            cam = wd.camera
            return {
                'robot': {'x': wd.x, 'y': wd.y, 'yaw': wd.yaw, 'v': wd.v, 'w': wd.w},
                'mcu_odom': dict(zip(('x', 'y', 'yaw'), odom_to_field(wd.ox, wd.oy, wd.oyaw))),
                'belief': None if self.belief is None else dict(zip(('x', 'y', 'yaw'), self.belief)),
                'trace': wd.trace, 'belief_trace': self.belief_trace,
                'bones': wd.bones, 'pylons': PYLONS, 'person': PERSON,
                'field': {'start_box': START_BOX, 'seg_r': [SEG_R_MIN, SEG_R_MAX],
                          'seg_half_angle': SEG_HALF_ANGLE, 'interact_radius': INTERACT_RADIUS},
                'camera': {'offset': cam.offset, 'half_hfov': cam.half_hfov, 'range': cam.range_max},
                'contact': self.contact,
                'motor': {k: self.motor[k] for k in ('linear', 'angular', 'posture', 'head', 'behavior')},
                'motor_alive': t - self.motor['t'] < self.p['watchdog'],
                'state': self.robot_state, 'led': self.led, 'buzzer': self.buzzer,
                'detections': [d for d in self.last_dets if t - d['t'] < 0.6],
                'slots': [{'key': k, 'label': s['label'], 'mode': s['mode'], 'modes': s['modes'],
                           'running': bool(s['proc'] and s['proc'].running),
                           'exit': None if s['proc'] is None else s['proc'].exit_code,
                           'perf': (self.rosout_perf.get(s.get('node', k), '') if s['mode'] == 'pi'
                                    else s['proc'].perf if s['proc'] else ''),
                           'seen': s.get('node', k) in self.seen_nodes,
                           'process': 'pkg' in s} for k, s in self.slots.items()],
                'profile': self.p['profile'],
                'network_nodes': sorted(self.seen_nodes - self.local_nodes()),
                'bridge': {'running': self.bridge.running, **self.bridge_status},
                'events': list(self.events)[-120:],
                'manual_status': {str(k): v for k, v in list(self.manual_status.items())[-30:]},
                'images': {k: round(time.time() - v[1], 1) for k, v in self.images.items()},
                'voice_scores': self.voice_scores if t - self.voice_scores['t'] < 2.0 else None,
            }

    def local_nodes(self):
        """Nodes this PC runs itself (simulator, its own processes, ros2 CLI helpers)."""
        own = {'simulator', 'pc_bridge'}
        own |= {s.get('node', k) for k, s in self.slots.items() if s['proc'] is not None and s['proc'].running}
        return own | {n for n in self.seen_nodes if n.startswith('_')}

    def logs(self, key):
        if key == 'pc_bridge':
            return self.bridge.tail(200)
        s = self.slots.get(key)
        if s and s['mode'] == 'pi':
            with self.lock:
                lines = list(self.rosout.get(s.get('node', key), []))
            return ['(Log vom Raspberry Pi, über /rosout)'] + lines[-200:]
        return s['proc'].tail(200) if s and s['proc'] else []

    def image(self, name):
        v = self.images.get(name)
        return v[0] if v else None

    def action(self, a):
        kind = a.get('type')
        if kind == 'mode':
            s = self.slots[a['slot']]
            if a['mode'] in s['modes']:
                s['mode'] = a['mode']
                self.event('Schalter', f'{s["label"]}: {a["mode"]}')
                if a['slot'] == 'bone_button' and a['mode'] == 'sim':
                    self.contact = None                     # publish the current contact state again
                self.apply_modes()
        elif kind == 'gesture':
            m = Gesture()
            m.header.stamp = self.get_clock().now().to_msg()
            m.command, m.confidence = a['command'], 1.0
            self.gesture_pub.publish(m)
        elif kind == 'voice':
            m = VoiceCommand()
            m.header.stamp = self.get_clock().now().to_msg()
            m.command, m.confidence = a['command'], 1.0
            self.voice_pub.publish(m)
        elif kind == 'search_button':
            self.search_pub.publish(Bool(data=bool(a['pressed'])))
        elif kind == 'bone_button':
            self.bone_contact_pub.publish(Bool(data=bool(a['pressed'])))
        elif kind == 'shutdown':
            self.shutdown_pub.publish(Empty())
        elif kind == 'robot_command':
            m = RobotCommand()
            m.header.stamp = self.get_clock().now().to_msg()
            with self.lock:
                self.manual_id += 1
                m.id = self.manual_id
            m.action = ACTIONS[a['action']]
            m.value = float(a.get('value', 0.0))
            m.linear, m.angular = float(a.get('linear', 0.0)), float(a.get('angular', 0.0))
            m.name = a.get('name', '')
            self.cmd_pub.publish(m)
            return {'id': m.id}
        elif kind == 'motor':
            with self.lock:
                mm = self.manual_motor
                for k in ('linear', 'angular', 'posture', 'head'):
                    if k in a:
                        mm[k] = a[k]
                mm['until'] = self.now() + float(a.get('hold', 0.35))
        elif kind == 'actuation':
            (self.led_pub if a['target'] == 'led' else self.speaker_pub).publish(String(data=a['data']))
        elif kind == 'place_bone':
            with self.lock:
                b = self.world.bone_at(float(a['x']), float(a['y']))
                if a.get('add'):
                    self.world.bones.append(b)
                else:
                    self.world.bones = [b]
            self.event('Welt', f'Knochen bei ({a["x"]:.2f}, {a["y"]:.2f})')
        elif kind == 'random_bone':
            with self.lock:
                self.world.bones = [self.world.random_bone()]
        elif kind == 'clear_bones':
            with self.lock:
                self.world.bones = []
        elif kind == 'reset':
            self.reset(restart=a.get('restart', True))
        else:
            return {'error': f'unknown action {kind}'}
        return {'ok': True}

    def reset(self, restart=True):
        """Robot back to the start; restart the core nodes so their state and odometry start fresh."""
        procs = [s['proc'] for k, s in self.slots.items()
                 if restart and k in ('state_coordinator', 'command_coordinator') and s['proc'] and s['proc'].running]
        for p in procs:
            p.stop()
        with self.lock:
            self.world.reset(random_bones=False)
            self.motor.update(linear=0.0, angular=0.0, t=0.0)
            self.belief, self.belief_trace, self.robot_state = None, [], '-'
            self.led = 'off'
        self.event('Welt', 'Neustart: Roboter im Startfeld' + (', Kern-Nodes neu gestartet' if procs else ''))
        if any(s['mode'] == 'pi' for k, s in self.slots.items() if k in ('state_coordinator', 'command_coordinator')):
            self.event('Welt', 'state_/command_coordinator laufen auf dem Pi weiter: für einen sauberen Start '
                               'dort tools/pi_hil.sh neu starten')
        self.apply_modes()

    def destroy_node(self):
        self.web.stop()
        for s in self.slots.values():
            if s['proc'] is not None:
                s['proc'].stop()
        self.bridge.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Simulator()
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
