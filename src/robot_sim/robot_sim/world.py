"""The simulated world, no ROS: field, true robot motion, wheel odometry with errors, bones,
bone contact and what the camera would see. Field geometry as in pren_ros_demo.

Coordinates: origin = centre of the Startfeld, +x into the search segment, the robot starts
facing the person (yaw = pi). The robot body is 0.5 x 0.2 m around base_link, the camera
sits 0.2 m in front of base_link.
"""
import math
import random

START_YAW = math.pi
START_BOX = (0.5, 0.2)
SEG_R_MIN, SEG_R_MAX = 1.0, 5.0
SEG_HALF_ANGLE = math.radians(15.0)
PERSON = (-1.5, 0.0)
INTERACT_RADIUS = 0.75
PYLONS = [(r * math.cos(SEG_HALF_ANGLE), s * r * math.sin(SEG_HALF_ANGLE))
          for r in (SEG_R_MIN, SEG_R_MAX) for s in (-1, 1)]
BONE_LEN = 0.21
ROBOT_LEN, ROBOT_WID = 0.5, 0.2


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Camera:
    """Pinhole model identical to bone_detector's (diagonal FOV, pitch 0, square pixels)."""

    def __init__(self, width=1280, height=720, dfov_deg=75.0, height_m=0.25, offset=0.2, range_max=3.5):
        self.w, self.h = width, height
        self.f = (math.hypot(width, height) / 2) / math.tan(math.radians(dfov_deg) / 2)
        self.cx, self.cy = width / 2, height / 2
        self.height_m, self.offset, self.range_max = height_m, offset, range_max
        self.half_hfov = math.atan(self.cx / self.f)

    def project(self, d, bearing, size):
        """Object on the floor at distance d / bearing -> bbox [x1, y1, x2, y2] or None if not visible."""
        if d <= 0.05 or abs(bearing) > self.half_hfov or d > self.range_max:
            return None
        u = self.cx - self.f * math.tan(bearing)
        v_floor = self.cy + self.f * self.height_m / d          # contact point with the floor
        if v_floor > self.h:                                    # below the image (too close)
            return None
        half_w = self.f * size[0] / d / 2
        top = self.cy + self.f * (self.height_m - size[1]) / d
        return [max(0.0, u - half_w), max(0.0, top), min(self.w, u + half_w), min(self.h, v_floor)]


class World:

    def __init__(self, seed=None, n_bones=1, camera=None):
        self.rng = random.Random(seed)
        self.camera = camera or Camera()
        # motion limits and odometry errors (as in the demo's loopback_sim)
        self.max_lin_acc, self.max_ang_acc = 0.8, 4.0
        self.wheel_scale_error = 0.03      # encoders over-report the distance by 3 %
        self.wheel_yaw_bias = 0.04         # rad heading error per metre driven
        self.turn_scale_error = 0.01       # 1 % error on turns
        self.n_bones = n_bones
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self, random_bones=True):
        self.x, self.y, self.yaw = 0.0, 0.0, START_YAW      # truth
        self.v = self.w = 0.0                               # actual velocities
        self.ox = self.oy = self.oyaw = 0.0                 # measured odometry, starts at 0 like the MCU
        self.trace, self.odom_trace = [], []
        if random_bones:
            self.bones = [self.random_bone() for _ in range(self.n_bones)]
        else:
            for b in self.bones:
                b['touched'] = False

    def random_bone(self):
        r = self.rng.uniform(SEG_R_MIN + 0.5, SEG_R_MAX - 0.3)
        a = self.rng.uniform(-SEG_HALF_ANGLE + 0.05, SEG_HALF_ANGLE - 0.05)
        return self.bone_at(r * math.cos(a), r * math.sin(a))

    def bone_at(self, x, y):
        return {'x': x, 'y': y, 'yaw': self.rng.uniform(-math.pi, math.pi), 'touched': False}

    # ------------------------------------------------------------------ motion
    def step(self, dt, cmd_v, cmd_w):
        """Advance the truth with acceleration limits and the measured odometry with errors."""
        dv = max(-self.max_lin_acc * dt, min(self.max_lin_acc * dt, cmd_v - self.v))
        dw = max(-self.max_ang_acc * dt, min(self.max_ang_acc * dt, cmd_w - self.w))
        self.v += dv
        self.w += dw
        self.yaw = wrap(self.yaw + self.w * dt)
        self.x += self.v * dt * math.cos(self.yaw)
        self.y += self.v * dt * math.sin(self.yaw)
        # what the wheels / gyro report
        ds = self.v * dt * (1 + self.wheel_scale_error)
        self.oyaw = wrap(self.oyaw + self.w * dt * (1 + self.turn_scale_error)
                         + self.wheel_yaw_bias * abs(self.v * dt))
        self.ox += ds * math.cos(self.oyaw)
        self.oy += ds * math.sin(self.oyaw)
        if not self.trace or math.hypot(self.x - self.trace[-1][0], self.y - self.trace[-1][1]) > 0.03:
            self.trace.append((round(self.x, 3), round(self.y, 3)))
            self.trace = self.trace[-1500:]

    def odom_in_field(self):
        """Measured odometry expressed in field coordinates (odom starts at the start pose)."""
        c, s = math.cos(START_YAW), math.sin(START_YAW)
        return (c * self.ox - s * self.oy, s * self.ox + c * self.oy, wrap(START_YAW + self.oyaw))

    # ------------------------------------------------------------------ sensing
    def camera_pose(self):
        return (self.x + self.camera.offset * math.cos(self.yaw),
                self.y + self.camera.offset * math.sin(self.yaw), self.yaw)

    def detections(self, noise=True):
        """What the camera sees: [{class_id, bearing, bbox, score, distance, x, y}] (bones and pylons)."""
        cx, cy, cyaw = self.camera_pose()
        out = []
        objects = [('bone', b['x'], b['y'], (BONE_LEN, 0.05)) for b in self.bones] + \
                  [('pylon', px, py, (0.15, 0.3)) for px, py in PYLONS]
        for cls, ox, oy, size in objects:
            d = math.hypot(ox - cx, oy - cy)
            bearing = wrap(math.atan2(oy - cy, ox - cx) - cyaw)
            bbox = self.camera.project(d, bearing, size)
            if bbox is None:
                continue
            p_detect = 0.97 - 0.45 * min(1.0, d / self.camera.range_max)   # far = fewer hits
            if noise and self.rng.random() > p_detect:
                continue
            if noise:
                bearing += self.rng.gauss(0.0, math.radians(1.0))
                jitter = self.rng.gauss(0.0, 3.0)
                bbox = [bbox[0] + jitter, bbox[1] + jitter, bbox[2] + jitter, min(self.camera.h, bbox[3] + jitter)]
            out.append({'class_id': cls, 'bearing': bearing, 'bbox': bbox, 'distance': d,
                        'score': round(p_detect * self.rng.uniform(0.85, 1.0), 2) if noise else 0.9,
                        'x': ox, 'y': oy})
        out.sort(key=lambda o: o['score'], reverse=True)
        return out

    def in_contact(self, margin=0.03):
        """Any bone point inside the robot footprint (+ margin) -> the bone button is pressed."""
        hx, hy = ROBOT_LEN / 2 + margin, ROBOT_WID / 2 + margin
        cyw, syw = math.cos(self.yaw), math.sin(self.yaw)
        hit = False
        for b in self.bones:
            c, s = math.cos(b['yaw']), math.sin(b['yaw'])
            for k in range(9):
                along = (k / 8 - 0.5) * BONE_LEN
                for side in (-0.025, 0.0, 0.025):
                    px = b['x'] + along * c - side * s - self.x
                    py = b['y'] + along * s + side * c - self.y
                    lx, ly = px * cyw + py * syw, -px * syw + py * cyw
                    if abs(lx) <= hx and abs(ly) <= hy:
                        b['touched'] = True
                        hit = True
                        break
                else:
                    continue
                break
        return hit
