"""Gesture decision logic from PythonProject/poc.py, without UI and without ROS.

Inputs per frame: MediaPipe GestureRecognizer result + MediaPipe PoseLandmarker result.
Output: a debounced command SITZ / PLATZ / AUF / TRICK or NONE.

  SITZ  : MediaPipe "Open_Palm" in front of the chest (inside the torso box from the pose)
  TRICK : MediaPipe "Thumb_Up" (Kunststueck)
  AUF   : MediaPipe "Pointing_Up", or index finger / "L" (thumb + index) points up (geometry)
  PLATZ : index finger points to the floor, or arm hangs down + loosely pointing index.
          MediaPipe has no "pointing down" class, so PLATZ comes from the hand landmarks
          (geometry), exactly as in poc.py.

Thresholds are identical to poc.py; all of them can be overridden via the constructor.
poc.py used YOLO26-pose (COCO keypoints); here MediaPipe Pose is mapped onto the same
COCO indices (mediapipe_persons), so the torso/arm logic is unchanged.
"""
from collections import Counter, deque

import numpy as np

SITZ, PLATZ, AUF, TRICK = 'SITZ', 'PLATZ', 'AUF', 'TRICK'
NONE = 'NONE'

# MediaPipe Pose landmark index -> COCO-17 keypoint index (what poc.py's logic uses)
MP_TO_COCO = {0: 0, 2: 1, 5: 2, 7: 3, 8: 4, 11: 5, 12: 6, 13: 7, 14: 8, 15: 9, 16: 10,
              23: 11, 24: 12, 25: 13, 26: 14, 27: 15, 28: 16}


def mediapipe_persons(pose_result, w, h):
    """PoseLandmarkerResult -> list of (17, 3) arrays (x px, y px, visibility) in COCO order."""
    persons = []
    for lms in (pose_result.pose_landmarks if pose_result is not None else []) or []:
        k = np.zeros((17, 3))
        for mp_i, coco_i in MP_TO_COCO.items():
            lm = lms[mp_i]
            vis = lm.visibility if lm.visibility is not None else 1.0
            k[coco_i] = (lm.x * w, lm.y * h, vis)
        persons.append(k)
    return persons

FINGERS = {
    'index': (5, 6, 7, 8),
    'middle': (9, 10, 11, 12),
    'ring': (13, 14, 15, 16),
    'pinky': (17, 18, 19, 20),
}

DEFAULTS = dict(
    kp_conf=0.5,                 # min. confidence of shoulder/hip keypoints
    torso_margin=0.25,           # widen torso box sideways by x * shoulder width
    ext_on=0.88, ext_off=0.78,   # finger extended (hysteresis)
    curl_on=0.62, curl_off=0.72,  # finger curled (hysteresis)
    point_others_max=0.74,
    point_others_min_count=2,
    down_max_deg=70.0,           # PLATZ: max. deviation from "down"
    up_max_deg=60.0,             # AUF: max. deviation from "up"
    min_2d_len=0.35,
    arm_down_max_deg=65.0,       # shoulder->wrist max. deviation from "down"
    arm_match_frac=0.8,          # hand <-> wrist keypoint: max. distance (x shoulder width)
    arm_wrist_conf=0.3,
    point_index_loose=0.80,
    ema_alpha=0.5,
    dom_min=1.35,
    open_palm_min=0.5,
    pointing_up_min=0.5,
    thumb_up_min=0.6,            # TRICK
    history_length=10,
    enter_votes=6,
    keep_votes=4,
)


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-6 else v


def straightness(pts, ids):
    mcp, pip, dip, tip = (pts[i] for i in ids)
    path = np.linalg.norm(pip - mcp) + np.linalg.norm(dip - pip) + np.linalg.norm(tip - dip)
    return np.linalg.norm(tip - mcp) / max(path, 1e-6)


class GestureLogic:

    def __init__(self, **overrides):
        c = dict(DEFAULTS)
        c.update({k: v for k, v in overrides.items() if k in DEFAULTS})
        self.c = c
        self.cos_down = np.cos(np.radians(c['down_max_deg']))
        self.cos_up = np.cos(np.radians(c['up_max_deg']))
        self.arm_down_cos = np.cos(np.radians(c['arm_down_max_deg']))
        self.reset()

    def reset(self):
        self.history = deque(maxlen=self.c['history_length'])
        self.stable = NONE
        self.sm_worlds, self.sm_imgs, self.states = {}, {}, {}
        self.torso, self.arms = None, []

    # ------------------------------------------------------------------ pose
    def _largest_person(self, persons):
        """Person in front of the robot = largest extent of the visible keypoints."""
        best, best_area = None, -1.0
        for k in persons:
            vis = k[k[:, 2] >= self.c['arm_wrist_conf'], :2]
            if len(vis) < 2:
                continue
            area = np.prod(vis.max(axis=0) - vis.min(axis=0))
            if area > best_area:
                best, best_area = k, area
        return best

    def update_pose(self, persons, w, h):
        """persons: list of (17, 3) keypoint arrays in COCO order (see mediapipe_persons)."""
        p = self._largest_person(persons)
        self.torso = self._torso_box(p, w, h)
        self.arms = self._arm_segments(p)

    def _torso_box(self, p, w, h):
        if p is None or len(p) < 13:
            return None
        xy, conf = p[:, :2], p[:, 2]
        kc = self.c['kp_conf']
        if conf[5] < kc or conf[6] < kc:
            return None
        ls, rs = xy[5], xy[6]
        sh_w = abs(ls[0] - rs[0])
        sh_y = (ls[1] + rs[1]) / 2
        x1 = min(ls[0], rs[0]) - self.c['torso_margin'] * sh_w
        x2 = max(ls[0], rs[0]) + self.c['torso_margin'] * sh_w
        y1 = sh_y - 0.3 * sh_w
        if conf[11] >= kc and conf[12] >= kc:
            y2 = (xy[11][1] + xy[12][1]) / 2
        else:                      # hips not visible -> estimate
            y2 = sh_y + 1.6 * sh_w
        return max(0, x1), max(0, y1), min(w, x2), min(h, y2)

    def _arm_segments(self, p):
        if p is None or len(p) < 11:
            return []
        xy, conf = p[:, :2], p[:, 2]
        if conf[5] < self.c['kp_conf'] or conf[6] < self.c['kp_conf']:
            return []
        sh_w = abs(xy[5][0] - xy[6][0])
        return [(xy[s], xy[wr], sh_w) for s, wr in ((5, 9), (6, 10))
                if conf[wr] >= self.c['arm_wrist_conf']]

    def _arm_down_value(self, palm):
        best, best_d = None, 1e9
        for sh, wr, sh_w in self.arms:
            d = np.linalg.norm(wr - palm)
            if d < best_d and d < self.c['arm_match_frac'] * max(sh_w, 1.0):
                best, best_d = (sh, wr), d
        if best is None:
            return None
        return float(unit(best[1] - best[0])[1])

    # ------------------------------------------------------------------ hands
    def _update_state(self, prev, s):
        c = self.c
        if prev == 'ext' and s >= c['ext_off']:
            return 'ext'
        if prev == 'curl' and s <= c['curl_off']:
            return 'curl'
        if s > c['ext_on']:
            return 'ext'
        if s < c['curl_on']:
            return 'curl'
        return 'mid'

    def _vertical(self, world, img, a, b, w, h):
        """y component (image: +y = down) of a->b. 2D preferred, else 3D."""
        ref_len = np.linalg.norm(np.array([(img[9][0] - img[0][0]) * w, (img[9][1] - img[0][1]) * h]))
        v2 = np.array([(img[b][0] - img[a][0]) * w, (img[b][1] - img[a][1]) * h])
        if ref_len > 1e-6 and np.linalg.norm(v2) > self.c['min_2d_len'] * ref_len:
            return unit(v2)[1]
        return unit(world[b] - world[a])[1]

    def update_hands(self, result, w, h):
        """result: mediapipe GestureRecognizerResult. Returns (raw command, per-hand debug info)."""
        c = self.c
        current = NONE
        seen, hands = set(), []
        if result is not None and result.hand_landmarks and result.hand_world_landmarks:
            for i, (hand_lms, world_lms) in enumerate(
                    zip(result.hand_landmarks, result.hand_world_landmarks)):
                label = (result.handedness[i][0].category_name
                         if result.handedness and i < len(result.handedness) else str(i))
                key = label if label not in seen else f'{label}_2'
                seen.add(key)

                g_name, g_score = '', 0.0
                if result.gestures and i < len(result.gestures) and result.gestures[i]:
                    g_name = result.gestures[i][0].category_name
                    g_score = result.gestures[i][0].score

                world = np.array([[p.x, p.y, p.z] for p in world_lms])
                img = np.array([[p.x, p.y] for p in hand_lms])
                if key not in self.sm_worlds:
                    self.sm_worlds[key], self.sm_imgs[key] = world, img
                    self.states[key] = {n: 'mid' for n in FINGERS}
                else:
                    a = c['ema_alpha']
                    self.sm_worlds[key] = a * world + (1 - a) * self.sm_worlds[key]
                    self.sm_imgs[key] = a * img + (1 - a) * self.sm_imgs[key]

                w3d = self.sm_worlds[key]
                values = {}
                for name, ids in FINGERS.items():
                    values[name] = straightness(w3d, ids)
                    self.states[key][name] = self._update_state(self.states[key][name], values[name])

                idx_v = self._vertical(w3d, self.sm_imgs[key], 5, 8, w, h)
                others = ('middle', 'ring', 'pinky')
                n_curled = sum(values[n] < c['point_others_max'] for n in others)
                n_ext = sum(self.states[key][n] == 'ext' for n in others)
                pointing = (self.states[key]['index'] == 'ext'
                            and n_curled >= c['point_others_min_count'] and n_ext == 0)

                palm = np.mean([[hand_lms[j].x * w, hand_lms[j].y * h] for j in (0, 5, 9, 13, 17)], axis=0)
                t = self.torso
                in_torso = t is not None and t[0] <= palm[0] <= t[2] and t[1] <= palm[1] <= t[3]

                arm_v = self._arm_down_value(palm)
                arm_down = arm_v is not None and arm_v > self.arm_down_cos
                palm_len = max(np.linalg.norm(w3d[9] - w3d[0]), 1e-6)
                reach_idx = np.linalg.norm(w3d[8] - w3d[0]) / palm_len
                reach_oth = max(np.linalg.norm(w3d[k] - w3d[0]) / palm_len for k in (12, 16, 20))
                dominance = reach_idx / max(reach_oth, 1e-6)
                pointing_loose = values['index'] >= c['point_index_loose'] and dominance >= c['dom_min']
                finger_down = pointing and idx_v > self.cos_down
                arm_pointing_down = pointing_loose and arm_down and idx_v > 0.0

                # priority per hand: SITZ > TRICK > AUF > PLATZ
                cmd = NONE
                if g_name == 'Open_Palm' and g_score >= c['open_palm_min'] and in_torso:
                    cmd = SITZ
                elif g_name == 'Thumb_Up' and g_score >= c['thumb_up_min']:
                    cmd = TRICK
                elif (g_name == 'Pointing_Up' and g_score >= c['pointing_up_min']) or (
                        pointing and idx_v < -self.cos_up):      # also covers the "L" pose
                    cmd = AUF
                elif finger_down or arm_pointing_down:
                    cmd = PLATZ

                if cmd != NONE:
                    current = cmd
                hands.append(dict(key=key, cmd=cmd, palm=palm, gesture=g_name, score=g_score,
                                  landmarks=img))

        for k in [k for k in self.sm_worlds if k not in seen]:   # forget vanished hands
            self.sm_worlds.pop(k), self.sm_imgs.pop(k), self.states.pop(k)
        return current, hands

    # ------------------------------------------------------------------ debounce
    def stabilize(self, current):
        """Majority vote over the last frames (same rule as poc.py). Returns the stable command."""
        c = self.c
        self.history.append(current)
        if len(self.history) < c['history_length']:
            return self.stable
        counts = Counter(self.history)
        candidate, votes = counts.most_common(1)[0]
        if candidate != self.stable and candidate != NONE and votes >= c['enter_votes']:
            self.stable = candidate
        elif counts[self.stable] < c['keep_votes']:
            self.stable = candidate if (candidate != NONE and votes >= c['enter_votes']) else NONE
        return self.stable

    def vote_share(self):
        """Fraction of the history that agrees with the stable command (used as confidence)."""
        if not self.history or self.stable == NONE:
            return 0.0
        return Counter(self.history)[self.stable] / len(self.history)
