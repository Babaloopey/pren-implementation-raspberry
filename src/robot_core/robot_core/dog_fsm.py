"""Behaviour state machine of the dog, exactly as in the team's state diagrams. No ROS here.

Top level (diagram "Hunde-Roboter"):
  Startzustand --System bereit--> Grundpositionen.Stehen
  Grundpositionen --voice HIER--> Vorwaerts 30 cm -> Drehen 180 -> Vorwaerts 30 cm -> Drehen 180 -> Stehen
  Grundpositionen --voice SUCH / search button--> Suche --done / 2 min--> Sitzen
  Stehen --voice BRAV--> Brav machen --done / 1 min--> Stehen
  Liegen --Kunststueck (gesture TRICK = thumbs up)--> Kunststueck machen --done / 1 min--> Stehen
  Sitzen --System ausschalten--> Ausschalten --> end

Grundpositionen (diagram "Grundpositionen"):
  Stehen --SITZ (gesture/voice)--> Sitzposition einnehmen --> Sitzen
  Stehen --PLATZ (gesture)--> Liegeposition einnehmen --> Liegen
  Sitzen --AUF (gesture)--> Stehposition einnehmen --> Stehen
  Sitzen --PLATZ (gesture)--> Liegeposition einnehmen --> Liegen
  Liegen --AUF (gesture)--> Stehposition einnehmen --> Stehen
  Liegen --SITZ (gesture/voice)--> Sitzposition einnehmen --> Sitzen

Suche (diagram "Such-Vorgang"):
  Drehen 180 -> Scannen --found--> Drehen (align) -> Vorwaerts --bone near--> Kopf senken
  -> Vorwaerts --bone touched--> Bellen -> Drehung (to start) -> Vorwaerts --at start-->
  Sitzposition einnehmen (exit / bellen) -> end
  Scannen --not found--> Vorwaerts 1 m -> Scannen

Every "entry / Ton + LED" is a SIGNAL ACK, "Bellen" a SIGNAL BARK.

Additions that the diagrams leave open (all marked "added:" below):
  * moving (HIER, SUCH) from Sitzen/Liegen first takes the standing posture
  * the bone is lost for `lost_timeout` s while aligning/approaching -> back to Scannen
  * the last forward step finds no contact within `touch_max_distance` -> head up, Scannen
  * the head goes up again when barking after the contact
  * no more 1 m steps beyond `max_search_distance` from the start (the segment ends at 5 m)

The class talks to the robot through `robot` (see RobotPort) and is driven by the node:
events (on_gesture, on_voice, on_search_button, on_bone_contact, on_bones, on_done,
on_shutdown) and tick(now, pose) at a fixed rate.
"""
import math

# --------------------------------------------------------------------------- states
STARTZUSTAND = 'STARTZUSTAND'
STEHEN = 'STEHEN'
SITZEN = 'SITZEN'
LIEGEN = 'LIEGEN'
SITZPOSITION_EINNEHMEN = 'SITZPOSITION_EINNEHMEN'
STEHPOSITION_EINNEHMEN = 'STEHPOSITION_EINNEHMEN'
LIEGEPOSITION_EINNEHMEN = 'LIEGEPOSITION_EINNEHMEN'
HIER_VORWAERTS_1 = 'HIER_VORWAERTS_1'
HIER_DREHEN_1 = 'HIER_DREHEN_1'
HIER_VORWAERTS_2 = 'HIER_VORWAERTS_2'
HIER_DREHEN_2 = 'HIER_DREHEN_2'
BRAV_MACHEN = 'BRAV_MACHEN'
KUNSTSTUECK_MACHEN = 'KUNSTSTUECK_MACHEN'
SUCHE_DREHEN = 'SUCHE_DREHEN'
SUCHE_SCANNEN = 'SUCHE_SCANNEN'
SUCHE_VORWAERTS_1M = 'SUCHE_VORWAERTS_1M'
SUCHE_AUSRICHTEN = 'SUCHE_AUSRICHTEN'
SUCHE_ANFAHREN = 'SUCHE_ANFAHREN'
SUCHE_KOPF_SENKEN = 'SUCHE_KOPF_SENKEN'
SUCHE_BERUEHREN = 'SUCHE_BERUEHREN'
SUCHE_BELLEN = 'SUCHE_BELLEN'
SUCHE_DREHUNG_START = 'SUCHE_DREHUNG_START'
SUCHE_RUECKWEG = 'SUCHE_RUECKWEG'
SUCHE_SITZPOSITION = 'SUCHE_SITZPOSITION'
SUCHE_ABBRUCH = 'SUCHE_ABBRUCH'
AUSSCHALTEN = 'AUSSCHALTEN'
AUS = 'AUS'

GRUNDPOSITIONEN = (STEHEN, SITZEN, LIEGEN)
SUCHE = (SUCHE_DREHEN, SUCHE_SCANNEN, SUCHE_VORWAERTS_1M, SUCHE_AUSRICHTEN, SUCHE_ANFAHREN,
         SUCHE_KOPF_SENKEN, SUCHE_BERUEHREN, SUCHE_BELLEN, SUCHE_DREHUNG_START, SUCHE_RUECKWEG,
         SUCHE_SITZPOSITION, SUCHE_ABBRUCH)

# posture / head / behaviour / signal names (same strings as robot_interfaces/RobotCommand)
STAND, SIT, LIE = 'STAND', 'SIT', 'LIE'
UP, DOWN = 'UP', 'DOWN'
ACK, BARK = 'ACK', 'BARK'

DEFAULTS = dict(
    hier_distance=0.30,          # m, "30cm bewegt"
    search_timeout=120.0,        # s, "2 Minuten abgelaufen"
    behavior_timeout=60.0,       # s, "1 Minute abgelaufen" (Brav, Kunststueck)
    scan_duration=2.0,           # s standing still per scan
    scan_min_hits=3,             # frames with a bone needed for "Knochen gefunden"
    scan_step=1.0,               # m, "1m bewegt"
    max_search_distance=5.0,     # m from the start, no 1 m steps beyond (added)
    align_tolerance_deg=4.0,     # "auf Knochen ausgerichtet"
    turn_gain=1.5,               # rad/s per rad of bearing
    max_angular=0.8,             # rad/s while aligning/approaching
    min_angular=0.15,            # rad/s, below that the robot doesn't turn at all
    approach_speed=0.15,         # m/s towards the bone
    touch_speed=0.05,            # m/s for the last bit with the head down
    touch_max_distance=0.40,     # m without contact -> missed (added)
    lost_timeout=2.0,            # s without bone while aligning/approaching (added)
    bark_duration=2.0,           # s, "Bellen beendet"
    start_tolerance=0.10,        # m, "beim Startfeld angelangt" (no move when closer)
    shutdown_delay=2.0,          # s for "Ton + LED" before shutting down
)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class RobotPort:
    """What the state machine needs from the robot. Awaited commands return an id that later
    comes back through DogFSM.on_done(id); fire-and-forget commands return nothing useful."""

    def stop(self): ...                          # fire-and-forget
    def velocity(self, linear, angular): ...     # fire-and-forget
    def signal(self, name): ...                  # fire-and-forget (ACK, BARK)
    def move(self, meters): ...                  # awaited
    def turn(self, radians): ...                 # awaited
    def posture(self, name): ...                 # awaited (STAND, SIT, LIE)
    def head(self, name): ...                    # awaited (UP, DOWN)
    def behavior(self, name): ...                # awaited (BRAV, TRICK)
    def power_off(self): ...                     # end of "Ausschalten"


AWAITED = ('move', 'turn', 'posture', 'head', 'behavior')


class DogFSM:

    def __init__(self, robot, log=print, **params):
        self.robot = robot
        self.log = log
        self.p = dict(DEFAULTS)
        self.p.update({k: v for k, v in params.items() if k in DEFAULTS})
        self.state = None
        self.posture = STAND
        self.now = 0.0
        self.t_enter = 0.0
        self.pose = (0.0, 0.0, 0.0)       # x, y, yaw relative to the start (odometry)
        self.on_state_change = None       # optional callback(old, new)
        # sequence runner
        self.steps, self.then, self.wait_id = [], None, None
        self.timed_out = False
        # search memory
        self.search_start = 0.0
        self.scan_start = 0.0
        self.scan_hits = 0
        self.last_bone = None             # (time, bone dict)
        self.touch_start = None

    # ======================================================================= events
    def start(self, now):
        """Power on: Startzustand, until the robot stands -> "System bereit" -> Stehen."""
        self.now = now
        self._enter(STARTZUSTAND)

    def on_gesture(self, command, now):
        self.now = now
        self._command('gesture', command)

    def on_voice(self, command, now):
        self.now = now
        self._command('voice', command)

    def on_search_button(self, now):
        self.now = now
        self._command('button', 'SUCH')

    def on_shutdown(self, now):
        self.now = now
        if self.state == SITZEN:
            self._enter(AUSSCHALTEN)
        else:
            self.log(f'"System ausschalten" ignored in {self.state} (only from SITZEN)')

    def on_bone_contact(self, pressed, now):
        self.now = now
        if pressed and self.state in (SUCHE_BERUEHREN, SUCHE_ANFAHREN):
            self.log('bone touched')
            self.robot.stop()
            self._enter(SUCHE_BELLEN)

    def on_bones(self, bones, now):
        """bones: list of dicts {bearing (rad, + left), score, near (bool)}, best first."""
        self.now = now
        if not bones:
            return
        best = bones[0]
        self.last_bone = (now, best)
        s = self.state
        if s == SUCHE_SCANNEN:
            self.scan_hits += 1
        elif s == SUCHE_AUSRICHTEN:
            if abs(best['bearing']) <= math.radians(self.p['align_tolerance_deg']):
                self.robot.stop()
                self._enter(SUCHE_ANFAHREN)
            else:
                self.robot.velocity(0.0, self._steer(best['bearing']))
        elif s == SUCHE_ANFAHREN:
            if best['near']:
                self.robot.stop()
                self._enter(SUCHE_KOPF_SENKEN)
            else:
                self.robot.velocity(self.p['approach_speed'], self._steer(best['bearing']))

    def on_done(self, cmd_id, ok, now):
        self.now = now
        if self.wait_id is None or cmd_id != self.wait_id:
            return
        if not ok:
            self.log(f'command {cmd_id} was aborted, continuing')
        self._next_step()

    def tick(self, now, pose=None):
        self.now = now
        if pose is not None:
            self.pose = pose
        s, dt = self.state, now - self.t_enter

        if s in SUCHE and s not in (SUCHE_ABBRUCH, SUCHE_SITZPOSITION) \
                and now - self.search_start > self.p['search_timeout']:
            self.log('search: 2 minutes over')
            self._enter(SUCHE_ABBRUCH)
        elif s in (BRAV_MACHEN, KUNSTSTUECK_MACHEN) and dt > self.p['behavior_timeout'] \
                and not self.timed_out:
            self.log(f'{s}: 1 minute over')
            self.timed_out = True
            self.robot.stop()
            self._run([('posture', STAND)], STEHEN)
        elif s == SUCHE_SCANNEN and now - self.scan_start >= self.p['scan_duration']:
            if self.scan_hits >= self.p['scan_min_hits']:
                self.log(f'bone found ({self.scan_hits} hits)')
                self._enter(SUCHE_AUSRICHTEN)
            elif math.hypot(self.pose[0], self.pose[1]) + self.p['scan_step'] <= self.p['max_search_distance']:
                self._enter(SUCHE_VORWAERTS_1M)
            else:                                   # end of the segment: keep looking (added)
                self.scan_start, self.scan_hits = now, 0
        elif s in (SUCHE_AUSRICHTEN, SUCHE_ANFAHREN) and self._bone_lost():
            self.log('bone lost')
            self.robot.stop()
            self._enter(SUCHE_SCANNEN)
        elif s == SUCHE_BERUEHREN and self.touch_start is not None:
            x0, y0 = self.touch_start
            if math.hypot(self.pose[0] - x0, self.pose[1] - y0) > self.p['touch_max_distance']:
                self.log('no contact, bone missed')
                self.robot.stop()
                self._run([('head', UP)], SUCHE_SCANNEN)
                self.touch_start = None
        elif s == SUCHE_BELLEN and dt >= self.p['bark_duration'] and not self.steps \
                and self.wait_id is None:
            self._enter(SUCHE_DREHUNG_START)
        elif s == AUSSCHALTEN and dt >= self.p['shutdown_delay']:
            self._enter(AUS)

    # ======================================================================= commands
    def _command(self, source, cmd):
        s = self.state
        handled = True
        if s not in GRUNDPOSITIONEN:
            handled = False
        elif cmd == 'HIER' and source == 'voice':
            self._enter(HIER_VORWAERTS_1)
        elif cmd == 'SUCH' and source in ('voice', 'button'):
            self.search_start = self.now
            self._enter(SUCHE_DREHEN)
        elif cmd == 'BRAV' and source == 'voice' and s == STEHEN:
            self._enter(BRAV_MACHEN)
        elif cmd == 'TRICK' and s == LIEGEN:
            self._enter(KUNSTSTUECK_MACHEN)
        elif cmd == 'SITZ' and source in ('gesture', 'voice') and s in (STEHEN, LIEGEN):
            self._enter(SITZPOSITION_EINNEHMEN)
        elif cmd == 'PLATZ' and source == 'gesture' and s in (STEHEN, SITZEN):
            self._enter(LIEGEPOSITION_EINNEHMEN)
        elif cmd == 'AUF' and source == 'gesture' and s in (SITZEN, LIEGEN):
            self._enter(STEHPOSITION_EINNEHMEN)
        else:
            handled = False
        if not handled:
            self.log(f'{source} {cmd} ignored in {s}')

    # ======================================================================= states
    def _enter(self, state):
        old, self.state, self.t_enter = self.state, state, self.now
        self.steps, self.then, self.wait_id = [], None, None
        self.timed_out = False
        self.log(f'state {old} -> {state}')
        if self.on_state_change:
            self.on_state_change(old, state)
        r, p = self.robot, self.p
        stand = [] if self.posture == STAND else [('posture', STAND)]   # added: drive standing

        if state == STARTZUSTAND:
            r.signal(ACK)
            self._run([('posture', STAND)], STEHEN)              # "System bereit"
        elif state in GRUNDPOSITIONEN:
            r.stop()                                            # do / Sensorik lauschen
        elif state == SITZPOSITION_EINNEHMEN:
            r.signal(ACK)
            self._run([('posture', SIT)], SITZEN)               # "Zielposition erreicht"
        elif state == STEHPOSITION_EINNEHMEN:
            r.signal(ACK)
            self._run([('posture', STAND)], STEHEN)
        elif state == LIEGEPOSITION_EINNEHMEN:
            r.signal(ACK)
            self._run([('posture', LIE)], LIEGEN)
        # ---- HIER
        elif state == HIER_VORWAERTS_1:
            r.signal(ACK)
            self._run(stand + [('move', p['hier_distance'])], HIER_DREHEN_1)
        elif state == HIER_DREHEN_1:
            self._run([('turn', math.pi)], HIER_VORWAERTS_2)
        elif state == HIER_VORWAERTS_2:
            self._run([('move', p['hier_distance'])], HIER_DREHEN_2)
        elif state == HIER_DREHEN_2:
            self._run([('turn', math.pi)], STEHEN)
        # ---- Brav / Kunststueck
        elif state == BRAV_MACHEN:
            r.signal(ACK)
            self._run([('behavior', 'BRAV')], STEHEN)
        elif state == KUNSTSTUECK_MACHEN:
            r.signal(ACK)
            self._run([('behavior', 'TRICK'), ('posture', STAND)], STEHEN)
        # ---- Suche
        elif state == SUCHE_DREHEN:
            r.signal(ACK)                                       # entry of "Suche"
            self._run(stand + [('turn', math.pi)], SUCHE_SCANNEN)
        elif state == SUCHE_SCANNEN:
            r.stop()
            self.scan_start, self.scan_hits = self.now, 0
        elif state == SUCHE_VORWAERTS_1M:
            self._run([('move', p['scan_step'])], SUCHE_SCANNEN)
        elif state in (SUCHE_AUSRICHTEN, SUCHE_ANFAHREN):
            pass                                                # steered in on_bones
        elif state == SUCHE_KOPF_SENKEN:
            self._run([('head', DOWN)], SUCHE_BERUEHREN)        # "Zielposition erreicht"
        elif state == SUCHE_BERUEHREN:
            self.touch_start = self.pose[:2]
            r.velocity(p['touch_speed'], 0.0)
        elif state == SUCHE_BELLEN:
            r.stop()
            r.signal(BARK)
            self._run([('head', UP)], None)                     # added: head up again
        elif state == SUCHE_DREHUNG_START:
            x, y, yaw = self.pose
            if math.hypot(x, y) < p['start_tolerance']:
                self._enter(SUCHE_SITZPOSITION)
                return
            self._run([('turn', wrap(math.atan2(-y, -x) - yaw))], SUCHE_RUECKWEG)
        elif state == SUCHE_RUECKWEG:
            x, y, _ = self.pose
            self._run([('move', math.hypot(x, y))], SUCHE_SITZPOSITION)
        elif state == SUCHE_SITZPOSITION:
            self._run([('posture', SIT)], self._search_done)
        elif state == SUCHE_ABBRUCH:
            r.stop()
            self._run([('head', UP), ('posture', SIT)], SITZEN)
        # ---- Ausschalten
        elif state == AUSSCHALTEN:
            r.stop()
            r.signal(ACK)
        elif state == AUS:
            r.power_off()

    def _search_done(self):
        self.robot.signal(BARK)                                 # exit / bellen
        self.log('search finished')
        self._enter(SITZEN)

    # ======================================================================= helpers
    def _run(self, steps, then):
        """Run awaited commands one after another, then go to state `then` (or call it)."""
        self.steps, self.then = list(steps), then
        self._next_step()

    def _next_step(self):
        if self.steps:
            kind, arg = self.steps.pop(0)
            if kind == 'posture':
                self.posture = arg
            self.wait_id = getattr(self.robot, kind)(arg)
            return
        self.wait_id = None
        then, self.then = self.then, None
        if callable(then):
            then()
        elif then is not None:
            self._enter(then)

    def _steer(self, bearing):
        w = max(-self.p['max_angular'], min(self.p['max_angular'], self.p['turn_gain'] * bearing))
        if abs(w) < self.p['min_angular']:
            w = math.copysign(self.p['min_angular'], bearing)
        return w

    def _bone_lost(self):
        if self.last_bone is None:
            return self.now - self.t_enter > self.p['lost_timeout']
        return self.now - max(self.last_bone[0], self.t_enter) > self.p['lost_timeout']
