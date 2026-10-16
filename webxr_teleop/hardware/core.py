"""Dependency-free safety state and trajectories, shared by all processes."""
import json
import hashlib
import math
from pathlib import Path

SIDES = ('left', 'right')
JOINTS = ('waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate')
MOTORS = ('waist', 'shoulder', 'shoulder_shadow', 'elbow', 'elbow_shadow',
          'forearm_roll', 'wrist_angle', 'wrist_rotate', 'gripper')
TTL = .150
FEEDBACK_TTL = .100
PERIOD = .020


def vector(q, n=7):
    if not isinstance(q, (list, tuple)) or len(q) != n or not all(
            isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in q):
        raise ValueError(f'Expected {n} finite numbers')
    return list(q)


def load_profile(path, live=False, sides=SIDES, require_motor_watchdog=True):
    p = json.loads(Path(path).read_text())
    if p.get('version') != 1:
        raise ValueError('Unsupported hardware profile version')
    for side in SIDES:
        a = p['arms'][side]
        for name in ('home_rad', 'sign', 'offset_deg', 'lower_rad', 'upper_rad'):
            vector(a[name], 6)
        vector(a['initial_rad'], 6)
        vector(a['base_position'], 3)
        vector(a['base_quaternion_wxyz'], 4)
        if abs(sum(x*x for x in a['base_quaternion_wxyz'])-1) > .001:
            raise ValueError('Base quaternion must be normalized')
        if any(x not in (-1, 1) for x in a['sign']):
            raise ValueError('Mapping signs must be -1 or +1')
        if any(lo >= hi or not lo <= h <= hi for lo, hi, h in zip(a['lower_rad'], a['upper_rad'], a['home_rad'])):
            raise ValueError('Invalid joint limits or home pose')
        if not 0 <= a['initial_gripper'] <= 1:
            raise ValueError('Invalid initial gripper')
        if not 0 < a['finger_open_m']-a['finger_closed_m'] <= .041:
            raise ValueError('Invalid finger opening conversion')
        if live and side in sides:
            if a.get('verified') is not True or not a.get('verification_note'):
                raise ValueError(f'{side}: commissioning verification required')
            if not Path(a['calibration']).is_file():
                raise ValueError(f'{side}: missing calibration')
            digest=hashlib.sha256(Path(a['calibration']).read_bytes()).hexdigest()
            if a.get('calibration_sha256') != digest:
                raise ValueError(f'{side}: calibration changed or has not been fingerprinted')
    if p['arms']['left']['port'] == p['arms']['right']['port']:
        raise ValueError('Follower ports must differ')
    if live and require_motor_watchdog and p.get('watchdog_verified') is not True:
        raise ValueError('Powered bus-watchdog stop must be verified before live operation')
    return p


class HomeTrajectory:
    """Synchronized quintic trajectory: zero endpoint velocity and acceleration."""
    def __init__(self, start, goal, speed=math.radians(5), acceleration=math.radians(10)):
        self.start = vector(start, len(start))
        self.goal = vector(goal, len(start))
        distance = max(abs(b-a) for a, b in zip(start, goal))
        # max s'=1.875; max |s''|=10/sqrt(3), for s=10u^3-15u^4+6u^5.
        self.duration = max(2., 1.875*distance/speed, math.sqrt(10/math.sqrt(3)*distance/acceleration))

    def at(self, t):
        u = min(1., max(0., t/self.duration))
        s = u**3*(10+u*(-15+6*u))
        return [a+s*(b-a) for a, b in zip(self.start, self.goal)]


class Guard:
    """Motor-local latch. No network reconnect or fresh target clears this latch."""
    def __init__(self, lower, upper, *, sequence=False):
        self.sequence = sequence
        self.lower, self.upper = lower, upper
        self.state = 'HOLD'
        self.reason = 'Startup: explicit reset required'
        self.goal = None
        self.velocity = [0.]*7
        self.epoch = -1
        self.seq = -1
        self.last_command = -math.inf
        self.error_since = None
        self.frozen = True

    def hold(self, measured, reason):
        if self.state != 'HOLD' or self.goal is None:
            self.goal = vector(measured)
        self.state, self.reason = 'HOLD', reason
        self.velocity = [0.]*7
        return self.goal[:]

    def clutch(self, measured):
        """Release motion authority without disarming; capture feedback only once."""
        if self.state == 'ARMED':
            if not self.frozen:
                self.goal = vector(measured)
            self.frozen = True
            self.velocity = [0.]*7
        return self.goal[:]

    def arm(self, measured, epoch):
        if not isinstance(epoch, int) or epoch <= self.epoch:
            raise ValueError('Reset requires a new generation')
        self.epoch, self.seq = epoch, -1
        self.goal = vector(measured)
        self.velocity = [0.]*7
        self.error_since = None
        self.frozen = True
        self.trajectory_start = None
        self.state, self.reason = 'ARMED', ''

    def command(self, msg, measured, now, dt=PERIOD):
        measured = vector(measured)
        if self.state == 'HOLD':
            return self.goal[:]
        try:
            if msg['epoch'] != self.epoch or not isinstance(msg['seq'], int) or msg['seq'] <= self.seq:
                raise ValueError('Old command sequence or generation')
            if not 0 <= now-msg['issued'] <= TTL:
                raise ValueError('Expired command')
            q = vector(msg['q'])
            if msg.get('freeze'):
                q = self.clutch(measured)

            else:
                self.frozen = False
            named_move = self.sequence and msg.get('mode') in ('opening', 'sleep')
            if not 0 <= q[6] <= 1 or (not named_move and not msg.get('freeze') and
                    not all(lo <= x <= hi for x, lo, hi in zip(q[:6], self.lower, self.upper))):
                raise ValueError('Joint target outside limits')
            # Tracking lag is handled by planner backpressure, not a latched
            # motor stop. Fresh valid targets continue driving the servo.
            # The first planner target is the trajectory's zero-velocity origin.
            # Encoder quantization can differ from the worker's capture by a count.
            # Adopt that bounded origin once, rather than inventing acceleration
            # from two independently sampled starting positions.
            if msg.get('trajectory_start') and named_move:
                identity = (self.epoch, msg['mode'], msg.get('trajectory_id'))
                if identity == self.trajectory_start:
                    raise ValueError('Repeated trajectory start')
                tolerance = [math.radians(2) if msg['mode']=='sleep' else 2*2*math.pi/4096+1e-9]*6+[.02]
                if any(abs(a-b)>limit for a,b,limit in zip(q,self.goal,tolerance)):
                    raise ValueError('Planner start differs from hold by more than encoder tolerance')
                if any(abs(v)>1e-9 for v in self.velocity):
                    raise ValueError('Planner start requires stationary command')
                self.goal = q[:]
                self.velocity = [0.]*7
                self.trajectory_start = identity
            # The planner owns trajectory speed/acceleration shaping. IPC may
            # skip frames, so differentiating targets at a fixed worker period
            # invents rate violations even for a bounded planner trajectory.
            velocity = [0.]*7 if msg.get('freeze') else [(x-y)/dt for x, y in zip(q, self.goal)]
            self.goal, self.velocity = q, velocity
            self.seq, self.last_command = msg['seq'], now
            return q[:]
        except (ValueError, KeyError, TypeError) as exc:
            return self.hold(measured, str(exc))

    def tick(self, measured, now):
        if self.state == 'ARMED' and now-self.last_command > TTL:
            return self.hold(measured, 'Command watchdog expired')
        return self.goal[:]


def validate_sequence(profile, sides=SIDES, live=False):
    """Named poses must be explicit; never substitute home for a missing sleep."""
    for side in sides:
        arm = profile['arms'][side]
        for name in ('opening_rad', 'sleep_rad'):
            if name not in arm:
                raise ValueError(f'{side}: sequence requires configured {name}')
            pose = vector(arm[name], 6)
            violations = [f'{joint}={math.degrees(q):.2f}° (allowed {math.degrees(lo):.2f}…{math.degrees(hi):.2f}°)'
                for joint, q, lo, hi in zip(JOINTS, pose, arm['lower_rad'], arm['upper_rad']) if not lo <= q <= hi]
            if violations and name != 'sleep_rad':
                raise ValueError(f'{side}: {name} outside joint limits: ' + '; '.join(violations))
            source = arm.get('sequence_sources', {}).get(name.removesuffix('_rad'))
            if source and (source['sign'] != arm['sign'] or source['offset_deg'] != arm['offset_deg']):
                raise ValueError(f'{side}: mapping changed; reimport {name}')
        verified = arm.get('sequence_verified') is True and bool(arm.get('sequence_verification_note'))
        authorized = arm.get('sequence_authorized') is True and bool(arm.get('sequence_authorization_note'))
        if live and not (verified or authorized):
            raise ValueError(f'{side}: opening/sleep commissioning verification or explicit operator authorization required')


class SettlingWindow:
    """Require 300 ms of bounded feedback, tolerating two encoder counts.

    The window bounds total excursion, not each adjacent step, so continuous
    drift cannot accumulate unnoticed. Targets and feedback gaps reset it.
    """
    def __init__(self, arm_error=math.radians(.5)):
        self.arm_error = arm_error
        self.samples = []
        self.target = None

    def update(self, q, target, timestamp):
        q, target = vector(q), vector(target)
        if (self.target != target or (self.samples and
                (timestamp < self.samples[-1][0] or timestamp-self.samples[-1][0] > FEEDBACK_TTL))):
            self.samples = []
        self.target = target[:]
        if self.samples and timestamp == self.samples[-1][0]:
            return False  # Repeated cached feedback cannot establish settling.
        self.samples.append((timestamp, q))
        while len(self.samples)>1 and self.samples[1][0] <= timestamp-.3:
            self.samples.pop(0)
        if timestamp-self.samples[0][0] < .3-1e-9:
            return False
        tolerance = [2*2*math.pi/4096+1e-9]*6+[.02]
        return all(max(row[1][j] for row in self.samples)-min(row[1][j] for row in self.samples) <= tolerance[j]
                   for j in range(7)) and all(
            abs(row[1][j]-target[j]) <= (self.arm_error if j<6 else .02)
            for row in self.samples for j in range(7))
