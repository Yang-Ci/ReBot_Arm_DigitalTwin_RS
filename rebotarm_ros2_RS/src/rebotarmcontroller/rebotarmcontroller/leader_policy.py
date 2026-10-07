"""ROS-independent mapping and watchdog for the Wiki Arm102 / RS pairing."""
from __future__ import annotations

import math
import secrets
import time

JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
WIKI_DIRECTIONS = (1, 1, -1, -1, -1, 1)
DEFAULT_LIMITS = ((-2.8, 2.8), (0, 3.14), (0, 3.14),
                  (-1.57, 1.57), (-1.57, 1.57), (-3.14, 3.14))
LEADER_RANGES = ((-150, 150), (-1, 170), (-200, 1),
                 (-80, 90), (-90, 90), (-130, 130), (0, 270))


def finite_vector(values, size):
    result = [float(v) for v in values]
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError(f"expected {size} finite values")
    return result


def fresh_angles(monitors):
    """The SDK returns cached monitor objects on timeout; reject reliable=False."""
    angles = []
    for servo_id in range(7):
        sample = monitors.get(servo_id)
        if sample is None or getattr(sample, "reliable", False) is not True:
            raise RuntimeError(f"leader servo {servo_id}: missing or unreliable sample")
        angles.append(float(sample.angle_deg))
    return finite_vector(angles, 7)


def unwrap_angles(values):
    values = finite_vector(values, 7)
    result = []
    for value, (lo, hi) in zip(values, LEADER_RANGES):
        center = (lo + hi) / 2
        result.append(value - round((value - center) / 360) * 360)
    return result


class LeaderMapping:
    def __init__(self, leader, follower, gripper, *, absolute=False,
                 limits=DEFAULT_LIMITS, directions=WIKI_DIRECTIONS,
                 gripper_close=0.0, gripper_open=5.0):
        self.leader = unwrap_angles(leader)
        self.follower = finite_vector(follower, 6)
        self.gripper = float(gripper)
        self.absolute = absolute
        self.directions = finite_vector(directions, 6)
        if not all(abs(d) == 1 for d in self.directions):
            raise ValueError("leader directions must be +1 or -1")
        self.limits = [finite_vector(pair, 2) for pair in limits]
        if len(self.limits) != 6 or any(lo >= hi for lo, hi in self.limits):
            raise ValueError("invalid follower joint limits")
        self.close, self.open = finite_vector([gripper_close, gripper_open], 2)
        if self.close >= self.open or not math.isfinite(self.gripper):
            raise ValueError("invalid gripper limits or feedback")
        self.previous = self.leader[:]

    def map(self, angles):
        values = unwrap_angles(angles)
        # Detect a reset, a turn discontinuity or an implausible per-frame step.
        if any(abs(a - b) > 25 for a, b in zip(values, self.previous)):
            raise ValueError("leader angle jumped >25 degrees; recheck zero and reconnect")
        self.previous = values[:]
        targets = []
        for i, (value, direction, (lo, hi)) in enumerate(zip(values, self.directions, self.limits)):
            delta = value if self.absolute else value - self.leader[i]
            target = math.radians(delta) * direction
            if not self.absolute:
                target += self.follower[i]
            targets.append(max(lo, min(hi, target)))
        grip = math.radians(values[6] * 6) if self.absolute else (
            self.gripper + math.radians((values[6] - self.leader[6]) * 6))
        return targets, max(self.close, min(self.open, grip))


class TeleopLease:
    """Call under the host's command lock; never refresh with cached frames."""
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.session_id = ""
        self.paused = False
        self.last_sample = 0.0
        self.last_heartbeat = 0.0
        self.last_seq = -1
        self.reason = ""

    def acquire(self):
        if self.session_id:
            raise RuntimeError("leader control is already owned by another session")
        self.session_id = secrets.token_hex(16)
        self.last_sample = self.last_heartbeat = self.clock()
        self.last_seq = -1
        self.paused = False
        self.reason = ""
        return self.session_id

    def require(self, session_id):
        if not self.session_id or session_id != self.session_id:
            raise RuntimeError("leader session expired or belongs to another page")

    def heartbeat(self, session_id):
        self.require(session_id)
        self.last_heartbeat = self.clock()

    def sample(self, session_id, seq, sampled_at):
        self.require(session_id)
        now = self.clock()
        if self.expired():
            raise RuntimeError("leader watchdog expired")
        if self.paused:
            raise RuntimeError("leader session is paused")
        if seq <= self.last_seq or not 0 <= now - sampled_at < 0.2:
            raise RuntimeError("stale or out-of-order leader sample")
        self.last_seq = seq
        self.last_sample = sampled_at

    def expired(self):
        if not self.session_id:
            return ""
        now = self.clock()
        if now - self.last_heartbeat > 1.0:
            return "browser heartbeat timed out"
        if not self.paused and now - self.last_sample > 0.3:
            return "leader sample timed out"
        return ""

    def release(self, reason="stopped"):
        self.session_id = ""
        self.paused = False
        self.reason = reason
