"""The same leader lifecycle/watchdog against FakeRsDriver, without CAN."""
from __future__ import annotations

import math
import time

from .leader_policy import DEFAULT_LIMITS, TeleopLease, finite_vector


class FakeLeaderHardware:
    teleop_speed_max = 1.0

    def __init__(self, node):
        self.node = node
        self._cmd_lock = node._leader_lock
        self._teleop_lease = TeleopLease()
        self.teleop_limits = DEFAULT_LIMITS
        configured_speed = float(getattr(node, "max_joint_speed", 1.0))
        self.teleop_speed_max = max(0.05, min(1.0, configured_speed))
        self.gripper_close_position = 0.0
        self.gripper_open_position = node.gripper_open_position
        self.has_gripper = True

    @property
    def teleop_owned(self):
        return bool(self._teleop_lease.session_id)

    def get_joint_state(self, request_feedback=False):
        return list(self.node.positions), list(self.node.velocities), [0] * 6

    def get_gripper_state(self, request_feedback=False):
        return self.node.gripper_position, self.node.gripper_velocity, 0, 0

    def teleop_acquire(self):
        with self._cmd_lock:
            if not self.node.enabled or self.node.state_machine != "IDLE":
                raise RuntimeError("fake follower is disabled or busy")
            self._hold()
            return self._teleop_lease.acquire()

    def teleop_heartbeat(self, session):
        with self._cmd_lock:
            self.teleop_watchdog()
            self._teleop_lease.heartbeat(session)

    def teleop_send(self, session, seq, sampled_at, targets, gripper, speed):
        with self._cmd_lock:
            self.teleop_watchdog()
            targets = finite_vector(targets, 6)
            if any(not lo <= value <= hi for value, (lo, hi) in zip(targets, self.teleop_limits)):
                raise ValueError("target out of limits")
            if not math.isfinite(speed) or not 0.05 <= speed <= self.teleop_speed_max:
                raise ValueError(
                    f"invalid leader speed: 0.05..{self.teleop_speed_max:g} rad/s"
                )
            if gripper is not None and (not math.isfinite(gripper) or not 0 <= gripper <= self.gripper_open_position):
                raise ValueError("invalid gripper target")
            self._teleop_lease.sample(session, seq, sampled_at)
            self.node.targets = list(targets)
            self.node.max_joint_speed = speed
            if gripper is not None:
                self.node.gripper_target = gripper
                self.node.max_gripper_speed = min(speed * 6, 3.0)
            self.node.state_machine = "LOWLEVEL_STREAMING"

    def _hold(self):
        self.node.targets = list(self.node.positions)
        self.node.gripper_target = self.node.gripper_position
        self.node.state_machine = "IDLE"

    def teleop_pause(self, session):
        with self._cmd_lock:
            self.teleop_watchdog()
            self._teleop_lease.require(session)
            self._hold()
            self._teleop_lease.paused = True

    def teleop_resume(self, session):
        with self._cmd_lock:
            self.teleop_watchdog()
            self._teleop_lease.require(session)
            self._teleop_lease.paused = False
            self._teleop_lease.last_sample = time.monotonic()
            self._teleop_lease.last_seq = -1

    def teleop_stop(self, session, reason="stopped"):
        with self._cmd_lock:
            self._teleop_lease.require(session)
            self._hold()
            self._teleop_lease.release(reason)

    def teleop_watchdog(self):
        with self._cmd_lock:
            reason = self._teleop_lease.expired()
            if reason:
                self._hold()
                self._teleop_lease.release(reason)
            return reason
