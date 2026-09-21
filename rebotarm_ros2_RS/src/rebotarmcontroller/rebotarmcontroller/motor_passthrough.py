from __future__ import annotations

import time

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import (
    JointMitCmd,
    JointPosVelCmd,
)


class MotorPassthrough:
    def __init__(self, node, hardware, namespace: str, arbitration: str) -> None:
        self._node = node
        self._hardware = hardware
        self._arbitration = arbitration
        self._last_joint_status_publish = 0.0
        self._joint_status_publish_period = 0.1
        # High-rate browser streams are state targets, not an event log.  A
        # depth-one queue makes the newest command replace stale pointer
        # samples instead of replaying them after the user has stopped.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._subscriptions = []

        joint_commands = (
            (
                JointMitCmd,
                "cmd/mit",
                lambda hw, name, msg: hw.send_joint_mit_cmd(
                    name,
                    msg.pos,
                    msg.vel,
                    msg.kp,
                    msg.kd,
                    msg.tau,
                ),
            ),
            (
                JointPosVelCmd,
                "cmd/pos_vel",
                lambda hw, name, msg: hw.send_joint_pos_vel_cmd(
                    name,
                    msg.pos,
                    msg.vlim,
                ),
            ),
        )
        gripper_commands = (
            (
                JointMitCmd,
                "cmd/mit",
                lambda hw, msg: hw.send_gripper_mit_cmd(
                    msg.pos,
                    msg.vel,
                    msg.kp,
                    msg.kd,
                    msg.tau,
                ),
            ),
            (
                JointPosVelCmd,
                "cmd/pos_vel",
                lambda hw, msg: hw.send_gripper_pos_vel_cmd(msg.pos, msg.vlim),
            ),
        )

        for joint_name in hardware.joint_names:
            for msg_type, label, command in joint_commands:
                self._subscribe(
                    msg_type,
                    f"/{namespace}/joints/{joint_name}/{label}",
                    self._make_joint_callback(
                        joint_name,
                        label,
                        command,
                    ),
                    qos,
                )
        if hardware.has_gripper:
            for msg_type, label, command in gripper_commands:
                self._subscribe(
                    msg_type,
                    f"/{namespace}/gripper/{label}",
                    self._make_gripper_callback(label, command),
                    qos,
                )

    def _subscribe(self, msg_type, topic: str, callback, qos: QoSProfile) -> None:
        self._subscriptions.append(
            self._node.create_subscription(
                msg_type,
                topic,
                callback,
                qos,
                callback_group=self._node.reentrant_group,
            )
        )

    def _make_joint_callback(self, joint_name: str, label: str, command) -> object:
        def _callback(msg) -> None:
            if not self._can_send_lowlevel(
                f"/joints/{joint_name}/{label}",
                allow_preempt=True,
            ):
                return

            try:
                command(self._hardware, joint_name, msg)
            except Exception as exc:
                self._node.get_logger().warn(
                    f"joint {label} failed for {joint_name}: {exc}"
                )
            finally:
                self._publish_joint_stream_status()

        return _callback

    def _publish_joint_stream_status(self) -> None:
        """Keep high-rate commands from flooding ROS with ArmStatus messages."""
        now = time.monotonic()
        if now - self._last_joint_status_publish < self._joint_status_publish_period:
            return
        self._last_joint_status_publish = now
        self._node.publish_arm_status(read_hardware=False)

    def _make_gripper_callback(self, label: str, command) -> object:
        def _callback(msg) -> None:
            if not self._can_send_lowlevel(
                f"/gripper/{label}",
                allow_preempt=False,
                allow_during_trajectory=True,
            ):
                return

            try:
                command(self._hardware, msg)
            except Exception as exc:
                self._node.get_logger().warn(f"gripper {label} failed: {exc}")
            finally:
                self._publish_joint_stream_status()

        return _callback

    def _can_send_lowlevel(
        self,
        label: str,
        *,
        allow_preempt: bool,
        allow_during_trajectory: bool = False,
    ) -> bool:
        state = self._hardware.state_machine
        if state in ("GRAVITY_COMP", "SAFE_HOMING"):
            self._node.get_logger().warn(f"rejecting {label} in state {state}")
            return False
        if state == "TRAJ_RUNNING":
            if allow_during_trajectory:
                return True
            if self._arbitration == "reject" or not allow_preempt:
                self._node.get_logger().warn(
                    f"rejecting {label} while trajectory is running"
                )
                return False
            self._node.get_logger().warn(
                f"preempting trajectory for {label}"
            )
            self._hardware.stop_motion()
        return True
