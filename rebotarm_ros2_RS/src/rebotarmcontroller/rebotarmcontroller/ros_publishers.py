from __future__ import annotations

import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from sensor_msgs.msg import JointState

_GRIPPER_FINGER_TRAVEL_M = 0.05


def _gripper_motor_to_joint_position(
    position: float,
    open_position: float,
    close_position: float,
) -> float:
    span = open_position - close_position
    ratio = 0.0 if span == 0.0 else (position - close_position) / span
    return max(0.0, min(_GRIPPER_FINGER_TRAVEL_M, ratio * _GRIPPER_FINGER_TRAVEL_M))


class JointStatePublisher:
    def __init__(
        self,
        node,
        hardware,
        namespace: str,
        rate_hz: float,
        feedback_poll_rate_hz: float,
    ) -> None:
        self._node = node
        self._hardware = hardware
        self._publisher = node.create_publisher(
            JointState,
            f"/{namespace}/joint_states",
            node.sensor_qos,
            callback_group=node.reentrant_group,
        )
        self._control_reference_publisher = node.create_publisher(
            JointState,
            f"/{namespace}/control_reference",
            node.sensor_qos,
            callback_group=node.reentrant_group,
        )
        self._control_target_publisher = node.create_publisher(
            JointState,
            f"/{namespace}/control_target",
            node.sensor_qos,
            callback_group=node.reentrant_group,
        )
        self._joint_state_publishers = {
            name: node.create_publisher(
                JointMotorState,
                f"/{namespace}/joints/{name}/state",
                node.sensor_qos,
                callback_group=node.reentrant_group,
            )
            for name in hardware.joint_names
        }
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_publisher = node.create_publisher(
            ArmStatus,
            f"/{namespace}/arm_status",
            latched_qos,
            callback_group=node.reentrant_group,
        )
        self._gripper_state_publisher = None
        if hardware.has_gripper:
            self._gripper_state_publisher = node.create_publisher(
                JointMotorState,
                f"/{namespace}/gripper/state",
                node.sensor_qos,
                callback_group=node.reentrant_group,
            )
        period = 1.0 / max(float(rate_hz), 1.0)
        self._feedback_poll_period = 1.0 / max(float(feedback_poll_rate_hz), 1.0)
        self._last_feedback_poll = 0.0
        self._timer = node.create_timer(
            period,
            self.publish,
            callback_group=node.reentrant_group,
        )
        self.publish_status()

    def publish(self) -> None:
        now = time.monotonic()
        request_feedback = now - self._last_feedback_poll >= self._feedback_poll_period
        if request_feedback:
            self._last_feedback_poll = now
        try:
            pos, vel, effort = self._hardware.get_joint_state(
                request_feedback=request_feedback
            )
        except Exception as exc:
            self._node.get_logger().warn(f"joint state read failed: {exc}")
            return

        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        status_codes = self._hardware.get_joint_status_codes()
        for i, name in enumerate(self._hardware.joint_names):
            motor_msg = JointMotorState()
            motor_msg.header = msg.header
            motor_msg.joint_name = name
            motor_msg.position = float(pos[i])
            motor_msg.velocity = float(vel[i])
            motor_msg.torque = float(effort[i])
            motor_msg.status_code = int(status_codes[i])
            self._joint_state_publishers[name].publish(motor_msg)

        msg.name = self._hardware.joint_names
        msg.position = [float(v) for v in pos]
        msg.velocity = [float(v) for v in vel]
        msg.effort = [float(v) for v in effort]

        target, reference, reference_velocity, reference_acceleration = (
            self._hardware.get_control_reference()
        )
        reference_msg = JointState()
        reference_msg.header = msg.header
        reference_msg.name = list(self._hardware.joint_names)
        reference_msg.position = [float(v) for v in reference]
        reference_msg.velocity = [float(v) for v in reference_velocity]
        # JointState has no acceleration field; effort carries qdd_ref on this
        # explicitly diagnostic topic.
        reference_msg.effort = [float(v) for v in reference_acceleration]
        self._control_reference_publisher.publish(reference_msg)

        target_msg = JointState()
        target_msg.header = msg.header
        target_msg.name = list(self._hardware.joint_names)
        target_msg.position = [float(v) for v in target]
        self._control_target_publisher.publish(target_msg)

        if self._gripper_state_publisher is not None:
            # The arm read above refreshes the full robot, including J7.  Do
            # not start a second synchronous CAN parameter read just for the
            # gripper on every display frame.
            g_pos, g_vel, g_torque, g_status = self._hardware.get_gripper_state(
                request_feedback=False
            )
            joint_pos = _gripper_motor_to_joint_position(
                float(g_pos),
                self._hardware.gripper_open_position,
                self._hardware.gripper_close_position,
            )
            joint_vel = (
                _gripper_motor_to_joint_position(
                    float(g_pos + g_vel),
                    self._hardware.gripper_open_position,
                    self._hardware.gripper_close_position,
                )
                - joint_pos
            )
            msg.name.extend(["gripper_joint1", "gripper_joint2"])
            msg.position.extend([joint_pos, joint_pos])
            msg.velocity.extend([joint_vel, joint_vel])
            msg.effort.extend([float(g_torque), float(g_torque)])

            gripper_msg = JointMotorState()
            gripper_msg.header = msg.header
            gripper_msg.joint_name = "gripper"
            gripper_msg.position = float(g_pos)
            gripper_msg.velocity = float(g_vel)
            gripper_msg.torque = float(g_torque)
            gripper_msg.status_code = int(g_status)
            self._gripper_state_publisher.publish(gripper_msg)

        self._publisher.publish(msg)

    def publish_status(self, *, read_hardware: bool = True) -> None:
        msg = ArmStatus()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.mode = self._hardware.mode
        msg.enabled = self._hardware.enabled
        msg.control_loop_active = self._hardware.control_loop_active
        msg.state_machine = self._hardware.state_machine
        msg.joint_names = self._hardware.joint_names
        if read_hardware:
            msg.per_joint_status_code = self._hardware.get_joint_status_codes()
        else:
            msg.per_joint_status_code = [0 for _ in self._hardware.joint_names]
        msg.error_codes = self._hardware.error_codes
        self._status_publisher.publish(msg)
