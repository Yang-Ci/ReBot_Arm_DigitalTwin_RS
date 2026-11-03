from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
from typing import Any

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from rebotarm_msgs.srv import MoveToPoseIK, SetGripper
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

try:
    from fastmcp import FastMCP
except ImportError as exc:  # pragma: no cover - runtime dependency guard
    FastMCP = None
    _FASTMCP_IMPORT_ERROR = exc
else:
    _FASTMCP_IMPORT_ERROR = None


_ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
_GRIPPER_OPEN_MM = 90.0
_GRIPPER_CLOSE_MM = 0.0
_GRIPPER_MOTOR_OPEN_RAD = 5.0
_GRIPPER_BASE_GAP_M = 0.0
_GRIPPER_VISUAL_TRAVEL_M = 0.100
_GRIPPER_EFFECTIVE_GAP_M = 0.100
_GRIPPER_MIN_OBJECT_GRASP_M = 0.014
_GRIPPER_SQUEEZE_M = 0.003
_GRIPPER_RELEASE_ANIMATION_MM = 70.0
_SIM_ANIMATION_TOPIC_SUFFIX = "sim/animation_event"
_VISION_TRANSIT_Z_M = 0.190
_VISION_TRANSIT_Z_BY_COLOR_M = {"blue": 0.185, "yellow": 0.195}
_VISION_VERTICAL_ALIGN_CLEARANCE_M = 0.035
_VISION_PREGRASP_CLEARANCE_M = 0.020
_VISION_FIRST_LIFT_CLEARANCE_M = 0.040
_VISION_MIN_VERTICAL_ALIGN_Z_M = 0.170
_VISION_FIRST_LIFT_MIN_M = 0.180
_VISION_FIRST_LIFT_MIN_BY_COLOR_M: dict[str, float] = {}
_VISION_SAFE_GRASP_Z_BY_COLOR_M = {
    "red": 0.140,
    "blue": 0.140,
    "yellow": 0.140,
}
_STORAGE_ZONES = {"red": (0.49, -0.16, 0.1225), "blue": (0.49, 0.0, 0.119), "yellow": (0.49, 0.16, 0.126)}
_FEEDBACK_MAX_AGE_SEC = 1.0
_VISION_MAX_AGE_SEC = 0.8


class _MissingFastMCP:
    def tool(self, *args, **kwargs):
        def decorator(func):
            return func

        return decorator

    def run(self, *args, **kwargs):
        raise RuntimeError(
            "fastmcp is not installed. Install it with: python3 -m pip install fastmcp"
        ) from _FASTMCP_IMPORT_ERROR


mcp = FastMCP("reBotArm MCP Server") if FastMCP is not None else _MissingFastMCP()
_bridge: "ReBotArmToolBridge | None" = None


class ReBotArmToolBridge(Node):
    """Bridge high-level MCP tools to the existing reBotArm ROS API."""

    def __init__(
        self,
        *,
        namespace: str,
        motion_allowed: bool,
        service_timeout_sec: float,
        action_timeout_padding_sec: float,
    ) -> None:
        super().__init__("rebotarm_mcp_bridge")
        self.namespace = namespace.strip("/") or "rebot_arm_rs_sim"
        self.motion_allowed = bool(motion_allowed)
        self.service_timeout_sec = max(float(service_timeout_sec), 0.1)
        self.action_timeout_padding_sec = max(float(action_timeout_padding_sec), 1.0)

        self._lock = threading.RLock()
        self._joint_state: dict[str, Any] | None = None
        self._joint_state_at = 0.0
        self._mujoco_joint_state: dict[str, Any] | None = None
        self._mujoco_joint_state_at = 0.0
        self._arm_status: dict[str, Any] | None = None
        self._arm_status_at = 0.0
        self._gripper_state: dict[str, Any] | None = None
        self._gripper_state_at = 0.0
        self._vision_payload: dict[str, Any] | None = None
        self._vision_payload_at = 0.0
        self._vision_parse_error: str | None = None

        self._trigger_clients = {
            "enable": self.create_client(Trigger, self._srv("enable")),
            "disable": self.create_client(Trigger, self._srv("disable")),
            "safe_home": self.create_client(Trigger, self._srv("safe_home")),
            "gravity_start": self.create_client(
                Trigger, self._srv("gravity_compensation/start")
            ),
            "gravity_stop": self.create_client(
                Trigger, self._srv("gravity_compensation/stop")
            ),
            "gravity_status": self.create_client(
                Trigger, self._srv("gravity_compensation/status")
            ),
            "record_start": self.create_client(Trigger, self._srv("mujoco/record/start")),
            "record_stop": self.create_client(Trigger, self._srv("mujoco/record/stop")),
            "record_replay": self.create_client(
                Trigger, self._srv("mujoco/record/replay")
            ),
            "record_clear": self.create_client(Trigger, self._srv("mujoco/record/clear")),
        }
        self._gripper_client = self.create_client(SetGripper, self._srv("gripper/set"))
        self._ik_client = self.create_client(MoveToPoseIK, self._srv("move_to_pose_ik"))
        self._move_to_pose_action = ActionClient(
            self, MoveToPose, f"/{self.namespace}/move_to_pose"
        )
        self._follow_joint_trajectory_action = ActionClient(
            self, FollowJointTrajectory, f"/{self.namespace}/follow_joint_trajectory"
        )
        self._sim_animation_pub = self.create_publisher(
            String,
            f"/{self.namespace}/{_SIM_ANIMATION_TOPIC_SUFFIX}",
            10,
        )

        latched_status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(
            JointState,
            f"/{self.namespace}/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        if self.namespace == "rebot_arm_rs_sim":
            self.create_subscription(
                JointState,
                f"/{self.namespace}/mujoco/joint_states",
                self._mujoco_joint_state_callback,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            ArmStatus,
            f"/{self.namespace}/arm_status",
            self._arm_status_callback,
            latched_status_qos,
        )
        self.create_subscription(
            JointMotorState,
            f"/{self.namespace}/gripper/state",
            self._gripper_state_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            f"/{self.namespace}/vision/color_blocks/detections",
            self._vision_callback,
            10,
        )

    def _srv(self, name: str) -> str:
        return f"/{self.namespace}/{name}"

    @staticmethod
    def _joint_state_dict(msg: JointState) -> dict[str, Any]:
        positions = {}
        velocities = {}
        efforts = {}
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                positions[name] = float(msg.position[index])
            if index < len(msg.velocity):
                velocities[name] = float(msg.velocity[index])
            if index < len(msg.effort):
                efforts[name] = float(msg.effort[index])
        return {
                "names": list(msg.name),
                "positions": positions,
                "velocities": velocities,
                "efforts": efforts,
        }

    def _joint_state_callback(self, msg: JointState) -> None:
        state = self._joint_state_dict(msg)
        with self._lock:
            self._joint_state = state
            self._joint_state_at = time.monotonic()

    def _mujoco_joint_state_callback(self, msg: JointState) -> None:
        state = self._joint_state_dict(msg)
        with self._lock:
            self._mujoco_joint_state = state
            self._mujoco_joint_state_at = time.monotonic()

    def _arm_status_callback(self, msg: ArmStatus) -> None:
        with self._lock:
            self._arm_status = {
                "mode": msg.mode,
                "enabled": bool(msg.enabled),
                "control_loop_active": bool(msg.control_loop_active),
                "state_machine": msg.state_machine,
                "joint_names": list(msg.joint_names),
                "per_joint_status_code": [int(value) for value in msg.per_joint_status_code],
                "error_codes": list(msg.error_codes),
            }
            self._arm_status_at = time.monotonic()

    def _gripper_state_callback(self, msg: JointMotorState) -> None:
        with self._lock:
            self._gripper_state = {
                "joint_name": msg.joint_name,
                "motor_position_rad": float(msg.position),
                "opening_mm": self._motor_rad_to_opening_mm(float(msg.position)),
                "velocity": float(msg.velocity),
                "torque": float(msg.torque),
                "status_code": int(msg.status_code),
            }
            self._gripper_state_at = time.monotonic()

    def _vision_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data or "{}")
            parse_error = None
        except Exception as exc:
            payload = None
            parse_error = str(exc)
        with self._lock:
            self._vision_payload = payload
            self._vision_parse_error = parse_error
            self._vision_payload_at = time.monotonic()

    def get_status(self) -> dict[str, Any]:
        with self._lock:
            now = time.monotonic()
            actual_positions = self._current_joint_positions()
            mujoco_positions = (self._mujoco_joint_state or {}).get("positions") or {}
            use_mujoco = (
                self.namespace == "rebot_arm_rs_sim" and actual_positions is not None
                and self._mujoco_joint_state_at > 0
                and now - self._mujoco_joint_state_at <= _FEEDBACK_MAX_AGE_SEC
                and all(name in mujoco_positions and math.isfinite(float(mujoco_positions[name])) for name in _ARM_JOINTS)
            )
            return {
                "ok": True,
                "namespace": self.namespace,
                "motion_mode": "allow" if self.motion_allowed else "locked",
                "topics": {
                    "joint_states_age_sec": self._age(now, self._joint_state_at),
                    "mujoco_joint_states_age_sec": self._age(now, self._mujoco_joint_state_at),
                    "arm_status_age_sec": self._age(now, self._arm_status_at),
                    "gripper_state_age_sec": self._age(now, self._gripper_state_at),
                    "vision_age_sec": self._age(now, self._vision_payload_at),
                },
                "arm_status": self._arm_status,
                "joint_state": self._mujoco_joint_state if use_mujoco else self._joint_state,
                "joint_state_source": "mujoco" if use_mujoco else "driver",
                "driver_joint_state": self._joint_state,
                "mujoco_joint_state": self._mujoco_joint_state,
                "gripper_state": self._gripper_state,
                "vision": self._vision_summary_locked(now),
            }

    def diagnose_ros(self) -> dict[str, Any]:
        services = {}
        for name, client in self._trigger_clients.items():
            services[name] = bool(client.wait_for_service(timeout_sec=0.05))
        services["gripper_set"] = bool(
            self._gripper_client.wait_for_service(timeout_sec=0.05)
        )
        services["move_to_pose_ik"] = bool(
            self._ik_client.wait_for_service(timeout_sec=0.05)
        )
        actions = {
            "move_to_pose": bool(
                self._move_to_pose_action.wait_for_server(timeout_sec=0.05)
            ),
            "follow_joint_trajectory": bool(
                self._follow_joint_trajectory_action.wait_for_server(timeout_sec=0.05)
            ),
        }
        status = self.get_status()
        return {
            "ok": True,
            "namespace": self.namespace,
            "motion_mode": status["motion_mode"],
            "services": services,
            "actions": actions,
            "topics": status["topics"],
        }

    def trigger(self, name: str, *, guard_motion: bool = False) -> dict[str, Any]:
        if guard_motion:
            blocked = self._motion_blocked(name)
            if blocked is not None:
                return blocked
        client = self._trigger_clients[name]
        try:
            result = self._call_service(client, Trigger.Request(), self._srv_name(name))
        except Exception as exc:
            return self._tool_error(name, exc)
        return self._trigger_result(name, result)

    def set_gripper_opening_mm(
        self,
        opening_mm: float,
        max_effort: float = 0.0,
        *,
        guard_motion: bool = True,
    ) -> dict[str, Any]:
        if guard_motion:
            blocked = self._motion_blocked("set_gripper_opening_mm")
            if blocked is not None:
                return blocked

        opening_mm = self._clamp(float(opening_mm), _GRIPPER_CLOSE_MM, _GRIPPER_OPEN_MM)
        request = SetGripper.Request()
        request.position = self._opening_mm_to_motor_rad(opening_mm)
        request.max_effort = float(max_effort)
        try:
            result = self._call_service(
                self._gripper_client,
                request,
                self._srv("gripper/set"),
            )
        except Exception as exc:
            return self._tool_error("set_gripper_opening_mm", exc)
        if result.success and opening_mm >= _GRIPPER_RELEASE_ANIMATION_MM:
            self._publish_sim_animation("release_object", source_tool="set_gripper_opening_mm")
        return {
            "ok": bool(result.success),
            "tool": "set_gripper_opening_mm",
            "requested_opening_mm": opening_mm,
            "ros_success": bool(result.success),
            "reached_opening_mm": self._motor_rad_to_opening_mm(
                float(result.reached_position)
            ),
        }

    def ik_check(
        self,
        x: float,
        y: float,
        z: float,
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        yaw_deg: float = 0.0,
    ) -> dict[str, Any]:
        request = MoveToPoseIK.Request()
        request.target_pose = self._pose_from_xyz_rpy_deg(
            x, y, z, roll_deg, pitch_deg, yaw_deg
        )
        try:
            result = self._call_service(
                self._ik_client,
                request,
                self._srv("move_to_pose_ik"),
            )
        except Exception as exc:
            return self._tool_error("ik_check", exc)
        return {
            "ok": True,
            "tool": "ik_check",
            "ros_success": bool(result.success),
            "message": str(result.message),
            "q_solution": [float(value) for value in result.q_solution],
        }

    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        duration: float = 2.0,
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        yaw_deg: float = 0.0,
        *,
        guard_motion: bool = True,
    ) -> dict[str, Any]:
        if guard_motion:
            blocked = self._motion_blocked("move_to_pose")
            if blocked is not None:
                return blocked

        duration = max(float(duration), 0.2)
        if not self._move_to_pose_action.wait_for_server(
            timeout_sec=self.service_timeout_sec
        ):
            return self._unavailable("move_to_pose", f"/{self.namespace}/move_to_pose")

        pose = self._pose_from_xyz_rpy_deg(x, y, z, roll_deg, pitch_deg, yaw_deg)
        return self._send_move_to_pose(pose, duration, tool="move_to_pose")

    def _move_to_pose_with_pose(
        self,
        pose: Pose,
        duration: float,
        *,
        tool: str,
    ) -> dict[str, Any]:
        if not self._move_to_pose_action.wait_for_server(
            timeout_sec=self.service_timeout_sec
        ):
            return self._unavailable(tool, f"/{self.namespace}/move_to_pose")
        return self._send_move_to_pose(pose, max(float(duration), 0.2), tool=tool)

    def _send_move_to_pose(
        self,
        pose: Pose,
        duration: float,
        *,
        tool: str,
    ) -> dict[str, Any]:
        goal = MoveToPose.Goal()
        goal.target_pose = pose
        goal.duration = max(float(duration), 0.2)
        send_future = self._move_to_pose_action.send_goal_async(goal)
        try:
            goal_handle = self._wait_future(
                send_future,
                self.service_timeout_sec,
                f"{tool} send_goal",
            )
        except Exception as exc:
            return self._tool_error(tool, exc)
        if not goal_handle.accepted:
            return {"ok": False, "tool": tool, "message": "goal rejected"}

        result_future = goal_handle.get_result_async()
        try:
            response = self._wait_future(
                result_future,
                duration + self.action_timeout_padding_sec,
                f"{tool} result",
            )
        except Exception as exc:
            return self._tool_error(tool, exc)
        result = response.result
        return {
            "ok": bool(result.success),
            "tool": tool,
            "ros_success": bool(result.success),
            "message": str(result.message),
            "final_pose": self._pose_to_dict(result.final_pose),
            "status": int(getattr(response, "status", 0)),
        }

    def _move_to_pose_via_ik_trajectory(
        self,
        pose: Pose,
        duration: float,
        *,
        tool: str,
        strict_ik: bool = False,
    ) -> dict[str, Any]:
        ik = self._solve_pose_ik(pose, tool=f"{tool}.ik")
        if not ik.get("ok"):
            return ik
        if strict_ik and not ik.get("ros_success"):
            return {**ik, "ok": False, "message": f"Target unreachable: {ik.get('message', '')}"}

        q_solution = ik.get("q_solution")
        if not isinstance(q_solution, list) or len(q_solution) < len(_ARM_JOINTS):
            return {
                "ok": False,
                "tool": tool,
                "message": "IK did not return a complete joint solution.",
                "ik": ik,
            }

        result = self._send_joint_positions(
            [float(value) for value in q_solution[: len(_ARM_JOINTS)]],
            max(float(duration), 0.2),
            tool=tool,
        )
        result["ik_ros_success"] = bool(ik.get("ros_success"))
        result["ik_best_effort"] = not bool(ik.get("ros_success"))
        result["ik_message"] = str(ik.get("message", ""))
        return result

    def _solve_pose_ik(self, pose: Pose, *, tool: str) -> dict[str, Any]:
        request = MoveToPoseIK.Request()
        request.target_pose = pose
        try:
            result = self._call_service(
                self._ik_client,
                request,
                self._srv("move_to_pose_ik"),
            )
        except Exception as exc:
            return self._tool_error(tool, exc)

        q_solution = [float(value) for value in result.q_solution]
        return {
            "ok": bool(q_solution),
            "tool": tool,
            "ros_success": bool(result.success),
            "message": str(result.message),
            "q_solution": q_solution,
        }

    def _send_joint_positions(
        self,
        positions: list[float],
        duration: float,
        *,
        tool: str,
    ) -> dict[str, Any]:
        if not self._follow_joint_trajectory_action.wait_for_server(
            timeout_sec=self.service_timeout_sec
        ):
            return self._unavailable(
                tool,
                f"/{self.namespace}/follow_joint_trajectory",
            )

        current = self._current_joint_positions()
        if current is None:
            return {
                "ok": False,
                "tool": tool,
                "message": "No /joint_states feedback yet; cannot build safe trajectory start point.",
            }

        duration = max(float(duration), 0.2)
        target = [float(value) for value in positions[: len(_ARM_JOINTS)]]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(_ARM_JOINTS)
        goal.trajectory.points = [
            self._trajectory_point([current[name] for name in _ARM_JOINTS], 0.05),
            self._trajectory_point(target, duration),
        ]

        send_future = self._follow_joint_trajectory_action.send_goal_async(goal)
        try:
            goal_handle = self._wait_future(
                send_future,
                self.service_timeout_sec,
                f"{tool} follow_joint_trajectory send_goal",
            )
        except Exception as exc:
            return self._tool_error(tool, exc)
        if not goal_handle.accepted:
            return {"ok": False, "tool": tool, "message": "goal rejected"}

        result_future = goal_handle.get_result_async()
        try:
            response = self._wait_future(
                result_future,
                duration + self.action_timeout_padding_sec,
                f"{tool} follow_joint_trajectory result",
            )
        except Exception as exc:
            return self._tool_error(tool, exc)
        result = response.result
        return {
            "ok": int(result.error_code) == FollowJointTrajectory.Result.SUCCESSFUL,
            "tool": tool,
            "target_joints": dict(zip(_ARM_JOINTS, target)),
            "error_code": int(result.error_code),
            "message": str(result.error_string),
            "status": int(getattr(response, "status", 0)),
        }

    def move_joints(
        self,
        joint1: float | None = None,
        joint2: float | None = None,
        joint3: float | None = None,
        joint4: float | None = None,
        joint5: float | None = None,
        joint6: float | None = None,
        duration: float = 3.0,
        *,
        guard_motion: bool = True,
    ) -> dict[str, Any]:
        if guard_motion:
            blocked = self._motion_blocked("move_joints")
            if blocked is not None:
                return blocked

        if not self._follow_joint_trajectory_action.wait_for_server(
            timeout_sec=self.service_timeout_sec
        ):
            return self._unavailable(
                "move_joints", f"/{self.namespace}/follow_joint_trajectory"
            )

        current = self._current_joint_positions()
        if current is None:
            return {
                "ok": False,
                "tool": "move_joints",
                "message": "No /joint_states feedback yet; cannot build safe trajectory start point.",
            }

        requested = {
            "joint1": joint1,
            "joint2": joint2,
            "joint3": joint3,
            "joint4": joint4,
            "joint5": joint5,
            "joint6": joint6,
        }
        target = dict(current)
        changed = {}
        for name, value in requested.items():
            if value is not None:
                target[name] = float(value)
                changed[name] = float(value)
        if not changed:
            return {
                "ok": False,
                "tool": "move_joints",
                "message": "At least one joint target must be provided.",
            }

        duration = max(float(duration), 0.2)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(_ARM_JOINTS)
        goal.trajectory.points = [
            self._trajectory_point([current[name] for name in _ARM_JOINTS], 0.05),
            self._trajectory_point([target[name] for name in _ARM_JOINTS], duration),
        ]

        send_future = self._follow_joint_trajectory_action.send_goal_async(goal)
        try:
            goal_handle = self._wait_future(
                send_future,
                self.service_timeout_sec,
                "follow_joint_trajectory send_goal",
            )
        except Exception as exc:
            return self._tool_error("move_joints", exc)
        if not goal_handle.accepted:
            return {"ok": False, "tool": "move_joints", "message": "goal rejected"}

        result_future = goal_handle.get_result_async()
        try:
            response = self._wait_future(
                result_future,
                duration + self.action_timeout_padding_sec,
                "follow_joint_trajectory result",
            )
        except Exception as exc:
            return self._tool_error("move_joints", exc)
        result = response.result
        return {
            "ok": int(result.error_code) == FollowJointTrajectory.Result.SUCCESSFUL,
            "tool": "move_joints",
            "changed": changed,
            "error_code": int(result.error_code),
            "message": str(result.error_string),
            "status": int(getattr(response, "status", 0)),
        }

    def detect_blocks(self, preferred_color: str = "auto") -> dict[str, Any]:
        with self._lock:
            now = time.monotonic()
            if self._vision_parse_error is not None:
                return {
                    "ok": False,
                    "tool": "detect_blocks",
                    "message": f"vision payload parse error: {self._vision_parse_error}",
                    "age_sec": self._age(now, self._vision_payload_at),
                }
            payload = self._copy_jsonable(self._vision_payload)
            age = self._age(now, self._vision_payload_at)

        if payload and age is not None:
            source_age = self._finite_float(payload.get("age_sec"), 0.0)
            if source_age >= 0:
                age += source_age

        if not payload:
            return {
                "ok": False,
                "tool": "detect_blocks",
                "message": "No color block detections have been received yet.",
                "age_sec": age,
            }

        detections = payload.get("detections") or []
        color = str(preferred_color or "auto").lower()
        if color != "auto":
            detections = [
                item for item in detections if str(item.get("color", "")).lower() == color
            ]
        target = self._choose_detection(payload, color)
        return {
            "ok": True,
            "tool": "detect_blocks",
            "age_sec": age,
            "target_color": payload.get("target_color"),
            "preferred_color": color,
            "count": len(detections),
            "target": target,
            "detections": detections,
        }

    def pick_color(
        self,
        color: str = "auto",
        approach_z: float = 0.18,
        grasp_z: float = 0.140,
        lift_z: float = 0.18,
        duration: float = 1.4,
    ) -> dict[str, Any]:
        blocked = self._motion_blocked("pick_color")
        if blocked is not None:
            return blocked

        detection_result = self._wait_for_detection(color, timeout_sec=0.8)
        if not detection_result.get("ok"):
            return detection_result
        target = detection_result.get("target")
        if not isinstance(target, dict):
            return {
                "ok": False,
                "tool": "pick_color",
                "message": f"No target found for color={color!r}.",
            }

        plan = self._build_pick_plan(
            target,
            requested_approach_z=float(approach_z),
            requested_grasp_z=float(grasp_z),
            requested_lift_z=float(lift_z),
        )
        duration = max(float(duration), 0.3)
        steps = []

        for label, action in [
            (
                "open_gripper",
                lambda: self.set_gripper_opening_mm(
                    _GRIPPER_OPEN_MM, guard_motion=False
                ),
            ),
            (
                "high_align",
                lambda: self._move_to_pose_via_ik_trajectory(
                    plan["transit_pose"],
                    max(duration, 1.4),
                    tool="pick_color.high_align",
                    strict_ik=plan["storage_pick"],
                ),
            ),
            (
                "vertical_align",
                lambda: self._move_to_pose_via_ik_trajectory(
                    plan["vertical_align_pose"],
                    max(duration * 0.75, 0.9),
                    tool="pick_color.vertical_align",
                    strict_ik=plan["storage_pick"],
                ),
            ),
            (
                "pregrasp",
                lambda: self._move_to_pose_via_ik_trajectory(
                    plan["pregrasp_pose"],
                    max(duration * 0.6, 0.8),
                    tool="pick_color.pregrasp",
                    strict_ik=plan["storage_pick"],
                ),
            ),
            (
                "move_down",
                lambda: self._move_to_pose_via_ik_trajectory(
                    plan["grasp_pose"],
                    max(duration * 0.65, 0.9),
                    tool="pick_color.move_down",
                    strict_ik=plan["storage_pick"],
                ),
            ),
            (
                "close_gripper",
                lambda: self.set_gripper_opening_mm(
                    plan["close_opening_mm"], guard_motion=False
                ),
            ),
            (
                "settle_after_close",
                lambda: self._sleep_step("pick_color.settle_after_close", 1.6),
            ),
            (
                "first_lift",
                lambda: self._move_to_pose_via_ik_trajectory(
                    plan["first_lift_pose"],
                    max(duration * 0.75, 1.1),
                    tool="pick_color.first_lift",
                    strict_ik=plan["storage_pick"],
                ),
            ),
            (
                "lift",
                lambda: self._move_to_pose_via_ik_trajectory(
                    plan["lift_pose"],
                    max(duration, 1.2),
                    tool="pick_color.lift",
                    strict_ik=plan["storage_pick"],
                ),
            ),
        ]:
            if label == "vertical_align":
                refined = self._wait_for_detection(
                    str(target.get("color") or color),
                    timeout_sec=0.35,
                )
                if not refined.get("ok"):
                    return {
                        "ok": False,
                        "tool": "pick_color",
                        "target": target,
                        "failed_step": "refresh_detection",
                        "message": "Fresh target detection unavailable; stopped before descent.",
                        "steps": steps,
                    }
                refined_target = refined.get("target")
                if isinstance(refined_target, dict) and self._target_shifted(
                    target, refined_target, 0.008
                ):
                    target = refined_target
                    plan = self._build_pick_plan(
                        target,
                        requested_approach_z=float(approach_z),
                        requested_grasp_z=float(grasp_z),
                        requested_lift_z=float(lift_z),
                    )
            result = action()
            steps.append({"step": label, "result": result})
            if not result.get("ok", result.get("ros_success", False)):
                return {
                    "ok": False,
                    "tool": "pick_color",
                    "target": target,
                    "failed_step": label,
                    "steps": steps,
                }

        verification = self._wait_for_detection(
            str(target.get("color") or color), timeout_sec=0.5
        )
        verified_target = verification.get("target") if verification.get("ok") else None
        start_z = self._finite_float(target.get("z"), 0.0)
        final_z = (
            self._finite_float(verified_target.get("z"), start_z)
            if isinstance(verified_target, dict)
            else start_z
        )
        lift_delta = final_z - start_z
        if lift_delta < 0.035:
            return {
                "ok": False,
                "tool": "pick_color",
                "target": target,
                "failed_step": "verify_physical_lift",
                "message": (
                    "Motion completed, but MuJoCo did not observe the object "
                    f"being lifted (delta_z={lift_delta:.4f} m)."
                ),
                "final_target": verified_target,
                "steps": steps,
            }

        self._publish_sim_animation("attach_object", target=verified_target, source_tool="pick_color")

        return {
            "ok": True,
            "tool": "pick_color",
            "target": target,
            "plan": {
                "yaw_deg": round(math.degrees(float(plan["yaw_rad"])), 1),
                "close_opening_mm": round(float(plan["close_opening_mm"]), 1),
                "approach_z": round(float(plan["approach_z"]), 4),
                "grasp_z": round(float(plan["grasp_z"]), 4),
                "first_lift_z": round(float(plan["first_lift_z"]), 4),
                "lift_z": round(float(plan["lift_z"]), 4),
            },
            "steps": steps,
            "verified_lift_m": round(lift_delta, 4),
        }

    def _call_service(self, client, request, name: str):
        if not client.wait_for_service(timeout_sec=self.service_timeout_sec):
            raise TimeoutError(f"ROS service unavailable: {name}")
        future = client.call_async(request)
        return self._wait_future(future, self.service_timeout_sec, name)

    def _wait_future(self, future, timeout_sec: float, label: str):
        deadline = time.monotonic() + max(float(timeout_sec), 0.1)
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Timed out waiting for {label}")
            time.sleep(0.02)
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} returned no result")
        return result

    def _trigger_result(self, tool: str, result) -> dict[str, Any]:
        return {
            "ok": bool(result.success),
            "tool": tool,
            "ros_success": bool(result.success),
            "message": str(result.message),
        }

    def _srv_name(self, key: str) -> str:
        return {
            "enable": self._srv("enable"),
            "disable": self._srv("disable"),
            "safe_home": self._srv("safe_home"),
            "gravity_start": self._srv("gravity_compensation/start"),
            "gravity_stop": self._srv("gravity_compensation/stop"),
            "gravity_status": self._srv("gravity_compensation/status"),
            "record_start": self._srv("mujoco/record/start"),
            "record_stop": self._srv("mujoco/record/stop"),
            "record_replay": self._srv("mujoco/record/replay"),
            "record_clear": self._srv("mujoco/record/clear"),
        }[key]

    def _motion_blocked(self, tool: str) -> dict[str, Any] | None:
        if self.motion_allowed:
            return None
        return {
            "ok": False,
            "blocked": True,
            "tool": tool,
            "message": (
                "Motion tools are locked. Restart the MCP server with "
                "--motion-mode allow after confirming you are connected to a safe "
                "simulation or intentionally controlling hardware."
            ),
        }

    def _unavailable(self, tool: str, endpoint: str) -> dict[str, Any]:
        return {
            "ok": False,
            "tool": tool,
            "message": f"ROS endpoint unavailable: {endpoint}",
        }

    @staticmethod
    def _tool_error(tool: str, exc: Exception) -> dict[str, Any]:
        return {
            "ok": False,
            "tool": tool,
            "message": str(exc),
            "error_type": exc.__class__.__name__,
        }

    def _current_joint_positions(self) -> dict[str, float] | None:
        with self._lock:
            now = time.monotonic()
            candidates = [(self._joint_state, self._joint_state_at)]
            if self.namespace == "rebot_arm_rs_sim":
                candidates.insert(0, (self._mujoco_joint_state, self._mujoco_joint_state_at))
            for state, received_at in candidates:
                if not state or not received_at or now - received_at > _FEEDBACK_MAX_AGE_SEC:
                    continue
                positions = state.get("positions") or {}
                if all(name in positions and math.isfinite(float(positions[name])) for name in _ARM_JOINTS):
                    return {name: float(positions[name]) for name in _ARM_JOINTS}
            return None

    def _vision_summary_locked(self, now: float) -> dict[str, Any]:
        if self._vision_parse_error:
            return {
                "ok": False,
                "parse_error": self._vision_parse_error,
                "age_sec": self._age(now, self._vision_payload_at),
            }
        if not self._vision_payload:
            return {
                "ok": False,
                "message": "no detections yet",
                "age_sec": self._age(now, self._vision_payload_at),
            }
        detections = self._vision_payload.get("detections") or []
        return {
            "ok": True,
            "age_sec": self._age(now, self._vision_payload_at),
            "target_color": self._vision_payload.get("target_color"),
            "count": len(detections),
            "target": self._vision_payload.get("target"),
        }

    @staticmethod
    def _choose_detection(payload: dict[str, Any], preferred_color: str) -> dict[str, Any] | None:
        detections = payload.get("detections") or []
        if preferred_color != "auto":
            for item in detections:
                if str(item.get("color", "")).lower() == preferred_color:
                    return item
            return None
        target = payload.get("target")
        if isinstance(target, dict):
            return target
        return detections[0] if detections else None

    @staticmethod
    def _estimate_grasp_opening_mm(target: dict[str, Any]) -> float:
        return float(ReBotArmToolBridge._estimate_grasp_plan(target)["command_mm"])

    def _wait_for_detection(
        self,
        preferred_color: str,
        *,
        timeout_sec: float,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        last = self.detect_blocks(preferred_color)
        while time.monotonic() < deadline:
            result = self.detect_blocks(preferred_color)
            if result.get("ok") and isinstance(result.get("target"), dict):
                age = result.get("age_sec")
                if age is not None and float(age) <= _VISION_MAX_AGE_SEC:
                    return result
            last = result
            time.sleep(0.08)
        return {"ok": False, "tool": "detect_blocks", "message": "No fresh color detection available.", "age_sec": last.get("age_sec")}

    def _build_pick_plan(
        self,
        target: dict[str, Any],
        *,
        requested_approach_z: float,
        requested_grasp_z: float,
        requested_lift_z: float,
    ) -> dict[str, Any]:
        x = self._finite_float(target.get("x"), 0.0)
        y = self._finite_float(target.get("y"), 0.0)
        color = self._target_color(target)
        grasp = self._estimate_grasp_plan(target)
        yaw_rad = float(grasp["yaw_rad"])

        detected_z = self._finite_float(target.get("z"), 0.122)
        zone = _STORAGE_ZONES.get(color)
        storage_pick = bool(self.namespace == "rebot_arm_rs_sim" and zone and
                            math.hypot(x - zone[0], y - zone[1]) <= 0.055 and
                            abs(detected_z - zone[2]) <= 0.035)
        safe_grasp_z = max(
            _VISION_SAFE_GRASP_Z_BY_COLOR_M.get(color, 0.122), detected_z
        )
        grasp_z = self._clamp(max(float(requested_grasp_z), safe_grasp_z), 0.06, 0.25)
        transit_floor = _VISION_TRANSIT_Z_BY_COLOR_M.get(color, _VISION_TRANSIT_Z_M)
        approach_z = self._clamp(
            max(float(requested_approach_z), transit_floor),
            0.08,
            0.35,
        )
        vertical_align_z = self._clamp(
            max(grasp_z + _VISION_VERTICAL_ALIGN_CLEARANCE_M, _VISION_MIN_VERTICAL_ALIGN_Z_M),
            grasp_z,
            approach_z,
        )
        pregrasp_z = self._clamp(
            max(grasp_z + _VISION_PREGRASP_CLEARANCE_M, grasp_z),
            grasp_z,
            approach_z,
        )
        first_lift_floor = _VISION_FIRST_LIFT_MIN_BY_COLOR_M.get(
            color,
            _VISION_FIRST_LIFT_MIN_M,
        )
        first_lift_z = self._clamp(
            max(grasp_z + _VISION_FIRST_LIFT_CLEARANCE_M, first_lift_floor),
            grasp_z,
            approach_z,
        )
        lift_z = self._clamp(
            max(float(requested_lift_z), first_lift_z, transit_floor),
            first_lift_z,
            0.35,
        )

        if storage_pick:
            approach_z = max(approach_z, detected_z + 0.10)
            lift_z = max(lift_z, detected_z + 0.13)
            first_lift_z = lift_z
            # Match the reachable web route: approach and lift at 50 degrees,
            # then descend directly at 65 degrees to the object.
            vertical_align_z = approach_z
            pregrasp_z = grasp_z
        pose = lambda z, pitch: self._pitched_pose(x, y, z, yaw_rad, pitch)
        transit_pitch = math.radians(50) if storage_pick else math.pi / 2
        grasp_pitch = math.radians(65) if storage_pick else math.pi / 2

        return {
            "x": x,
            "y": y,
            "color": color,
            "storage_pick": storage_pick,
            "yaw_rad": yaw_rad,
            "approach_z": approach_z,
            "grasp_z": grasp_z,
            "first_lift_z": first_lift_z,
            "lift_z": lift_z,
            "close_opening_mm": float(grasp["command_mm"]),
            "physical_gap_mm": float(grasp["physical_gap_m"]) * 1000.0,
            "transit_pose": pose(approach_z, transit_pitch),
            "vertical_align_pose": pose(vertical_align_z, transit_pitch),
            "pregrasp_pose": pose(pregrasp_z, grasp_pitch),
            "grasp_pose": pose(grasp_z, grasp_pitch),
            "first_lift_pose": pose(first_lift_z, transit_pitch),
            "lift_pose": pose(lift_z, transit_pitch),
        }

    @staticmethod
    def _sleep_step(tool: str, seconds: float) -> dict[str, Any]:
        delay = max(float(seconds), 0.0)
        time.sleep(delay)
        return {"ok": True, "tool": tool, "slept_sec": round(delay, 3)}

    def _publish_sim_animation(
        self,
        event: str,
        *,
        target: dict[str, Any] | None = None,
        color: str | None = None,
        source_tool: str,
    ) -> dict[str, Any]:
        resolved_color = str(color or "").strip().lower()
        if not resolved_color and isinstance(target, dict):
            resolved_color = self._target_color(target)

        payload: dict[str, Any] = {
            "event": str(event),
            "source": "rebotarm_agent",
            "source_tool": str(source_tool),
            "namespace": self.namespace,
            "stamp": round(time.time(), 3),
        }
        if resolved_color:
            payload["color"] = resolved_color
        if isinstance(target, dict):
            payload["target"] = self._copy_jsonable(target)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._sim_animation_pub.publish(msg)
        return {
            "ok": True,
            "tool": f"{source_tool}.sim_animation",
            "event": payload["event"],
            "color": resolved_color or None,
            "topic": f"/{self.namespace}/{_SIM_ANIMATION_TOPIC_SUFFIX}",
        }

    @staticmethod
    def _target_shifted(
        left: dict[str, Any],
        right: dict[str, Any],
        threshold: float,
    ) -> bool:
        if ReBotArmToolBridge._target_color(left) != ReBotArmToolBridge._target_color(right):
            return False
        dx = ReBotArmToolBridge._finite_float(left.get("x"), math.nan) - ReBotArmToolBridge._finite_float(
            right.get("x"),
            math.nan,
        )
        dy = ReBotArmToolBridge._finite_float(left.get("y"), math.nan) - ReBotArmToolBridge._finite_float(
            right.get("y"),
            math.nan,
        )
        dz = ReBotArmToolBridge._finite_float(left.get("z"), 0.0) - ReBotArmToolBridge._finite_float(
            right.get("z"),
            0.0,
        )
        if not all(math.isfinite(value) for value in (dx, dy, dz)):
            return False
        return math.hypot(dx, dy, dz) > float(threshold)

    @staticmethod
    def _target_color(target: dict[str, Any]) -> str:
        return str(target.get("color") or "").strip().lower()

    @staticmethod
    def _estimate_grasp_plan(target: dict[str, Any]) -> dict[str, float]:
        fallback_by_color = {
            "red": 0.045,
            "yellow": 0.040,
            "blue": 0.032,
        }
        color = ReBotArmToolBridge._target_color(target)
        width = ReBotArmToolBridge._finite_float(target.get("width_m"), math.nan)
        height = ReBotArmToolBridge._finite_float(target.get("height_m"), math.nan)
        shortest = ReBotArmToolBridge._finite_float(target.get("shortest_m"), math.nan)
        yaw_rad = 0.0

        if width > 0 and height > 0:
            candidates = [
                {"cross_section": height, "yaw_rad": 0.0},
                {"cross_section": width, "yaw_rad": -math.pi / 2.0},
            ]
            candidates.sort(
                key=lambda item: (
                    item["cross_section"] > _GRIPPER_EFFECTIVE_GAP_M,
                    item["cross_section"],
                )
            )
            cross_section = float(candidates[0]["cross_section"])
            yaw_rad = float(candidates[0]["yaw_rad"])
        else:
            cross_section = shortest
            if not math.isfinite(cross_section) or cross_section <= 0:
                cross_section = fallback_by_color.get(color, 0.05)
            reported_yaw = ReBotArmToolBridge._finite_float(
                target.get("grasp_yaw_rad"),
                math.nan,
            )
            if math.isfinite(reported_yaw):
                yaw_rad = reported_yaw

        physical_gap = ReBotArmToolBridge._clamp(
            float(cross_section) - _GRIPPER_SQUEEZE_M,
            _GRIPPER_MIN_OBJECT_GRASP_M,
            _GRIPPER_EFFECTIVE_GAP_M,
        )
        command_m = ReBotArmToolBridge._physical_gap_to_gripper_command(physical_gap)
        return {
            "command_mm": command_m * 1000.0,
            "physical_gap_m": physical_gap,
            "yaw_rad": yaw_rad,
        }

    @staticmethod
    def _physical_gap_to_gripper_command(physical_gap_m: float) -> float:
        travel = max(_GRIPPER_VISUAL_TRAVEL_M, 0.001)
        open_m = _GRIPPER_OPEN_MM / 1000.0
        return ReBotArmToolBridge._clamp(
            ((float(physical_gap_m) - _GRIPPER_BASE_GAP_M) / travel) * open_m,
            _GRIPPER_CLOSE_MM / 1000.0,
            open_m,
        )

    @staticmethod
    def _top_down_pose(x: float, y: float, z: float, yaw_rad: float) -> Pose:
        return ReBotArmToolBridge._pitched_pose(x, y, z, yaw_rad, math.pi / 2)

    @staticmethod
    def _pitched_pose(x: float, y: float, z: float, yaw_rad: float, pitch: float) -> Pose:
        yaw = yaw_rad if math.isfinite(float(yaw_rad)) else 0.0
        sy = math.sin(yaw / 2.0)
        cy = math.cos(yaw / 2.0)
        sp = math.sin(pitch / 2)
        cp = math.cos(pitch / 2)
        return ReBotArmToolBridge._pose_from_xyz_quat(
            x,
            y,
            z,
            qx=-sy * sp,
            qy=cy * sp,
            qz=sy * cp,
            qw=cy * cp,
        )

    @staticmethod
    def _opening_mm_to_motor_rad(opening_mm: float) -> float:
        ratio = ReBotArmToolBridge._clamp(
            float(opening_mm) / _GRIPPER_OPEN_MM, 0.0, 1.0
        )
        return ratio * _GRIPPER_MOTOR_OPEN_RAD

    @staticmethod
    def _motor_rad_to_opening_mm(position_rad: float) -> float:
        ratio = ReBotArmToolBridge._clamp(
            float(position_rad) / _GRIPPER_MOTOR_OPEN_RAD, 0.0, 1.0
        )
        return ratio * _GRIPPER_OPEN_MM

    @staticmethod
    def _pose_from_xyz_quat(
        x: float,
        y: float,
        z: float,
        *,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> Pose:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose

    @staticmethod
    def _finite_float(value: Any, default: float) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return float(default)
        if not math.isfinite(number):
            return float(default)
        return number

    @staticmethod
    def _pose_from_xyz_rpy_deg(
        x: float,
        y: float,
        z: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> Pose:
        qx, qy, qz, qw = ReBotArmToolBridge._quat_from_rpy_deg(
            roll_deg, pitch_deg, yaw_deg
        )
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    @staticmethod
    def _quat_from_rpy_deg(
        roll_deg: float, pitch_deg: float, yaw_deg: float
    ) -> tuple[float, float, float, float]:
        roll = math.radians(float(roll_deg))
        pitch = math.radians(float(pitch_deg))
        yaw = math.radians(float(yaw_deg))
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return float(qx), float(qy), float(qz), float(qw)

    @staticmethod
    def _pose_to_dict(pose: Pose) -> dict[str, Any]:
        return {
            "position": {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
            },
            "orientation": {
                "x": float(pose.orientation.x),
                "y": float(pose.orientation.y),
                "z": float(pose.orientation.z),
                "w": float(pose.orientation.w),
            },
        }

    @staticmethod
    def _trajectory_point(positions: list[float], seconds: float) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        sec = int(math.floor(float(seconds)))
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int(round((float(seconds) - sec) * 1e9))
        return point

    @staticmethod
    def _copy_jsonable(value):
        return json.loads(json.dumps(value)) if value is not None else None

    @staticmethod
    def _age(now: float, stamp: float) -> float | None:
        if stamp <= 0.0:
            return None
        return round(max(now - stamp, 0.0), 3)

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(float(minimum), min(float(maximum), float(value)))


def _require_bridge() -> ReBotArmToolBridge:
    if _bridge is None:
        raise RuntimeError("reBotArm MCP bridge is not initialized")
    return _bridge


@mcp.tool()
def get_robot_status() -> dict[str, Any]:
    """Return the latest robot status, joint feedback, gripper feedback, and vision freshness."""
    return _require_bridge().get_status()


@mcp.tool()
def diagnose_ros() -> dict[str, Any]:
    """Check whether the expected reBotArm ROS services, actions, and feedback topics are available."""
    return _require_bridge().diagnose_ros()


@mcp.tool()
def enable_robot() -> dict[str, Any]:
    """Enable the robot controller. This does not send a motion target by itself."""
    return _require_bridge().trigger("enable")


@mcp.tool()
def disable_robot() -> dict[str, Any]:
    """Disable the robot controller."""
    return _require_bridge().trigger("disable")


@mcp.tool()
def safe_home() -> dict[str, Any]:
    """Move the arm to the configured safe home position. Requires motion_mode=allow."""
    return _require_bridge().trigger("safe_home", guard_motion=True)


@mcp.tool()
def gravity_compensation_status() -> dict[str, Any]:
    """Query whether controller-side gravity compensation is active."""
    return _require_bridge().trigger("gravity_status")


@mcp.tool()
def gravity_compensation_start() -> dict[str, Any]:
    """Start controller-side gravity compensation. Requires motion_mode=allow."""
    return _require_bridge().trigger("gravity_start", guard_motion=True)


@mcp.tool()
def gravity_compensation_stop() -> dict[str, Any]:
    """Stop controller-side gravity compensation."""
    return _require_bridge().trigger("gravity_stop")


@mcp.tool()
def set_gripper_opening_mm(opening_mm: float, max_effort: float = 0.0) -> dict[str, Any]:
    """Command the gripper opening in millimeters from 0 closed to 90 open. Requires motion_mode=allow."""
    return _require_bridge().set_gripper_opening_mm(opening_mm, max_effort)


@mcp.tool()
def ik_check(
    x: float,
    y: float,
    z: float,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
) -> dict[str, Any]:
    """Run an IK reachability check for a target pose without executing motion."""
    return _require_bridge().ik_check(x, y, z, roll_deg, pitch_deg, yaw_deg)


@mcp.tool()
def move_to_pose(
    x: float,
    y: float,
    z: float,
    duration: float = 2.0,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
) -> dict[str, Any]:
    """Move the end effector to a Cartesian pose. Requires motion_mode=allow."""
    return _require_bridge().move_to_pose(x, y, z, duration, roll_deg, pitch_deg, yaw_deg)


@mcp.tool()
def move_joints(
    joint1: float | None = None,
    joint2: float | None = None,
    joint3: float | None = None,
    joint4: float | None = None,
    joint5: float | None = None,
    joint6: float | None = None,
    duration: float = 3.0,
) -> dict[str, Any]:
    """Move one or more arm joints in radians using a safe two-point trajectory. Requires motion_mode=allow."""
    return _require_bridge().move_joints(
        joint1, joint2, joint3, joint4, joint5, joint6, duration
    )


@mcp.tool()
def detect_blocks(preferred_color: str = "auto") -> dict[str, Any]:
    """Return the latest simulated color-block detections, optionally filtered by color."""
    return _require_bridge().detect_blocks(preferred_color)


@mcp.tool()
def pick_color(
    color: str = "auto",
    approach_z: float = 0.18,
    grasp_z: float = 0.140,
    lift_z: float = 0.18,
    duration: float = 1.4,
) -> dict[str, Any]:
    """Pick the latest detected color block with a simple open-approach-close-lift sequence. Requires motion_mode=allow."""
    return _require_bridge().pick_color(color, approach_z, grasp_z, lift_z, duration)


@mcp.tool()
def record_start() -> dict[str, Any]:
    """Start MuJoCo task recording if the simulation task server is running."""
    return _require_bridge().trigger("record_start")


@mcp.tool()
def record_stop() -> dict[str, Any]:
    """Stop MuJoCo task recording and save the captured CSV."""
    return _require_bridge().trigger("record_stop")


@mcp.tool()
def record_replay() -> dict[str, Any]:
    """Replay the latest in-memory MuJoCo task recording. Requires motion_mode=allow."""
    return _require_bridge().trigger("record_replay", guard_motion=True)


@mcp.tool()
def record_clear() -> dict[str, Any]:
    """Clear the current MuJoCo task recording buffer."""
    return _require_bridge().trigger("record_clear")


def _parse_args(argv: list[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="Run the reBotArm MCP server.")
    parser.add_argument(
        "--namespace",
        default=os.getenv("REBOTARM_NAMESPACE", "rebot_arm_rs_sim"),
        help="ROS namespace without a leading slash.",
    )
    parser.add_argument(
        "--host",
        default=os.getenv("REBOTARM_MCP_HOST", "127.0.0.1"),
        help="HTTP host for streamable-http transport.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.getenv("REBOTARM_MCP_PORT", "8081")),
        help="HTTP port for streamable-http transport.",
    )
    parser.add_argument(
        "--transport",
        choices=("streamable-http", "stdio"),
        default=os.getenv("REBOTARM_MCP_TRANSPORT", "streamable-http"),
        help="MCP transport mode.",
    )
    parser.add_argument(
        "--motion-mode",
        choices=("locked", "allow"),
        default=os.getenv("REBOTARM_MCP_MOTION_MODE", "locked"),
        help="Use 'allow' only for safe simulation or intentional hardware control.",
    )
    parser.add_argument(
        "--service-timeout-sec",
        type=float,
        default=float(os.getenv("REBOTARM_MCP_SERVICE_TIMEOUT_SEC", "2.0")),
    )
    parser.add_argument(
        "--action-timeout-padding-sec",
        type=float,
        default=float(os.getenv("REBOTARM_MCP_ACTION_TIMEOUT_PADDING_SEC", "8.0")),
    )
    args, ros_args = parser.parse_known_args(argv)
    return args, ros_args


def main(argv: list[str] | None = None) -> int:
    global _bridge
    args, ros_args = _parse_args(argv)

    if FastMCP is None:
        raise RuntimeError(
            "fastmcp is not installed. Install it with: python3 -m pip install fastmcp"
        ) from _FASTMCP_IMPORT_ERROR

    rclpy.init(args=ros_args)
    executor = MultiThreadedExecutor(num_threads=4)
    _bridge = ReBotArmToolBridge(
        namespace=args.namespace,
        motion_allowed=args.motion_mode == "allow",
        service_timeout_sec=args.service_timeout_sec,
        action_timeout_padding_sec=args.action_timeout_padding_sec,
    )
    executor.add_node(_bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    _bridge.get_logger().info(
        "reBotArm MCP server starting: "
        f"namespace=/{_bridge.namespace}, transport={args.transport}, "
        f"motion_mode={args.motion_mode}"
    )

    try:
        try:
            if args.transport == "stdio":
                mcp.run(transport="stdio")
            else:
                mcp.run(transport="streamable-http", host=args.host, port=args.port)
        except KeyboardInterrupt:
            pass
    finally:
        executor.shutdown()
        if _bridge is not None:
            _bridge.destroy_node()
            _bridge = None
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
