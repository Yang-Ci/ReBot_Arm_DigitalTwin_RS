from __future__ import annotations

from contextlib import nullcontext
import json
from pathlib import Path
import time

import mujoco
import numpy as np
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .wrist_cameras import CAMERA_MODELS, WristCameraAssemblies


_ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
_RS_ROS_VISUAL_OPEN_M = 0.045
_LEGACY_VISUAL_OPEN_M = 0.0285
_MUJOCO_GRIPPER_OPEN_M = 0.05
_GRASP_SCENE_NAME = "rs_grasp_scene.xml"
_DEFAULT_OBJECTS = ("red_cube", "blue_block", "yellow_cylinder")
_TARGET_MATERIAL_RGBA = np.array([1.0, 0.55, 0.12, 0.72], dtype=np.float32)
_TARGET_SITE_RGBA = np.array([1.0, 0.55, 0.12, 0.35], dtype=np.float32)
_TARGET_HIDDEN_POS = np.array([0.0, 0.0, -10.0], dtype=np.float64)
_TARGET_HIDDEN_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def find_default_model() -> Path:
    try:
        installed_model = (
            Path(get_package_share_directory("rebotarm_mujoco_rs"))
            / "models"
            / _GRASP_SCENE_NAME
        )
        if installed_model.is_file():
            return installed_model
    except PackageNotFoundError:
        pass
    here = Path(__file__).resolve()
    for parent in here.parents:
        grasp_scene = parent / "models" / _GRASP_SCENE_NAME
        if grasp_scene.is_file():
            return grasp_scene
    raise FileNotFoundError(
        "Integrated B601-RS MuJoCo model was not found in rebotarm_mujoco_rs. "
        "Build the ROS workspace or pass model_path explicitly."
    )


class RsMujocoSync(Node):
    """Run RS MuJoCo dynamics and synchronize them with ROS joint targets."""

    def __init__(self) -> None:
        super().__init__("rebotarm_rs_mujoco")

        self.declare_parameter("arm_namespace", "rebotarm_rs")
        self.declare_parameter("input_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("target_pose_topic", "")
        self.declare_parameter("target_visible_timeout", 0.7)
        self.declare_parameter("model_path", "")
        self.declare_parameter("wrist_camera_model", "d405")
        self.declare_parameter("simulation_mode", "kinematic")
        self.declare_parameter("update_rate", 250.0)
        self.declare_parameter("smoothing_alpha", 1.0)
        self.declare_parameter("stale_timeout", 1.0)
        self.declare_parameter("use_viewer", False)
        self.declare_parameter("show_tcp_marker", False)
        self.declare_parameter("object_names", list(_DEFAULT_OBJECTS))
        self.declare_parameter("object_publish_rate", 30.0)
        self.declare_parameter("arm_kp", [80.0, 100.0, 100.0, 35.0, 25.0, 18.0])
        self.declare_parameter("arm_kd", [8.0, 10.0, 10.0, 4.0, 3.0, 2.5])
        # Match the validated DM finger dynamics.  RS drives one coupler that
        # is equality-linked to both fingers, so its force limit is the sum of
        # DM's two 32 N finger limits.
        self.declare_parameter("gripper_kp", 1800.0)
        self.declare_parameter("gripper_kd", 18.0)
        self.declare_parameter("gripper_tau_limit", 64.0)

        namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        input_topic = str(self.get_parameter("input_topic").value).strip()
        output_topic = str(self.get_parameter("output_topic").value).strip()
        input_topic = input_topic or f"/{namespace}/joint_states"
        output_topic = output_topic or f"/{namespace}/mujoco/joint_states"
        target_pose_topic = str(
            self.get_parameter("target_pose_topic").value
        ).strip()
        target_pose_topic = target_pose_topic or f"/{namespace}/mujoco/target_pose"

        requested_path = str(self.get_parameter("model_path").value).strip()
        self.model_path = (
            Path(requested_path).expanduser().resolve()
            if requested_path
            else find_default_model()
        )
        if not self.model_path.is_file():
            raise FileNotFoundError(f"MuJoCo model not found: {self.model_path}")

        self.simulation_mode = str(
            self.get_parameter("simulation_mode").value
        ).strip().lower()
        if self.simulation_mode not in ("kinematic", "physics"):
            raise ValueError("simulation_mode must be 'kinematic' or 'physics'")

        self.update_rate = max(float(self.get_parameter("update_rate").value), 1.0)
        self.smoothing_alpha = float(
            np.clip(self.get_parameter("smoothing_alpha").value, 0.01, 1.0)
        )
        self.stale_timeout = max(
            float(self.get_parameter("stale_timeout").value), 0.0
        )
        self.arm_kp = self._vector_parameter("arm_kp")
        self.arm_kd = self._vector_parameter("arm_kd")
        self.arm_tau_limit = np.array([36.0, 36.0, 36.0, 14.0, 14.0, 14.0])
        self.gripper_kp = float(self.get_parameter("gripper_kp").value)
        self.gripper_kd = float(self.get_parameter("gripper_kd").value)
        self.gripper_tau_limit = float(self.get_parameter("gripper_tau_limit").value)
        self.target_visible_timeout = max(
            float(self.get_parameter("target_visible_timeout").value),
            0.0,
        )

        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.tcp_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "tcp"
        )
        self.show_tcp_marker = bool(self.get_parameter("show_tcp_marker").value)
        self._set_tcp_marker_visible_locked(self.show_tcp_marker)
        self.wrist_cameras = WristCameraAssemblies(self.model, self.model_path)
        self.wrist_cameras.select(str(self.get_parameter("wrist_camera_model").value))
        self._pending_camera_cycle = 0
        self._pending_tcp_toggle = 0
        mujoco.mj_forward(self.model, self.data)
        target_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "ik_target"
        )
        if target_body_id >= 0:
            self.target_mocap_id = int(self.model.body_mocapid[target_body_id])
            if self.target_mocap_id < 0:
                self.get_logger().warn(
                    "ik_target is not a mocap body; target pose visualization disabled"
                )
        else:
            self.target_mocap_id = -1
            self.get_logger().warn(
                "ik_target body not found; target pose visualization disabled"
            )
        self.target_mat_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_MATERIAL, "target_mat"
        )
        self.target_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "ik_target_sphere"
        )
        self.target_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "ik_target_site"
        )
        self.arm_joint_ids = np.array(
            [self._required_id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in _ARM_JOINTS]
        )
        self.arm_qpos_addrs = self.model.jnt_qposadr[self.arm_joint_ids]
        self.arm_dof_addrs = self.model.jnt_dofadr[self.arm_joint_ids]
        self.arm_actuator_ids = np.array(
            [
                self._required_id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_motor")
                for name in _ARM_JOINTS
            ]
        )
        self.gripper_joint_id = self._required_id(
            mujoco.mjtObj.mjOBJ_JOINT, "joint7"
        )
        self.gripper_qpos_addr = int(self.model.jnt_qposadr[self.gripper_joint_id])
        self.gripper_dof_addr = int(self.model.jnt_dofadr[self.gripper_joint_id])
        self.gripper_actuator_id = self._required_id(
            mujoco.mjtObj.mjOBJ_ACTUATOR, "joint7_motor"
        )
        self.left_joint_id = self._required_id(
            mujoco.mjtObj.mjOBJ_JOINT, "joint_left"
        )
        self.right_joint_id = self._required_id(
            mujoco.mjtObj.mjOBJ_JOINT, "joint_right"
        )
        self.left_qpos_addr = int(self.model.jnt_qposadr[self.left_joint_id])
        self.right_qpos_addr = int(self.model.jnt_qposadr[self.right_joint_id])
        self.left_dof_addr = int(self.model.jnt_dofadr[self.left_joint_id])
        self.right_dof_addr = int(self.model.jnt_dofadr[self.right_joint_id])
        self.object_names = [
            str(name) for name in self.get_parameter("object_names").value
        ]
        self.object_body_ids = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in self.object_names
        }
        self.object_body_ids = {
            name: body_id
            for name, body_id in self.object_body_ids.items()
            if body_id >= 0
        }
        self.object_publish_period = 1.0 / max(
            float(self.get_parameter("object_publish_rate").value), 1.0
        )

        self._target_pose: tuple[np.ndarray, np.ndarray] | None = None
        self._target_pose_monotonic: float | None = None
        self._target_visible = True
        self._set_target_visible_locked(False)

        self.target_arm = self.data.qpos[self.arm_qpos_addrs].copy()
        self.target_gripper = 0.0
        self.last_input_time = 0.0
        self.last_publish_time = 0.0
        self.last_object_publish_time = 0.0

        self.publisher = self.create_publisher(
            JointState,
            output_topic,
            qos_profile_sensor_data,
        )
        self.object_publisher = self.create_publisher(
            String,
            f"/{namespace}/mujoco/object_states",
            10,
        )
        self.subscription = self.create_subscription(
            JointState,
            input_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self.target_pose_subscription = self.create_subscription(
            PoseStamped,
            target_pose_topic,
            self._target_pose_callback,
            qos_profile_sensor_data,
        )
        self.reset_service = self.create_service(
            Trigger,
            f"/{namespace}/mujoco/reset",
            self._reset,
        )

        self.viewer = None
        if bool(self.get_parameter("use_viewer").value):
            from mujoco import viewer as mujoco_viewer

            self.viewer = mujoco_viewer.launch_passive(
                self.model, self.data, key_callback=self._viewer_key_callback
            )

        self.camera_model_publisher = self.create_publisher(
            String, f"/{namespace}/mujoco/wrist_camera_model",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )
        self.tcp_marker_publisher = self.create_publisher(
            Bool, f"/{namespace}/mujoco/tcp_marker_visible",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )
        self.add_on_set_parameters_callback(self._visual_parameter_callback)
        self._publish_camera_model()
        self._publish_tcp_marker_visible()

        self.timer = self.create_timer(1.0 / self.update_rate, self._update)
        self.get_logger().info(
            f"RS MuJoCo ready: mode={self.simulation_mode}, input={input_topic}, "
            f"output={output_topic}, target_pose={target_pose_topic}, "
            f"model={self.model_path}"
        )
        self.get_logger().info(
            f"Wrist camera: {self.wrist_cameras.label}; press C in MuJoCo to cycle cameras"
        )
        self.get_logger().info(
            f"TCP marker: {'shown' if self.show_tcp_marker else 'hidden'}; "
            "press T in MuJoCo to show/hide"
        )

    def _viewer_key_callback(self, keycode):
        # GLFW invokes this on its own thread. Only queue work; ROS and model
        # mutations stay on the node's executor thread under the viewer lock.
        if keycode in (ord("C"), ord("c")):
            self._pending_camera_cycle += 1
        elif keycode in (ord("T"), ord("t")):
            self._pending_tcp_toggle += 1

    def _publish_camera_model(self):
        self.camera_model_publisher.publish(String(data=self.wrist_cameras.active))

    def _set_tcp_marker_visible_locked(self, visible):
        if self.tcp_site_id >= 0:
            self.model.site_rgba[self.tcp_site_id, 3] = 0.3 if visible else 0.0

    def _publish_tcp_marker_visible(self):
        self.tcp_marker_publisher.publish(Bool(data=self.show_tcp_marker))

    def _visual_parameter_callback(self, parameters):
        selected = [p.value for p in parameters if p.name == "wrist_camera_model"]
        tcp_selected = [p for p in parameters if p.name == "show_tcp_marker"]
        if not selected and not tcp_selected:
            return SetParametersResult(successful=True)
        if selected and selected[-1] not in CAMERA_MODELS:
            return SetParametersResult(successful=False,
                                       reason="Choose d405, d435i, gemini2 or uvc32")
        if tcp_selected and tcp_selected[-1].type_ != Parameter.Type.BOOL:
            return SetParametersResult(successful=False,
                                       reason="show_tcp_marker must be a boolean")
        if tcp_selected and tcp_selected[-1].value and self.tcp_site_id < 0:
            return SetParametersResult(successful=False,
                                       reason="TCP site not found in this model")
        viewer_lock = self.viewer.lock() if self.viewer is not None else nullcontext()
        with viewer_lock:
            if selected:
                self.wrist_cameras.select(selected[-1])
                mujoco.mj_forward(self.model, self.data)
            if tcp_selected:
                self.show_tcp_marker = tcp_selected[-1].value
                self._set_tcp_marker_visible_locked(self.show_tcp_marker)
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()
        if selected:
            self._publish_camera_model()
            self.get_logger().info(f"Wrist camera switched to {self.wrist_cameras.label}")
        if tcp_selected:
            self._publish_tcp_marker_visible()
        return SetParametersResult(successful=True)

    def _vector_parameter(self, name: str) -> np.ndarray:
        values = np.asarray(self.get_parameter(name).value, dtype=np.float64)
        if values.shape != (6,):
            raise ValueError(f"{name} must contain 6 values")
        return values

    def _required_id(self, object_type, name: str) -> int:
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f"required MuJoCo object not found: {name}")
        return int(object_id)

    def _joint_state_callback(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        for index, name in enumerate(_ARM_JOINTS):
            if name in values and np.isfinite(values[name]):
                self.target_arm[index] = float(values[name])

        if "gripper_joint1" in values:
            self.target_gripper = self._visual_to_mujoco_gripper(
                values["gripper_joint1"], _RS_ROS_VISUAL_OPEN_M
            )
        elif "finger_left" in values:
            self.target_gripper = self._visual_to_mujoco_gripper(
                values["finger_left"], _LEGACY_VISUAL_OPEN_M
            )
        self.last_input_time = time.monotonic()

    def _target_pose_callback(self, msg: PoseStamped) -> None:
        pos = np.array(
            [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ],
            dtype=np.float64,
        )
        quat = np.array(
            [
                float(msg.pose.orientation.w),
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
            ],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(quat))
        quat = _TARGET_HIDDEN_QUAT.copy() if norm < 1e-9 else quat / norm
        self._target_pose = (pos, quat)
        self._target_pose_monotonic = time.monotonic()

    @staticmethod
    def _visual_to_mujoco_gripper(position: float, visual_open: float) -> float:
        ratio = np.clip(float(position) / visual_open, 0.0, 1.0)
        return float(ratio * _MUJOCO_GRIPPER_OPEN_M)

    def _update(self) -> None:
        if self._pending_tcp_toggle:
            count = self._pending_tcp_toggle
            self._pending_tcp_toggle -= count
            if count % 2:
                self.set_parameters([
                    Parameter("show_tcp_marker", value=not self.show_tcp_marker)
                ])
        if self._pending_camera_cycle:
            count = self._pending_camera_cycle
            self._pending_camera_cycle -= count
            index = (CAMERA_MODELS.index(self.wrist_cameras.active) + count) % len(CAMERA_MODELS)
            self.set_parameters([Parameter("wrist_camera_model", value=CAMERA_MODELS[index])])
        if self.last_input_time == 0.0:
            return
        if (
            self.stale_timeout > 0.0
            and time.monotonic() - self.last_input_time > self.stale_timeout
        ):
            return

        viewer_lock = self.viewer.lock() if self.viewer is not None else nullcontext()
        with viewer_lock:
            if self.simulation_mode == "kinematic":
                self._update_kinematic()
            else:
                self._update_physics()
            if self._apply_target_pose_locked():
                mujoco.mj_forward(self.model, self.data)
            self._publish_state()
            self._publish_object_states()
        if self.viewer is not None:
            if self.viewer.is_running():
                self.viewer.sync()
            else:
                self.viewer.close()
                self.viewer = None

    def _update_kinematic(self) -> None:
        current = self.data.qpos[self.arm_qpos_addrs]
        next_arm = current + self.smoothing_alpha * (self.target_arm - current)
        self.data.qpos[self.arm_qpos_addrs] = next_arm
        self.data.qvel[self.arm_dof_addrs] = 0.0

        current_gripper = float(self.data.qpos[self.gripper_qpos_addr])
        next_gripper = current_gripper + self.smoothing_alpha * (
            self.target_gripper - current_gripper
        )
        self.data.qpos[self.gripper_qpos_addr] = next_gripper
        self.data.qpos[self.left_qpos_addr] = next_gripper
        self.data.qpos[self.right_qpos_addr] = next_gripper
        self.data.qvel[
            [self.gripper_dof_addr, self.left_dof_addr, self.right_dof_addr]
        ] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _apply_target_pose_locked(self) -> bool:
        if self.target_mocap_id < 0:
            return False
        if self._target_pose is None:
            return self._set_target_visible_locked(False)

        now = time.monotonic()
        visible = (
            self._target_pose_monotonic is not None
            and now - self._target_pose_monotonic <= self.target_visible_timeout
        )
        visual_changed = self._set_target_visible_locked(visible)
        if not visible:
            if not np.allclose(
                self.data.mocap_pos[self.target_mocap_id], _TARGET_HIDDEN_POS
            ):
                self.data.mocap_pos[self.target_mocap_id] = _TARGET_HIDDEN_POS
                self.data.mocap_quat[self.target_mocap_id] = _TARGET_HIDDEN_QUAT
                return True
            return visual_changed

        pos, quat = self._target_pose
        changed = not (
            np.allclose(self.data.mocap_pos[self.target_mocap_id], pos)
            and np.allclose(self.data.mocap_quat[self.target_mocap_id], quat)
        )
        self.data.mocap_pos[self.target_mocap_id] = pos
        self.data.mocap_quat[self.target_mocap_id] = quat
        return bool(changed or visual_changed)

    def _set_target_visible_locked(self, visible: bool) -> bool:
        if self._target_visible == visible:
            return False
        self._target_visible = visible
        alpha = 1.0 if visible else 0.0
        if self.target_mat_id >= 0:
            rgba = _TARGET_MATERIAL_RGBA.copy()
            rgba[3] *= alpha
            self.model.mat_rgba[self.target_mat_id] = rgba
        if self.target_geom_id >= 0:
            rgba = _TARGET_MATERIAL_RGBA.copy()
            rgba[3] *= alpha
            self.model.geom_rgba[self.target_geom_id] = rgba
        if self.target_site_id >= 0:
            rgba = _TARGET_SITE_RGBA.copy()
            rgba[3] *= alpha
            self.model.site_rgba[self.target_site_id] = rgba
        return True

    def _update_physics(self) -> None:
        timestep = float(self.model.opt.timestep)
        steps = max(1, int(round((1.0 / self.update_rate) / timestep)))
        for _ in range(steps):
            q = self.data.qpos[self.arm_qpos_addrs]
            qd = self.data.qvel[self.arm_dof_addrs]
            gravity_bias = self.data.qfrc_bias[self.arm_dof_addrs]
            tau = gravity_bias + self.arm_kp * (self.target_arm - q) - self.arm_kd * qd
            self.data.ctrl[self.arm_actuator_ids] = np.clip(
                tau, -self.arm_tau_limit, self.arm_tau_limit
            )

            gripper_q = float(self.data.qpos[self.gripper_qpos_addr])
            gripper_qd = float(self.data.qvel[self.gripper_dof_addr])
            gripper_tau = self.gripper_kp * (
                self.target_gripper - gripper_q
            ) - self.gripper_kd * gripper_qd
            self.data.ctrl[self.gripper_actuator_id] = float(
                np.clip(gripper_tau, -self.gripper_tau_limit, self.gripper_tau_limit)
            )
            mujoco.mj_step(self.model, self.data)

    def _publish_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [*_ARM_JOINTS, "gripper_joint1", "gripper_joint2"]
        msg.position = [
            *[float(value) for value in self.data.qpos[self.arm_qpos_addrs]],
            float(self.data.qpos[self.left_qpos_addr]),
            float(self.data.qpos[self.right_qpos_addr]),
        ]
        msg.velocity = [
            *[float(value) for value in self.data.qvel[self.arm_dof_addrs]],
            float(self.data.qvel[self.left_dof_addr]),
            float(self.data.qvel[self.right_dof_addr]),
        ]
        msg.effort = [
            *[float(value) for value in self.data.qfrc_actuator[self.arm_dof_addrs]],
            0.0,
            0.0,
        ]
        self.publisher.publish(msg)
        self.last_publish_time = time.monotonic()

    def _publish_object_states(self) -> None:
        now = time.monotonic()
        if now - self.last_object_publish_time < self.object_publish_period:
            return
        objects = []
        for name, body_id in self.object_body_ids.items():
            quat_wxyz = [float(value) for value in self.data.xquat[body_id]]
            objects.append(
                {
                    "name": name,
                    "position": [float(value) for value in self.data.xpos[body_id]],
                    "quaternion": quat_wxyz,
                    "quat_wxyz": quat_wxyz,
                }
            )
        msg = String()
        msg.data = json.dumps(
            {"objects": objects, "simulation_mode": self.simulation_mode},
            separators=(",", ":"),
        )
        self.object_publisher.publish(msg)
        self.last_object_publish_time = now

    def _reset(self, _request, response):
        mujoco.mj_resetData(self.model, self.data)
        self.target_arm.fill(0.0)
        self.target_gripper = 0.0
        mujoco.mj_forward(self.model, self.data)
        response.success = True
        response.message = "RS MuJoCo reset"
        return response

    def destroy_node(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RsMujocoSync()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
