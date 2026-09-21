from __future__ import annotations

import functools
import threading
import time

import numpy as np

from .conversions import fk_to_pose
from .hardware_config import resolve_hardware_config
from .motion_profiles import advance_jerk_limited_reference_over_elapsed

_GRIPPER_GOAL_TOLERANCE_RAD = 0.12
_DEFAULT_JOINT_POS_VEL_VLIM_RAD_S = 1.20
_MAX_JOINT_POS_VEL_VLIM_RAD_S = 1.50
_DEFAULT_GRIPPER_POS_VEL_VLIM_RAD_S = 5.0
_MAX_GRIPPER_POS_VEL_VLIM_RAD_S = 5.0
_SAFE_HOME_TOLERANCE_RAD = np.deg2rad(2.0)
_SAFE_HOME_VELOCITY_TOLERANCE_RAD_S = 0.15
_GRIPPER_ASSIST_TORQUE_MAX_NM = 0.08
_GRIPPER_ASSIST_LIMIT_MARGIN_RAD = 0.12


def _locked(method):
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        with self._cmd_lock:
            return method(self, *args, **kwargs)

    return wrapper


class HardwareManager:
    """ROS-facing adapter for the new grouped reBotArm SDK."""

    def __init__(
        self,
        hardware_config: str | None = None,
        model: str = "",
        channel: str = "",
    ) -> None:
        self._cmd_lock = threading.RLock()
        hardware_config_path, hardware_data = resolve_hardware_config(
            hardware_config,
            model,
            channel,
        )

        from reBotArm_control_py.actuator import RebotArm
        from reBotArm_control_py.controllers import RebotArmEndPose
        from reBotArm_control_py.dynamics import compute_generalized_gravity
        from reBotArm_control_py.kinematics import (
            compute_fk,
            load_robot_model,
            pad_q_for_model,
        )

        self._compute_fk = compute_fk
        self._pad_q_for_model = pad_q_for_model

        runtime_config = hardware_data["_runtime"]
        control_runtime = runtime_config["control"]
        self._arm_control_mode = control_runtime["arm_control_mode"]

        self._robot = RebotArm(hw_yaml=str(hardware_config_path))
        self._arm_group = self._robot.groups.get("arm")
        if self._arm_group is None:
            raise ValueError("hardware config must define groups.arm")
        self._gripper_group = self._robot.groups.get("gripper")
        self._robot_get_state = self._robot.get_state
        self._robot.get_state = self._get_arm_state
        self._arm_mit_kp = np.array(control_runtime["mit_kp"], dtype=np.float64)
        self._arm_mit_kd = np.array(control_runtime["mit_kd"], dtype=np.float64)
        self._stream_acceleration_limit = float(
            control_runtime["stream_acceleration_limit"]
        )
        self._stream_jerk_limit = float(control_runtime["stream_jerk_limit"])
        self._stream_natural_frequency = float(
            control_runtime["stream_natural_frequency"]
        )
        self._endpos_ctrl = RebotArmEndPose(
            self._robot,
            arm_control_mode=self._arm_control_mode,
            # The SDK implementation reads all six RobStride mechPos
            # parameters synchronously before every MIT command in order to
            # compute gravity feed-forward.  On a shared CAN bus that blocks
            # the real-time sender for tens/hundreds of milliseconds.  This
            # adapter computes the same feed-forward from its asynchronously
            # refreshed state cache in _endpos_loop_cb instead.
            use_gravity_ff=False,
        )

        self._gripper_name = (
            self._gripper_group.joint_names[0]
            if self.has_gripper and self._gripper_group.joint_names
            else ""
        )
        self._gripper_mit_kp = (
            np.array(getattr(self._gripper_group, "_mit_kp"), dtype=np.float64)
            if self.has_gripper
            else np.zeros(0, dtype=np.float64)
        )
        self._gripper_mit_kd = (
            np.array(getattr(self._gripper_group, "_mit_kd"), dtype=np.float64)
            if self.has_gripper
            else np.zeros(0, dtype=np.float64)
        )
        gripper_limits = hardware_data.get("gripper", {}).get("position_limits", {})
        self.gripper_open_position = float(gripper_limits.get("open", 0.0))
        self.gripper_close_position = float(gripper_limits.get("close", 0.0))
        self._gripper_target_position: float | None = None

        self._gc_model = load_robot_model()
        self._gc_data = self._gc_model.createData()
        self._gc_compute_generalized_gravity = compute_generalized_gravity
        gc_runtime = runtime_config["gravity_compensation"]
        self._gravity_comp_kp = np.array(gc_runtime["kp"], dtype=np.float64)
        self._gravity_comp_kd = np.array(gc_runtime["kd"], dtype=np.float64)
        self._gravity_comp_transition_duration = float(
            gc_runtime["transition_duration"]
        )
        self._gravity_comp_joint_direction = np.array(
            gc_runtime["joint_direction"],
            dtype=np.float64,
        )
        self._gravity_comp_tau_scale = np.array(
            gc_runtime["tau_scale"],
            dtype=np.float64,
        )
        self._gravity_comp_torque_limit = np.array(
            gc_runtime["torque_limit"],
            dtype=np.float64,
        )
        assist_runtime = runtime_config["gripper_assist"]
        self._gripper_assist_torque = float(
            np.clip(
                assist_runtime["torque"],
                0.0,
                _GRIPPER_ASSIST_TORQUE_MAX_NM,
            )
        )
        self._gripper_assist_kd = float(np.clip(assist_runtime["kd"], 0.0, 0.2))
        self._gripper_assist_velocity_threshold = float(
            assist_runtime["velocity_threshold"]
        )
        self._gripper_assist_velocity_full = float(assist_runtime["velocity_full"])
        self._gripper_assist_speed_limit = float(assist_runtime["speed_limit"])
        self._gripper_assist_breakaway_fraction = float(
            np.clip(assist_runtime["breakaway_fraction"], 0.0, 0.5)
        )

        self._connected = False
        self._enabled = False
        self._control_output_enabled = False
        self._state_machine = "IDLE"
        self._error_codes: list[str] = []
        self._gravity_comp_active = False
        self._gravity_comp_q_target: np.ndarray | None = None
        self._gravity_comp_q_last: np.ndarray | None = None
        self._gravity_comp_transition_q_hold: np.ndarray | None = None
        self._gravity_comp_transition_started_at: float | None = None
        self._gravity_comp_fault = ""
        self._gripper_manual_free = False
        self._gripper_assist_active = False
        self._gripper_assist_velocity = 0.0
        self._gripper_hold_position: float | None = None
        self._cached_arm_position = np.zeros(len(self.joint_names), dtype=np.float64)
        self._cached_arm_velocity = np.zeros(len(self.joint_names), dtype=np.float64)
        self._cached_arm_torque = np.zeros(len(self.joint_names), dtype=np.float64)
        self._cached_gripper_position = 0.0
        self._cached_gripper_velocity = 0.0
        self._cached_gripper_torque = 0.0
        self._homing_thread: int | None = None
        self._last_motion_duration: float = 0.0
        self._lowlevel_pos_target: np.ndarray | None = None
        self._mit_stream_target: np.ndarray | None = None
        self._mit_stream_vlim = np.full(
            len(self.joint_names),
            _DEFAULT_JOINT_POS_VEL_VLIM_RAD_S,
            dtype=np.float64,
        )
        self._mit_stream_velocity = np.zeros(len(self.joint_names), dtype=np.float64)
        self._mit_stream_acceleration = np.zeros(
            len(self.joint_names), dtype=np.float64
        )
        self._mit_stream_last_time = time.perf_counter()
        self._mit_gripper_target: float | None = None
        self._mit_gripper_vlim = _DEFAULT_GRIPPER_POS_VEL_VLIM_RAD_S

    # ------------------------------------------------------------------
    # state
    # ------------------------------------------------------------------

    @property
    def joint_names(self) -> list[str]:
        return list(self._arm_group.joint_names)

    @property
    def mode(self) -> str:
        return str(self._arm_group.mode)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def control_loop_active(self) -> bool:
        return bool(self._robot.control_loop_active)

    @property
    def has_gripper(self) -> bool:
        return bool(self._robot.has_gripper)

    @property
    def last_motion_duration(self) -> float:
        return float(self._last_motion_duration)

    @property
    def state_machine(self) -> str:
        return self._state_machine

    @property
    def error_codes(self) -> list[str]:
        return list(self._error_codes)

    @_locked
    def get_control_reference(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return target, reference position/velocity/acceleration snapshots."""
        reference = np.array(self._endpos_ctrl._q_target, dtype=np.float64, copy=True)
        velocity = np.array(self._endpos_ctrl._qd_target, dtype=np.float64, copy=True)
        acceleration = np.array(
            self._mit_stream_acceleration, dtype=np.float64, copy=True
        )
        target = (
            np.array(self._mit_stream_target, dtype=np.float64, copy=True)
            if self._mit_stream_target is not None
            else reference.copy()
        )
        return target, reference, velocity, acceleration

    @_locked
    def set_state_machine(self, state: str) -> None:
        if state not in (
            "IDLE",
            "TRAJ_RUNNING",
            "LOWLEVEL_STREAMING",
            "GRAVITY_COMP",
            "SAFE_HOMING",
        ):
            raise ValueError(f"unsupported state machine value: {state}")
        self._state_machine = state

    # ------------------------------------------------------------------
    # arm
    # ------------------------------------------------------------------

    @_locked
    def connect(self) -> None:
        if self._connected:
            return
        try:
            self._robot.connect()
            if self.has_gripper:
                self._gripper_target_position = self.get_gripper_state()[0]
            self._start_endpos_loop()
            self._connected = True
            self._enabled = True
        except Exception:
            self._control_output_enabled = False
            self._endpos_ctrl._running = False
            try:
                self._robot.stop_control_loop()
                self._robot.disconnect()
            finally:
                self._connected = False
                self._enabled = False
            raise

    def shutdown(self, disable_after_safe_home: bool = True) -> None:
        if not self._connected:
            return
        try:
            self.safe_home()
            with self._cmd_lock:
                self._robot.stop_control_loop()

            if disable_after_safe_home:
                self.disable()

            with self._cmd_lock:
                self._robot.disconnect()
        finally:
            self._connected = False
            self._enabled = False
            self._control_output_enabled = False
            self.set_state_machine("IDLE")

    def get_joint_state(
        self, request_feedback: bool = True
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self._get_arm_state(request_feedback=request_feedback)

    def _get_arm_state(
        self,
        request_feedback: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not request_feedback:
            return (
                self._cached_arm_position.copy(),
                self._cached_arm_velocity.copy(),
                self._cached_arm_torque.copy(),
            )
        pos, vel, torq = self._robot_get_state(request_feedback=request_feedback)
        n = len(self.joint_names)
        # Replace complete arrays atomically so the control thread can take a
        # lock-free snapshot while ROS performs the slower hardware refresh.
        self._cached_arm_position = np.array(pos[:n], dtype=np.float64, copy=True)
        self._cached_arm_velocity = np.array(vel[:n], dtype=np.float64, copy=True)
        self._cached_arm_torque = np.array(torq[:n], dtype=np.float64, copy=True)
        if self.has_gripper and len(pos) > n:
            self._cached_gripper_position = float(pos[n])
            self._cached_gripper_velocity = float(vel[n])
            self._cached_gripper_torque = float(torq[n])
        return (
            self._cached_arm_position.copy(),
            self._cached_arm_velocity.copy(),
            self._cached_arm_torque.copy(),
        )

    def get_joint_positions(self, request: bool = False) -> np.ndarray:
        if request:
            current = self._arm_group.get_positions(request_feedback=True)
            self._cached_arm_position = np.array(
                current, dtype=np.float64, copy=True
            )
        return self._cached_arm_position.copy()

    def is_near_safe_home(
        self,
        tolerance_rad: float = _SAFE_HOME_TOLERANCE_RAD,
        velocity_tolerance_rad_s: float = _SAFE_HOME_VELOCITY_TOLERANCE_RAD_S,
    ) -> bool:
        """Return whether every arm joint is close enough to the zero pose.

        A fresh hardware read is intentional here: this check gates motor
        disable, so a cached browser or controller target must not be used.
        If feedback cannot be read, the exception propagates and disable is
        aborted instead of cutting power at an unknown pose.
        """
        positions, velocities, _ = self.get_joint_state()
        positions = np.asarray(positions, dtype=np.float64).reshape(-1)
        velocities = np.asarray(velocities, dtype=np.float64).reshape(-1)
        if len(positions) != len(self.joint_names):
            raise RuntimeError(
                f"expected {len(self.joint_names)} joint positions, got {len(positions)}"
            )
        if not np.all(np.isfinite(positions)):
            raise RuntimeError("joint feedback contains non-finite values")
        if len(velocities) != len(self.joint_names):
            raise RuntimeError(
                f"expected {len(self.joint_names)} joint velocities, got {len(velocities)}"
            )
        if not np.all(np.isfinite(velocities)):
            raise RuntimeError("joint velocity feedback contains non-finite values")
        return bool(
            np.all(np.abs(positions) <= float(tolerance_rad))
            and np.all(np.abs(velocities) <= float(velocity_tolerance_rad_s))
        )

    def wait_until_near_safe_home(self, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + max(float(timeout), 0.0)
        while True:
            if self.is_near_safe_home():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)

    def get_joint_velocities(self, request: bool = False) -> np.ndarray:
        return self._arm_group.get_velocities(request_feedback=request)

    @_locked
    def hold_current_position(self) -> np.ndarray:
        current = self.get_joint_positions(request=True).copy()
        self._cached_arm_position = current.copy()
        if self._state_machine != "SAFE_HOMING":
            self._endpos_ctrl._q_target[:] = current
            self._endpos_ctrl._qd_target[:] = 0.0
        return current

    @_locked
    def set_joint_position_target(self, positions, velocities=None) -> None:
        if self._state_machine == "SAFE_HOMING":
            raise RuntimeError("rejecting joint target during safe home")
        target = np.asarray(positions, dtype=np.float64).reshape(-1)
        if len(target) != len(self.joint_names):
            raise ValueError(
                f"expected {len(self.joint_names)} joint targets, got {len(target)}"
            )
        self.stop_motion()
        self._endpos_ctrl._q_target[:] = target
        if velocities is None:
            self._endpos_ctrl._qd_target[:] = 0.0
        else:
            velocity = np.asarray(velocities, dtype=np.float64).reshape(-1)
            if len(velocity) != len(self.joint_names):
                raise ValueError(
                    f"expected {len(self.joint_names)} joint velocities, got {len(velocity)}"
                )
            self._endpos_ctrl._qd_target[:] = velocity

    @_locked
    def update_trajectory_reference(self, positions, velocities) -> None:
        """Update a running trajectory without stopping its control stream."""
        if self._state_machine != "TRAJ_RUNNING":
            raise RuntimeError(
                f"rejecting trajectory reference in state {self._state_machine}"
            )
        target = np.asarray(positions, dtype=np.float64).reshape(-1)
        velocity = np.asarray(velocities, dtype=np.float64).reshape(-1)
        expected = len(self.joint_names)
        if len(target) != expected or len(velocity) != expected:
            raise ValueError(
                f"expected {expected} trajectory positions and velocities"
            )
        self._endpos_ctrl._q_target[:] = target
        self._endpos_ctrl._qd_target[:] = velocity

    @_locked
    def start_endpos_control(self) -> None:
        if self._gravity_comp_active:
            raise RuntimeError("stop gravity compensation before starting endpos control")
        if self._state_machine in ("SAFE_HOMING", "TRAJ_RUNNING"):
            raise RuntimeError(f"rejecting endpos control in state {self._state_machine}")

        if self.control_loop_active:
            self.set_state_machine("IDLE")
            return

        self._robot.stop_control_loop()
        self._start_endpos_loop()
        self._enabled = True
        self.set_state_machine("IDLE")

    def enable(self) -> None:
        self.start_endpos_control()

    @_locked
    def disable(self) -> None:
        if self._gravity_comp_active:
            raise RuntimeError("stop gravity compensation before disable")
        self._control_output_enabled = False
        self.stop_motion()
        self._endpos_ctrl._running = False
        self._robot.stop_control_loop()
        self._robot.disable_all()
        self._enabled = False
        self.set_state_machine("IDLE")

    def safe_home(self) -> None:
        with self._cmd_lock:
            self.stop_motion()
            # A previous browser stream may still contain arm targets. Clear
            # them before homing, but preserve the measured J7 pose: closing a
            # hand-positioned gripper implicitly can drop or pinch an object.
            self._lowlevel_pos_target = None
            self._mit_stream_target = None
            self._mit_stream_velocity.fill(0.0)
            self._mit_stream_acceleration.fill(0.0)
            self._mit_gripper_target = None
            if self.has_gripper:
                self._hold_gripper_current_locked()
            self.set_state_machine("IDLE")
            if self._gravity_comp_active:
                if self._arm_control_mode == "mit" and self.control_loop_active:
                    hold_target = (
                        self._gravity_comp_q_last.copy()
                        if self._gravity_comp_q_last is not None
                        else self._cached_arm_position.copy()
                    )
                    self._arm_group._mit_kp = self._arm_mit_kp.copy()
                    self._arm_group._mit_kd = self._arm_mit_kd.copy()
                    self._endpos_ctrl._q_target[:] = hold_target
                    self._endpos_ctrl._qd_target[:] = 0.0
                    self._endpos_ctrl._running = True
                    self._control_output_enabled = True
                    self._send_endpos_hold_once()
                    self._robot._ctrl_fn = self._endpos_loop_cb
                    self._gravity_comp_active = False
                    self._clear_gravity_compensation_state()
                else:
                    self.stop_gravity_compensation()
            else:
                self.start_endpos_control()
            self.set_state_machine("SAFE_HOMING")
            self._homing_thread = threading.get_ident()
        try:
            self._endpos_ctrl.safe_home()
        finally:
            if self.has_gripper:
                self._mit_gripper_target = None
            self._homing_thread = None
            self.set_state_machine("IDLE")

    @_locked
    def set_zero(self, joint_name: str = "") -> bool:
        self.stop_motion()
        self._robot.stop_control_loop()
        self._endpos_ctrl._running = False
        if joint_name:
            if joint_name not in self._robot._motor_map:
                raise KeyError(f"unknown joint: {joint_name}")
            self._set_zero_single(joint_name)
        else:
            self._robot.set_zero()
        self._enabled = False
        self.set_state_machine("IDLE")
        return True

    @_locked
    def send_joint_mit_cmd(
        self,
        joint_name: str,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau: float,
    ) -> None:
        index = self._joint_index(joint_name)
        continuing = self.state_machine == "LOWLEVEL_STREAMING" and self.mode == "mit"
        self._begin_lowlevel_streaming("mit")
        self._lowlevel_pos_target = None
        if not continuing or self._mit_stream_target is None:
            self._mit_stream_target = self._endpos_ctrl._q_target.copy()
            self._mit_stream_vlim.fill(_DEFAULT_JOINT_POS_VEL_VLIM_RAD_S)
            self._mit_stream_velocity.fill(0.0)
            self._mit_stream_acceleration.fill(0.0)
            self._mit_stream_last_time = time.perf_counter()
        self._mit_stream_target[index] = float(pos)
        self._mit_stream_vlim[index] = self._safe_positive_limit(
            vel,
            default=_DEFAULT_JOINT_POS_VEL_VLIM_RAD_S,
            maximum=_MAX_JOINT_POS_VEL_VLIM_RAD_S,
        )
        self.set_state_machine("LOWLEVEL_STREAMING")

    @_locked
    def send_joint_pos_vel_cmd(
        self,
        joint_name: str,
        pos: float,
        vlim: float,
    ) -> None:
        index = self._joint_index(joint_name)
        self._begin_lowlevel_streaming("pos_vel")
        if self._lowlevel_pos_target is None:
            self._lowlevel_pos_target = np.array(
                self._arm_group.get_positions(request_feedback=True),
                dtype=np.float64,
                copy=True,
            )
        target_pos = self._lowlevel_pos_target.copy()
        target_vlim = np.array(
            [
                self._safe_positive_limit(
                    value,
                    default=_DEFAULT_JOINT_POS_VEL_VLIM_RAD_S,
                    maximum=_MAX_JOINT_POS_VEL_VLIM_RAD_S,
                )
                for value in getattr(self._arm_group, "_pv_vlim")
            ],
            dtype=np.float64,
        )
        safe_vlim = self._safe_positive_limit(
            vlim,
            default=_DEFAULT_JOINT_POS_VEL_VLIM_RAD_S,
            maximum=_MAX_JOINT_POS_VEL_VLIM_RAD_S,
        )
        target_pos[index] = float(pos)
        target_vlim[index] = safe_vlim
        self._lowlevel_pos_target = target_pos.copy()
        self._arm_group.send_pos_vel(target_pos, vlim=target_vlim)
        self.set_state_machine("LOWLEVEL_STREAMING")

    def current_pose(self):
        q, _, _ = self.get_joint_state()
        q_padded = self._pad_q_for_model(self._gc_model, q, len(self.joint_names))
        position, rotation, _ = self._compute_fk(self._gc_model, q_padded)
        return fk_to_pose(position, rotation)

    def _require_idle(self, what: str) -> None:
        state = self._state_machine
        if state in ("TRAJ_RUNNING", "GRAVITY_COMP", "SAFE_HOMING"):
            raise RuntimeError(f"rejecting {what} in state {state}")

    @_locked
    def begin_trajectory_stream(self) -> None:
        self._require_idle("trajectory stream")
        self.start_endpos_control()
        self._mit_stream_velocity.fill(0.0)
        self._mit_stream_acceleration.fill(0.0)
        self._endpos_ctrl._qd_target[:] = 0.0
        self.set_state_machine("TRAJ_RUNNING")

    def move_to_pose_traj(
        self, x, y, z, roll, pitch, yaw, duration: float
    ) -> bool:
        self.begin_trajectory_stream()
        ok = False
        try:
            ok = bool(
                self._endpos_ctrl.move_to_traj(x, y, z, roll, pitch, yaw, duration)
            )
            self._last_motion_duration = float(
                getattr(self._endpos_ctrl, "_last_traj_duration", duration)
            )
        finally:
            if not ok and self._state_machine == "TRAJ_RUNNING":
                self.set_state_machine("IDLE")
        return ok

    @_locked
    def move_to_pose_ik(self, x, y, z, roll, pitch, yaw) -> tuple[bool, list[float]]:
        self._require_idle("IK target")
        self.start_endpos_control()
        ok = self._endpos_ctrl.move_to_ik(x, y, z, roll, pitch, yaw)
        return bool(ok), [float(v) for v in self._endpos_ctrl._q_target]

    def get_joint_status_codes(self) -> list[int]:
        codes: list[int] = []
        for name in self.joint_names:
            try:
                st = self._robot._motor_map[name].get_state()
                codes.append(int(st.status_code if st is not None else 0))
            except Exception:
                codes.append(0)
        return codes

    # ------------------------------------------------------------------
    # gravity compensation
    # ------------------------------------------------------------------

    @_locked
    def start_gravity_compensation(self) -> None:
        # Starting gravity compensation is idempotent.  Restarting an active
        # loop briefly drops back into position hold and switches every motor
        # mode again, which is both unnecessary and perceptible on hardware.
        if self._gravity_comp_active:
            return
        if self._state_machine in ("TRAJ_RUNNING", "SAFE_HOMING"):
            raise RuntimeError(
                f"rejecting gravity compensation in state {self._state_machine}"
            )
        self.stop_motion()
        self._robot.stop_control_loop()
        self._endpos_ctrl._running = False
        self._control_output_enabled = False

        # Read the real pose before changing mode.  This pose, not zero, is
        # the first MIT target when gravity compensation starts away from home.
        q_hold, qd_hold, _ = self.get_joint_state(request_feedback=True)
        q_hold = q_hold.copy()
        self._cached_arm_position = q_hold.copy()
        self._cached_arm_velocity = qd_hold.copy()
        tau_hold = self._gravity_comp_torque(q_hold)
        self._gravity_comp_q_target = q_hold.copy()
        self._gravity_comp_q_last = q_hold.copy()
        self._gravity_comp_transition_q_hold = q_hold.copy()
        self._gravity_comp_transition_started_at = None
        self._gravity_comp_fault = ""
        self._error_codes = [
            code for code in self._error_codes
            if not code.startswith("GRAVITY_COMP_FAULT:")
        ]

        try:
            # Begin at the already-active position-hold gains.  The control
            # loop follows measured position while smoothly reducing these
            # gains to the compliant gravity-compensation values.
            self._enter_group_mit_at_current(
                self._arm_group,
                q_hold,
                tau_hold,
                self._arm_mit_kp,
                self._arm_mit_kd,
            )
            gripper_hold = None
            if self.has_gripper:
                gripper_hold = self._gripper_group.get_positions().copy()
                self._enter_group_mit_at_current(
                    self._gripper_group,
                    gripper_hold,
                    np.zeros_like(gripper_hold),
                    self._gripper_mit_kp,
                    self._gripper_mit_kd,
                )
                self._cached_gripper_position = float(gripper_hold[0])
                self._gripper_hold_position = float(gripper_hold[0])
                self._gripper_manual_free = False
                self._gripper_assist_velocity = 0.0
                self._gripper_assist_active = True

            if not self._enabled:
                self._arm_group.enable()
                if self.has_gripper:
                    self._gripper_group.enable()
                # Re-send immediately after enabling in case a motor does not
                # retain a command received while disabled.
                self._arm_group.send_mit(
                    q_hold,
                    vel=np.zeros(len(self.joint_names)),
                    kp=self._arm_mit_kp,
                    kd=self._arm_mit_kd,
                    tau=tau_hold,
                )
                if gripper_hold is not None:
                    self._gripper_group.send_mit(gripper_hold)
                self._enabled = True

            # Start the fade only after every motor has entered MIT and is
            # holding the measured pose.  Mode-switch latency must not consume
            # part of the configured transition interval.
            self._gravity_comp_transition_started_at = time.perf_counter()
            self._gravity_comp_active = True
            arm_rate = float(getattr(self._robot, "_rate", 500.0))
            self._gravity_comp_tick(self._robot, 1.0 / arm_rate)
            self._robot.start_control_loop(
                self._gravity_comp_tick_guarded,
                rate=arm_rate,
            )
            self.set_state_machine("GRAVITY_COMP")
        except Exception as exc:
            self._gravity_comp_fault = f"{type(exc).__name__}: {exc}"
            self._gravity_comp_active = False
            self._clear_gravity_compensation_state(keep_fault=True)
            if self._enabled and not self.control_loop_active:
                self._hold_gripper_current_locked()
                self._start_endpos_hold(target=q_hold)
            self.set_state_machine("IDLE")
            raise RuntimeError(
                f"gravity compensation startup failed; restored position hold: {exc}"
            ) from exc

    @_locked
    def stop_gravity_compensation(self) -> None:
        if not self._gravity_comp_active:
            # An idempotent stop must not energize a disabled J7.  Only leave
            # an explicitly requested free/assist mode when the robot is
            # already enabled.
            if (
                self._enabled
                and self.has_gripper
                and (self._gripper_manual_free or self._gripper_assist_active)
            ):
                self._hold_gripper_current_locked()
            return
        hold_target = (
            self._gravity_comp_q_last.copy()
            if self._gravity_comp_q_last is not None
            else None
        )
        self._robot.stop_control_loop()
        self._gravity_comp_active = False
        self._clear_gravity_compensation_state()
        self._error_codes = [
            code for code in self._error_codes
            if not code.startswith("GRAVITY_COMP_FAULT:")
        ]
        if self._enabled:
            # Capture the hand-adjusted J7 pose before the normal holding loop
            # resumes. Otherwise it would jump back to the pre-teach target.
            self._hold_gripper_current_locked()
            self._start_endpos_hold(target=hold_target)
        self.set_state_machine("IDLE")

    def _clear_gravity_compensation_state(self, *, keep_fault: bool = False) -> None:
        self._gravity_comp_q_target = None
        self._gravity_comp_q_last = None
        self._gravity_comp_transition_q_hold = None
        self._gravity_comp_transition_started_at = None
        if not keep_fault:
            self._gravity_comp_fault = ""

    def gravity_compensation_active(self) -> bool:
        return self._gravity_comp_active

    def gravity_compensation_target(self) -> np.ndarray | None:
        target = self._gravity_comp_q_target
        return None if target is None else target.copy()

    def gravity_compensation_fault(self) -> str:
        return self._gravity_comp_fault

    def _gravity_comp_torque(self, q: np.ndarray) -> np.ndarray:
        q_for_model = q * self._gravity_comp_joint_direction
        q_model = self._pad_q_for_model(
            self._gc_model, q_for_model, len(self.joint_names)
        )
        tau_model = self._gc_compute_generalized_gravity(
            self._gc_model,
            q_model,
            self._gc_data,
        )[: len(self.joint_names)]
        tau_motor = (
            tau_model
            * self._gravity_comp_joint_direction
            * self._gravity_comp_tau_scale
        )
        configured_limits = getattr(
            self,
            "_gravity_comp_torque_limit",
            np.zeros(len(self.joint_names), dtype=np.float64),
        )
        limits = np.where(
            configured_limits > 0.0,
            configured_limits,
            np.inf,
        )
        return np.clip(tau_motor, -limits, limits)

    @staticmethod
    def _enter_group_mit_at_current(group, q_hold, tau_hold, kp, kd) -> None:
        """Switch each motor to MIT and immediately command its measured pose."""
        from motorbridge import Mode

        q_hold = np.asarray(q_hold, dtype=np.float64).reshape(-1)
        tau_hold = np.asarray(tau_hold, dtype=np.float64).reshape(-1)
        kp = np.asarray(kp, dtype=np.float64).reshape(-1)
        kd = np.asarray(kd, dtype=np.float64).reshape(-1)
        for index, joint_name in enumerate(group.joint_names):
            motor = group._mm[joint_name]
            motor.ensure_mode(Mode.MIT, 1000)
            motor.send_mit(
                float(q_hold[index]),
                0.0,
                float(kp[index]),
                float(kd[index]),
                float(tau_hold[index]),
            )
        group._mode = "mit"
        group._mit_kp = kp.copy()
        group._mit_kd = kd.copy()

    @staticmethod
    def _angles_near_reference(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
        delta = values - reference
        delta = (delta + np.pi) % (2.0 * np.pi) - np.pi
        return reference + delta

    def _read_gravity_comp_positions(
        self,
        *,
        request: bool = False,
        reference: np.ndarray | None = None,
    ) -> np.ndarray:
        if request:
            q = self._arm_group.get_positions(request_feedback=True)
            self._cached_arm_position = np.array(q, dtype=np.float64, copy=True)
        else:
            # Never issue synchronous RobStride parameter reads from a control
            # loop.  The joint-state publisher refreshes this cache at the
            # configured hardware_feedback_poll_rate.
            q = self._cached_arm_position.copy()
        ref = reference if reference is not None else self._gravity_comp_q_last
        if ref is not None:
            q = self._angles_near_reference(q, ref)
        self._gravity_comp_q_last = np.array(q, dtype=np.float64, copy=True)
        return self._gravity_comp_q_last.copy()

    def _gravity_comp_tick(self, _robot, dt: float) -> None:
        del dt
        if not self._gravity_comp_active:
            return

        q = self._read_gravity_comp_positions(request=False)
        tau_motor = self._gravity_comp_torque(q)
        transition_started_at = self._gravity_comp_transition_started_at
        elapsed = (
            self._gravity_comp_transition_duration
            if transition_started_at is None
            else max(0.0, time.perf_counter() - transition_started_at)
        )
        blend = self._gravity_comp_transition_blend(elapsed)
        kp, kd = self._gravity_comp_transition_gains(elapsed)
        q_hold = self._gravity_comp_transition_q_hold
        q_target = (
            q.copy()
            if q_hold is None
            else q_hold + blend * (q - q_hold)
        )
        if blend >= 1.0:
            self._gravity_comp_transition_q_hold = None
        self._gravity_comp_q_target = q_target.copy()

        self._arm_group.send_mit(
            q_target,
            vel=np.zeros(len(self.joint_names)),
            kp=kp,
            kd=kd,
            tau=tau_motor,
        )
        if self.has_gripper:
            self._send_gripper_guidance_tick()

    def _gravity_comp_tick_guarded(self, robot, dt: float) -> None:
        try:
            self._gravity_comp_tick(robot, dt)
        except Exception as exc:
            fault = f"{type(exc).__name__}: {exc}"
            self._gravity_comp_fault = fault
            self._error_codes = [
                code for code in self._error_codes
                if not code.startswith("GRAVITY_COMP_FAULT:")
            ]
            self._error_codes.append(f"GRAVITY_COMP_FAULT: {fault}")
            self._gravity_comp_active = False
            recovery_target = (
                self._gravity_comp_q_last.copy()
                if self._gravity_comp_q_last is not None
                else None
            )
            # The SDK control loop cannot join itself. Ask it to exit and
            # restore normal holding from a separate recovery thread.
            robot._running = False
            threading.Thread(
                target=self._recover_from_gravity_compensation_fault,
                args=(recovery_target,),
                name="rebotarm-rs-gravity-recovery",
                daemon=True,
            ).start()

    def _recover_from_gravity_compensation_fault(
        self,
        target: np.ndarray | None,
    ) -> None:
        control_thread = getattr(self._robot, "_ctrl_thread", None)
        if (
            control_thread is not None
            and control_thread is not threading.current_thread()
        ):
            control_thread.join(timeout=1.0)
        with self._cmd_lock:
            self._clear_gravity_compensation_state(keep_fault=True)
            try:
                if self._enabled:
                    self._hold_gripper_current_locked()
                    self._start_endpos_hold(target=target)
            except Exception as exc:
                self._gravity_comp_fault += (
                    f"; hold recovery failed: {type(exc).__name__}: {exc}"
                )
                self._error_codes = [
                    f"GRAVITY_COMP_FAULT: {self._gravity_comp_fault}"
                ]
            finally:
                self.set_state_machine("IDLE")

    def _gravity_comp_transition_gains(
        self, elapsed: float
    ) -> tuple[np.ndarray, np.ndarray]:
        blend = self._gravity_comp_transition_blend(elapsed)
        kp = self._arm_mit_kp + blend * (
            self._gravity_comp_kp - self._arm_mit_kp
        )
        kd = self._arm_mit_kd + blend * (
            self._gravity_comp_kd - self._arm_mit_kd
        )
        return kp, kd

    def _gravity_comp_transition_blend(self, elapsed: float) -> float:
        ratio = float(
            np.clip(
                elapsed / max(self._gravity_comp_transition_duration, 1e-6),
                0.0,
                1.0,
            )
        )
        # Smoothstep has zero slope at both ends, avoiding a gain derivative
        # step when entering the compliant mode.
        return ratio * ratio * (3.0 - 2.0 * ratio)

    # ------------------------------------------------------------------
    # gripper
    # ------------------------------------------------------------------

    def _gripper_assist_feedforward(self, position: float, velocity: float) -> float:
        speed = abs(float(velocity))
        if speed <= self._gripper_assist_velocity_threshold:
            return 0.0
        if speed >= self._gripper_assist_speed_limit:
            return 0.0

        direction = 1.0 if velocity > 0.0 else -1.0
        lower = min(self.gripper_close_position, self.gripper_open_position)
        upper = max(self.gripper_close_position, self.gripper_open_position)
        if position <= lower + _GRIPPER_ASSIST_LIMIT_MARGIN_RAD and direction < 0.0:
            return 0.0
        if position >= upper - _GRIPPER_ASSIST_LIMIT_MARGIN_RAD and direction > 0.0:
            return 0.0

        ramp_span = max(
            self._gripper_assist_velocity_full
            - self._gripper_assist_velocity_threshold,
            1e-6,
        )
        ramp = float(
            np.clip(
                (speed - self._gripper_assist_velocity_threshold) / ramp_span,
                0.0,
                1.0,
            )
        )
        ramp = max(ramp, self._gripper_assist_breakaway_fraction)
        return direction * self._gripper_assist_torque * ramp

    def _send_gripper_guidance_tick(self) -> None:
        if not self.has_gripper or self._gripper_manual_free:
            return
        position = float(self._cached_gripper_position)
        if self._gripper_assist_active:
            self._gripper_assist_velocity = (
                0.70 * self._gripper_assist_velocity
                + 0.30 * float(self._cached_gripper_velocity)
            )
            torque = self._gripper_assist_feedforward(
                position,
                self._gripper_assist_velocity,
            )
            self._gripper_group.send_mit(
                np.array([position], dtype=np.float64),
                vel=np.zeros(1, dtype=np.float64),
                kp=np.zeros(1, dtype=np.float64),
                kd=np.array([self._gripper_assist_kd], dtype=np.float64),
                tau=np.array([torque], dtype=np.float64),
            )
            return

        target = (
            position
            if self._gripper_hold_position is None
            else self._gripper_hold_position
        )
        self._gripper_group.send_mit(
            np.array([target], dtype=np.float64),
            kp=self._gripper_mit_kp,
            kd=self._gripper_mit_kd,
        )

    @_locked
    def release_gripper_for_manual(self) -> None:
        """Disable only J7 so the fingers can be positioned by hand."""
        if not self.has_gripper:
            return
        if self._gripper_manual_free and not self._gripper_assist_active:
            return
        self.get_gripper_state(request_feedback=True)
        self._gripper_assist_active = False
        self._gripper_assist_velocity = 0.0
        self._gripper_hold_position = None
        self._gripper_target_position = None
        self._gripper_group.disable()
        self._gripper_manual_free = True

    def _hold_gripper_current_locked(self) -> None:
        if not self.has_gripper:
            return
        current = float(self._cached_gripper_position)
        try:
            current = float(self.get_gripper_state(request_feedback=True)[0])
        except Exception:
            pass
        current = float(
            np.clip(
                current,
                min(self.gripper_close_position, self.gripper_open_position),
                max(self.gripper_close_position, self.gripper_open_position),
            )
        )
        self._gripper_group.mode_mit(
            kp=self._gripper_mit_kp,
            kd=self._gripper_mit_kd,
        )
        self._gripper_group.enable()
        self._gripper_group.send_mit(
            np.array([current], dtype=np.float64),
            kp=self._gripper_mit_kp,
            kd=self._gripper_mit_kd,
        )
        self._cached_gripper_position = current
        self._endpos_ctrl._gripper_target = current
        self._gripper_target_position = current
        self._gripper_hold_position = current
        self._mit_gripper_target = None
        self._gripper_manual_free = False
        self._gripper_assist_active = False
        self._gripper_assist_velocity = 0.0

    @_locked
    def hold_gripper_current(self) -> None:
        """Hold the measured J7 pose without returning to a stale target."""
        self._hold_gripper_current_locked()

    @_locked
    def start_gripper_assist(self) -> None:
        """Enable conservative motion-following torque assistance for J7."""
        if not self.has_gripper:
            raise RuntimeError("gripper is not initialized")
        self.get_gripper_state(request_feedback=True)
        if not self.control_loop_active:
            self._start_endpos_loop()
        self._gripper_group.mode_mit(
            kp=np.zeros(1, dtype=np.float64),
            kd=np.array([self._gripper_assist_kd], dtype=np.float64),
        )
        self._gripper_group.enable()
        self._gripper_manual_free = False
        self._gripper_hold_position = None
        self._gripper_assist_velocity = 0.0
        self._gripper_assist_active = True

    def gripper_assist_active(self) -> bool:
        return bool(self._gripper_assist_active)

    def gripper_manual_free(self) -> bool:
        return bool(self._gripper_manual_free)

    @_locked
    def set_gripper_target(self, position: float) -> None:
        self._begin_gripper_command(allow_endpos=True)
        target = float(position)
        self._endpos_ctrl.set_gripper_target(target)
        self._gripper_group.send_mit(
            np.array([target], dtype=np.float64),
            kp=self._gripper_mit_kp,
            kd=self._gripper_mit_kd,
        )
        self._gripper_target_position = target

    def wait_gripper_target(self, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.gripper_reached_target():
                return True
            time.sleep(0.02)
        return False

    def set_gripper_position(
        self,
        position: float,
        timeout: float = 3.0,
    ) -> tuple[bool, float]:
        self.set_gripper_target(position)
        reached = self.wait_gripper_target(timeout)
        return reached, self.get_gripper_state()[0]

    def get_gripper_state(
        self,
        request_feedback: bool = True,
    ) -> tuple[float, float, float, int]:
        if not self.has_gripper or not self._gripper_name:
            return 0.0, 0.0, 0.0, 0
        if request_feedback:
            # A robot-level refresh includes the arm and gripper in one pass
            # and updates all local caches.
            self._get_arm_state(request_feedback=True)
        pos = self._cached_gripper_position
        vel = self._cached_gripper_velocity
        status = 0
        torque = self._cached_gripper_torque
        try:
            st = self._robot._motor_map[self._gripper_name].get_state()
            if st is not None:
                torque = float(st.torq)
                status = int(st.status_code)
        except Exception:
            status = 0
        return float(pos), float(vel), float(torque), status

    def gripper_reached_target(self) -> bool:
        if self._gripper_target_position is None:
            return True
        pos = self.get_gripper_state()[0]
        return abs(pos - self._gripper_target_position) < _GRIPPER_GOAL_TOLERANCE_RAD

    @_locked
    def send_gripper_mit_cmd(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau: float,
    ) -> None:
        self._begin_gripper_command(allow_endpos=True)
        self._begin_gripper_lowlevel("mit")
        self._mit_gripper_target = float(pos)
        self._mit_gripper_vlim = self._safe_positive_limit(
            vel,
            default=_DEFAULT_GRIPPER_POS_VEL_VLIM_RAD_S,
            maximum=_MAX_GRIPPER_POS_VEL_VLIM_RAD_S,
        )
        self._gripper_target_position = None

    @_locked
    def send_gripper_pos_vel_cmd(self, pos: float, vlim: float) -> None:
        self._begin_gripper_command()
        self._begin_gripper_lowlevel("pos_vel")
        safe_vlim = self._safe_positive_limit(
            vlim,
            default=_DEFAULT_GRIPPER_POS_VEL_VLIM_RAD_S,
            maximum=_MAX_GRIPPER_POS_VEL_VLIM_RAD_S,
        )
        self._gripper_group.send_pos_vel(
            np.array([float(pos)], dtype=np.float64),
            vlim=np.array([safe_vlim], dtype=np.float64),
        )
        self._gripper_target_position = None

    def _begin_gripper_command(self, *, allow_endpos: bool = False) -> None:
        if not self._enabled:
            raise RuntimeError("rejecting gripper command while arm is disabled")
        if self._gravity_comp_active or self.state_machine == "GRAVITY_COMP":
            raise RuntimeError("rejecting gripper command during gravity compensation")
        if (
            self.state_machine == "SAFE_HOMING"
            and threading.get_ident() != self._homing_thread
        ):
            raise RuntimeError("rejecting gripper command during safe home")
        if self.state_machine == "TRAJ_RUNNING" and not allow_endpos:
            raise RuntimeError("rejecting gripper command while trajectory is running")
        if not self.has_gripper or not self._gripper_name:
            raise RuntimeError("gripper is not initialized")
        if self._gripper_manual_free or self._gripper_assist_active:
            self._gripper_group.mode_mit(
                kp=self._gripper_mit_kp,
                kd=self._gripper_mit_kd,
            )
            self._gripper_group.enable()
            self._gripper_manual_free = False
            self._gripper_assist_active = False
            self._gripper_assist_velocity = 0.0
            self._gripper_hold_position = None
        if allow_endpos and not self.control_loop_active:
            # A gripper-only MIT command needs the shared periodic sender. Hold
            # the arm at its measured pose and clear the arm streaming state so
            # a stale web target cannot resume when the gripper starts moving.
            self._start_endpos_loop()
            self.set_state_machine("IDLE")
        if not allow_endpos and self.control_loop_active:
            self.stop_motion()
            self._robot.stop_control_loop()
            self._endpos_ctrl._running = False

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _begin_lowlevel_streaming(self, required_mode: str) -> None:
        if not self._enabled:
            raise RuntimeError("rejecting low-level command while arm is disabled")
        if self._gravity_comp_active or self.state_machine == "GRAVITY_COMP":
            raise RuntimeError("rejecting low-level command during gravity compensation")
        if self.state_machine == "SAFE_HOMING":
            raise RuntimeError("rejecting low-level command during safe home")
        if self.state_machine == "TRAJ_RUNNING":
            self.stop_motion()

        if required_mode == "pos_vel":
            # POS_VEL is executed by the motor controller using its velocity
            # limit.  A concurrent end-pose loop would continuously overwrite
            # these commands, so stop it before entering POS_VEL streaming.
            refresh_target = self.control_loop_active or required_mode != self.mode
            if self.control_loop_active:
                self.stop_motion()
                self._control_output_enabled = False
                self._robot.stop_control_loop()
                self._endpos_ctrl._running = False
            if refresh_target:
                self._lowlevel_pos_target = np.array(
                    self._arm_group.get_positions(request_feedback=True),
                    dtype=np.float64,
                    copy=True,
                )
        elif not self.control_loop_active:
            # Explicit MIT commands retain the short interpolation path.
            self._lowlevel_pos_target = None
            self._start_endpos_loop()

        if required_mode != self.mode:
            self._enter_mode(
                self._arm_group,
                required_mode,
                "arm",
                kp=self._arm_mit_kp,
                kd=self._arm_mit_kd,
            )
        self.set_state_machine("LOWLEVEL_STREAMING")

    def _begin_gripper_lowlevel(self, required_mode: str) -> None:
        # Teaching replay can update J7 several times per second while the arm
        # trajectory uses the same command lock.  JointGroup.mode_mit() waits
        # for hardware mode confirmation and includes setup delays, so calling
        # it for every already-MIT sample starves the 125 Hz arm sender and
        # makes the whole arm jerk.  A mode transition is only needed when the
        # requested mode actually changes.
        if getattr(self._gripper_group, "mode", None) == required_mode:
            return
        self._enter_mode(self._gripper_group, required_mode, "gripper")

    @staticmethod
    def _enter_mode(group, required_mode: str, label: str, **mit_gains) -> None:
        if required_mode == "mit":
            ok = group.mode_mit(**mit_gains)
        elif required_mode == "pos_vel":
            ok = group.mode_pos_vel()
        else:
            raise ValueError(f"unsupported low-level mode: {required_mode}")
        if not ok:
            raise RuntimeError(f"{label} did not enter {required_mode} mode")

    @staticmethod
    def _safe_positive_limit(value: float, *, default: float, maximum: float) -> float:
        try:
            requested = abs(float(value))
        except (TypeError, ValueError):
            requested = default
        if not np.isfinite(requested) or requested <= 0.0:
            requested = default
        return float(np.clip(requested, 0.05, maximum))

    def _start_endpos_hold(self, target: np.ndarray | None = None) -> None:
        if self.control_loop_active:
            return
        self._start_endpos_loop(target)

    def _start_endpos_loop(self, target: np.ndarray | None = None) -> None:
        self._configure_groups_for_endpos()
        if target is None:
            self.hold_current_position()
        else:
            self._endpos_ctrl._q_target[:] = np.asarray(target, dtype=np.float64)
            self._endpos_ctrl._qd_target[:] = 0.0
        self._control_output_enabled = True
        self._robot.start_control_loop(self._endpos_loop_cb)
        self._endpos_ctrl._running = True

    def _configure_groups_for_endpos(self) -> None:
        if self._arm_control_mode == "mit":
            self._arm_group.mode_mit(
                kp=self._arm_mit_kp,
                kd=self._arm_mit_kd,
            )
        else:
            self._arm_group.mode_pos_vel()
        self._arm_group.enable()
        if self.has_gripper:
            if not self._gripper_group.mode_mit(
                kp=self._gripper_mit_kp,
                kd=self._gripper_mit_kd,
            ):
                raise RuntimeError("gripper did not enter mit mode")
            self._gripper_group.enable()

    @_locked
    def stop_motion(self) -> None:
        self._endpos_ctrl._stop_send.set()
        if self._endpos_ctrl._send_thread is not None:
            self._endpos_ctrl._send_thread.join(timeout=5.0)
        self._endpos_ctrl._moving = False
        self._endpos_ctrl._stop_send.clear()

    def motion_active(self) -> bool:
        return bool(self._endpos_ctrl._moving)

    def _endpos_loop_cb(self, robot, dt: float) -> None:
        del dt
        if not self._cmd_lock.acquire(blocking=False):
            return
        try:
            if not self._control_output_enabled:
                return
            now = time.perf_counter()
            loop_dt = float(np.clip(now - self._mit_stream_last_time, 0.0005, 0.1))
            self._mit_stream_last_time = now
            if (
                self.state_machine == "LOWLEVEL_STREAMING"
                and self._mit_stream_target is not None
            ):
                (
                    next_position,
                    next_velocity,
                    next_acceleration,
                ) = advance_jerk_limited_reference_over_elapsed(
                    self._endpos_ctrl._q_target,
                    self._mit_stream_velocity,
                    self._mit_stream_acceleration,
                    self._mit_stream_target,
                    self._mit_stream_vlim,
                    loop_dt,
                    acceleration_limit=self._stream_acceleration_limit,
                    jerk_limit=self._stream_jerk_limit,
                    natural_frequency=self._stream_natural_frequency,
                )
                self._endpos_ctrl._q_target[:] = next_position
                self._mit_stream_velocity[:] = next_velocity
                self._mit_stream_acceleration[:] = next_acceleration
                self._endpos_ctrl._qd_target[:] = self._mit_stream_velocity
            if self._mit_gripper_target is not None:
                gripper_delta = (
                    self._mit_gripper_target - self._endpos_ctrl._gripper_target
                )
                gripper_step = float(
                    np.clip(
                        gripper_delta,
                        -self._mit_gripper_vlim * loop_dt,
                        self._mit_gripper_vlim * loop_dt,
                    )
                )
                self._endpos_ctrl._gripper_target += gripper_step
            if self._arm_control_mode == "mit":
                # Gravity is feed-forward only; feedback remains the motor's
                # MIT position/velocity loop.  Use the most recent measured
                # pose without a CAN transaction in this timing-critical path.
                tau_ff = self._gravity_comp_torque(
                    self._cached_arm_position.copy()
                )
                self._arm_group.send_mit(
                    self._endpos_ctrl._q_target,
                    vel=self._endpos_ctrl._qd_target,
                    kp=getattr(self._arm_group, "_mit_kp"),
                    kd=getattr(self._arm_group, "_mit_kd"),
                    tau=tau_ff,
                )
            else:
                self._arm_group.send_pos_vel(
                    self._endpos_ctrl._q_target,
                    vlim=getattr(self._arm_group, "_pv_vlim"),
                )
            if self.has_gripper:
                if self._gripper_manual_free:
                    pass
                elif self._gripper_assist_active:
                    self._send_gripper_guidance_tick()
                else:
                    self._gripper_group.send_mit(
                        np.array([self._endpos_ctrl._gripper_target]),
                        kp=self._gripper_mit_kp,
                        kd=self._gripper_mit_kd,
                    )
        finally:
            self._cmd_lock.release()

    def _send_endpos_hold_once(self) -> None:
        if self._arm_control_mode == "mit":
            self._arm_group.send_mit(
                self._endpos_ctrl._q_target,
                vel=self._endpos_ctrl._qd_target,
                kp=getattr(self._arm_group, "_mit_kp"),
                kd=getattr(self._arm_group, "_mit_kd"),
            )
        else:
            self._arm_group.send_pos_vel(
                self._endpos_ctrl._q_target,
                vlim=getattr(self._arm_group, "_pv_vlim"),
            )

    def _joint_index(self, joint_name: str) -> int:
        try:
            return self.joint_names.index(joint_name)
        except ValueError as exc:
            raise KeyError(f"unknown joint: {joint_name}") from exc

    def _set_zero_single(self, joint_name: str) -> None:
        self._robot.disable_all()
        time.sleep(0.3)
        motor = self._robot._motor_map[joint_name]
        ctrl = None
        for joint in self._robot._all_joints:
            if joint.name == joint_name:
                ctrl = self._robot._ctrl_map[str(joint.vendor)]
                break
        if ctrl is None:
            raise KeyError(f"unknown joint: {joint_name}")
        for _ in range(200):
            try:
                motor.request_feedback()
                ctrl.poll_feedback_once()
            except Exception:
                pass
            st = motor.get_state()
            if st is not None and st.status_code == 0:
                break
            time.sleep(0.05)
        motor.set_zero_position()
