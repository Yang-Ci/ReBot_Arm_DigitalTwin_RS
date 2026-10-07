from __future__ import annotations

import time

from control_msgs.action import FollowJointTrajectory, GripperCommand
import numpy as np
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rebotarm_msgs.action import MoveToPose

from .conversions import pose_to_xyz_rpy
from .trajectory_profiles import (
    retime_cubic_hermite,
    sample_cubic_hermite,
)

_MAX_ACTION_JOINT_SPEED_RAD_S = 0.60
_MAX_TEACH_REPLAY_JOINT_SPEED_RAD_S = 1.20
_TEACH_REPLAY_FRAME_ID = "rebotarm_teaching_replay"


class ArmActions:
    def __init__(self, node, hardware, namespace: str) -> None:
        self._node = node
        self._hardware = hardware
        self._namespace = namespace
        self._move_to_pose_server = ActionServer(
            node,
            MoveToPose,
            f"/{namespace}/move_to_pose",
            execute_callback=self.execute_move_to_pose,
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )
        self._follow_joint_trajectory_server = ActionServer(
            node,
            FollowJointTrajectory,
            f"/{namespace}/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )
        self._gripper_command_server = ActionServer(
            node,
            GripperCommand,
            f"/{namespace}/gripper/command",
            execute_callback=self.execute_gripper_command,
            goal_callback=self.gripper_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )

    def arm_goal_callback(self, _goal_request):
        return self._gate_goal(
            ("TRAJ_RUNNING", "GRAVITY_COMP", "SAFE_HOMING"), "arm motion"
        )

    def gripper_goal_callback(self, _goal_request):
        return self._gate_goal(("GRAVITY_COMP", "SAFE_HOMING"), "gripper")

    def _gate_goal(self, blocked, label):
        if self._hardware.teleop_owned:
            self._node.get_logger().warn(f"rejecting {label}: leader owns control")
            return GoalResponse.REJECT
        state = self._hardware.state_machine
        if state in blocked:
            self._node.get_logger().warn(f"rejecting {label} goal in state {state}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _fail_move_to_pose(self, goal_handle, result, message, *, canceled=False):
        self._release_action_state()
        if canceled:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        result.success = False
        result.message = message
        result.final_pose = self._hardware.current_pose()
        return result

    def _hold_after_failure(self):
        with self._hardware._cmd_lock:
            if not self._hardware.teleop_owned:
                self._hardware.hold_current_position()

    def _release_action_state(self):
        # A goal accepted before leader acquisition can fail during execution.
        # Its cleanup must not overwrite the newly acquired leader state.
        with self._hardware._cmd_lock:
            if not self._hardware.teleop_owned and self._hardware.state_machine != "SAFE_HOMING":
                self._hardware.set_state_machine("IDLE")
                self._node.publish_arm_status()

    def execute_move_to_pose(self, goal_handle):
        goal = goal_handle.request
        result = MoveToPose.Result()

        try:
            x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(goal.target_pose)
            ok = self._hardware.move_to_pose_traj(
                x, y, z, roll, pitch, yaw, float(goal.duration)
            )
        except Exception as exc:
            self._hold_after_failure()
            return self._fail_move_to_pose(goal_handle, result, str(exc))

        if not ok:
            return self._fail_move_to_pose(
                goal_handle, result, "trajectory planning failed"
            )
        self._node.publish_arm_status()

        effective_duration = max(
            float(goal.duration),
            float(self._hardware.last_motion_duration),
            0.0,
        )
        deadline = time.monotonic() + effective_duration + 2.0
        while self._hardware.motion_active():
            if self._hardware.state_machine == "SAFE_HOMING":
                self._hardware.stop_motion()
                break
            if goal_handle.is_cancel_requested:
                self._hardware.stop_motion()
                self._hardware.hold_current_position()
                return self._fail_move_to_pose(
                    goal_handle, result, "move_to_pose canceled", canceled=True
                )
            if time.monotonic() > deadline:
                self._hardware.stop_motion()
                self._hardware.hold_current_position()
                return self._fail_move_to_pose(
                    goal_handle, result, "move_to_pose timeout"
                )
            time.sleep(0.02)

        if self._hardware.state_machine == "SAFE_HOMING":
            return self._fail_move_to_pose(
                goal_handle, result, "move_to_pose preempted by safe_home"
            )

        positions = self._hardware.get_joint_positions()
        velocities = self._hardware.get_joint_velocities()
        result.success = True
        result.message = (
            f"move_to_traj accepted duration={effective_duration:.2f}s "
            f"positions={[float(v) for v in positions]} "
            f"velocities={[float(v) for v in velocities]}"
        )
        result.final_pose = self._hardware.current_pose()
        self._hardware.set_state_machine("IDLE")
        self._node.publish_arm_status()
        goal_handle.succeed()
        return result

    def execute_follow_joint_trajectory(self, goal_handle):
        goal = goal_handle.request
        result = FollowJointTrajectory.Result()
        trajectory = goal.trajectory
        joint_names = list(trajectory.joint_names)

        if not joint_names or not trajectory.points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory must include joint_names and points"
            return result

        if joint_names != self._hardware.joint_names:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = (
                f"trajectory joint_names must be {self._hardware.joint_names}"
            )
            return result

        targets = [np.array(point.positions, dtype=np.float64) for point in trajectory.points]
        if any(len(target) != len(self._hardware.joint_names) for target in targets):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "point positions length must match joint_names"
            return result

        try:
            self._hardware.begin_trajectory_stream()
            self._node.publish_arm_status()
            point_times = [
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) * 1e-9
                for point in trajectory.points
            ]
            current = self._hardware.get_joint_positions().copy()
            if point_times[0] > 0.0 or np.max(np.abs(targets[0] - current)) > 1e-6:
                targets.insert(0, current)
                point_times.insert(0, 0.0)
            velocity_limit = (
                _MAX_TEACH_REPLAY_JOINT_SPEED_RAD_S
                if trajectory.header.frame_id == _TEACH_REPLAY_FRAME_ID
                else _MAX_ACTION_JOINT_SPEED_RAD_S
            )
            prepare_started_at = time.monotonic()
            point_times, waypoint_velocities = retime_cubic_hermite(
                targets,
                point_times,
                velocity_limit=velocity_limit,
            )
            prepare_elapsed = time.monotonic() - prepare_started_at
            if trajectory.header.frame_id == _TEACH_REPLAY_FRAME_ID:
                self._node.get_logger().info(
                    "teaching replay prepared: "
                    f"points={len(targets)}, preparation={prepare_elapsed:.3f}s, "
                    f"duration={point_times[-1]:.3f}s"
                )
            # Retiming a full teaching recording may process up to 1500
            # waypoints.  Start the playback clock only after that work is
            # complete; otherwise the first samples are already overdue and
            # are emitted back-to-back as the loop tries to catch up.
            start = time.monotonic()

            for index in range(1, len(targets)):
                q0 = targets[index - 1]
                q1 = targets[index]
                v0 = waypoint_velocities[index - 1]
                v1 = waypoint_velocities[index]
                t0 = point_times[index - 1]
                t1 = max(point_times[index], t0)

                while True:
                    if self._hardware.state_machine == "SAFE_HOMING":
                        goal_handle.abort()
                        result.error_code = (
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        )
                        result.error_string = (
                            "follow_joint_trajectory preempted by safe_home"
                        )
                        return result
                    now = time.monotonic() - start
                    ratio = 1.0 if t1 <= t0 else max(0.0, min(1.0, (now - t0) / (t1 - t0)))
                    target, desired_velocities = sample_cubic_hermite(
                        q0, q1, v0, v1, t1 - t0, ratio
                    )
                    self._hardware.update_trajectory_reference(
                        target, desired_velocities
                    )

                    positions = self._hardware.get_joint_positions()
                    velocities = self._hardware.get_joint_velocities()
                    feedback = FollowJointTrajectory.Feedback()
                    feedback.header.stamp = self._node.get_clock().now().to_msg()
                    feedback.joint_names = self._hardware.joint_names
                    feedback.desired.positions = [float(v) for v in target]
                    feedback.desired.velocities = [float(v) for v in desired_velocities]
                    feedback.actual.positions = [float(v) for v in positions]
                    feedback.actual.velocities = [float(v) for v in velocities]
                    feedback.error.positions = [float(v) for v in target - positions]
                    feedback.error.velocities = [
                        float(v) for v in desired_velocities - velocities
                    ]
                    goal_handle.publish_feedback(feedback)

                    if goal_handle.is_cancel_requested:
                        self._hardware.hold_current_position()
                        goal_handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "follow_joint_trajectory canceled"
                        return result

                    if now >= t1:
                        break
                    time.sleep(0.01)

            final_target = targets[-1]
            self._hardware.update_trajectory_reference(
                final_target, np.zeros_like(final_target)
            )
            settle_deadline = time.monotonic() + 2.0
            while time.monotonic() < settle_deadline:
                if self._hardware.state_machine == "SAFE_HOMING":
                    goal_handle.abort()
                    result.error_code = (
                        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    )
                    result.error_string = (
                        "follow_joint_trajectory preempted by safe_home"
                    )
                    return result
                if goal_handle.is_cancel_requested:
                    self._hardware.hold_current_position()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "follow_joint_trajectory canceled"
                    return result
                positions = self._hardware.get_joint_positions()
                velocities = self._hardware.get_joint_velocities()
                if (
                    np.max(np.abs(final_target - positions)) <= 0.025
                    and np.max(np.abs(velocities)) <= 0.15
                ):
                    break
                time.sleep(0.02)

        except Exception as exc:
            self._hold_after_failure()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = f"execution failed: {exc}"
            return result
        finally:
            self._release_action_state()

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        positions = self._hardware.get_joint_positions()
        velocities = self._hardware.get_joint_velocities()
        result.error_string = (
            "joint target accepted "
            f"positions={[float(v) for v in positions]} "
            f"velocities={[float(v) for v in velocities]}"
        )
        return result

    def execute_gripper_command(self, goal_handle):
        goal = goal_handle.request.command
        result = GripperCommand.Result()
        feedback = GripperCommand.Feedback()

        try:
            self._hardware.set_gripper_target(goal.position)
        except Exception:
            goal_handle.abort()
            result.position = 0.0
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            return result

        start = time.monotonic()
        last_pos = self._hardware.get_gripper_state()[0]
        stalled = False
        while time.monotonic() - start < 5.0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                pos, _, effort, _ = self._hardware.get_gripper_state()
                result.position = pos
                result.effort = effort
                result.stalled = stalled
                result.reached_goal = False
                return result

            pos, _, effort, _ = self._hardware.get_gripper_state()
            reached = self._hardware.gripper_reached_target()
            stalled = abs(pos - last_pos) < 1e-4 and abs(effort) >= float(goal.max_effort)
            feedback.position = pos
            feedback.effort = effort
            feedback.stalled = stalled
            feedback.reached_goal = reached
            goal_handle.publish_feedback(feedback)
            if reached:
                break
            last_pos = pos
            time.sleep(0.05)

        pos, _, effort, _ = self._hardware.get_gripper_state()
        result.position = pos
        result.effort = effort
        result.stalled = stalled
        result.reached_goal = self._hardware.gripper_reached_target()
        goal_handle.succeed()
        return result
