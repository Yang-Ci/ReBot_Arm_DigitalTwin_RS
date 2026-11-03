#!/usr/bin/env python3
"""Cancel an in-flight reBotArm action goal from the command line.

ROS 2 ships no ``ros2 action cancel`` subcommand, and Jazzy removed the
``ActionClient.async_cancel_all_goals`` convenience method, so this node drives
the underlying ``CancelGoal`` service directly: an all-zero goal id + zero
timestamp means "cancel every executing goal" per the action protocol spec.
"""

from __future__ import annotations

import argparse

import rclpy
from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from rebotarm_msgs.action import MoveToPose
from unique_identifier_msgs.msg import UUID

_ACTIONS = {
    "move_to_pose": MoveToPose,
    "follow_joint_trajectory": FollowJointTrajectory,
    "gripper/command": GripperCommand,
}


def _cancel_all_goals(client: ActionClient) -> Future:
    """Send a cancel-all request through the internal action client handle.

    Mirrors ``ActionClient._cancel_goal_async`` but with an all-zero GoalInfo so
    the server cancels every executing goal instead of a single tracked one.
    """
    # GoalInfo with zero UUID + zero timestamp == cancel all goals.
    cancel_request = CancelGoal.Request()
    cancel_request.goal_info = GoalInfo()
    cancel_request.goal_info.goal_id = UUID()

    future: Future = Future()
    with client._lock:
        sequence_number = client._client_handle.send_cancel_request(
            cancel_request,
        )
        client._pending_cancel_requests[sequence_number] = future
        future.add_done_callback(client._remove_pending_cancel_request)
        client.add_future(future)
    return future


class CancelAction(Node):
    def __init__(self, action_name: str, namespace: str) -> None:
        super().__init__("cancel_action")
        action_type = _ACTIONS.get(action_name)
        if action_type is None:
            raise ValueError(
                f"unknown action '{action_name}', choose from {sorted(_ACTIONS)}"
            )
        self._action_name = action_name
        self._client = ActionClient(
            self,
            action_type,
            f"/{namespace}/{action_name}",
        )

    def run(self) -> int:
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"action server /{self._action_name} not available"
            )
            return 1

        self.get_logger().info(f"requesting cancellation on {self._action_name}")
        future = _cancel_all_goals(self._client)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done() and future.result() is not None:
            response = future.result()
            if response.goals_canceling:
                names = [
                    "".join(f"{b:02x}" for b in g.goal_id.uuid)
                    for g in response.goals_canceling
                ]
                self.get_logger().info(
                    f"canceled {len(response.goals_canceling)} goal(s): {names}"
                )
            else:
                self.get_logger().info(
                    "no active goals matched (nothing to cancel)"
                )
            return 0
        self.get_logger().warn("cancel request timed out or returned no response")
        return 1


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action",
        nargs="?",
        default="move_to_pose",
        choices=sorted(_ACTIONS),
        help="action name to cancel (default: move_to_pose)",
    )
    parser.add_argument("--namespace", default="rebot_arm_rs")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init()
    node = CancelAction(args.action, args.namespace.strip("/"))
    try:
        code = node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
