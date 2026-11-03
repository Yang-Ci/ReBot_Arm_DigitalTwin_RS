import json
import math
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from rebotarm_agent import rebotarm_mcp_server as server
from rebotarm_agent import rebotarm_text_agent as text_agent


def bridge(namespace="rebot_arm_rs_sim"):
    node = object.__new__(server.ReBotArmToolBridge)
    node.namespace = namespace
    node._lock = threading.RLock()
    node._joint_state = node._mujoco_joint_state = None
    node._joint_state_at = node._mujoco_joint_state_at = 0.0
    return node


def state(value):
    return {"positions": {name: value for name in server._ARM_JOINTS}}


def test_simulation_prefers_fresh_mujoco_and_falls_back_to_fresh_driver():
    node = bridge()
    node._joint_state, node._joint_state_at = state(1.0), time.monotonic()
    node._mujoco_joint_state, node._mujoco_joint_state_at = state(2.0), time.monotonic()
    assert node._current_joint_positions()["joint1"] == 2.0
    node._mujoco_joint_state_at -= 2
    assert node._current_joint_positions()["joint1"] == 1.0
    node._joint_state_at -= 2
    assert node._current_joint_positions() is None


def test_hardware_uses_driver_feedback():
    node = bridge("rebot_arm_rs")
    node._joint_state, node._joint_state_at = state(1.0), time.monotonic()
    node._mujoco_joint_state, node._mujoco_joint_state_at = state(2.0), time.monotonic()
    assert node._current_joint_positions()["joint1"] == 1.0


def test_old_detection_is_rejected_even_if_topic_is_publishing():
    node = bridge()
    node._vision_payload_at = time.monotonic()
    node._vision_parse_error = None
    node._vision_payload = {"age_sec": 3.0, "detections": [{"color": "red"}]}
    assert not node._wait_for_detection("red", timeout_sec=0)["ok"]
    node._vision_payload["age_sec"] = 0.01
    assert node._wait_for_detection("red", timeout_sec=0.2)["ok"]


def test_storage_pose_matches_the_reachable_web_route():
    for color, (x, y, z) in server._STORAGE_ZONES.items():
        node = bridge()
        plan = node._build_pick_plan({"color": color, "x": x, "y": y, "z": z}, requested_approach_z=.18, requested_grasp_z=.14, requested_lift_z=.18)
        assert plan["storage_pick"]
        assert plan["approach_z"] == pytest.approx(z + .10)
        assert plan["lift_z"] == pytest.approx(z + .13)
        assert plan["transit_pose"].orientation.y == pytest.approx(math.sin(math.radians(25)))
        assert plan["grasp_pose"].orientation.y == pytest.approx(math.sin(math.radians(32.5)))
    hardware = bridge("rebot_arm_rs")
    plan = hardware._build_pick_plan({"color": "yellow", "x": .49, "y": .16, "z": .126}, requested_approach_z=.18, requested_grasp_z=.14, requested_lift_z=.18)
    assert not plan["storage_pick"]


def test_storage_ik_failure_does_not_send_trajectory():
    node = bridge()
    node._solve_pose_ik = lambda *a, **kw: {"ok": True, "ros_success": False, "q_solution": [0.0] * 6, "message": "large error"}
    node._send_joint_positions = lambda *a, **kw: pytest.fail("unreachable solution was sent")
    result = node._move_to_pose_via_ik_trajectory(None, 1.0, tool="test", strict_ik=True)
    assert not result["ok"]


def test_deepseek_default_and_provider_keys_do_not_cross(monkeypatch):
    for key in ("REBOTARM_LLM_BASE_URL", "REBOTARM_LLM_MODEL", "REBOTARM_LLM_API_KEY", "OPENAI_API_KEY", "DASHSCOPE_API_KEY", "DEEPSEEK_API_KEY"):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv("DASHSCOPE_API_KEY", "wrong-provider")
    args = text_agent._parse_args([])
    assert (args.base_url, args.model, args.api_key) == ("https://api.deepseek.com", "deepseek-flash", "")
    monkeypatch.setenv("DEEPSEEK_API_KEY", "deepseek-test")
    assert text_agent._parse_args([]).api_key == "deepseek-test"
    assert text_agent._parse_args(["--base-url", "https://api.openai.com/v1"]).api_key == ""


def test_deepseek_request_disables_thinking_without_affecting_custom_provider():
    requests = []
    class Response:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self): return b'{}'
    def fake_open(request, timeout):
        requests.append(json.loads(request.data))
        return Response()
    with patch("urllib.request.urlopen", fake_open):
        for url in ("https://api.deepseek.com", "http://localhost:8000/v1"):
            text_agent.ChatCompletionsLLM(base_url=url, api_key="test", model="test", timeout_sec=5, temperature=.7).complete([], [])
    assert requests[0]["thinking"] == {"type": "disabled"}
    assert "thinking" not in requests[1]


def test_deepseek_multi_round_tool_calls(monkeypatch):
    import asyncio

    calls = []
    tool_call = {"id": "call_1", "type": "function", "function": {"name": "get_robot_status", "arguments": "{}"}}
    replies = [
        {"choices": [{"message": {"role": "assistant", "content": None, "tool_calls": [tool_call]}}]},
        {"choices": [{"message": {"role": "assistant", "content": "机械臂已连接。"}}]},
    ]
    class Response:
        def __init__(self, payload): self.payload = payload
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self): return json.dumps(self.payload).encode()
    def fake_open(request, timeout):
        calls.append(json.loads(request.data))
        return Response(replies.pop(0))
    class FakeClient:
        def __init__(self, url): pass
        async def __aenter__(self): return self
        async def __aexit__(self, *args): pass
        async def list_tools(self):
            return [SimpleNamespace(name="get_robot_status", description="status", inputSchema={"type": "object", "properties": {}})]
        async def call_tool(self, name, arguments):
            return SimpleNamespace(data={"ok": True, "namespace": "rebot_arm_rs_sim"})
    monkeypatch.setattr(text_agent, "Client", FakeClient)
    monkeypatch.setattr(text_agent.urllib.request, "urlopen", fake_open)
    monkeypatch.setattr(text_agent, "_parse_builtin_intent", lambda _: None)
    args = text_agent._parse_args(["--api-key", "test"])
    result = asyncio.run(text_agent._http_handle_chat(args, {"text": "请检查机器人", "reset": True}))
    assert result["ok"] and result["text"] == "机械臂已连接。"
    assert all(call["model"] == "deepseek-flash" and call["thinking"] == {"type": "disabled"} for call in calls)
    assert calls[1]["messages"][-2]["tool_calls"] == [tool_call]
    assert calls[1]["messages"][-1]["tool_call_id"] == "call_1"


def test_pick_stops_before_descent_if_detection_expires():
    node = bridge()
    node.motion_allowed = True
    target = {"color": "red", "x": .49, "y": -.16, "z": .1225}
    detections = iter([{"ok": True, "target": target}, {"ok": False, "message": "stale"}])
    node._wait_for_detection = lambda *a, **kw: next(detections)
    node.set_gripper_opening_mm = lambda *a, **kw: {"ok": True}
    movements = []
    def move(*args, **kwargs):
        movements.append(kwargs["tool"])
        return {"ok": True}
    node._move_to_pose_via_ik_trajectory = move
    result = node.pick_color("red")
    assert not result["ok"] and result["failed_step"] == "refresh_detection"
    assert movements == ["pick_color.high_align"]
