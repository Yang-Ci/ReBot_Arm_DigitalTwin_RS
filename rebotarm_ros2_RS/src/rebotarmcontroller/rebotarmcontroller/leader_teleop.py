"""ROS service/UI host. UART lives in a killable worker; CAN stays in the driver."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rebotarm_msgs.msg import LeaderDevice, LeaderStatus
from rebotarm_msgs.srv import LeaderControl

from .leader_policy import LeaderMapping, finite_vector, SAMPLE_MAX_AGE_S, SAMPLE_CLOCK_SKEW_S


class UartProcess:
    def __init__(self, port, on_frame=None, *, probe=False, zero=False, unlock=False):
        self.events = queue.Queue()
        self.on_frame = on_frame
        self.active = True
        command = [sys.executable, "-u", "-m", "rebotarmcontroller.leader_worker", "--port", port]
        if probe:
            command.append("--probe")
        if zero:
            command.append("--zero")
        if unlock:
            command.append("--unlock")
        # Ensure this package is importable in both colcon symlink and installed builds.
        env = os.environ.copy()
        env["PYTHONPATH"] = str(Path(__file__).resolve().parent.parent) + os.pathsep + env.get("PYTHONPATH", "")
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                        text=True, bufsize=1, env=env)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.process.stdout:
            try:
                frame = json.loads(line)
            except (ValueError, TypeError):
                continue
            if not self.active:
                break
            if frame.get("event") in ("ready", "probe", "fatal"):
                self.events.put(frame)
            if self.on_frame:
                self.on_frame(self, frame)
        if self.active:
            frame = {"event": "fatal", "message": "UART worker exited; check leader USB and SDK"}
            self.events.put(frame)
            if self.on_frame:
                self.on_frame(self, frame)

    def ready(self, timeout=3.0):
        try:
            frame = self.events.get(timeout=timeout)
        except queue.Empty:
            self.close()
            raise RuntimeError("leader probe timed out; check port/power/permissions")
        if frame["event"] == "fatal":
            self.close()
            raise RuntimeError(frame["message"])
        return frame

    def close(self):
        self.active = False
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=0.5)
        self.reader.join(timeout=0.5)
        if not self.reader.is_alive():
            self.process.stdout.close()


class LeaderTeleop:
    def __init__(self, node, hardware, namespace, *, allow_mock=False):
        self.node, self.hw, self.namespace = node, hardware, namespace
        self.allow_mock = allow_mock
        self.lock = threading.RLock()
        self.operations = threading.Lock()
        self.worker = None
        self.port = ""
        self.state = "DISCONNECTED"
        self.message = "Connect ROS, then scan for Arm102 leader"
        self.calibrated = False
        self.angles = []
        self.sample_at = 0.0
        self.sample_seq = -1
        self.sample_hz = 0.0
        self.session = ""
        self.mapping = None
        self.targets = []
        self.speed = 0.3
        self.follow_gripper = True
        self.absolute = False
        # Configurable without depending on the full LeRobot training environment.
        node.declare_parameter("leader_joint_directions", [1.0, 1.0, -1.0, -1.0, -1.0, 1.0])
        self.directions = list(node.get_parameter("leader_joint_directions").value)
        self.group = ReentrantCallbackGroup()
        self.pub = node.create_publisher(LeaderStatus, f"/{namespace}/leader/status", 1)
        self.service = node.create_service(LeaderControl, f"/{namespace}/leader/control",
                                           self.control, callback_group=self.group)
        self.timer = node.create_timer(0.05, self.tick, callback_group=self.group)

    def control(self, request, response):
        try:
            operation = request.operation
            # Keep heartbeat and stop responsive while a scan/connection probes UART.
            if operation in ("heartbeat", "pause", "stop"):
                with self.lock:
                    self._require_session(request.session_id)
                    if operation == "heartbeat":
                        self.hw.teleop_heartbeat(self.session)
                    elif operation == "pause":
                        self.hw.teleop_pause(self.session)
                        self.state, self.message = "PAUSED", "Paused: follower holds its current pose"
                    else:
                        self._stop("Stopped: follower holds its current pose")
            else:
                if not self.operations.acquire(blocking=False):
                    raise RuntimeError("leader operation is busy; wait for it to finish")
                try:
                    self._operation(operation, request, response)
                finally:
                    self.operations.release()
            response.success = True
            response.message = self.message if operation != "heartbeat" else "heartbeat accepted"
            response.session_id = self.session
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _operation(self, operation, request, response):
        if operation == "scan":
            with self.lock:
                if self.worker or self.session:
                    raise RuntimeError("disconnect leader before scanning serial ports")
            response.devices = self.scan(request.port)
            self.message = "Scan complete; select a full leader (servo IDs 0–6)"
        elif operation in ("connect", "unlock", "calibrate"):
            with self.lock:
                if self.session:
                    raise RuntimeError("stop teleoperation before connecting or calibrating")
                port = request.port.strip() if operation == "connect" else self.port
                if not port or (port == "mock" and not self.allow_mock):
                    raise RuntimeError("select an actual serial port; mock is simulation-only")
                if operation == "calibrate" and (not self.worker or not request.confirm_zero):
                    raise RuntimeError("confirm leader is at the reference zero with gripper closed")
                if operation == "unlock" and not self.worker:
                    raise RuntimeError("connect leader before unlocking it")
                self.calibrated = False
                self.state = "CONNECTING" if operation == "connect" else "CALIBRATING"
                old, self.worker = self.worker, None
                self.angles, self.sample_at, self.sample_seq = [], 0, -1
            if old:
                old.close()
            worker = UartProcess(port, self.on_frame, zero=operation == "calibrate", unlock=operation == "unlock")
            try:
                worker.ready()
            except Exception:
                worker.close()
                with self.lock:
                    self.state = "FAULT"
                    self.message = "Leader connection/calibration failed; retry explicitly"
                raise
            with self.lock:
                self.worker = worker
                self.port = port
                self.calibrated = operation == "calibrate"
                self.state = "READY" if self.calibrated else "PREVIEW"
                self.message = "Leader ready; begin with relative follow" if self.calibrated else (
                    "Read-only preview; confirm zero to unlock/calibrate leader before following")
                if operation == "unlock":
                    self.message = "Leader unlocked; position it at reference zero, close gripper, then confirm zero"
        elif operation == "disconnect":
            with self.lock:
                if self.session:
                    self._require_session(request.session_id)
                    self._stop("Leader disconnected; follower holding")
                old, self.worker = self.worker, None
                self.state, self.port, self.calibrated = "DISCONNECTED", "", False
                self.angles, self.sample_at = [], 0
                self.message = "Leader disconnected"
            if old:
                old.close()
        elif operation in ("start", "resume"):
            with self.lock:
                if not self.calibrated or not self.worker or time.monotonic() - self.sample_at > SAMPLE_MAX_AGE_S:
                    raise RuntimeError("leader is uncalibrated or its samples are stale")
                if operation == "start":
                    if self.session:
                        raise RuntimeError("leader session already running")
                    speed_limit = float(getattr(self.hw, "teleop_speed_max", 0.6))
                    if not math.isfinite(request.speed) or not 0.05 <= request.speed <= speed_limit:
                        raise ValueError(
                            f"speed must be 0.05..{speed_limit:g} rad/s"
                        )
                    self.speed = request.speed
                    self.follow_gripper = request.follow_gripper and self.hw.has_gripper
                    self.absolute = request.absolute
                else:
                    self._require_session(request.session_id)
                    if self.state != "PAUSED":
                        raise RuntimeError("only a paused session can resume")
                # Snapshot and acquisition share the driver command lock.
                with self.hw._cmd_lock:
                    positions = self.hw.get_joint_state(request_feedback=False)[0]
                    gripper = self.hw.get_gripper_state(request_feedback=False)[0] if self.hw.has_gripper else 0
                    mapping = LeaderMapping(self.angles, positions, gripper,
                        absolute=self.absolute, directions=self.directions, limits=self.hw.teleop_limits,
                        gripper_close=self.hw.gripper_close_position if self.hw.has_gripper else 0.0,
                        gripper_open=self.hw.gripper_open_position if self.hw.has_gripper else 5.0)
                    targets, grip = mapping.map(self.angles)
                    if self.absolute and (max(abs(a - b) for a, b in zip(targets, positions)) > 0.15 or
                                          (self.follow_gripper and abs(grip - gripper) > 0.3)):
                        raise RuntimeError("absolute poses differ; align arms or use relative follow")
                    if operation == "start":
                        self.session = self.hw.teleop_acquire()
                    else:
                        self.hw.teleop_resume(self.session)
                    self.mapping = mapping
                    try:
                        self.hw.teleop_send(self.session, self.sample_seq, self.sample_at,
                            targets, grip if self.follow_gripper else None, self.speed)
                    except Exception:
                        self._stop("Failed to start; follower holding")
                        raise
                self.targets = targets
                self.state, self.message = "FOLLOWING", "Following leader; pause to hold, stop to release control"
        else:
            raise ValueError(f"unsupported leader operation: {operation}")

    def scan(self, selected=""):
        if selected == "mock":
            if not self.allow_mock:
                raise RuntimeError("mock is simulation-only")
            device = LeaderDevice()
            device.port, device.description = "mock", "Mock leader (simulation only)"
            device.complete, device.responding_ids = True, list(range(7))
            return [device]
        try:
            from serial.tools import list_ports
        except ImportError:
            if not self.allow_mock:
                raise RuntimeError("missing pyserial; install requirements-rs-leader.txt")
            ports = []
        else:
            ports = [p for p in list_ports.comports()
                     if not p.device.startswith("/dev/ttyS")]
        if selected and selected not in [p.device for p in ports]:
            from types import SimpleNamespace
            ports = [SimpleNamespace(device=selected, description="Manual port", serial_number="")]
        elif selected:
            ports = [p for p in ports if p.device == selected]
        devices = []
        for port in ports[:12]:
            device = LeaderDevice()
            device.port = port.device
            device.description = port.description or "USB UART"
            device.serial_number = port.serial_number or ""
            if "can" in device.description.lower():
                device.error = "CAN adapter skipped; manually enter UART port if needed"
            else:
                probe = None
                try:
                    probe = UartProcess(device.port, probe=True)
                    frame = probe.ready(timeout=1.5)
                    responding_ids = [int(i) for i in frame.get("ids", [])]
                    device.responding_ids = responding_ids
                    device.complete = responding_ids == list(range(7))
                    if not device.complete:
                        device.error = "Missing IDs: " + str(sorted(set(range(7)) - set(responding_ids)))
                except Exception as exc:
                    device.error = str(exc)
                finally:
                    if probe:
                        probe.close()
            devices.append(device)
        if self.allow_mock and (not selected or selected == "mock"):
            device = LeaderDevice()
            device.port, device.description = "mock", "Mock leader (simulation only)"
            device.complete, device.responding_ids = True, list(range(7))
            devices.append(device)
        return devices

    def on_frame(self, worker, frame):
        with self.lock:
            if worker is not self.worker:
                return
            if frame.get("event") == "sample":
                try:
                    sampled_at = float(frame["at"])
                    age = time.monotonic() - sampled_at
                    if not -SAMPLE_CLOCK_SKEW_S <= age < SAMPLE_MAX_AGE_S:
                        return
                    angles = finite_vector(frame["angles"], 7)
                    seq = int(frame["seq"])
                    if seq <= self.sample_seq:
                        raise RuntimeError("UART frame sequence went backwards")
                    if self.sample_at:
                        self.sample_hz = 1 / max(sampled_at - self.sample_at, 1e-6)
                    self.sample_at, self.sample_seq, self.angles = sampled_at, seq, angles
                    if self.state == "FOLLOWING" and self.session:
                        targets, grip = self.mapping.map(angles)
                        self.hw.teleop_send(self.session, seq, sampled_at, targets,
                            grip if self.follow_gripper else None, self.speed)
                        self.targets = targets
                except Exception as exc:
                    self._fault(str(exc))
            elif frame.get("event") in ("error", "fatal"):
                self.message = frame.get("message", "leader read failed")
                if self.session:
                    self._fault(self.message)
                elif frame["event"] == "fatal":
                    self.state, self.calibrated = "FAULT", False

    def _require_session(self, session):
        if not session or session != self.session:
            raise RuntimeError("leader session expired or belongs to another page")

    def _stop(self, reason):
        if self.session and self.hw.teleop_owned:
            self.hw.teleop_stop(self.session, reason)
        self.session, self.mapping = "", None
        self.state = "READY" if self.calibrated else "PREVIEW"
        self.message = reason

    def _fault(self, reason):
        self._stop(reason)
        self.state, self.message, self.calibrated = "FAULT", reason, False

    def tick(self):
        with self.lock:
            reason = self.hw.teleop_watchdog()
            if self.session and not self.hw.teleop_owned:
                self._fault(reason or self.hw._teleop_lease.reason or "leader session ended")
            if self.worker and self.sample_at and time.monotonic() - self.sample_at > SAMPLE_MAX_AGE_S:
                if self.session:
                    self._fault("leader sample timed out")
            status = LeaderStatus()
            status.stamp = self.node.get_clock().now().to_msg()
            status.state, status.message, status.port = self.state, self.message, self.port
            status.session_id, status.calibrated = self.session, self.calibrated
            status.following, status.paused = self.state == "FOLLOWING", self.state == "PAUSED"
            status.sample_age = time.monotonic() - self.sample_at if self.sample_at else -1.0
            status.sample_hz = self.sample_hz
            status.angles_deg, status.targets_rad = list(self.angles), list(self.targets)
            self.pub.publish(status)

    def shutdown(self):
        with self.lock:
            self._stop("controller shutdown")
            old, self.worker = self.worker, None
        if old:
            old.close()
