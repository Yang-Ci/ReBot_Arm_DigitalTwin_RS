"""Desktop controls for the ROS MuJoCo wrist camera and TCP marker."""
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

LABELS = {"d405": "RealSense D405", "d435i": "RealSense D435i", "gemini2": "Orbbec Gemini 2", "uvc32": "32×32 UVC"}


class WristCameraSelector(Node):
    def __init__(self):
        super().__init__("rebotarm_rs_wrist_camera_selector")
        self.declare_parameter("arm_namespace", "rebotarm_rs")
        self.declare_parameter("simulation_node", "/rebotarm_rs_mujoco")
        namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self.client = AsyncParameterClient(self, str(self.get_parameter("simulation_node").value))
        self.active = None
        self.pending = False
        self.tcp_active = None
        self.tcp_pending = False
        self.root = tk.Tk()
        self.root.title("MuJoCo · Wrist Camera")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.root.quit)
        frame = ttk.Frame(self.root, padding=16)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="Wrist camera", font=("", 13, "bold")).pack(anchor="w")
        self.value = tk.StringVar()
        self.combo = ttk.Combobox(frame, textvariable=self.value,
                                  values=list(LABELS.values()), state="disabled", width=28)
        self.combo.pack(fill="x", pady=(10, 8))
        self.combo.bind("<<ComboboxSelected>>", self._select)
        self.tcp_value = tk.BooleanVar(value=False)
        self.tcp_checkbox = ttk.Checkbutton(
            frame, text="显示 TCP 红点 / Show TCP marker", variable=self.tcp_value,
            command=self._select_tcp, state="disabled",
        )
        self.tcp_checkbox.pack(anchor="w", pady=(0, 8))
        self.status = tk.StringVar(value="Connecting to MuJoCo…")
        ttk.Label(frame, textvariable=self.status, wraplength=290).pack(anchor="w")
        ttk.Label(frame, text="MuJoCo: C switches cameras; T shows/hides the TCP marker.",
                  wraplength=290).pack(anchor="w", pady=(12, 0))
        self.create_subscription(
            String, f"/{namespace}/mujoco/wrist_camera_model", self._state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            Bool, f"/{namespace}/mujoco/tcp_marker_visible", self._tcp_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )
        self.root.after(30, self._spin)

    def _state(self, message):
        if message.data not in LABELS:
            return
        self.active = message.data
        self.value.set(LABELS[self.active])
        self.status.set(f"Installed: {LABELS[self.active]}")

    def _select(self, _event=None):
        name = next((key for key, label in LABELS.items() if label == self.value.get()), None)
        if name is None or self.pending:
            return
        if not self.client.services_are_ready():
            self.status.set("MuJoCo is unavailable; reconnecting…")
            if self.active:
                self.value.set(LABELS[self.active])
            return
        self.pending = True
        self.combo.configure(state="disabled")
        self.status.set(f"Switching to {LABELS[name]}…")
        future = self.client.set_parameters([Parameter("wrist_camera_model", value=name)])
        future.add_done_callback(self._done)

    def _tcp_state(self, message):
        self.tcp_active = message.data
        self.tcp_value.set(self.tcp_active)

    def _select_tcp(self):
        if self.tcp_pending or not self.client.services_are_ready():
            self.tcp_value.set(bool(self.tcp_active))
            return
        self.tcp_pending = True
        self.tcp_checkbox.configure(state="disabled")
        future = self.client.set_parameters([
            Parameter("show_tcp_marker", value=self.tcp_value.get())
        ])
        future.add_done_callback(self._tcp_done)

    def _tcp_done(self, future):
        self.tcp_pending = False
        try:
            result = future.result().results[0]
            if not result.successful:
                raise RuntimeError(result.reason)
        except Exception as error:
            self.status.set(f"TCP marker switch failed: {error}")
            self.tcp_value.set(bool(self.tcp_active))

    def _done(self, future):
        self.pending = False
        try:
            result = future.result().results[0]
            if not result.successful:
                raise RuntimeError(result.reason)
            self.status.set(f"Installed: {self.value.get()}")
        except Exception as error:
            self.status.set(f"Switch failed: {error}")
            if self.active:
                self.value.set(LABELS[self.active])

    def _spin(self):
        if not rclpy.ok():
            self.root.quit()
            return
        rclpy.spin_once(self, timeout_sec=0.0)
        connected = self.active is not None and self.client.services_are_ready()
        self.combo.configure(state="readonly" if connected and not self.pending else "disabled")
        tcp_connected = self.tcp_active is not None and self.client.services_are_ready()
        self.tcp_checkbox.configure(
            state="normal" if tcp_connected and not self.tcp_pending else "disabled"
        )
        self.root.after(30, self._spin)

    def destroy_node(self):
        self.root.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WristCameraSelector()
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
