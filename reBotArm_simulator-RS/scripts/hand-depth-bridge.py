"""Local, on-demand Gemini 336 RGB-D capture. Never sends arm commands."""
import json
import os
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from pyorbbecsdk import (AlignFilter, Config, Context, OBFormat,
                        OBFrameAggregateOutputMode, OBLogLevel, OBSensorType,
                        OBStreamType, Pipeline)

Context.set_logger_level(OBLogLevel.NONE)
TOKEN = os.environ["REBOT_DEPTH_TOKEN"]


class Capture:
    def __init__(self):
        self.lock = threading.Lock()
        self.thread = None
        self.stop_event = threading.Event()
        self.packet = None
        self.seq = 0
        self.state = "stopped"
        self.error = None
        self.last_client = time.monotonic()
        self.device = None

    def status(self):
        with self.lock:
            return dict(state=self.state, error=self.error, device=self.device, seq=self.seq)

    def start(self):
        self.last_client = time.monotonic()
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        with self.lock:
            self.state, self.error, self.packet = "starting", None, None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=4)
        with self.lock:
            self.packet = None

    def run(self):
        pipeline = None
        try:
            context = Context()
            devices = context.query_devices()
            device = next((devices.get_device_by_index(i) for i in range(devices.get_count())
                           if "336" in devices.get_device_by_index(i).get_device_info().get_name()), None)
            if device is None:
                raise RuntimeError("未找到 Gemini 336，请检查 USB 连接。")
            self.device = device.get_device_info().get_name()
            pipeline = Pipeline(device)
            config = Config()
            config.enable_stream(pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
                                 .get_video_stream_profile(640, 480, OBFormat.MJPG, 30))
            config.enable_stream(pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
                                 .get_video_stream_profile(640, 480, OBFormat.Y16, 30))
            config.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
            pipeline.enable_frame_sync()
            align = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
            pipeline.start(config)
            last_frame = time.monotonic()
            clock_offset = None
            while not self.stop_event.is_set():
                if time.monotonic() - self.last_client > 10:
                    break
                frames = pipeline.wait_for_frames(100)
                if not frames:
                    if time.monotonic() - last_frame > 5:
                        raise RuntimeError("336 没有返回图像，请关闭其他相机程序后重试。")
                    continue
                aligned = align.process(frames)
                if not aligned:
                    continue
                frames = aligned.as_frame_set()
                color, depth = frames.get_color_frame(), frames.get_depth_frame()
                if not color or not depth:
                    continue
                width, height = color.get_width(), color.get_height()
                if depth.get_width() != width or depth.get_height() != height:
                    raise RuntimeError("RGB 与深度对齐尺寸不一致，已停止跟随。")
                system_ms = color.get_system_timestamp_us() / 1000
                unix_ms = time.time() * 1000
                if clock_offset is None:
                    clock_offset = 0 if abs(unix_ms - system_ms) < 5000 else unix_ms - system_ms
                captured_ms = system_ms + clock_offset
                depth_mm = np.frombuffer(depth.get_data(), dtype=np.uint16).reshape(height, width)
                depth_mm = np.clip(np.rint(depth_mm * depth.get_depth_scale()), 0, 65535).astype("<u2")
                jpeg = bytes(color.get_data())
                self.seq += 1
                meta = json.dumps(dict(seq=self.seq, capturedUnixMs=captured_ms,
                                       width=width, height=height, jpegBytes=len(jpeg)),
                                  separators=(",", ":")).encode()
                packet = b"RBD1" + struct.pack("<I", len(meta)) + meta + jpeg + depth_mm.tobytes()
                with self.lock:
                    self.packet, self.state = packet, "running"
                last_frame = time.monotonic()
        except Exception as error:
            with self.lock:
                self.error = "336 采集失败，请关闭其他相机预览后重试。" if "HResult" in str(error) else str(error)
                self.state = "error"
        finally:
            if pipeline:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            with self.lock:
                if self.state != "error":
                    self.state = "stopped"
                self.packet = None


capture = Capture()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def handle_request(self):
        if self.headers.get("Authorization") != "Bearer " + TOKEN:
            self.send_error(403)
            return
        capture.last_client = time.monotonic()
        if self.path == "/start" and self.command == "POST":
            capture.start()
        elif self.path == "/stop" and self.command == "POST":
            capture.stop()
        elif self.path == "/frame" and self.command == "GET":
            with capture.lock:
                packet = capture.packet
            self.send_response(200 if packet else 204)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(packet) if packet else 0))
            self.end_headers()
            if packet:
                self.wfile.write(packet)
            return
        elif self.path != "/status" or self.command != "GET":
            self.send_error(404)
            return
        data = json.dumps(capture.status(), ensure_ascii=False).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    do_GET = handle_request
    do_POST = handle_request


if __name__ == "__main__":
    def parent_watchdog():
        # Node owns this pipe. EOF releases the camera if the web server exits.
        sys.stdin.buffer.read()
        capture.stop()
        os._exit(0)

    threading.Thread(target=parent_watchdog, daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    print("REBOT_DEPTH_READY " + str(server.server_port), flush=True)
    server.serve_forever()
