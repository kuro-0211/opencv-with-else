#!/usr/bin/env python3
"""Vision node: reads a video file, runs one of three processing modes
(motion / edge / raw), publishes metrics on ROS2 topics, and writes the
latest annotated frame + a state.json into the shared volume.

Topics published:
    /vision/object_count   std_msgs/Int32
    /vision/fps            std_msgs/Float32
    /vision/proc_time_ms   std_msgs/Float32
    /vision/mode_state     std_msgs/String

Topic subscribed:
    /vision/mode_cmd       std_msgs/String   ("motion" | "edge" | "raw")
"""
import json
import os
import time
import tempfile
from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Float32, String
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult

try:
    from vision_interfaces.srv import SetMode
    _HAS_SETMODE = True
except Exception:  # interface not built (e.g. CPU image) -> service disabled
    SetMode = None
    _HAS_SETMODE = False

VIDEO_PATH = os.environ.get("VIDEO_PATH", "/test_video.mp4")
SHARED_DIR = os.environ.get("SHARED_DIR", "/shared")
FRAME_PATH = os.path.join(SHARED_DIR, "latest_frame.jpg")
STATE_PATH = os.path.join(SHARED_DIR, "state.json")

VALID_MODES = ("motion", "edge", "raw")
MIN_CONTOUR_AREA = 500  # px, ignore tiny noise blobs

# Processing rate control:
#   TARGET_FPS="auto"  -> match the video file's native fps (default)
#   TARGET_FPS="0"/"max" -> uncapped, process as fast as possible
#   TARGET_FPS="<n>"   -> cap at n fps
TARGET_FPS = os.environ.get("TARGET_FPS", "auto").strip().lower()
# Disk writes (latest_frame.jpg + state.json) are throttled to this rate
# regardless of processing speed, so uncapped processing isn't bottlenecked by I/O.
WRITE_FPS = float(os.environ.get("WRITE_FPS", "15"))

# Video decode backend:
#   "gpu"/"nvdec" -> NVIDIA NVDEC via PyNvVideoCodec (falls back to CPU on failure)
#   "cpu"         -> OpenCV VideoCapture (CPU/ffmpeg)
DECODE_BACKEND = os.environ.get("DECODE_BACKEND", "cpu").strip().lower()


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        os.makedirs(SHARED_DIR, exist_ok=True)

        self.mode = "motion"
        self.fps = 0.0
        self.proc_time_ms = 0.0
        self.object_count = 0
        # rolling window of recent frame timestamps for accurate fps
        self._frame_times = deque()

        # MOG2 background subtractor for motion mode (CPU fallback path)
        self.bg_sub = cv2.createBackgroundSubtractorMOG2(
            history=300, varThreshold=25, detectShadows=False
        )

        # ---- CUDA / GPU acceleration (auto-detect, graceful CPU fallback) ----
        self.use_gpu = False
        self.device = "cpu"
        try:
            if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                self.cuda_stream = cv2.cuda.Stream()
                # GPU MOG2 + Canny (constructed once, reused every frame)
                self.gpu_mog2 = cv2.cuda.createBackgroundSubtractorMOG2(
                    300, 25.0, False
                )
                self.gpu_canny = cv2.cuda.createCannyEdgeDetector(80, 160)
                self.gpu_mat = cv2.cuda_GpuMat()
                self.use_gpu = True
                self.device = "cuda"
                name = cv2.cuda.printShortCudaDeviceInfo(0) if hasattr(
                    cv2.cuda, "printShortCudaDeviceInfo") else ""
                self.get_logger().info(
                    f"CUDA enabled: {cv2.cuda.getCudaEnabledDeviceCount()} device(s) "
                    f"-> GPU pipeline active {name}"
                )
            else:
                self.get_logger().info("No CUDA device visible -> CPU pipeline")
        except Exception as e:  # noqa: BLE001
            self.use_gpu = False
            self.device = "cpu"
            self.get_logger().warn(f"CUDA init failed, using CPU: {e}")

        # Publishers
        self.pub_count = self.create_publisher(Int32, "/vision/object_count", 10)
        self.pub_fps = self.create_publisher(Float32, "/vision/fps", 10)
        self.pub_proc = self.create_publisher(Float32, "/vision/proc_time_ms", 10)
        self.pub_mode = self.create_publisher(String, "/vision/mode_state", 10)

        # Annotated frame over ROS2 (sensor_msgs/Image) + full dashboard state.
        # web_bridge_node subscribes to these and writes ./shared, so the data
        # the dashboard reads flows THROUGH ROS2 instead of being written here.
        img_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub_image = self.create_publisher(Image, "/vision/image_annotated", img_qos)
        self.pub_state = self.create_publisher(String, "/vision/state", 10)

        # Subscriber for mode commands.
        # IMPORTANT: put it in its own callback group so it runs concurrently
        # with the processing timer. With the default (shared mutually-exclusive)
        # group, an uncapped/high-rate timer starves this callback and mode
        # commands are never applied.
        self.cmd_cbg = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            String, "/vision/mode_cmd", self._on_mode_cmd, 10,
            callback_group=self.cmd_cbg,
        )

        # Mode can be changed THREE ways, all converging on _apply_mode():
        #   1) topic   /vision/mode_cmd        (above)
        #   2) service /vision/set_mode        (custom SetMode.srv)
        #   3) parameter `mode`                (ros2 param set)
        if _HAS_SETMODE:
            self.create_service(
                SetMode, "/vision/set_mode", self._on_set_mode_srv,
                callback_group=self.cmd_cbg,
            )
            self.get_logger().info("Service ready: /vision/set_mode (vision_interfaces/SetMode)")
        else:
            self.get_logger().warn("vision_interfaces not found -> /vision/set_mode service disabled")
        self.declare_parameter("mode", self.mode)
        self.add_on_set_parameters_callback(self._on_set_param)

        # ---- Open video: GPU NVDEC decode (PyNvVideoCodec) or CPU (OpenCV) ----
        self.cap = None
        self._gpu_decoder = None
        self._gpu_iter = None
        self.decode_backend = "cpu"
        src_fps = 30.0

        if DECODE_BACKEND in ("gpu", "nvdec", "cuda"):
            try:
                import PyNvVideoCodec as nvc  # noqa: N813
                self._nvc = nvc
                self._gpu_decoder = nvc.SimpleDecoder(
                    VIDEO_PATH,
                    use_device_memory=False,  # NVDEC decodes on GPU, copies to host
                    output_color_type=nvc.OutputColorType.RGB,
                )
                self._gpu_iter = iter(self._gpu_decoder)
                self.decode_backend = "nvdec"
                self.get_logger().info(
                    f"Decode backend: GPU NVDEC (PyNvVideoCodec) on {VIDEO_PATH}"
                )
            except Exception as e:  # noqa: BLE001
                self._gpu_decoder = None
                self.get_logger().warn(f"GPU decode unavailable, using CPU: {e}")

        if self._gpu_decoder is None:
            self.cap = cv2.VideoCapture(VIDEO_PATH)
            if not self.cap.isOpened():
                self.get_logger().error(f"Could not open video: {VIDEO_PATH}")
            else:
                self.get_logger().info(f"Decode backend: CPU (OpenCV) on {VIDEO_PATH}")
            sf = self.cap.get(cv2.CAP_PROP_FPS)
            if sf and 0 < sf <= 120:
                src_fps = sf

        if TARGET_FPS in ("0", "max", "uncapped", "unlimited"):
            # Uncapped: fire as fast as the executor allows; real throughput is
            # limited by per-frame processing time, not this period.
            self.timer_period = 0.0
            self.get_logger().info("FPS cap: UNCAPPED (max throughput)")
        elif TARGET_FPS in ("auto", ""):
            self.timer_period = 1.0 / src_fps
            self.get_logger().info(f"FPS cap: auto ({src_fps:.1f} from video)")
        else:
            try:
                tf = float(TARGET_FPS)
                self.timer_period = 1.0 / tf if tf > 0 else 0.0
                self.get_logger().info(f"FPS cap: {tf:.1f}")
            except ValueError:
                self.timer_period = 1.0 / src_fps
                self.get_logger().warn(f"Bad TARGET_FPS '{TARGET_FPS}', using {src_fps:.1f}")

        self._write_period = (1.0 / WRITE_FPS) if WRITE_FPS > 0 else 0.0
        self._last_write = 0.0
        self.create_timer(self.timer_period, self._process_frame)

        # Publish initial mode state immediately
        self._publish_mode_state()
        self.get_logger().info(f"vision_node started in '{self.mode}' mode @ {src_fps:.1f} fps")

    # ------------------------------------------------------------------ #
    def _apply_mode(self, mode):
        """Central mode setter used by the topic, service and parameter paths.
        Returns (success, message)."""
        m = (mode or "").strip().lower()
        if m not in VALID_MODES:
            self.get_logger().warn(f"Ignoring invalid mode: '{mode}'")
            return False, f"invalid mode '{mode}' (valid: {', '.join(VALID_MODES)})"
        if m != self.mode:
            self.mode = m
            self.get_logger().info(f"Mode -> {self.mode}")
            self._publish_mode_state()
        return True, f"mode={m}"

    # 1) topic /vision/mode_cmd
    def _on_mode_cmd(self, msg: String):
        self._apply_mode(msg.data)

    # 2) service /vision/set_mode
    def _on_set_mode_srv(self, request, response):
        ok, msg = self._apply_mode(request.mode)
        response.success = ok
        response.message = msg
        return response

    # 3) parameter `mode`
    def _on_set_param(self, params):
        for p in params:
            if p.name == "mode":
                ok, msg = self._apply_mode(p.value)
                if not ok:
                    return SetParametersResult(successful=False, reason=msg)
        return SetParametersResult(successful=True)

    def _publish_mode_state(self):
        m = String()
        m.data = self.mode
        self.pub_mode.publish(m)

    # ------------------------------------------------------------------ #
    def _read_frame(self):
        """Return the next BGR frame (looping the video), or None."""
        if self._gpu_decoder is not None:
            try:
                frame = next(self._gpu_iter)
            except StopIteration:
                self._gpu_iter = iter(self._gpu_decoder)  # loop
                try:
                    frame = next(self._gpu_iter)
                except StopIteration:
                    return None
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"GPU decode error, switching to CPU: {e}")
                self._gpu_decoder = None
                self.decode_backend = "cpu"
                self.cap = cv2.VideoCapture(VIDEO_PATH)
                return self._read_frame()
            # NVDEC gave host RGB (HWC); convert to BGR for the pipeline
            rgb = np.from_dlpack(frame)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if self.cap is None or not self.cap.isOpened():
            return None
        ok, frame = self.cap.read()
        if not ok:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # loop
            ok, frame = self.cap.read()
            if not ok:
                return None
        return frame

    def _process_frame(self):
        frame = self._read_frame()
        if frame is None:
            return

        t0 = time.perf_counter()

        if self.mode == "motion":
            annotated, count = self._do_motion(frame)
        elif self.mode == "edge":
            annotated, count = self._do_edge(frame)
        else:  # raw
            annotated, count = frame.copy(), 0

        proc_ms = (time.perf_counter() - t0) * 1000.0
        self.proc_time_ms = proc_ms
        self.object_count = count

        # Measure real fps as a true 1-second sliding-window average
        # (matches `ros2 topic hz`; robust to scheduling jitter).
        now = time.perf_counter()
        self._frame_times.append(now)
        cutoff = now - 1.0
        while self._frame_times and self._frame_times[0] < cutoff:
            self._frame_times.popleft()
        span = now - self._frame_times[0]
        self.fps = (len(self._frame_times) - 1) / span if span > 0 else 0.0

        self._draw_overlay(annotated)
        self._publish(annotated)

    def _gpu_fallback(self, err):
        """Disable the GPU path permanently after a runtime CUDA error so the
        node keeps running on CPU instead of crashing mid-demo."""
        if self.use_gpu:
            self.use_gpu = False
            self.device = "cpu"
            self.get_logger().warn(f"GPU op failed, falling back to CPU: {err}")

    def _do_motion(self, frame):
        # ---- foreground mask: GPU (cuda MOG2) or CPU (MOG2) ----
        mask = None
        if self.use_gpu:
            try:
                self.gpu_mat.upload(frame)
                gpu_mask = self.gpu_mog2.apply(self.gpu_mat, -1.0, self.cuda_stream)
                self.cuda_stream.waitForCompletion()
                mask = gpu_mask.download()
            except Exception as e:  # noqa: BLE001
                self._gpu_fallback(e)
        if mask is None:
            mask = self.bg_sub.apply(frame)

        # contour analysis is CPU-only in OpenCV
        mask = cv2.medianBlur(mask, 5)
        _, mask = cv2.threshold(mask, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        count = 0
        annotated = frame.copy()
        for c in contours:
            if cv2.contourArea(c) < MIN_CONTOUR_AREA:
                continue
            count += 1
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return annotated, count

    def _do_edge(self, frame):
        # ---- Canny edges: GPU (cuda) or CPU ----
        edges = None
        if self.use_gpu:
            try:
                self.gpu_mat.upload(frame)
                gpu_gray = cv2.cuda.cvtColor(self.gpu_mat, cv2.COLOR_BGR2GRAY)
                gpu_edges = self.gpu_canny.detect(gpu_gray)
                edges = gpu_edges.download()
            except Exception as e:  # noqa: BLE001
                self._gpu_fallback(e)
        if edges is None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 80, 160)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        count = sum(1 for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA)
        annotated = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        return annotated, count

    def _draw_overlay(self, img):
        h = img.shape[0]
        cv2.rectangle(img, (0, 0), (img.shape[1], 28), (0, 0, 0), -1)
        dev = self.device.upper()
        text = (f"[{dev}|dec:{self.decode_backend}] mode={self.mode}  "
                f"objects={self.object_count}  fps={self.fps:.1f}  "
                f"proc={self.proc_time_ms:.1f}ms")
        color = (0, 255, 0) if self.device == "cuda" else (0, 200, 255)
        cv2.putText(img, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    # ------------------------------------------------------------------ #
    def _publish(self, annotated):
        # Throttle ALL telemetry (topic publishes + disk writes) to WRITE_FPS.
        # The processing loop runs at TARGET_FPS; publishing every frame floods
        # the logger's subscriptions and caps real throughput, so we decouple it.
        now = time.perf_counter()
        if self._write_period and (now - self._last_write) < self._write_period:
            return
        self._last_write = now

        # Metric topics (consumed by logger_node; also demonstrable via ros2 CLI)
        c = Int32(); c.data = int(self.object_count); self.pub_count.publish(c)
        f = Float32(); f.data = float(self.fps); self.pub_fps.publish(f)
        p = Float32(); p.data = float(self.proc_time_ms); self.pub_proc.publish(p)
        self._publish_mode_state()

        # Annotated frame over ROS2 (sensor_msgs/Image, bgr8) -> web_bridge_node
        try:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            msg.height, msg.width = annotated.shape[:2]
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = annotated.shape[1] * 3
            msg.data = annotated.tobytes()
            self.pub_image.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"image publish failed: {e}")

        # Full dashboard state as JSON on /vision/state -> web_bridge_node
        state = {
            "mode": self.mode,
            "device": self.device,
            "decode": self.decode_backend,
            "object_count": int(self.object_count),
            "fps": round(float(self.fps), 2),
            "proc_time_ms": round(float(self.proc_time_ms), 2),
            "timestamp": time.time(),
        }
        s = String(); s.data = json.dumps(state); self.pub_state.publish(s)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
