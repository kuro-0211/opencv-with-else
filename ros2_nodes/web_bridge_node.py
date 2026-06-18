#!/usr/bin/env python3
"""Web bridge node: the dashboard's data path through ROS2.

Subscribes to the frame + state that vision_node PUBLISHES on ROS2 topics and
writes them into the shared volume for the Django dashboard:
    /vision/image_annotated (sensor_msgs/Image) -> ./shared/latest_frame.jpg
    /vision/state           (std_msgs/String JSON) -> ./shared/state.json

This means the live frame/metrics the web UI shows are produced by a ROS2
subscriber, i.e. they actually flow over ROS2 (vision_node -> [ROS2] -> bridge
-> file -> Django), rather than vision_node writing files directly.
"""
import os
import tempfile

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image

SHARED_DIR = os.environ.get("SHARED_DIR", "/shared")
FRAME_PATH = os.path.join(SHARED_DIR, "latest_frame.jpg")
STATE_PATH = os.path.join(SHARED_DIR, "state.json")


class WebBridgeNode(Node):
    def __init__(self):
        super().__init__("web_bridge_node")
        os.makedirs(SHARED_DIR, exist_ok=True)

        img_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, "/vision/image_annotated", self._on_image, img_qos)
        self.create_subscription(String, "/vision/state", self._on_state, 10)
        self.get_logger().info(f"web_bridge_node started -> writing {SHARED_DIR}")

    def _atomic_write(self, path, data, text=False):
        """Write atomically (temp + rename) and world-readable so the host's
        Django user can read these root-owned files from the bind mount."""
        fd, tmp = tempfile.mkstemp(dir=SHARED_DIR)
        with os.fdopen(fd, "w" if text else "wb") as fh:
            fh.write(data)
        os.chmod(tmp, 0o644)
        os.replace(tmp, path)

    def _on_image(self, msg: Image):
        try:
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            arr = arr.reshape(msg.height, msg.width, 3)
            ok, buf = cv2.imencode(".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                self._atomic_write(FRAME_PATH, buf.tobytes())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"frame write failed: {e}")

    def _on_state(self, msg: String):
        try:
            self._atomic_write(STATE_PATH, msg.data, text=True)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"state write failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WebBridgeNode()
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
