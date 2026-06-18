#!/usr/bin/env python3
"""Launch vision_node and logger_node together in a single process using a
MultiThreadedExecutor. This is the container's entrypoint.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.executors import MultiThreadedExecutor

from vision_node import VisionNode
from logger_node import LoggerNode
from web_bridge_node import WebBridgeNode


def main():
    rclpy.init()
    vision = VisionNode()
    logger = LoggerNode()
    bridge = WebBridgeNode()

    executor = MultiThreadedExecutor()
    executor.add_node(vision)
    executor.add_node(logger)
    executor.add_node(bridge)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        vision.destroy_node()
        logger.destroy_node()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
