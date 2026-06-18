#!/usr/bin/env python3
"""Logger node: subscribes to the vision topics and logs detection events
into a SQLite database in the shared volume (./shared/detections.db).

A row is written at most once per second, recording the latest metric values.
This keeps the DB small and avoids lock contention with the Django reader.
"""
import os
import sqlite3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String

SHARED_DIR = os.environ.get("SHARED_DIR", "/shared")
DB_PATH = os.path.join(SHARED_DIR, "detections.db")
LOG_INTERVAL = 1.0  # seconds between DB rows


class LoggerNode(Node):
    def __init__(self):
        super().__init__("logger_node")
        os.makedirs(SHARED_DIR, exist_ok=True)

        self.object_count = 0
        self.fps = 0.0
        self.proc_time_ms = 0.0
        self.mode = "motion"
        self._last_log = 0.0

        self._init_db()

        self.create_subscription(Int32, "/vision/object_count", self._on_count, 10)
        self.create_subscription(Float32, "/vision/fps", self._on_fps, 10)
        self.create_subscription(Float32, "/vision/proc_time_ms", self._on_proc, 10)
        self.create_subscription(String, "/vision/mode_state", self._on_mode, 10)

        # Periodic writer so we always log even if a topic is quiet
        self.create_timer(LOG_INTERVAL, self._log_row)
        self.get_logger().info(f"logger_node started, writing {DB_PATH}")

    def _init_db(self):
        conn = sqlite3.connect(DB_PATH, timeout=5.0)
        # WAL improves concurrent read (Django) + write (here)
        conn.execute("PRAGMA journal_mode=WAL;")
        conn.execute(
            """
            CREATE TABLE IF NOT EXISTS detections (
                id           INTEGER PRIMARY KEY AUTOINCREMENT,
                ts           REAL    NOT NULL,
                mode         TEXT    NOT NULL,
                object_count INTEGER NOT NULL,
                fps          REAL    NOT NULL,
                proc_time_ms REAL    NOT NULL
            )
            """
        )
        conn.commit()
        conn.close()

    def _on_count(self, msg: Int32):
        self.object_count = int(msg.data)

    def _on_fps(self, msg: Float32):
        self.fps = float(msg.data)

    def _on_proc(self, msg: Float32):
        self.proc_time_ms = float(msg.data)

    def _on_mode(self, msg: String):
        self.mode = msg.data

    def _log_row(self):
        now = time.time()
        if now - self._last_log < LOG_INTERVAL:
            return
        self._last_log = now
        try:
            conn = sqlite3.connect(DB_PATH, timeout=5.0)
            conn.execute(
                "INSERT INTO detections (ts, mode, object_count, fps, proc_time_ms) "
                "VALUES (?, ?, ?, ?, ?)",
                (now, self.mode, self.object_count, self.fps, self.proc_time_ms),
            )
            conn.commit()
            conn.close()
            if self.object_count > 0:
                self.get_logger().info(
                    f"[{self.mode}] objects={self.object_count} fps={self.fps:.1f}"
                )
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"DB write failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
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
