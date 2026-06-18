"""Views for the vision monitor dashboard.

The dashboard reads everything the ROS2 container writes into ./shared/:
    state.json        -> /api/state
    detections.db     -> /api/logs, /api/history
    latest_frame.jpg  -> /video_feed
Mode switching is done by `docker exec`-ing into the ROS2 container and
publishing on /vision/mode_cmd.
"""
import json
import os
import shlex
import sqlite3
import subprocess

from django.conf import settings
from django.http import HttpResponse, JsonResponse, HttpResponseBadRequest
from django.shortcuts import render
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_POST

SHARED_DIR = settings.SHARED_DIR
STATE_PATH = os.path.join(SHARED_DIR, "state.json")
FRAME_PATH = os.path.join(SHARED_DIR, "latest_frame.jpg")
DB_PATH = os.path.join(SHARED_DIR, "detections.db")
PLACEHOLDER_PATH = os.path.join(
    os.path.dirname(__file__), "static", "monitor", "placeholder.jpg"
)

VALID_MODES = ("motion", "edge", "raw")


def index(request):
    return render(request, "monitor/index.html")


def api_state(request):
    """Latest vision state from state.json."""
    try:
        with open(STATE_PATH) as fh:
            state = json.load(fh)
        state["online"] = True
    except (FileNotFoundError, json.JSONDecodeError):
        state = {
            "online": False,
            "mode": "unknown",
            "object_count": 0,
            "fps": 0,
            "proc_time_ms": 0,
            "timestamp": 0,
        }
    return JsonResponse(state)


def _query_db(query, params=()):
    if not os.path.exists(DB_PATH):
        return []
    # read-only connection so we never block the logger node's writes
    uri = f"file:{DB_PATH}?mode=ro"
    conn = sqlite3.connect(uri, uri=True, timeout=3.0)
    conn.row_factory = sqlite3.Row
    try:
        rows = conn.execute(query, params).fetchall()
        return [dict(r) for r in rows]
    finally:
        conn.close()


def api_logs(request):
    """Most recent detection rows (newest first)."""
    try:
        limit = min(int(request.GET.get("limit", 25)), 200)
    except ValueError:
        limit = 25
    try:
        rows = _query_db(
            "SELECT id, ts, mode, object_count, fps, proc_time_ms "
            "FROM detections ORDER BY id DESC LIMIT ?",
            (limit,),
        )
    except sqlite3.Error:
        rows = []
    return JsonResponse({"logs": rows})


def api_history(request):
    """Time-series of object_count for charting (oldest -> newest)."""
    try:
        limit = min(int(request.GET.get("limit", 60)), 500)
    except ValueError:
        limit = 60
    try:
        rows = _query_db(
            "SELECT ts, object_count, fps FROM detections "
            "ORDER BY id DESC LIMIT ?",
            (limit,),
        )
    except sqlite3.Error:
        rows = []
    rows.reverse()
    return JsonResponse(
        {
            "ts": [r["ts"] for r in rows],
            "object_count": [r["object_count"] for r in rows],
            "fps": [r["fps"] for r in rows],
        }
    )


def video_feed(request):
    """Serve the latest annotated frame, or a placeholder if unavailable."""
    path = FRAME_PATH if os.path.exists(FRAME_PATH) else PLACEHOLDER_PATH
    try:
        with open(path, "rb") as fh:
            data = fh.read()
    except FileNotFoundError:
        return HttpResponse(status=404)
    resp = HttpResponse(data, content_type="image/jpeg")
    resp["Cache-Control"] = "no-store, must-revalidate"
    return resp


@csrf_exempt
@require_POST
def set_mode(request):
    """Change vision_node's mode by publishing on /vision/mode_cmd."""
    mode = (request.POST.get("mode") or "").strip().lower()
    if mode not in VALID_MODES:
        return HttpResponseBadRequest(
            json.dumps({"ok": False, "error": f"invalid mode '{mode}'"}),
            content_type="application/json",
        )

    container = settings.ROS2_CONTAINER

    def run(inner):
        cmd = ["docker", "exec", container, "bash", "-lc", inner]
        return subprocess.run(cmd, capture_output=True, text=True, timeout=15)

    # Primary: ROS2 service call (custom vision_interfaces/srv/SetMode).
    svc_inner = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /ros2_ws/install/setup.bash && "
        f"ros2 service call /vision/set_mode vision_interfaces/srv/SetMode '{{mode: {mode}}}'"
    )
    # Fallback: topic publish (works even without the custom interface, e.g. CPU image).
    # -w 1 waits for the subscription to be matched so the one-shot isn't lost.
    topic_inner = (
        "source /opt/ros/jazzy/setup.bash && "
        f"ros2 topic pub --once -w 1 /vision/mode_cmd std_msgs/msg/String '{{data: {mode}}}'"
    )

    try:
        proc = run(svc_inner)
        via = "service"
        if proc.returncode != 0 or "response:" not in (proc.stdout or ""):
            proc = run(topic_inner)
            via = "topic"
    except subprocess.TimeoutExpired:
        return JsonResponse({"ok": False, "error": "docker exec timed out"}, status=504)
    except FileNotFoundError:
        return JsonResponse({"ok": False, "error": "docker not found on host"}, status=500)

    if proc.returncode != 0:
        return JsonResponse(
            {"ok": False, "error": proc.stderr.strip() or "docker exec failed"},
            status=500,
        )
    return JsonResponse({"ok": True, "mode": mode, "via": via})


@csrf_exempt
@require_POST
def clear_logs(request):
    """Clear the detection log via logger_node's ROS2 service (std_srvs/Trigger)."""
    container = settings.ROS2_CONTAINER
    inner = (
        "source /opt/ros/jazzy/setup.bash && "
        "ros2 service call /logger/clear_log std_srvs/srv/Trigger '{}'"
    )
    cmd = ["docker", "exec", container, "bash", "-lc", inner]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    except subprocess.TimeoutExpired:
        return JsonResponse({"ok": False, "error": "docker exec timed out"}, status=504)
    except FileNotFoundError:
        return JsonResponse({"ok": False, "error": "docker not found on host"}, status=500)

    ok = proc.returncode == 0 and "response:" in (proc.stdout or "")
    return JsonResponse(
        {"ok": ok, "error": None if ok else (proc.stderr.strip() or "clear failed")},
        status=200 if ok else 500,
    )
