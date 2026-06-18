# NOTE: switched from osrf/ros:jazzy-ros-base -> ros:jazzy-ros-base.
# The osrf/ros repo has no "jazzy-ros-base" tag (manifest unknown); the
# official library image is the same ROS2 Jazzy base and is what's published.
FROM ros:jazzy-ros-base

# System OpenCV (pulls in all native libs: libGL, libglib, etc.) plus numpy.
# Using the apt package avoids the Ubuntu 24.04 "externally-managed-environment"
# pip restriction and guarantees the shared libraries OpenCV needs are present.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-opencv \
        python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# ROS2 nodes
COPY ros2_nodes/ /ros2_nodes/

ENV VIDEO_PATH=/test_video.mp4 \
    SHARED_DIR=/shared \
    PYTHONUNBUFFERED=1

# Source the ROS2 environment then launch both nodes.
CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && exec python3 /ros2_nodes/run_nodes.py"]
