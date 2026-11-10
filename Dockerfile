# Go1 simulator: Webots R2025a, ROS 2 Jazzy and the Go2 walking policy's runtime (MNN).
# Build with ./docker/run.sh build. The repo, unitree_ws and policies are mounted at run time, not copied in.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LIBGL_ALWAYS_SOFTWARE=1

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg python3-venv python3-numpy \
        python3-colcon-common-extensions ros-jazzy-sensor-msgs \
 && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://cyberbotics.com/Cyberbotics.asc | gpg --dearmor -o /usr/share/keyrings/cyberbotics.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/cyberbotics.gpg] https://cyberbotics.com/debian/ binary-amd64/" \
        > /etc/apt/sources.list.d/cyberbotics.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends webots \
 && rm -rf /var/lib/apt/lists/*

# Same as the laptop setup: the venv sees the system numpy and rclpy, MNN is the only extra package.
RUN python3 -m venv --system-site-packages /opt/sim-venv \
 && /opt/sim-venv/bin/pip install --no-cache-dir --timeout 300 --retries 10 "MNN>=3.0,<4"

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

WORKDIR /ws/go1_sim
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["webots", "--mode=realtime", "/ws/go1_sim/worlds/go1.wbt"]
