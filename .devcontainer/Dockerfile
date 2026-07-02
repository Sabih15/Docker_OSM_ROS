# syntax=docker/dockerfile:1

# ──────────────────────────────────────────────────────────────────────────
#  Stage 1 — builder: compile the C++ node and install the Python package.
#  All the heavy build tooling (colcon, gcc, cmake) lives ONLY here and is
#  discarded; the runtime image never carries it.
# ──────────────────────────────────────────────────────────────────────────
FROM ros:jazzy-ros-base AS builder
SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-dev-tools \
        build-essential \
        cmake \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY ros2_ws/src ./src

# merge-install gives a single, easy-to-copy install tree.
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# ──────────────────────────────────────────────────────────────────────────
#  Stage 2 — runtime: ros-base already provides rclpy, rclcpp, sensor_msgs
#  and geometry_msgs. We add only python3-websockets and copy the built
#  workspace + the static web UI. No compilers, no colcon.
# ──────────────────────────────────────────────────────────────────────────

FROM ros:jazzy-ros-base AS runtime
SHELL ["/bin/bash", "-lc"]

# ros-base already provides rclpy, rclcpp, sensor_msgs, geometry_msgs.
# Add only the websockets lib via apt (v12.0 on Noble) — no pip, no PEP 668.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-websockets \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY --from=builder /ros2_ws/install /ros2_ws/install
COPY web /web
COPY .devcontainer/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV IMPL=py
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ARG USERNAME=ubuntu

RUN apt-get update && apt-get install -y --no-install-recommends sudo && \
    rm -rf /var/lib/apt/lists/* && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

EXPOSE 8000 9090
ENTRYPOINT ["/entrypoint.sh"]
