FROM ros:humble

SHELL ["/bin/bash", "-c"]

WORKDIR /workspace/ros2demo

# keep the image simple: ROS2 base + colcon + numpy
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY config/ ./config/
COPY launch/ ./launch/
COPY scripts/ ./scripts/
COPY src/ ./src/
COPY README.md ./
COPY results/.gitkeep ./results/.gitkeep

RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select lidar_perception

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch lidar_perception demo_launch.py"]
