# ROS2 LiDAR Perception Demo

ROS2 demo for LiDAR point cloud playback, BEV projection, simple clustering, and RViz2 visualization.

This project grew out of my MEng research discussions at Western University. My research project was on cooperative BEV reconstruction, where the main question was how to recover an occluded area using LiDAR information from another vehicle view. During that work, my supervisor and I narrowed a lot of the discussion down to the perception side, especially how much can be done from LiDAR-only input before getting into heavier model design. Based on that, I wanted a separate ROS2 project that is smaller, cleaner, and easier to explain as an engineering demo.

So this repo focuses on one practical slice of the problem: LiDAR in, perception outputs out. The idea is to keep the system simple enough to run and inspect, but still structured enough to talk about playback, projection, detection, visualization, and later profiling.

## Architecture

```text
pointcloud_player
    -> /points_raw
    -> bev_projection
        -> /bev_tensor
        -> /bev_image
    -> cluster_detector
        -> /detections
    -> latency_logger
        -> results/latency_stats.csv
        -> results/detection_summary.csv
```

## Quick start

### Local

```bash
colcon build --packages-select lidar_perception
source install/setup.bash
ros2 launch lidar_perception demo_launch.py
```

### Docker

```bash
docker compose up
```

Docker support is planned for a later step. Right now the local ROS2 path is the first target.

## Nodes

### pointcloud_player

Publishes a synthetic `sensor_msgs/PointCloud2` stream for now.
This is the first step so the rest of the pipeline can be tested before I wire in a public rosbag or PCD sample.

### bev_projection

Subscribes to the raw point cloud and builds a fixed 8-height-channel BEV.
It publishes:

- `/bev_tensor` for the full `(H, W, 8)` representation
- `/bev_image` as a simple `mono8` preview for RViz2

### cluster_detector

Planned ROS2 node for simple Euclidean clustering and RViz2 marker publishing.

### latency_logger

Planned logger for per-frame timing and simple detection-rate statistics.

## Configuration

Current parameters live in `config/demo.yaml`.

Current parameters cover:

- synthetic point cloud topic and publish settings
- BEV tensor and preview topics
- fixed x/y/z projection range
- BEV resolution
- number of height bins
- preview image mode

Later I will expand this with clustering thresholds and logging options.

## Results

- Avg latency: TODO
- Detection summary: TODO

I will add real numbers only after the profiling and evaluation pieces are in place.

## Design direction

The design here is intentionally narrow.
My research repo is closer to a real experiment workspace.
This repo is meant to be a cleaner engineering demo around LiDAR perception in ROS2.

That means a few choices are deliberate:

- Python first, so iteration stays fast
- synthetic input first, so the pipeline is debuggable early
- simple clustering first, so the behavior stays easy to inspect in RViz2
- Docker later, once the core nodes are stable

## Limitations

- The current input is synthetic, not a public rosbag yet
- Clustering is not added yet
- Latency and detection-rate logging are not added yet

## Future work

- Replace synthetic input with a small public LiDAR dataset
- Add Euclidean clustering and marker visualization
- Add latency logging and simple summary export
- Add Docker and RViz2 config for a one-command demo
