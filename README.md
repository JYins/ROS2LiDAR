# ROS2 LiDAR Perception Demo

[![CI](https://github.com/JYins/ROS2LiDAR/actions/workflows/ci.yml/badge.svg)](https://github.com/JYins/ROS2LiDAR/actions/workflows/ci.yml)

ROS2 demo for LiDAR point cloud playback, BEV projection, simple object detection, latency logging, and RViz2 visualization.

This project came out of my MEng research discussions at Western University. In my research work I spent a lot of time thinking about LiDAR-based BEV representations and what information is worth keeping for later 3D reasoning. This repo is a smaller and more engineering-focused version of that thinking: one clean ROS2 pipeline, beginner-friendly code, simple outputs, and enough structure to be useful in an interview or as a learning project.

## Architecture

```text
pointcloud_player
    -> /points_raw

bev_projection
    <- /points_raw
    -> /bev_tensor
    -> /bev_image

cluster_detector
    <- /points_raw
    -> /detections

latency_logger
    <- /points_raw
    <- /bev_image
    <- /detections
    -> results/latency_stats.csv
    -> results/detection_summary.csv
```

## Quick start

### Local ROS2

```bash
colcon build --packages-select lidar_perception
source install/setup.bash
ros2 launch lidar_perception demo_launch.py
```

Open RViz2 with:

```bash
rviz2 -d config/rviz_config.rviz
```

After the pipeline runs for a bit:

```bash
python scripts/summarize_results.py
```

### Docker

```bash
docker compose up --build
```

This starts the ROS2 nodes inside a container and writes result CSV files into the local `results/` folder.

## Nodes

### pointcloud_player

Publishes a synthetic `sensor_msgs/PointCloud2` stream.
Right now this is the default input so the project is runnable without downloading a public dataset first.

### bev_projection

Subscribes to `/points_raw` and projects the cloud into a fixed 8-height-channel BEV.

Outputs:

- `/bev_tensor` as the full `(500, 500, 8)` representation
- `/bev_image` as a simple `mono8` preview for RViz2

### cluster_detector

Runs a simple Euclidean clustering pass on the point cloud and publishes `/detections` as `MarkerArray`.
This is a basic object-level demo, not a production detector.

### latency_logger

Watches the main pipeline topics and writes:

- `results/latency_stats.csv`
- `results/detection_summary.csv`

## Configuration

Main parameters live in `config/demo.yaml`.

Current config covers:

- synthetic input settings
- BEV range and resolution
- number of height bins
- clustering thresholds
- result file locations

Important defaults:

- BEV `x/y`: `[-40 m, 40 m]`
- BEV `z`: `[-3 m, 2 m]`
- BEV resolution: `0.16 m`
- height bins: `8`

## Results

Current generation flow:

```bash
ros2 launch lidar_perception demo_launch.py
python scripts/summarize_results.py
```

Current checked-in result files:

- `results/latency_stats.csv`
- `results/detection_summary.csv`

If you want fresh numbers, rerun the launch and regenerate from the current run instead of trusting old CSVs.

## Screenshots

Screenshot placeholder for now:

- RViz2 point cloud view
- RViz2 cluster marker view
- RViz2 BEV image view

## Design decisions

Short version:

- Python first, so iteration stays fast
- synthetic input first, so the pipeline always runs
- 8 height channels, so the BEV keeps more structure
- simple clustering, so the outputs stay easy to inspect
- CSV logging, so the results are easy to read

More detail:

- `docs/architecture.md`
- `docs/design_decisions.md`

## Limitations

- The default input is synthetic, not a public rosbag yet
- The detector is intentionally simple
- The nodes are regular `rclpy` nodes right now, not ROS2 lifecycle nodes
- This repo does not claim production-grade tracking or classification
- Docker runs the ROS2 pipeline, but RViz2 is still easiest to use locally

## Future work

- Add a public rosbag or PCD-based playback path
- Add a cleaner RViz2 screenshot set
- Tighten latency measurement with more explicit per-node timing hooks
- Add a stronger detector after the current baseline is stable
