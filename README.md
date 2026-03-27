# ROS2 LiDAR Perception Demo

Small ROS2 project for LiDAR point cloud playback, BEV projection, simple clustering, and RViz2 visualization.

I am building this repo as a clean robotics portfolio project. The scope is intentionally small: one LiDAR pipeline, one launch flow, one Docker setup later, and a few simple metrics. If something is not finished yet, I will say so directly.

## Why this repo exists

I wanted one robotics project in my portfolio that feels focused and practical.
Not a full autonomy stack.
Not a research code dump.
Just a clear ROS2 demo that shows I can work with sensor data, pipeline structure, visualization, and basic profiling.

## Planned pipeline

```text
pointcloud_player
    -> /points_raw (sensor_msgs/PointCloud2)
    -> bev_projection
        -> /bev_image (sensor_msgs/Image)
    -> cluster_detector
        -> /detections (visualization_msgs/MarkerArray)
    -> latency_logger
        -> results/latency_stats.csv
        -> results/detection_summary.csv
```

## Current status

- Repo created and connected to GitHub
- ROS2 workspace skeleton added
- Synthetic point cloud publisher code added
- Docker setup: not added yet
- Public rosbag / sample data: not added yet
- Metrics: not generated yet
- BEV projection: not added yet
- Clustering: not added yet

## Planned features

- Synthetic point cloud publisher first, so the pipeline is testable before I bring in a public rosbag
- BEV projection node with configurable range and resolution
- Simple Euclidean clustering for basic object-level visualization
- RViz2 config for one-command demo startup later
- Latency logging and simple per-frame detection stats
- Dockerized setup so the whole thing is easier to reproduce

## Results

- Avg latency: TODO
- Detection summary: TODO

I will only fill these numbers after the pipeline actually runs.

## Roadmap

1. Add a synthetic point cloud publisher
2. Add BEV projection
3. Add simple clustering
4. Add latency logging and result summaries
5. Dockerize the pipeline
6. Add tests and CI
7. Clean up docs and visualization assets

## Local run note

I have not run this in the current Windows workspace yet because `ros2` is not installed here right now.
Once ROS2 Humble is installed, the next target command will be:

```bash
ros2 launch lidar_perception demo_launch.py
```

## Limitations right now

- Only the synthetic input step is started
- No latency or detection metrics yet
- No screenshots yet
- No public rosbag wired in yet

## Future work

- Switch from synthetic input to a small public rosbag or PCD sample
- Add BEV projection and clustering outputs
- Add cleaner evaluation scripts once there is real output to evaluate

## Project note

I want this repo to stay straightforward and engineering-focused.
Simple pipeline, clear files, honest results.
