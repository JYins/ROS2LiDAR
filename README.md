# ROS2 LiDAR Perception Demo

> Work in progress. Building this repo step by step like a normal engineer, not magic.

This repo is for a small ROS2 LiDAR perception demo I can actually explain in an interview without hand-waving. The goal is pretty simple: play LiDAR point clouds, turn them into a BEV view, run a basic clustering-based detector, and visualize everything in RViz2. Anyways, I want this project to stay practical, reproducible, and honest. If something is not finished yet, I will mark it clearly instead of pretending it already works.

## Why this repo exists

I wanted one robotics project in my portfolio that feels focused and grounded.
Not a full autonomy stack.
Not a research code dump.
Just one clean ROS2 demo that shows I can work with sensor data, structure a pipeline, and think a little about latency, visualization, and reproducibility.

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
- README started
- Real ROS2 workspace files: not added yet
- Docker setup: not added yet
- Public rosbag / sample data: not added yet
- Metrics: not generated yet

## Planned features

- Synthetic point cloud publisher first, so the pipeline is testable even without a public rosbag on day one
- BEV projection node with configurable range and resolution
- Simple Euclidean clustering for basic object-level visualization
- RViz2 config for one-command demo startup later
- Latency logging and simple per-frame detection stats
- Dockerized setup so the whole thing is easier to reproduce

## Results

- Avg latency: TODO
- Detection summary: TODO

I will only fill these numbers after the pipeline actually runs.

## Quick note

If you are reading this early, yes, this repo is still in the "starting from first principles" phase.
That is intentional.
I want the commit history to look like how I really build things: one small piece working, then the next one, then cleanup after.

## Roadmap

1. Initialize the ROS2 workspace skeleton
2. Add a synthetic point cloud publisher
3. Add BEV projection
4. Add simple clustering
5. Add latency logging and result summaries
6. Dockerize the pipeline
7. Add tests and CI
8. Clean up docs and visualization assets

## Limitations right now

- Nothing runnable yet
- No latency or detection metrics yet
- No screenshots yet

## Future work

- Switch from synthetic input to a small public rosbag or PCD sample
- Try stronger clustering or lightweight tracking if the basic version is stable
- Add cleaner evaluation scripts once there is real output to evaluate
