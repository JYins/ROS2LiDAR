# Architecture

## Big picture

This repo is a small ROS2 LiDAR perception demo.
The goal is not to build a full autonomy stack.
The goal is to show one clean data flow from point cloud input to simple perception outputs.

## Node graph

```text
pointcloud_player
    -> /points_raw (sensor_msgs/PointCloud2)

bev_projection
    <- /points_raw
    -> /bev_tensor (std_msgs/Float32MultiArray)
    -> /bev_image (sensor_msgs/Image)

cluster_detector
    <- /points_raw
    -> /detections (visualization_msgs/MarkerArray)

latency_logger
    <- /points_raw
    <- /bev_image
    <- /detections
    -> results/latency_stats.csv
    -> results/detection_summary.csv
```

## Data flow

### 1. Synthetic point cloud input

`pointcloud_player` publishes a synthetic LiDAR-style point cloud.
I started with synthetic input on purpose so the whole pipeline can run even before I wire in a public rosbag.

### 2. BEV projection

`bev_projection` takes the raw point cloud and maps it into a fixed bird's-eye-view grid.
The current representation follows the BEV layout that became more stable in my MEng project:

- channels `0-3`: density in 4 fixed height bins
- channels `4-7`: max height in those same 4 bins

That still gives an 8-channel tensor, but it is more informative than just stacking 8 raw occupancy slices.

Two outputs come out of this node:

- `bev_tensor` for the full 8-channel data
- `bev_image` for a simple RViz preview

### 3. Simple object detection

`cluster_detector` runs a basic Euclidean clustering pass.
This is intentionally simple.
The point is to create clear object-level outputs in RViz, not to claim a production detector.

### 4. Logging and results

`latency_logger` listens to the pipeline topics and writes:

- per-frame latency rows
- cluster-count rows

Then `scripts/summarize_results.py` prints a short summary from those CSV files.

## Why this structure

I kept the repo split by node because it makes the pipeline easy to explain in an interview:

- where the cloud comes from
- how it becomes a BEV
- how detections are produced
- where the metrics go

That keeps the project small enough to understand quickly, but still realistic enough to discuss ROS2 topics, launch files, and reproducibility.
