# Design Decisions

## Python first

I kept the nodes in Python because this project is mainly a portfolio demo and learning project.
That let me move faster and keep the code easy to read.

I still list C++ on the resume because I use it elsewhere, but this repo itself is Python-first right now.

## Synthetic input first

I did not want the whole project blocked by dataset conversion on day one.
So I started with a synthetic point cloud publisher.

This is useful for three reasons:

1. the launch file always has something to run
2. RViz can show the full pipeline early
3. later dataset integration becomes a swap, not a rewrite

## 8 height channels for BEV

Instead of a single grayscale BEV image, I used 8 height channels.
This is more useful for later 3D reasoning than a flat occupancy image.

It also matches the way I was thinking about BEV in my research work: keep the vertical structure instead of throwing it away too early.

## Simple Euclidean clustering

The detector here is intentionally basic.
I did not want to overclaim object detection quality.

The current goal is:

- produce visible object-like outputs
- show a clean ROS2 perception step
- keep the logic easy to explain

If I want a stronger detector later, I can swap this part out.

## Plain CSV logging

For latency and detection stats, I used CSV files instead of a heavier metrics stack.
That keeps the project easy to inspect:

- run the launch
- open the CSVs
- run the summary script

For a portfolio repo, that simplicity is a feature.

## Docker for reproducibility

I added Docker so the environment is not tied to one laptop setup.
That matters a lot for ROS2 work because dependency drift is common.

The Docker setup here is meant to answer a practical question:
"Can someone else pull this repo and run the same pipeline without rebuilding my whole environment by hand?"

