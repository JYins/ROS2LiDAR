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

## Research-style 8-channel BEV

Instead of a single grayscale BEV image, I used an 8-channel layout:

- 4 channels for density in fixed height bins
- 4 channels for max height in the same bins

This is closer to the version that became more stable in my MEng project.
It still keeps vertical structure, but it is a bit more informative than just counting occupancy in 8 separate slices.

The current default bins are:

- `[-3.0, -1.5)`
- `[-1.5, 0.0)`
- `[0.0, 1.0)`
- `[1.0, 2.0)`

I kept the range and resolution the same as my research-style setup:

- `x/y`: `[-40 m, 40 m]`
- `z`: `[-3 m, 2 m]`
- resolution: `0.16 m`

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
