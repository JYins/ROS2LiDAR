import sys
from pathlib import Path

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "lidar_perception"))

from lidar_perception.utils import build_preview_image
from lidar_perception.utils import make_bev_config
from lidar_perception.utils import point_to_bev_index
from lidar_perception.utils import project_points_to_bev


def test_point_lands_in_expected_cell_and_bin():
    config = make_bev_config(
        x_min=-40.0,
        x_max=40.0,
        y_min=-40.0,
        y_max=40.0,
        z_min=-3.0,
        z_max=2.0,
        resolution=0.16,
        num_height_bins=8,
    )

    idx = point_to_bev_index(0.0, 0.0, -0.5, config)

    assert idx == (250, 250, 4)


def test_points_outside_bounds_are_ignored():
    config = make_bev_config()
    points = [
        (0.0, 0.0, 0.0, 1.0),
        (45.0, 0.0, 0.0, 1.0),
        (0.0, 45.0, 0.0, 1.0),
        (0.0, 0.0, 3.0, 1.0),
    ]

    bev = project_points_to_bev(points, config)

    assert bev.sum() == 1.0


def test_bev_shape_is_fixed():
    config = make_bev_config()
    bev = project_points_to_bev([(0.0, 0.0, 0.0, 1.0)], config)

    assert bev.shape == (500, 500, 8)


def test_preview_image_is_not_empty_for_valid_points():
    config = make_bev_config()
    bev = project_points_to_bev([(1.0, 1.0, 1.5, 1.0)], config)
    img = build_preview_image(bev)

    assert img.shape == (500, 500)
    assert img.dtype == np.uint8
    assert img.max() > 0
