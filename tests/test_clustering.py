import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "lidar_perception"))

from lidar_perception.utils import run_euclidean_clustering
from lidar_perception.utils import summarize_cluster


def test_two_clusters_are_found():
    points = [
        (1.0, 1.0, 0.2, 1.0),
        (1.1, 1.0, 0.2, 1.0),
        (1.0, 1.1, 0.3, 1.0),
        (8.0, -2.0, 0.4, 1.0),
        (8.1, -2.1, 0.5, 1.0),
        (7.9, -2.0, 0.4, 1.0),
        (20.0, 20.0, 0.0, 1.0),
    ]

    clusters = run_euclidean_clustering(
        points,
        distance_threshold=0.5,
        min_cluster_size=3,
        max_cluster_size=10,
        cluster_z_min=-1.0,
        cluster_z_max=2.0,
    )

    assert len(clusters) == 2
    assert sorted(len(cluster) for cluster in clusters) == [3, 3]


def test_cluster_summary_has_reasonable_box():
    cluster = [
        (4.0, 2.0, 0.0, 1.0),
        (4.4, 2.2, 0.5, 1.0),
        (4.2, 1.8, 0.2, 1.0),
    ]

    summary = summarize_cluster(cluster)

    assert summary["count"] == 3
    assert 4.0 <= summary["center_x"] <= 4.4
    assert 1.8 <= summary["center_y"] <= 2.2
    assert summary["size_x"] >= 0.2
    assert summary["size_y"] >= 0.2
    assert summary["size_z"] >= 0.2
