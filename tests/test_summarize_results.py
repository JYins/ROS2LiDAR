import csv
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.summarize_results import avg
from scripts.summarize_results import load_detections
from scripts.summarize_results import load_latency


def test_avg_handles_empty_and_values():
    assert avg([]) == 0.0
    assert avg([1.0, 2.0, 3.0]) == 2.0


def test_load_latency_groups_by_node(tmp_path):
    path = tmp_path / "latency.csv"
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame_stamp", "node", "latency_ms"])
        writer.writerow(["1.0", "bev_projection", "4.5"])
        writer.writerow(["1.1", "bev_projection", "5.5"])
        writer.writerow(["1.1", "cluster_detector", "2.0"])

    rows = load_latency(path)

    assert rows["bev_projection"] == [4.5, 5.5]
    assert rows["cluster_detector"] == [2.0]


def test_load_detections_reads_cluster_counts(tmp_path):
    path = tmp_path / "detections.csv"
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame_stamp", "cluster_count"])
        writer.writerow(["1.0", "3"])
        writer.writerow(["1.1", "4"])

    counts = load_detections(path)

    assert counts == [3, 4]
