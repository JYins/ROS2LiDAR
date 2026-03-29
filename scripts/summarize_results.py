import csv
from collections import defaultdict
from pathlib import Path


def load_latency(path):
    rows = defaultdict(list)
    with path.open() as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows[row["node"]].append(float(row["latency_ms"]))
    return rows


def load_detections(path):
    counts = []
    with path.open() as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            counts.append(int(row["cluster_count"]))
    return counts


def avg(values):
    if not values:
        return 0.0
    return sum(values) / len(values)


def main():
    results_dir = Path("results")
    latency_path = results_dir / "latency_stats.csv"
    detection_path = results_dir / "detection_summary.csv"

    if not latency_path.exists():
        print(f"missing file: {latency_path}")
        return

    if not detection_path.exists():
        print(f"missing file: {detection_path}")
        return

    latency_rows = load_latency(latency_path)
    detection_counts = load_detections(detection_path)

    print("Latency summary")
    for node_name in sorted(latency_rows):
        values = latency_rows[node_name]
        print(
            f"- {node_name}: count={len(values)} avg={avg(values):.2f} ms "
            f"min={min(values):.2f} ms max={max(values):.2f} ms"
        )

    print("\nDetection summary")
    if detection_counts:
        print(
            f"- frames={len(detection_counts)} avg_clusters={avg(detection_counts):.2f} "
            f"min={min(detection_counts)} max={max(detection_counts)}"
        )
    else:
        print("- frames=0 avg_clusters=0.00 min=0 max=0")


if __name__ == "__main__":
    main()
