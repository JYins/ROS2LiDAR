import csv
from pathlib import Path
from time import perf_counter

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import MarkerArray


class LatencyLogger(Node):
    def __init__(self):
        super().__init__("latency_logger")

        self.declare_parameter("pointcloud_topic", "/points_raw")
        self.declare_parameter("bev_image_topic", "/bev_image")
        self.declare_parameter("cluster_topic", "/detections")
        self.declare_parameter("results_dir", "results")
        self.declare_parameter("latency_file", "latency_stats.csv")
        self.declare_parameter("detection_file", "detection_summary.csv")

        pointcloud_topic = self.get_parameter("pointcloud_topic").value
        bev_image_topic = self.get_parameter("bev_image_topic").value
        cluster_topic = self.get_parameter("cluster_topic").value

        self.frame_times = {}
        self.results_dir = Path(str(self.get_parameter("results_dir").value))
        self.results_dir.mkdir(parents=True, exist_ok=True)

        self.latency_path = self.results_dir / str(self.get_parameter("latency_file").value)
        self.detection_path = self.results_dir / str(
            self.get_parameter("detection_file").value
        )

        self._init_csvs()

        self.points_sub = self.create_subscription(
            PointCloud2, pointcloud_topic, self.on_points, 10
        )
        self.bev_sub = self.create_subscription(Image, bev_image_topic, self.on_bev, 10)
        self.cluster_sub = self.create_subscription(
            MarkerArray, cluster_topic, self.on_clusters, 10
        )

        self.get_logger().info(
            f"latency_logger writing to {self.latency_path} and {self.detection_path}"
        )

    def _init_csvs(self):
        if not self.latency_path.exists():
            with self.latency_path.open("w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(["frame_stamp", "node", "latency_ms"])

        if not self.detection_path.exists():
            with self.detection_path.open("w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(["frame_stamp", "cluster_count"])

    def _stamp_key(self, stamp):
        return f"{stamp.sec}.{stamp.nanosec:09d}"

    def _trim_frames(self):
        while len(self.frame_times) > 500:
            first_key = next(iter(self.frame_times))
            self.frame_times.pop(first_key, None)

    def _append_latency(self, stamp_key, node_name, latency_ms):
        with self.latency_path.open("a", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow([stamp_key, node_name, f"{latency_ms:.3f}"])

    def _append_detection(self, stamp_key, cluster_count):
        with self.detection_path.open("a", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow([stamp_key, cluster_count])

    def on_points(self, cloud):
        stamp_key = self._stamp_key(cloud.header.stamp)
        self.frame_times[stamp_key] = perf_counter()
        self._trim_frames()

        now = self.get_clock().now()
        msg_time = Time.from_msg(cloud.header.stamp)
        latency_ms = max((now - msg_time).nanoseconds / 1e6, 0.0)
        self._append_latency(stamp_key, "pointcloud_player", latency_ms)

    def on_bev(self, img):
        stamp_key = self._stamp_key(img.header.stamp)
        start_time = self.frame_times.get(stamp_key)
        if start_time is None:
            return

        latency_ms = (perf_counter() - start_time) * 1000.0
        self._append_latency(stamp_key, "bev_projection", latency_ms)

    def on_clusters(self, markers):
        if not markers.markers:
            return

        stamp = markers.markers[0].header.stamp
        stamp_key = self._stamp_key(stamp)
        start_time = self.frame_times.get(stamp_key)
        if start_time is not None:
            latency_ms = (perf_counter() - start_time) * 1000.0
            self._append_latency(stamp_key, "cluster_detector", latency_ms)

        cluster_count = max(len(markers.markers) - 1, 0)
        self._append_detection(stamp_key, cluster_count)


def main(args=None):
    rclpy.init(args=args)
    node = LatencyLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
