import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from lidar_perception.utils import build_pointcloud2


class PointCloudPlayer(Node):
    def __init__(self):
        super().__init__("pointcloud_player")

        self.declare_parameter("topic_name", "/points_raw")
        self.declare_parameter("frame_id", "lidar")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("cluster_count", 4)
        self.declare_parameter("points_per_cluster", 45)
        self.declare_parameter("cluster_spread", 0.8)
        self.declare_parameter("ground_points", 120)
        self.declare_parameter("range_x", 20.0)
        self.declare_parameter("range_y", 12.0)
        self.declare_parameter("ground_z", -1.4)

        topic_name = self.get_parameter("topic_name").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.cloud_pub = self.create_publisher(PointCloud2, topic_name, 10)
        self.frame_count = 0
        self.rng = random.Random(7)

        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_cloud)
        self.get_logger().info(f"publishing synthetic point cloud on {topic_name}")

    def publish_cloud(self):
        points = []
        points.extend(self._build_ground_points())
        points.extend(self._build_cluster_points())

        msg = self._to_pointcloud2(points)
        self.cloud_pub.publish(msg)
        self.frame_count += 1

    def _build_ground_points(self):
        ground_points = int(self.get_parameter("ground_points").value)
        range_x = float(self.get_parameter("range_x").value)
        range_y = float(self.get_parameter("range_y").value)
        ground_z = float(self.get_parameter("ground_z").value)

        points = []
        for _ in range(ground_points):
            x = self.rng.uniform(0.0, range_x)
            y = self.rng.uniform(-range_y, range_y)
            z = ground_z + self.rng.uniform(-0.05, 0.05)
            intensity = self.rng.uniform(0.05, 0.2)
            points.append((x, y, z, intensity))
        return points

    def _build_cluster_points(self):
        cluster_count = int(self.get_parameter("cluster_count").value)
        points_per_cluster = int(self.get_parameter("points_per_cluster").value)
        cluster_spread = float(self.get_parameter("cluster_spread").value)
        range_x = float(self.get_parameter("range_x").value)
        range_y = float(self.get_parameter("range_y").value)

        points = []
        for idx in range(cluster_count):
            angle = (self.frame_count * 0.08) + idx * 1.1
            center_x = 4.0 + idx * 3.0 + math.sin(angle) * 0.6
            center_y = math.cos(angle) * (range_y * 0.45)
            center_z = -0.2 + self.rng.uniform(-0.05, 0.05)

            center_x = min(max(center_x, 1.0), range_x - 1.0)
            center_y = min(max(center_y, -range_y + 1.0), range_y - 1.0)

            for _ in range(points_per_cluster):
                x = center_x + self.rng.uniform(-cluster_spread, cluster_spread)
                y = center_y + self.rng.uniform(-cluster_spread, cluster_spread)
                z = center_z + self.rng.uniform(-0.6, 0.8)
                intensity = self.rng.uniform(0.6, 1.0)
                points.append((x, y, z, intensity))
        return points

    def _to_pointcloud2(self, points):
        return build_pointcloud2(
            header_stamp=self.get_clock().now().to_msg(),
            frame_id=str(self.get_parameter("frame_id").value),
            points=points,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPlayer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
