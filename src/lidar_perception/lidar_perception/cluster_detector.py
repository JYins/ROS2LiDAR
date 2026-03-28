import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

from lidar_perception.utils import build_cluster_markers
from lidar_perception.utils import run_euclidean_clustering
from lidar_perception.utils import summarize_cluster
from lidar_perception.utils import unpack_pointcloud2


class ClusterDetector(Node):
    def __init__(self):
        super().__init__("cluster_detector")

        self.declare_parameter("input_topic", "/points_raw")
        self.declare_parameter("cluster_topic", "/detections")
        self.declare_parameter("frame_id", "lidar")
        self.declare_parameter("distance_threshold", 1.2)
        self.declare_parameter("min_cluster_size", 12)
        self.declare_parameter("max_cluster_size", 400)
        self.declare_parameter("cluster_z_min", -1.0)
        self.declare_parameter("cluster_z_max", 2.0)

        input_topic = self.get_parameter("input_topic").value
        cluster_topic = self.get_parameter("cluster_topic").value

        self.frame_id = str(self.get_parameter("frame_id").value)

        self.cloud_sub = self.create_subscription(
            PointCloud2, input_topic, self.process_cloud, 10
        )
        self.marker_pub = self.create_publisher(MarkerArray, cluster_topic, 10)

        self.get_logger().info(
            "cluster_detector ready "
            f"topic={cluster_topic} dist={self.get_parameter('distance_threshold').value}"
        )

    def process_cloud(self, cloud):
        points = unpack_pointcloud2(cloud)
        clusters = run_euclidean_clustering(
            points,
            distance_threshold=float(self.get_parameter("distance_threshold").value),
            min_cluster_size=int(self.get_parameter("min_cluster_size").value),
            max_cluster_size=int(self.get_parameter("max_cluster_size").value),
            cluster_z_min=float(self.get_parameter("cluster_z_min").value),
            cluster_z_max=float(self.get_parameter("cluster_z_max").value),
        )

        summaries = [summarize_cluster(cluster) for cluster in clusters]
        frame_id = self.frame_id or cloud.header.frame_id
        markers = build_cluster_markers(cloud.header.stamp, frame_id, summaries)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ClusterDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
