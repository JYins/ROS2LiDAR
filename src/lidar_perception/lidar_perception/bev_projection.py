import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32MultiArray

from lidar_perception.utils import build_bev_tensor_msg
from lidar_perception.utils import build_mono_image_msg
from lidar_perception.utils import build_preview_image
from lidar_perception.utils import make_bev_config
from lidar_perception.utils import project_points_to_bev
from lidar_perception.utils import unpack_pointcloud2


class BevProjection(Node):
    def __init__(self):
        super().__init__("bev_projection")

        self.declare_parameter("input_topic", "/points_raw")
        self.declare_parameter("bev_tensor_topic", "/bev_tensor")
        self.declare_parameter("bev_image_topic", "/bev_image")
        self.declare_parameter("frame_id", "lidar")
        self.declare_parameter("x_min", -40.0)
        self.declare_parameter("x_max", 40.0)
        self.declare_parameter("y_min", -40.0)
        self.declare_parameter("y_max", 40.0)
        self.declare_parameter("z_min", -3.0)
        self.declare_parameter("z_max", 2.0)
        self.declare_parameter("resolution", 0.16)
        self.declare_parameter("num_height_bins", 8)
        self.declare_parameter("preview_normalize_mode", "max_bin")

        input_topic = self.get_parameter("input_topic").value
        bev_tensor_topic = self.get_parameter("bev_tensor_topic").value
        bev_image_topic = self.get_parameter("bev_image_topic").value

        self.config = self._load_config()
        self.preview_mode = str(self.get_parameter("preview_normalize_mode").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.cloud_sub = self.create_subscription(
            PointCloud2, input_topic, self.process_cloud, 10
        )
        self.tensor_pub = self.create_publisher(Float32MultiArray, bev_tensor_topic, 10)
        self.image_pub = self.create_publisher(Image, bev_image_topic, 10)

        self.get_logger().info(
            "bev_projection ready "
            f"grid={self.config['height']}x{self.config['width']}x{self.config['num_height_bins']}"
        )

    def _load_config(self):
        return make_bev_config(
            x_min=float(self.get_parameter("x_min").value),
            x_max=float(self.get_parameter("x_max").value),
            y_min=float(self.get_parameter("y_min").value),
            y_max=float(self.get_parameter("y_max").value),
            z_min=float(self.get_parameter("z_min").value),
            z_max=float(self.get_parameter("z_max").value),
            resolution=float(self.get_parameter("resolution").value),
            num_height_bins=int(self.get_parameter("num_height_bins").value),
        )

    def process_cloud(self, cloud):
        points = unpack_pointcloud2(cloud)
        bev = project_points_to_bev(points, self.config)
        img = build_preview_image(bev, mode=self.preview_mode)

        stamp = cloud.header.stamp
        frame_id = self.frame_id or cloud.header.frame_id

        tensor_msg = build_bev_tensor_msg(bev)
        image_msg = build_mono_image_msg(stamp, frame_id, img)

        self.tensor_pub.publish(tensor_msg)
        self.image_pub.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BevProjection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
