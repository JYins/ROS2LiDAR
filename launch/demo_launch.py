from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("lidar_perception"), "config", "demo.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="lidar_perception",
                executable="pointcloud_player",
                name="pointcloud_player",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                package="lidar_perception",
                executable="bev_projection",
                name="bev_projection",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                package="lidar_perception",
                executable="cluster_detector",
                name="cluster_detector",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
