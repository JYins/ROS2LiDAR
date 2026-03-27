from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lidar_perception",
                executable="pointcloud_player",
                name="pointcloud_player",
                output="screen",
                parameters=["config/demo.yaml"],
            )
        ]
    )

