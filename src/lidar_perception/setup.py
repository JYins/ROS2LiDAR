from setuptools import find_packages, setup


package_name = "lidar_perception"
launch_file = "../../launch/demo_launch.py"
config_file = "../../config/demo.yaml"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [launch_file]),
        (f"share/{package_name}/config", [config_file]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="JYins",
    maintainer_email="todo@example.com",
    description="ROS2 LiDAR perception demo package.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bev_projection = lidar_perception.bev_projection:main",
            "cluster_detector = lidar_perception.cluster_detector:main",
            "pointcloud_player = lidar_perception.pointcloud_player:main",
        ],
    },
)
