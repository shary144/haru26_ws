from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    keywords = [
        ("lidar_imu", "lidar_imu_bringup.launch.py"),
        ("odometry_plugins", "odometry_bridge_launch.xml")
    ]

    launch_path_series = []

    for pkg, launch_file in keywords:
        pkg_dir = get_package_share_directory(pkg)
        launch_path = os.path.join(pkg_dir, "launch", launch_file)

        if not os.path.exists(launch_path):
            raise FileNotFoundError(f"{launch_path} が見つかりません")

        launch_path_series.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(launch_path)
            )
        )

    return LaunchDescription(launch_path_series)