from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    keywords= [("lidar_imu","lidar_imu_bringup.launch.py"),
               ("odometry_plugins","odometry_bridge_launch.xml"),
               ("urg_node2","urg_node2.launch.py")]
    
    launch_path_series = []
    for (pkg, launch_file) in keywords:
        pkg_dir = get_package_share_directory(pkg)
        launch_path = os.path.join(pkg_dir, 'launch', launch_file)
        if not os.path.exists(launch_path):
            raise FileNotFoundError(f"Launch file '{launch_file}' not found in package '{pkg}' at path: {launch_path}")
        launch_path_series.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path)))
        
    pkg_dir = get_package_share_directory('self_driving')

    return LaunchDescription(launch_path_series)