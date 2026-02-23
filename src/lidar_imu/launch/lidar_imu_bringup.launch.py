from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---- Launch args ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_csv      = LaunchConfiguration('map_csv')

    # パッケージ share からデフォルトの map を引く
    pkg_share = get_package_share_directory('lidar_imu')
    default_map_csv = os.path.join(pkg_share, 'lidar_1211.csv')

    # ---- Nodes ----
    imu_pubsub = Node(
        package='lidar_imu',
        executable='imu_pubsub',
        name='imu_pubsub',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    lidar_pubsub = Node(
        package='lidar_imu',
        executable='lidar_pubsub',
        name='lidar_pubsub',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    imu_gyro = Node(
        package='lidar_imu',
        executable='imu_gyro',
        name='imu_gyro',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    box_markers = Node(
        package='lidar_imu',
        executable='box_markers',
        name='box_markers',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_points_markers = Node(
        package='lidar_imu',
        executable='robot_points_markers',
        name='robot_points_markers',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # icp_localization_node が map_csv みたいなパラメータを読める想定で渡す
    # もしあなたのコード側のパラメータ名が違うなら、ここを合わせればOK。
    icp_localization_node = Node(
        package='lidar_imu',
        executable='icp_localization_node',
        name='icp_localization_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'map_csv': map_csv},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map_csv',
            default_value=default_map_csv,
            description='Path to map CSV installed in share/lidar_imu'
        ),

        imu_pubsub,
        lidar_pubsub,
        imu_gyro,
        box_markers,
        robot_points_markers,
        icp_localization_node,
    ])