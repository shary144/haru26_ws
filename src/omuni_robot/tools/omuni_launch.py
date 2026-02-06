from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # robomas_package_2 の launch ファイルへのパス
    robomas_launch = os.path.join(
        get_package_share_directory('robomas_package_2'),
        'launch',
        'robomas_launch.py'
    )

    return LaunchDescription([

        # robomas_package_2 の launch を起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robomas_launch)
        ),

        # joy
        Node(
            package='joy',
            executable='joy_node',
            output='screen'
        ),

        # omuni
        Node(
            package='omuni_robot',
            executable='omuni_3rin_node',
            output='screen'
        ),

    ])
