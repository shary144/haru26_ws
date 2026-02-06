from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'robomas_package_2'

    # パッケージ直下の sender_params.yaml を取得
    pkg_share = get_package_share_directory(pkg_name)
    sender_param_file_can0 = os.path.join(pkg_share, 'sender_params_can0.yaml')
    sender_param_file_can1 = os.path.join(pkg_share, 'sender_params_can1.yaml')

    sender_node_can0 = Node(
        package=pkg_name,
        executable='sender_can0',
        parameters=[sender_param_file_can0],
        output='screen'
    )

    sender_node_can1 = Node(
        package=pkg_name,
        executable='sender_can1',
        parameters=[sender_param_file_can1],
        output='screen'
    )

    # receiver_node = Node(
    #     package=pkg_name,
    #     executable='receiver',
    #     output='screen'
    # )

    return LaunchDescription([
        sender_node_can0,
        sender_node_can1,
        # receiver_node,
    ])
