from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    debug_param = {"debug": True}

    ball_detector_node = Node(
        package="ball_detector",
        executable="ball_detector_node",
        name="ball_detector_node",
        parameters=[debug_param],
        output="screen",
    )

    ball_color_marker_node = Node(
        package="ball_detector",
        executable="ball_color_marker_node",
        name="ball_color_marker_node",
        output="screen",
    )

    Rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", "/home/mihiro/.rviz2/default.rviz"],# Specify your RViz config file path here
        output="screen",
    )

    return LaunchDescription([
        ball_detector_node,
        ball_color_marker_node,
        Rviz_node,
    ])