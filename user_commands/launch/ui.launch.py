from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    moveit_config_share = get_package_share_directory("jaka_robot_moveit_config")
    rviz_launch_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "demo.launch.py")
        )
    )
    main_window_node = Node(
        package="user_commands",
        executable="real_main_window",
        output="screen",
    )


    start_main_window = TimerAction(period=4.0, actions=[main_window_node])

    return LaunchDescription([
        rviz_launch_node,
        start_main_window
    ])