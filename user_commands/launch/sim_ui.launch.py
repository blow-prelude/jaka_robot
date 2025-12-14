from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # 1) 先启动仿真：jaka_robot_moveit_config/launch/rviz_gazebo.launch.py，加载出rviz_gazebo
    moveit_config_share = get_package_share_directory("jaka_robot_moveit_config")
    demo_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "rviz_gazebo.launch.py")
        )
    )

    # 2) 然后启动 user_commands/src/moveit_client.cpp 编译出的节点，加载moveit客户端
    moveit_client_node = Node(
        package="user_commands",
        executable="moveit_client",
        output="screen",
    )


    # 4) 最后启动 user_commands/script/sim_main_window.py 安装后的可执行脚本
    sim_main_window_node = Node(
        package="user_commands",
        executable="sim_main_window",
        output="screen",
    )

    # 使用 TimerAction 做简单的先后顺序控制：
    # - demo_gazebo.launch.py 立即启动
    # - moveit_client 在 4 秒后启动
    # - gripper_mimic 在 7 秒后启动
    # - sim_main_window 在 8 秒后启动
    start_moveit_client = TimerAction(period=4.0, actions=[moveit_client_node])
    start_sim_main_window = TimerAction(period=8.0, actions=[sim_main_window_node])

    return LaunchDescription(
        [
            demo_gazebo_launch,
            start_moveit_client,
            start_sim_main_window,
        ]
    )
