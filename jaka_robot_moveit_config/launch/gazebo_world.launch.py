import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils.moveit_configs_builder import MoveItConfigsBuilder


def generate_launch_description():
    """
    Launch an empty Ignition Gazebo world and spawn the robot model
    using the same URDF as MoveIt.

    这里直接通过 ros_gz_sim 的 `-param` 选项从本节点参数中加载模型，
    不依赖 /robot_description 话题，也不依赖外部的 use_gazebo LaunchArgument。
    """

    # Arguments
    gz_args = LaunchConfiguration("gz_args")

    declare_gz_args = DeclareLaunchArgument(
        "gz_args",
        default_value="empty.sdf -r",
        description=(
            "Arguments passed to ros_gz_sim gz_sim.launch.py, "
            "e.g. 'empty.sdf -r' for an empty world running immediately."
        ),
    )


    # 1) 启动 Gazebo 仿真世界（这里使用 ros_gz_sim 的空世界）
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    # 2) 通过 ros_gz_sim create，从本节点参数 robot_description 中读取模型并生成实体
    spawn_robot = TimerAction(
        period=5.0,  # 等待仿真世界就绪
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_robot",
                output="screen",
                arguments=[
                    "-topic",
                    "/robot_description",
                    "-z",
                    "0.06",  # 略微抬高，避免与地面初始穿插
                ],
            )
        ],
    )

    # 3) 后续如需加载其他模型，可在此处追加新的 Node / IncludeLaunchDescription
    #    例如：通过 ros_gz_sim create 从 SDF 文件生成环境模型等。

    return LaunchDescription(
        [
            declare_gz_args,
            gazebo_launch,
            spawn_robot,
        ]
    )
