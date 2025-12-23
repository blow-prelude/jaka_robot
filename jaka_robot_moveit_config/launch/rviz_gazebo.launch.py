from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from moveit_configs_utils.moveit_configs_builder import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch

from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch a full RViz + Gazebo demo:
      1) 声明常用参数（config_para.launch.py）
      2) 启动 robot_state_publisher（发布 /robot_description 与 TF）
      3) 启动 Gazebo 空世界并从 /robot_description 生成机器人实体
      4) 启动 MoveIt move_group 进行规划
      5) 启动 RViz 可视化规划结果
      6) 启动 ros2_control 控制器，让 Gazebo 中的机械臂执行规划轨迹
    """

    # 构建 MoveIt 配置（此处直接启用 Gazebo 仿真相关的 xacro 分支）
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_zu5", package_name="jaka_robot_moveit_config"
        )
        .robot_description(
            mappings={
                # 该启动文件专门用于 Gazebo 仿真，因此这里固定为 true
                "use_gazebo": "true",
                # Gazebo 下由 gz_ros2_control 提供硬件接口，不使用 RViz 假仿真硬件
                "use_rviz_sim": "false"
            }
        )
        .to_moveit_configs()
    )

    launch_package_path = moveit_config.package_path / "launch"

    launch_description = LaunchDescription()

    # 1) 声明通用参数：entity / use_gazebo / db / debug / use_sim_time / paused / use_rviz
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "config_arguments.launch.py")
            )
        )
    )

    # 如果存在虚拟关节，发布对应的静态 TF
    virtual_joints_launch = launch_package_path / "static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        launch_description.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch))
            )
        )

    # 2) robot_state_publisher：发布 /robot_description 和 TF
    launch_description.add_action(generate_rsp_launch(moveit_config))

    # 3) Gazebo 仿真世界 + 从 /robot_description 生成机器人实体
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "gazebo_world.launch.py")
            )
        )
    )

    # 4) 启动 MoveIt move_group，允许执行轨迹
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "move_group.launch.py")
            ),
            launch_arguments={"allow_trajectory_execution": "true"}.items(),
        )
    )

    # 5) 启动 RViz，进行规划与可视化
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # 如需使用数据库，启动 warehouse_db
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )


    # Start the controller manager
    # launch_description.add_action(
    #     Node(
    #         package="controller_manager",
    #         executable="ros2_control_node",
    #         parameters=[
    #             moveit_config.robot_description,
    #             str(moveit_config.package_path / "config/ros2_controllers.yaml"),
    #         ],
    #         condition=IfCondition(LaunchConfiguration("use_gazebo")),
    #     )
    # )

    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "spawn_controllers.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_gazebo")),
        )
    )

    return launch_description
