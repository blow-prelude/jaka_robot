import os
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils.moveit_configs_builder import MoveItConfigsBuilder
from launch.event_handlers import OnProcessExit


def generate_move_group_launch(moveit_config):
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )

    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    # 启动仿真时钟
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation time"
        )
    )

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        {"use_sim_time": True},
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    return ld



def generate_demo_launch(moveit_config, launch_package_path=None):
    """
    Launches a self contained demo

    launch_package_path is optional to use different launch and config packages

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners (for RViz simulation)
    """
    if launch_package_path == None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    ld.add_action(
       DeclareBooleanLaunchArg(
           "use_rviz_sim",
           default_value=True,
           description="Set to True to use Rviz simulation mode, False for real robot",
       )
    )
    ld.add_action(
       LogInfo(
           condition=IfCondition(LaunchConfiguration("use_rviz_sim")),
           msg="Rviz simulation mode is enabled"
       )
    )

    ld.add_action(
       LogInfo(
           condition=UnlessCondition(LaunchConfiguration("use_rviz_sim")),
           msg="Rviz simulation mode is disabled"
       )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="jaka_zu5",
            package_name=os.path.basename(str(moveit_config.package_path))
        )
        .robot_description(
            mappings={
                "use_rviz_sim": LaunchConfiguration("use_rviz_sim"), # Ensure RViz simulation is mode enabled
            }
        )
        .to_moveit_configs()
    )

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )

    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )


    move_group = generate_move_group_launch(moveit_config)
    ld.add_action(move_group)

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz_sim")),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/spawn_controllers.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz_sim")),  # Disable controller manager for real robots
        )
    )

    return ld


def generate_gazebo_launch(moveit_config, launch_package_path=None):
    """
    Mimics the ROS1 gazebo.launch:
      - Launches an empty world from gazebo_ros (using empty_world.launch.py)
      - Reads the robot URDF from file and sets it as the 'robot_description' parameter.
      - Spawns the robot in Gazebo using the spawn_entity node.
      - Includes the controllers launch file.
    """
    if launch_package_path is None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "entity", default_value="jaka_zu5", 
            description="Robot entity name (derived from package name)"
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "use_gazebo",
            default_value=True,
            description="Use Gazebo simulation mode",
        )
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="jaka_zu5",
            package_name=os.path.basename(str(moveit_config.package_path))
        )
        .robot_description(
            mappings={
                "use_gazebo": LaunchConfiguration("use_gazebo"),  # Ensure Gazebo simulation is enabled
            }
        )
        .to_moveit_configs()
    )

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )

    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # 1) Given the published joint states, publish tf for the robot links
    #  使用路径中的launches.py中的函数
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # # 2) Launch Gazebo Classic (from gazebo_ros)
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
    #     ),
    #     launch_arguments={
    #         "gazebo_world": "worlds/empty.world",
    #         "gazebo_ros_init": "true",
    #         "gazebo_ros_factory": "true",
    #     }.items()
    # )
    # ld.add_action(gazebo_launch)

    # 2) Launch Ignition Gazebo (from ros_gz_sim)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "empty.sdf -r",
        }.items(),
    )
    ld.add_action(gazebo_launch)


    # # 3) Spawn the robot into Gazebo Classic
    # spawn_robot = TimerAction(
    #     period=5.0,  # Wait for Gazebo to fully load
    #     actions=[
    #         Node(
    #             package="gazebo_ros",
    #             executable="spawn_entity.py",
    #             name="urdf_spawner",
    #             arguments=[
    #                 "-entity", LaunchConfiguration("entity"),
    #                 "-topic", "/robot_description",
    #                 "-x", "0", "-y", "0", "-z", "0.06" 
    #             ],
    #             output="screen"
    #         ),
    #     ],
    # )
    # ld.add_action(spawn_robot)

    # 3) Spawn the robot in Ignition Gazebo using ros_gz_sim's create.py
    spawn_robot = TimerAction(
        period=5.0,  # Wait for robot_state_publisher to load the URDF
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_robot",
                output="screen",
                arguments=[
                    "-topic", "/robot_description",
                    "-z", "0.06",  # Slightly lift robot above ground to avoid collision issues
                ],
            ),
        ],
    )
    ld.add_action(spawn_robot)

    # Run MoveIt! move_group

    move_group = generate_move_group_launch(moveit_config)
    ld.add_action(move_group)

    # Start the controller manager
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
            condition=IfCondition(LaunchConfiguration("use_gazebo")),
        )
    )

    # Start controllers
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/spawn_controllers.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_gazebo")),  # Disable controller manager for real robots
        )
    )

    return ld


def generate_demo_gazebo_launch(moveit_config, launch_package_path=None):
    """
    Launches a self-contained demo in Ignition Gazebo:
      1) Starts Gazebo and spawns the robot
      2) Starts robot_state_publisher
      3) Launches MoveIt! move_group
      4) Launches RViz
      5) Starts controllers
    """

    if launch_package_path is None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(
        DeclareLaunchArgument(
            "entity", default_value="jaka_zu5", 
            description="Robot entity name (derived from package name)"
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "use_gazebo",
            default_value=True, 
            description="Use Gazebo simulation mode",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="jaka_zu5",
            package_name=os.path.basename(str(moveit_config.package_path))
        )
        .robot_description(
            mappings={
                "use_gazebo": LaunchConfiguration("use_gazebo"),  # Ensure Gazebo simulation is enabled
            }
        )
        .to_moveit_configs()
    )

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )

    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # 1) Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )

    # # 2) Launch Gazebo Classic (from gazebo_ros)
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
    #     ),
    #     launch_arguments={
    #         "gazebo_world": "worlds/empty.world",
    #         "gazebo_ros_init": "true",
    #         "gazebo_ros_factory": "true",
    #     }.items()
    # )
    # ld.add_action(gazebo_launch)

    # 2) Launch Ignition Gazebo (from ros_gz_sim)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "empty.sdf -r",
        }.items() 
    )
    ld.add_action(gazebo_launch)

    # # 3) Spawn the robot into Gazebo Classic
    # spawn_robot = TimerAction(
    #     period=5.0,  # Wait for Gazebo to start
    #     actions=[
    #         Node(
    #             package="gazebo_ros",
    #             executable="spawn_entity.py",
    #             arguments=[
    #                 "-entity", LaunchConfiguration("entity"),
    #                 "-topic", "/robot_description",
    #                 "-x", "0", "-y", "0", "-z", "0.06"
    #             ],
    #             output="screen"
    #         ),
    #     ],
    # )
    # ld.add_action(spawn_robot)

    # 3) Spawn the robot in Ignition Gazebo using ros_gz_sim's create.py
    spawn_robot = TimerAction(
        period=5.0,  # Wait for robot_state_publisher to load the URDF
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_robot",
                output="screen",
                arguments=[
                    "-topic", "/robot_description",
                    "-z", "0.06",  # Slightly lift robot above ground to avoid collision issues
                ],
            ),
        ],
    )
    ld.add_action(spawn_robot)

    # Run MoveIt! move_group
    move_group = generate_move_group_launch(moveit_config)
    ld.add_action(move_group)

    # Run Rviz if enabled
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Start the controller manager
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
            condition=IfCondition(LaunchConfiguration("use_gazebo")),
        )
    )

    # Start controllers
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/spawn_controllers.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_gazebo")),  # Disable controller manager for real robots
        )
    )

    return ld