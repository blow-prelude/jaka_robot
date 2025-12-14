import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    """
    Declare common launch parameters for the JAKA simulation / MoveIt setup.

    This file only declares arguments and does not start any nodes.
    Other launch files can include these arguments to keep configuration
    consistent.
    """

    # Build MoveIt config to derive the default entity name from the package path
    moveit_config = MoveItConfigsBuilder(
        "jaka_zu5", package_name="jaka_robot_moveit_config"
    ).to_moveit_configs()

    entity = DeclareLaunchArgument(
        "entity",
        default_value="jaka_zu5",
        description="Robot entity name (derived from package name)",
    )

    use_gazebo = DeclareBooleanLaunchArg(
        "use_gazebo",
        default_value=True,
        description="Use Gazebo simulation mode",
    )

    database = DeclareBooleanLaunchArg(
        "db",
        default_value=False,
        description="By default, we do not start a database (it can be large)",
    )

    debug = DeclareBooleanLaunchArg(
        "debug",
        default_value=False,
        description="By default, we are not in debug mode",
    )

    sim_time = DeclareBooleanLaunchArg(
        "use_sim_time",
        default_value=True,
        description="By default, we use sim time instead of real time",
    )

    pause = DeclareBooleanLaunchArg(
        "paused",
        default_value=True,
        description="By default, we  pause the time",
    )

    use_rviz = DeclareBooleanLaunchArg(
        "use_rviz",
        default_value=True,
        description="By default, we use rviz to show robot mode",
    )

    return LaunchDescription(
        [
            entity,
            use_gazebo,
            database,
            debug,
            sim_time,
            pause,
            use_rviz
        ]
    )