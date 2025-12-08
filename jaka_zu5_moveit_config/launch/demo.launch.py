from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    """
    RViz-only / real-robot MoveIt demo for JAKA ZU5.

    - 使用 `use_rviz_sim:=true` 时：启用 mock_components/GenericSystem 作为虚拟硬件，
      在 RViz 中仿真执行轨迹（无真实机器人、无 Gazebo）。
    - 使用默认或 `use_rviz_sim:=false` 时：走真实机器人路径（需要你自己后续接入）。
    """

    moveit_config = MoveItConfigsBuilder(
        "jaka_zu5", package_name="jaka_zu5_moveit_config"
    ).to_moveit_configs()

    return generate_demo_launch(moveit_config)
