from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch

# 启动控制器 jaka_zu5_controller 和joint_state_broadcaster
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jaka_zu5", package_name="jaka_zu5_moveit_config").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
