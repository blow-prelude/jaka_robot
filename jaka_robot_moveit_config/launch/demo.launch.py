from moveit_configs_utils import MoveItConfigsBuilder

import os, sys
from ament_index_python.packages import get_package_share_directory
pkg_share = get_package_share_directory('jaka_robot_moveit_config')
sys.path.append(os.path.join(pkg_share, 'launch'))

from my_launch_utils import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jaka_zu5", package_name="jaka_robot_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
