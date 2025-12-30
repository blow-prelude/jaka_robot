from moveit_configs_utils import MoveItConfigsBuilder
import os, sys
from ament_index_python.packages import get_package_share_directory
pkg_share = get_package_share_directory('jaka_robot_moveit_config')
sys.path.append(os.path.join(pkg_share, 'launch'))

from my_launch_utils import generate_gazebo_ros_bridge


def generate_launch_description():
   return generate_gazebo_ros_bridge()