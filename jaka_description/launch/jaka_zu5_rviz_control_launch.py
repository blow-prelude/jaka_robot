import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    # ------------------------------------
    # 请在这里修改你的 xacro 文件绝对路径
    xacro_file = '/home/wtr/programs/cpp/ros2/jaka/src/jaka_description/urdf/my_robot.urdf.xacro'
    # ------------------------------------

    # 1. 使用 xacro 命令将文件转换为 XML 格式的机器人描述
    robot_description_content = Command(['xacro ', xacro_file])

    # 2. 创建 robot_state_publisher 节点，它将发布模型数据
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 3. 创建 joint_state_publisher_gui 节点，用于发布关节角度（非必须，但推荐）
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 4. 启动 RViz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_rviz,
    ])


