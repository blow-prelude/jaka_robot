from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# 必须包含这个函数！函数名不能错
def generate_launch_description():
    # 1. 配置包名和文件路径（直接用包名，避免引用安装目录）
    moveit_config_pkg = "jaka_zu5_moveit_config"  # MoveIt! 配置包名（存放 demo_gazebo.launch.py 的包）
    cmd_control_pkg = "jaka_zu5_moveit_config"    # 命令行脚本所在包名（和脚本存放位置一致）

    # 2. 拼接原 Gazebo+MoveIt! 启动文件的绝对路径（从源码包读取，而非安装目录）
    demo_gazebo_launch_path = os.path.join(
        get_package_share_directory(moveit_config_pkg),  # 获取包的源码路径
        "launch",
        "demo_gazebo.launch.py"  # 你原来的 Gazebo 启动文件名（必须和实际一致）
    )

    # 3. 引入原 Gazebo+MoveIt! 启动文件（先启动仿真环境）
    include_gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_gazebo_launch_path)
    )

    # 4. 定义命令行控制节点（延迟启动，确保依赖就绪）
    cmd_vel_node = Node(
        package=cmd_control_pkg,
        executable="cmd_vel_control.py",  # 脚本文件名（无需加 .py，Colcon 会自动识别可执行文件）
        name="cmd_line_arm_control",
        output="screen",  # 日志输出到终端
        emulate_tty=True,  # 关键：启用终端交互，允许输入指令
        shell=True  # 可选：解决部分终端无法输入的问题
    )

    # 5. 延迟启动命令行节点（根据电脑性能调整延迟时间，10-20 秒）
    delayed_cmd_node = TimerAction(
        period=15.0,  # 性能差就调大（如 20），避免节点启动过早报错
        actions=[cmd_vel_node]
    )

    # 6. 组装所有启动动作，返回 LaunchDescription 对象
    return LaunchDescription([
        include_gazebo_moveit,
        delayed_cmd_node
    ])