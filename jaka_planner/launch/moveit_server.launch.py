import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare 'ip' and 'model' arguments
        DeclareLaunchArgument('ip', default_value='192.168.1.100', description='IP address'),
        DeclareLaunchArgument('model', default_value='default_model', description='Model name'),

        # 输出ip和model
        LogInfo(
            msg=["The IP address is: ", LaunchConfiguration('ip')] 
        ),
        LogInfo(
            msg=["The model is: ", LaunchConfiguration('model')]  
        ),

        # Launch the 'moveit_server' node from the 'jaka_planner' package
        Node(
            package='jaka_planner',
            executable='moveit_server',  # the executable to run
            name='moveit_server',
            output='screen',
            parameters=[
                {'ip': LaunchConfiguration('ip')},  # Pass 'ip' parameter
                {'model': LaunchConfiguration('model')}  # Pass 'model' parameter
            ],
        ),
    ])
