"""The whole robot: perception + GPIO drivers + behaviour (state_coordinator, command_coordinator).

    ros2 launch robot_core robot.launch.py [gpio:=true] [camera:=true] [bones:=true]
        [gestures:=true] [voice:=true] [debug:=false] [mic:=MV7]

Logs: ~/pren_logs/states_*.log (state changes), ~/pren_logs/commands_*.log (motor commands)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg = LaunchConfiguration
    config = os.path.join(get_package_share_directory('robot_core'), 'config', 'core.yaml')
    perception = os.path.join(get_package_share_directory('robot_vision'), 'launch',
                              'perception.launch.py')
    pass_through = ['camera', 'bones', 'gestures', 'voice', 'debug', 'mic']

    gpio = [Node(package='robot_gpio', executable=exe, name=exe, output='screen',
                 condition=IfCondition(arg('gpio')))
            for exe in ('search_button_driver', 'bone_button_driver', 'led_driver', 'speaker_driver')]

    return LaunchDescription([
        DeclareLaunchArgument('gpio', default_value='true',
                              description='start the button, LED and speaker drivers'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('bones', default_value='true'),
        DeclareLaunchArgument('gestures', default_value='true'),
        DeclareLaunchArgument('voice', default_value='true'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('mic', default_value='MV7'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(perception),
                                 launch_arguments={k: arg(k) for k in pass_through}.items()),
        *gpio,
        Node(package='robot_core', executable='command_coordinator', name='command_coordinator',
             output='screen', parameters=[config]),
        Node(package='robot_core', executable='state_coordinator', name='state_coordinator',
             output='screen', parameters=[config]),
    ])
