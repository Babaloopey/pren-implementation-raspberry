"""Audio chain: microphone_driver -> /audio/raw -> voice_command_detector -> /perception/voice_cmds

    ros2 launch robot_audio voice.launch.py [mic:=USB] [scores:=true]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    config = os.path.join(get_package_share_directory('robot_audio'), 'config', 'voice.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('mic', default_value='',
                              description='substring of the microphone name, empty = default input'),
        DeclareLaunchArgument('scores', default_value='false',
                              description='publish /perception/voice_scores for threshold tuning'),
        Node(package='robot_audio', executable='microphone_driver', name='microphone_driver',
             output='screen', parameters=[config, {'device_name': arg('mic')}]),
        Node(package='robot_audio', executable='voice_command_detector',
             name='voice_command_detector', output='screen',
             parameters=[config, {'publish_scores': ParameterValue(arg('scores'), value_type=bool)}]),
    ])
