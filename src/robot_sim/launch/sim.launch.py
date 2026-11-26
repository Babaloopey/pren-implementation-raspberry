"""PC simulator: ros2 launch robot_sim sim.launch.py [port:=8080] [bones:=1] [camera_height:=0.25] [profile:=local|hil]

Starts only the simulator; it starts/stops the real robot nodes itself (GUI switches).
GUI: http://localhost:8080
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('bones', default_value='1'),
        DeclareLaunchArgument('camera_height', default_value='0.25'),
        DeclareLaunchArgument('log_dir', default_value='/ws/logs'),
        DeclareLaunchArgument('profile', default_value='local',
                              description='local: all nodes on this PC, hil: robot nodes on the Raspberry Pi'),
        Node(package='robot_sim', executable='simulator', name='simulator', output='screen',
             parameters=[{
                 'http_port': ParameterValue(arg('port'), value_type=int),
                 'n_bones': ParameterValue(arg('bones'), value_type=int),
                 'camera_height': ParameterValue(arg('camera_height'), value_type=float),
                 'log_dir': arg('log_dir'),
                 'profile': arg('profile'),
             }]),
    ])
