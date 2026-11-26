"""All perception nodes of the robot (camera, bones, gestures, voice).

    ros2 launch robot_vision perception.launch.py [camera:=true] [bones:=true] [gestures:=true]
        [voice:=true] [debug:=false] [mic:=USB]

Data flow (no image_processor / voice_processor: the drivers deliver the final format):
  camera_driver     -> /camera/image_raw -> gesture_detector, bone_detector
  microphone_driver -> /audio/raw        -> voice_command_detector

Outputs (consumed by state_coordinator), types from robot_interfaces:
  /perception/bone_position  BoneDetectionArray (bearing always, position only with camera_height > 0)
  /perception/gestures       Gesture
  /perception/voice_cmds     VoiceCommand
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    config = os.path.join(get_package_share_directory('robot_vision'), 'config', 'vision.yaml')
    debug = {'publish_debug_image': ParameterValue(arg('debug'), value_type=bool)}

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('bones', default_value='true'),
        DeclareLaunchArgument('gestures', default_value='true'),
        DeclareLaunchArgument('voice', default_value='true'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='publish annotated images on /perception/*_debug'),
        DeclareLaunchArgument('mic', default_value='', description='microphone name substring'),

        Node(package='robot_vision', executable='camera_driver', name='camera_driver',
             output='screen', condition=IfCondition(arg('camera'))),
        Node(package='robot_vision', executable='bone_detector', name='bone_detector',
             output='screen', parameters=[config, debug], condition=IfCondition(arg('bones'))),
        Node(package='robot_vision', executable='gesture_detector', name='gesture_detector',
             output='screen', parameters=[config, debug], condition=IfCondition(arg('gestures'))),
        # camera_link as in the simulator: 0.2 m in front of base_link, 0.25 m above the floor.
        # Adjust to the real mount (and keep camera_height in vision.yaml consistent).
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_tf',
             arguments=['--x', '0.2', '--z', '0.25',
                        '--frame-id', 'base_link', '--child-frame-id', 'camera_link']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robot_audio'), 'launch', 'voice.launch.py')),
            launch_arguments={'mic': arg('mic')}.items(),
            condition=IfCondition(arg('voice'))),
    ])
