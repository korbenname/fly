"""目标制导接口入口；第1步默认不激活任务，不代替原生RC/到高授权。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('control_enabled', default_value='false'),
        DeclareLaunchArgument('engage_mode', default_value='track'),
        DeclareLaunchArgument('use_vision_state', default_value='true'),
        DeclareLaunchArgument('control_debug_hz', default_value='0.0'),
        DeclareLaunchArgument('platform_state_topic', default_value='/fc01/platform/state'),
        DeclareLaunchArgument('native_flight_topic', default_value='/fc01/native/flight_state'),
        DeclareLaunchArgument('mission_config', default_value=os.path.join(get_package_share_directory('fc01_mission'), 'config', 'mission.yaml')),
        DeclareLaunchArgument('target_topic', default_value='/fc01/vision/target_observation'),
        DeclareLaunchArgument('navigation_topic', default_value='/fc01/navigation/command'),
        DeclareLaunchArgument('velocity_topic', default_value='/fc01/vision/velocity_command_v2'),
        DeclareLaunchArgument('horizontal_topic', default_value='/fc01/vision/horizontal_command'),
    ]
    node = Node(
        package='fc01_mission',
        executable='mission',
        name='fc01_mission',
        output='screen',
        parameters=[
            LaunchConfiguration('mission_config'),
            {
                'control_enabled': ParameterValue(
                    LaunchConfiguration('control_enabled'), value_type=bool
                ),
                'engage_mode': LaunchConfiguration('engage_mode'),
                'use_vision_state': ParameterValue(
                    LaunchConfiguration('use_vision_state'), value_type=bool
                ),
                'control_debug_hz': ParameterValue(
                    LaunchConfiguration('control_debug_hz'), value_type=float
                ),
                'vision_state_topic': '/fc01/vision/target_observation',
                'platform_input_mode': 'px4',
                'platform_state_topic': '/fc01/platform/state',
                'native_flight_topic': '/fc01/native/flight_state',
                'navigation_command_topic': '/navigation_command',
                'vision_control_topic': '/visioncontrol',
                'horizontal_command_topic': '/fc01/vision/horizontal_command',
            },
        ],
        # 映射实际作用于发布/订阅者，不只写在说明里；V1诊断topic不参与映射。
        remappings=[
            ('/fc01/vision/target_observation', LaunchConfiguration('target_topic')),
            ('/fc01/platform/state', LaunchConfiguration('platform_state_topic')),
            ('/fc01/native/flight_state', LaunchConfiguration('native_flight_topic')),
            ('/navigation_command', LaunchConfiguration('navigation_topic')),
            ('/visioncontrol', LaunchConfiguration('velocity_topic')),
            ('/fc01/vision/horizontal_command', LaunchConfiguration('horizontal_topic')),
        ],
    )
    return LaunchDescription(args + [node])
