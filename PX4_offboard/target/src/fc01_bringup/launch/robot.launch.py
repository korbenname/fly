"""生产视觉接口入口：相机/mission/只读接收器；现有飞控由systemd独占。

接通真实PX4自机反馈与相机目标；不启动第二个fc01_system_node，默认关闭任务。
相机缺失不影响已有定高服务；原生独占RC/到高授权与最终姿态推力发布。
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
    defaults = {
        'control_enabled': 'false',
        'engage_mode': 'track',
        'control_debug_hz': '0.0',
        'start_camera': 'true',
        'start_px4_bridge': 'true',
        'platform_state_topic': '/fc01/platform/state',
        'native_flight_topic': '/fc01/native/flight_snapshot',
        'mission_config': os.path.join(get_package_share_directory('fc01_mission'), 'config', 'mission.yaml'),
        'bridge_config': os.path.join(get_package_share_directory('fc01_mission'), 'config', 'platform.yaml'),
        'read_only_receiver': 'true',
        'target_topic': '/fc01/vision/target_observation',
        'navigation_topic': '/fc01/navigation/command',
        'velocity_topic': '/fc01/vision/velocity_command_v2',
        'horizontal_topic': '/fc01/vision/horizontal_command',
        'publish_debug_image': 'false',
        'show_tuning_window': 'false',
        'vision_config': os.path.join(
            get_package_share_directory('fc01_vision'), 'config', 'vision.yaml'
        ),
    }
    actions = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    camera = Node(
        package='fc01_vision',
        executable='fc01_vision_node',
        name='fc01_vision',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_camera')),
        parameters=[
            LaunchConfiguration('vision_config'),
            {
                'state_topic': '/fc01/vision/target_observation',
                'publish_debug_image': ParameterValue(
                    LaunchConfiguration('publish_debug_image'), value_type=bool
                ),
                'show_tuning_window': ParameterValue(
                    LaunchConfiguration('show_tuning_window'), value_type=bool
                ),
            },
        ],
        remappings=[('/fc01/vision/target_observation', LaunchConfiguration('target_topic'))],
    )
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fc01_mission'), 'launch', 'mission.launch.py')
        ),
        launch_arguments={
            k: LaunchConfiguration(k)
            for k in (
                'control_enabled',
                'engage_mode',
                'control_debug_hz',
                'target_topic',
                'platform_state_topic',
                'native_flight_topic',
                'mission_config',
                'navigation_topic',
                'velocity_topic',
                'horizontal_topic',
            )
        }.items(),
    )
    receiver = Node(
        package='jikl_fc01_system',
        executable='fc01_target_interface_node',
        name='fc01_target_interface',
        output='screen',
        condition=IfCondition(LaunchConfiguration('read_only_receiver')),
        parameters=[
            {
                'velocity_topic': '/fc01/vision/velocity_command_v2',
                'navigation_topic': '/fc01/navigation/command',
                'target_topic': '/fc01/vision/target_observation',
            }
        ],
        remappings=[
            ('/fc01/vision/velocity_command_v2', LaunchConfiguration('velocity_topic')),
            ('/fc01/navigation/command', LaunchConfiguration('navigation_topic')),
            ('/fc01/vision/target_observation', LaunchConfiguration('target_topic')),
        ],
    )
    bridge = Node(package='fc01_mission', executable='px4_state_bridge',
                  name='fc01_px4_state_bridge', output='screen',
                  condition=IfCondition(LaunchConfiguration('start_px4_bridge')),
                  parameters=[LaunchConfiguration('bridge_config')],
                  remappings=[('/fc01/platform/state',LaunchConfiguration('platform_state_topic'))])
    return LaunchDescription(actions + [bridge, camera, mission, receiver])
