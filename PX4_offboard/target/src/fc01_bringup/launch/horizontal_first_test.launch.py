"""首次水平候选试验入口：固定track，必须显式提供可追查的现场mission配置。

这里只启动视觉、实际PX4状态桥和任务，现有systemd独占原生控制器。任务默认关闭；
现场配置确认内外参/安全间距后可显式control_enabled:=true，仍不能越过RC和原生到高门。
本文件不改PX4参数，不启动或解锁电机，也不把仓库候选标定值视为现场确认。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('site_mission_config', description='现场核验后的完整mission YAML路径'),
        DeclareLaunchArgument('control_enabled', default_value='false'),
        DeclareLaunchArgument('vision_config', default_value=os.path.join(
            get_package_share_directory('fc01_vision'),'config','vision.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('fc01_bringup'),'launch','robot.launch.py')),
            launch_arguments={
                'mission_config':LaunchConfiguration('site_mission_config'),
                'vision_config':LaunchConfiguration('vision_config'),
                # 不暴露push/impact覆盖口；实机入口硬性选track，模拟分支保持原兼容。
                'engage_mode':'track',
                'control_enabled':LaunchConfiguration('control_enabled'),
            }.items()),
    ])
