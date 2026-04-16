"""
coverage_execute.launch.py
启动覆盖路径执行节点

使用方法:
    ros2 launch auto_construct coverage_execute.launch.py \
        map_dir:=/home/nic/ROS/ROS/map/maps/map_20260329_211503

可选参数:
    map_dir         地图/路径目录 (必填，与 navigation.launch.py 保持一致)
    path_filename   路径 YAML 文件名 (默认 path.yaml)
    frame_id        坐标系 (默认 map)
    skip_on_failure 单点失败后是否跳过 (默认 true)
    autostart       是否自动开始 (默认 false，需手动调用 ~/start 服务)

目录结构:
    map_dir/
    ├── map.yaml
    ├── map.pgm
    ├── map.pcd
    └── path.yaml   ← 你的覆盖路径 YAML

手动控制:
    ros2 service call /coverage_path/start  std_srvs/srv/Trigger {}
    ros2 service call /coverage_path/pause  std_srvs/srv/Trigger {}
    ros2 service call /coverage_path/resume std_srvs/srv/Trigger {}
    ros2 service call /coverage_path/cancel std_srvs/srv/Trigger {}

监控进度:
    ros2 topic echo /coverage_path/progress
    ros2 topic echo /coverage_path/status
    ros2 topic echo /coverage_path/done
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    map_dir       = LaunchConfiguration('map_dir').perform(context)
    path_filename = LaunchConfiguration('path_filename').perform(context)
    frame_id      = LaunchConfiguration('frame_id').perform(context)
    skip_on_fail  = LaunchConfiguration('skip_on_failure').perform(context)
    autostart     = LaunchConfiguration('autostart').perform(context)

    path_file = os.path.join(map_dir, path_filename)

    if not os.path.exists(path_file):
        raise FileNotFoundError(
            f'[coverage_execute.launch] 路径文件不存在: {path_file}\n'
            f'请确认 map_dir 和 path_filename 参数正确。\n'
            f'当前: map_dir={map_dir}, path_filename={path_filename}'
        )

    executor_node = Node(
        package='auto_construct',
        executable='coverage_path',
        name='coverage_path',
        output='screen',
        emulate_tty=True,     # 让进度条 \r 在终端正常刷新
        parameters=[{
            'path_file':       path_file,
            'frame_id':        frame_id,
            'skip_on_failure': skip_on_fail.lower() in ('true', '1', 'yes'),
            'autostart':       autostart.lower() in ('true', '1', 'yes'),
        }],
    )

    return [
        LogInfo(msg=f'[coverage_execute.launch] 路径文件: {path_file}'),
        LogInfo(msg=f'[coverage_execute.launch] 坐标系:   {frame_id}'),
        LogInfo(msg=f'[coverage_execute.launch] 自动开始: {autostart}'),
        executor_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_dir',
            default_value='',
            description='地图目录完整路径 (与 navigation.launch.py 保持一致)',
        ),
        DeclareLaunchArgument(
            'path_filename',
            default_value='coverage_path_cache.yaml',
            description='路径 YAML 文件名 (map_dir 目录下的文件名)',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='路径坐标系，与 FASTLIO2 固定帧保持一致',
        ),
        DeclareLaunchArgument(
            'skip_on_failure',
            default_value='true',
            description='单个路径点失败时是否跳过继续 (true/false)',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='false',
            description='节点启动后是否自动开始 (false=手动调用 ~/start 服务)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
