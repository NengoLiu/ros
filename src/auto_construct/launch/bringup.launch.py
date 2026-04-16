"""
bringup.launch.py
一键启动：Nav2 导航栈 + 覆盖路径执行节点

使用方法:
    ros2 launch auto_construct bringup.launch.py \
        map_dir:=/home/nic/ROS/ROS/map/maps/map_20260329_211503

启动完成后，用 Service 一键设定路径并开始导航:
    ros2 service call /coverage_path_executor/set_path_and_start \
        auto_construct/srv/SetPathAndStart \
        "{path_file: '/home/nic/ROS/ROS/map/maps/map_20260329_211503/coverage_path_cache.yaml'}"

其他控制:
    ros2 service call /coverage_path_executor/pause  std_srvs/srv/Trigger {}
    ros2 service call /coverage_path_executor/resume std_srvs/srv/Trigger {}
    ros2 service call /coverage_path_executor/cancel std_srvs/srv/Trigger {}

监控进度:
    ros2 topic echo /coverage_path_executor/status
    ros2 topic echo /coverage_path_executor/progress
    ros2 topic echo /coverage_path_executor/done

可选参数:
    map_dir         地图目录完整路径 (必填)
    map_filename    2D 地图 YAML 文件名 (默认 pcd2pgm_map.yaml)
    params_file     Nav2 参数文件路径 (默认 auto_construct/config/nav2_params.yaml)
    use_sim_time    是否使用仿真时间 (默认 false)
    frame_id        路径坐标系 (默认 map)
    skip_on_failure 单点失败后跳过 (默认 true)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    map_dir      = LaunchConfiguration('map_dir').perform(context)
    map_filename = LaunchConfiguration('map_filename').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    params_file  = LaunchConfiguration('params_file').perform(context)
    frame_id     = LaunchConfiguration('frame_id').perform(context)
    skip_on_fail = LaunchConfiguration('skip_on_failure').perform(context)

    map_yaml_file = os.path.join(map_dir, map_filename)

    if not os.path.exists(map_yaml_file):
        raise FileNotFoundError(
            f'[bringup.launch] 地图文件不存在: {map_yaml_file}\n'
            f'请确认 map_dir 和 map_filename 参数正确。'
        )

    sim_time_bool = use_sim_time.lower() in ('true', '1', 'yes')

    # ── Map Server ────────────────────────────────────────────────────────────
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_yaml_file},
            {'use_sim_time': sim_time_bool},
        ],
    )

    # ── Lifecycle Manager (map_server) ────────────────────────────────────────
    lc_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': sim_time_bool,
            'autostart':    True,
            'bond_timeout': 4.0,
            'node_names':   ['map_server'],
        }],
    )

    # ── Nav2 核心导航栈 ────────────────────────────────────────────────────────
    nav2_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time':                  use_sim_time,
            'params_file':                   params_file,
            'use_lifecycle_mgr':             'true',
            'map_subscribe_transient_local': 'true',
        }.items(),
    )

    # ── 覆盖路径执行节点 ──────────────────────────────────────────────────────
    executor_node = Node(
        package='auto_construct',
        executable='coverage_path_executor',
        name='coverage_path_executor',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'path_file':       '',           # 启动时不预加载，通过 set_path_and_start 设定
            'frame_id':        frame_id,
            'skip_on_failure': skip_on_fail.lower() in ('true', '1', 'yes'),
            'autostart':       False,
        }],
    )

    return [
        LogInfo(msg='[bringup.launch] ═══════════════════════════════════'),
        LogInfo(msg=f'[bringup.launch] 地图目录: {map_dir}'),
        LogInfo(msg=f'[bringup.launch] 地图文件: {map_yaml_file}'),
        LogInfo(msg=f'[bringup.launch] 参数文件: {params_file}'),
        LogInfo(msg='[bringup.launch] 启动完成后执行:'),
        LogInfo(msg='[bringup.launch]   ros2 service call /coverage_path_executor/set_path_and_start \\'),
        LogInfo(msg='[bringup.launch]       auto_construct/srv/SetPathAndStart \\'),
        LogInfo(msg=f'[bringup.launch]       "{{path_file: \'{map_dir}/coverage_path_cache.yaml\'}}"'),
        LogInfo(msg='[bringup.launch] ═══════════════════════════════════'),
        map_server_node,
        lc_map,
        nav2_launch,
        executor_node,
    ]


def generate_launch_description():
    pkg = FindPackageShare('auto_construct').find('auto_construct')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_dir',
            default_value='',
            description='地图目录完整路径 (例: /home/nic/ROS/ROS/map/maps/map_20260329_211503)',
        ),
        DeclareLaunchArgument(
            'map_filename',
            default_value='pcd2pgm_map.yaml',
            description='2D 地图 YAML 文件名 (map_dir 目录下)',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 参数文件路径',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='路径坐标系',
        ),
        DeclareLaunchArgument(
            'skip_on_failure',
            default_value='true',
            description='单个路径点失败时是否跳过继续',
        ),
        OpaqueFunction(function=launch_setup),
    ])
