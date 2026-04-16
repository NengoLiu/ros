"""
bringup.launch.py
一键启动：Nav2 导航栈 + 覆盖路径执行节点

无需在启动时指定任何路径，地图和路径均通过 Service 动态传入。

使用方法:
    ros2 launch auto_construct bringup.launch.py

启动后，用 Service 传入地图 + 路径并开始导航:
    ros2 service call /coverage_path_executor/set_path_and_start \\
        auto_construct/srv/SetPathAndStart \\
        "{map_file:  '/home/nic/ROS/ROS/map/maps/map_A/pcd2pgm_map.yaml',
          path_file: '/home/nic/ROS/ROS/map/maps/map_A/coverage_path_cache.yaml'}"

切换到另一张地图并重新导航 (先 cancel 当前任务):
    ros2 service call /coverage_path_executor/cancel std_srvs/srv/Trigger {}
    ros2 service call /coverage_path_executor/set_path_and_start \\
        auto_construct/srv/SetPathAndStart \\
        "{map_file:  '/home/nic/ROS/ROS/map/maps/map_B/pcd2pgm_map.yaml',
          path_file: '/home/nic/ROS/ROS/map/maps/map_B/coverage_path_cache.yaml'}"

仅换路径 (沿用当前地图):
    ros2 service call /coverage_path_executor/set_path_and_start \\
        auto_construct/srv/SetPathAndStart \\
        "{path_file: '/home/nic/ROS/ROS/map/maps/map_A/coverage_path_cache.yaml'}"

其他控制:
    ros2 service call /coverage_path_executor/pause  std_srvs/srv/Trigger {}
    ros2 service call /coverage_path_executor/resume std_srvs/srv/Trigger {}
    ros2 service call /coverage_path_executor/cancel std_srvs/srv/Trigger {}

监控进度:
    ros2 topic echo /coverage_path_executor/status
    ros2 topic echo /coverage_path_executor/progress
    ros2 topic echo /coverage_path_executor/done

可选参数:
    params_file     Nav2 参数文件路径 (默认 auto_construct/config/nav2_params.yaml)
    use_sim_time    是否使用仿真时间 (默认 false)
    frame_id        路径坐标系默认值 (默认 map)
    skip_on_failure 单点失败后跳过 (默认 true)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    params_file  = LaunchConfiguration('params_file').perform(context)
    frame_id     = LaunchConfiguration('frame_id').perform(context)
    skip_on_fail = LaunchConfiguration('skip_on_failure').perform(context)

    sim_time_bool = use_sim_time.lower() in ('true', '1', 'yes')

    # ── Map Server — 以空地图启动，等待 load_map service 传入实际地图 ────────────
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': ''},   # 空地图，运行时由 set_path_and_start 动态加载
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
    nav2_launch_path = os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
    from launch.actions import IncludeLaunchDescription
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
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
            'path_file':       '',
            'frame_id':        frame_id,
            'skip_on_failure': skip_on_fail.lower() in ('true', '1', 'yes'),
            'autostart':       False,
        }],
    )

    return [
        LogInfo(msg='[bringup] ══════════════════════════════════════════════'),
        LogInfo(msg='[bringup] Nav2 + CoveragePathExecutor 启动中...'),
        LogInfo(msg='[bringup] 就绪后调用 set_path_and_start service 开始导航'),
        LogInfo(msg='[bringup] 示例:'),
        LogInfo(msg='[bringup]   ros2 service call /coverage_path_executor/set_path_and_start \\'),
        LogInfo(msg='[bringup]       auto_construct/srv/SetPathAndStart \\'),
        LogInfo(msg='[bringup]       "{map_file: \'<地图目录>/pcd2pgm_map.yaml\','),
        LogInfo(msg='[bringup]         path_file: \'<地图目录>/coverage_path_cache.yaml\'}"'),
        LogInfo(msg='[bringup] ══════════════════════════════════════════════'),
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
            description='路径坐标系默认值',
        ),
        DeclareLaunchArgument(
            'skip_on_failure',
            default_value='true',
            description='单个路径点失败时是否跳过继续',
        ),
        OpaqueFunction(function=launch_setup),
    ])
