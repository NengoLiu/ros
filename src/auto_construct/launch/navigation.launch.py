"""
navigation.launch.py
启动 Nav2 导航栈（适配 FASTLIO2 定位，无 AMCL）

使用方法:
    ros2 launch auto_construct navigation.launch.py \
        map_dir:=/home/nic/ROS/ROS/map/maps/map_20260329_211503

map_dir 目录结构要求:
    map_20260329_211503/
    ├── map.pgm          ← 2D 占用栅格地图图像
    ├── map.yaml         ← Nav2 地图服务配置
    ├── map.pcd          ← 3D 点云地图 (FASTLIO2 使用，此处无需加载)
    └── path.yaml        ← 覆盖路径 (由 coverage_execute.launch.py 使用)

FASTLIO2 TF 帧名说明:
    FASTLIO2 默认输出: camera_init → body
    Nav2 期望:         map → odom → base_link
    若帧名不同，在本文件末尾取消注释 tf_bridge 节点。
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
    """OpaqueFunction：在此处组合路径，避免 LaunchConfiguration 不可拼接的问题。"""

    map_dir       = LaunchConfiguration('map_dir').perform(context)
    use_sim_time  = LaunchConfiguration('use_sim_time').perform(context)
    params_file   = LaunchConfiguration('params_file').perform(context)
    map_filename  = LaunchConfiguration('map_filename').perform(context)

    map_yaml_file = os.path.join(map_dir, map_filename)

    if not os.path.exists(map_yaml_file):
        raise FileNotFoundError(
            f'[navigation.launch] 地图文件不存在: {map_yaml_file}\n'
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
            'use_sim_time':  sim_time_bool,
            'autostart':     True,
            'bond_timeout':  4.0,
            'node_names':    ['map_server'],
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

    # ─────────────────────────────────────────────────────────────────────────
    # FASTLIO2 → Nav2 TF 帧名桥接
    # 若 FASTLIO2 已配置输出 map/odom/base_link，则注释掉下面这段。
    # 若 FASTLIO2 输出 camera_init/body，则取消注释。
    # ─────────────────────────────────────────────────────────────────────────
    # tf_bridge = GroupAction([
    #     # camera_init → map (零偏移重命名)
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='camera_init_to_map',
    #         arguments=['0','0','0','0','0','0','map','camera_init'],
    #         parameters=[{'use_sim_time': sim_time_bool}],
    #     ),
    #     # body → base_link (零偏移重命名)
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='body_to_base_link',
    #         arguments=['0','0','0','0','0','0','base_link','body'],
    #         parameters=[{'use_sim_time': sim_time_bool}],
    #     ),
    #     # FASTLIO2 发布 /Odometry，Nav2 订阅 /odom
    #     # 在 FASTLIO2 launch 文件中加 remap 更简洁，这里备用
    # ])

    return [
        LogInfo(msg=f'[navigation.launch] 地图目录: {map_dir}'),
        LogInfo(msg=f'[navigation.launch] 地图文件: {map_yaml_file}'),
        LogInfo(msg=f'[navigation.launch] 参数文件: {params_file}'),
        map_server_node,
        lc_map,
        nav2_launch,
        # tf_bridge,   ← 需要桥接时取消注释
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
            'map_filename',
            default_value='pcd2pgm_map.yaml',
            description='2D 地图 YAML 文件名 (map_dir 目录下)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
