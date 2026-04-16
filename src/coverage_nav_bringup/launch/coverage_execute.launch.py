"""
coverage_execute.launch.py
启动覆盖路径执行节点

使用方法:
    ros2 launch coverage_nav_bringup coverage_execute.launch.py \
        path_file:=/path/to/coverage.yaml

可配置参数:
    path_file        覆盖路径 YAML 文件完整路径 (必填)
    frame_id         坐标系名称 (默认 map)
    skip_on_failure  单点失败后是否跳过继续 (默认 true)
    autostart        是否自动开始 (默认 false，需手动调用 ~/start 服务)

示例 — 自动开始:
    ros2 launch coverage_nav_bringup coverage_execute.launch.py \
        path_file:=/home/user/paths/coverage.yaml \
        autostart:=true

示例 — 手动控制:
    ros2 launch coverage_nav_bringup coverage_execute.launch.py \
        path_file:=/home/user/paths/coverage.yaml
    # 然后:
    ros2 service call /coverage_path_executor/start std_srvs/srv/Trigger {}
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── 参数声明 ────────────────────────────────────────────────────────────────
    declare_path_file = DeclareLaunchArgument(
        'path_file',
        default_value='',
        description='覆盖路径 YAML 文件的完整路径 (必填)',
    )
    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='路径坐标系 (与 FASTLIO2 固定帧保持一致)',
    )
    declare_skip = DeclareLaunchArgument(
        'skip_on_failure',
        default_value='true',
        description='单个路径点导航失败时是否跳过继续 (true/false)',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='节点启动后是否自动开始执行 (true/false)',
    )

    # ── 执行节点 ─────────────────────────────────────────────────────────────────
    executor_node = Node(
        package='coverage_nav_bringup',
        executable='coverage_path_executor',
        name='coverage_path_executor',
        output='screen',
        emulate_tty=True,          # 让进度条 \r 在终端中正常刷新
        parameters=[{
            'path_file':       LaunchConfiguration('path_file'),
            'frame_id':        LaunchConfiguration('frame_id'),
            'skip_on_failure': LaunchConfiguration('skip_on_failure'),
            'autostart':       LaunchConfiguration('autostart'),
        }],
    )

    return LaunchDescription([
        declare_path_file,
        declare_frame_id,
        declare_skip,
        declare_autostart,
        LogInfo(msg=['[coverage_execute] 路径文件: ', LaunchConfiguration('path_file')]),
        LogInfo(msg=['[coverage_execute] 自动开始: ', LaunchConfiguration('autostart')]),
        executor_node,
    ])
