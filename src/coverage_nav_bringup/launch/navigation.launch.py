"""
navigation.launch.py
启动 Nav2 导航栈，适配 FASTLIO2_ROS2 定位 + 麦轮底盘 + opennav_coverage

使用方法:
  ros2 launch coverage_nav_bringup navigation.launch.py map:=/path/to/map.yaml

可配置参数:
  map           - 2D 地图 yaml 文件路径 (必填)
  params_file   - Nav2 参数文件路径 (默认使用包内 nav2_params.yaml)
  use_sim_time  - 是否使用仿真时间 (默认 false)
  log_level     - 日志级别 (默认 info)
"""

import os
from ament_python_package import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ------------------------------------------------------------------ #
    # 包路径
    # ------------------------------------------------------------------ #
    pkg_share = FindPackageShare("coverage_nav_bringup")
    nav2_bringup_dir = FindPackageShare("nav2_bringup")

    # ------------------------------------------------------------------ #
    # Launch 参数声明
    # ------------------------------------------------------------------ #
    declare_map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="2D 占用栅格地图 yaml 文件的完整路径 (必填)",
    )

    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"]),
        description="Nav2 参数 yaml 文件路径",
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="是否使用仿真时钟 (Gazebo/Isaac Sim 时设为 true)",
    )

    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="日志级别: debug / info / warn / error",
    )

    # ------------------------------------------------------------------ #
    # Launch 配置变量
    # ------------------------------------------------------------------ #
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    # ------------------------------------------------------------------ #
    # FASTLIO2 → Nav2 TF 帧名桥接
    #
    # FASTLIO2_ROS2 (liangheming) 默认帧名:
    #   固定帧:  camera_init
    #   机器人:  body
    #   里程计:  /Odometry (child: body, frame: camera_init)
    #
    # Nav2 期望帧名:
    #   固定帧:  map
    #   机器人:  base_link
    #   里程计:  /odom (child: base_link, frame: odom)
    #
    # 方案A (推荐): 修改 FASTLIO2 配置文件，直接输出 map/odom/base_link
    #   → 在 FASTLIO2 的 config/lio_sam.yaml 中修改:
    #       mapFrame: map
    #       odometryFrame: odom
    #       pointCloudFrame: base_link
    #
    # 方案B (快速): 用 static_transform_publisher 做零偏移帧名重定向
    #   → 下方 tf_bridge_nodes 已配置好，取消注释即可使用
    # ------------------------------------------------------------------ #

    # 方案B: TF 帧名桥接节点 (方案A时注释掉这段)
    # tf_bridge_nodes = GroupAction([
    #     # camera_init → map (零偏移，仅重命名)
    #     Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         name="camera_init_to_map",
    #         arguments=["0", "0", "0", "0", "0", "0", "map", "camera_init"],
    #         parameters=[{"use_sim_time": use_sim_time}],
    #     ),
    #     # body → base_link (零偏移，仅重命名)
    #     Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         name="body_to_base_link",
    #         arguments=["0", "0", "0", "0", "0", "0", "base_link", "body"],
    #         parameters=[{"use_sim_time": use_sim_time}],
    #     ),
    # ])

    # ------------------------------------------------------------------ #
    # Map Server — 加载已保存的 2D 占用栅格地图
    # ------------------------------------------------------------------ #
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {"yaml_filename": map_yaml_file},
            {"use_sim_time": use_sim_time},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ------------------------------------------------------------------ #
    # Nav2 核心导航栈
    # ------------------------------------------------------------------ #
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "use_lifecycle_mgr": "true",
            "map_subscribe_transient_local": "true",
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # Lifecycle Manager — 管理 map_server 的生命周期
    # (nav2_bringup 中的 navigation_launch 管理其余节点)
    # ------------------------------------------------------------------ #
    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 4.0},
            {"node_names": ["map_server"]},
        ],
    )

    # ------------------------------------------------------------------ #
    # /Odometry → /odom topic 重映射节点
    # FASTLIO2 默认发布 /Odometry，Nav2 订阅 /odom
    # 方法: 用 topic_tools 做转发，或在 FASTLIO2 launch 中 remap
    # ------------------------------------------------------------------ #
    # odom_relay_node = Node(
    #     package="topic_tools",
    #     executable="relay",
    #     name="odom_relay",
    #     parameters=[{"use_sim_time": use_sim_time}],
    #     arguments=["--ros-args", "-r", "/Odometry:=/odom"],
    # )

    # ------------------------------------------------------------------ #
    # Launch 描述组装
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        # 参数声明
        declare_map_arg,
        declare_params_file_arg,
        declare_use_sim_time_arg,
        declare_log_level_arg,

        # 日志提示
        LogInfo(msg=["[navigation.launch] 地图文件: ", map_yaml_file]),
        LogInfo(msg=["[navigation.launch] 参数文件: ", params_file]),

        # 节点启动
        map_server_node,
        lifecycle_manager_map,
        nav2_stack,

        # 如需 TF 桥接，取消下行注释:
        # tf_bridge_nodes,
    ])
