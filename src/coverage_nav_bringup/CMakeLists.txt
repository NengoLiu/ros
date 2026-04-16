cmake_minimum_required(VERSION 3.8)
project(coverage_nav_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python 脚本安装为可执行文件 (ros2 run 可直接调用)
install(PROGRAMS
  scripts/coverage_path_executor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME coverage_path_executor
)

# 安装配置文件、launch 文件、地图目录、路径目录
install(
  DIRECTORY config launch maps paths bt_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
