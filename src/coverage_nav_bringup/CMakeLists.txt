cmake_minimum_required(VERSION 3.8)
project(coverage_nav_bringup)

find_package(ament_cmake REQUIRED)

# 安装配置文件、launch 文件、地图目录、路径目录
install(
  DIRECTORY config launch maps paths bt_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
