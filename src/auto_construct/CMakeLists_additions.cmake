# =============================================================================
# 将以下内容追加/合并到你的 auto_construct/CMakeLists.txt 中
# =============================================================================

# ── 新增依赖 (在 find_package(ament_cmake REQUIRED) 之后添加) ────────────────
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── 可执行目标 (在 ament_target_dependencies 之前添加) ───────────────────────
add_executable(coverage_path_executor
  src/coverage_path_executor.cpp
)

target_include_directories(coverage_path_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_target_dependencies(coverage_path_executor
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
  std_srvs
  action_msgs
)

target_link_libraries(coverage_path_executor
  yaml-cpp
)

# ── 安装可执行文件 ────────────────────────────────────────────────────────────
install(TARGETS coverage_path_executor
  DESTINATION lib/${PROJECT_NAME}
)

# ── 安装 launch / config 目录 ─────────────────────────────────────────────────
# 如果你的 CMakeLists.txt 中已有类似的 install(DIRECTORY ...) 语句，
# 只需在 DIRECTORY 后追加 launch config 即可，不要重复写 install()
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
