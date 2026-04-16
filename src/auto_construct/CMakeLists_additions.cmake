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
find_package(rosidl_default_generators REQUIRED)

# ── 自定义 Service 生成 (必须在 add_executable 之前) ─────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPathAndStart.srv"
)

# ── 可执行目标 ───────────────────────────────────────────────────────────────
add_executable(coverage_path
  src/coverage_path.cpp
)

target_include_directories(coverage_path PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_target_dependencies(coverage_path
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
  std_srvs
  action_msgs
)

# 链接 yaml-cpp 和自定义 Service 的 C++ typesupport
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(coverage_path
  yaml-cpp
  "${cpp_typesupport_target}"
)

# ── 安装可执行文件 ────────────────────────────────────────────────────────────
install(TARGETS coverage_path
  DESTINATION lib/${PROJECT_NAME}
)

# ── 安装 launch / config / srv 目录 ──────────────────────────────────────────
# 如果你的 CMakeLists.txt 中已有类似的 install(DIRECTORY ...) 语句，
# 只需在 DIRECTORY 后追加 launch config 即可，不要重复写 install()
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
