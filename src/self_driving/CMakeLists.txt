cmake_minimum_required(VERSION 3.8)
project(self_driving)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robomas_interfaces REQUIRED)

# --- Generate messages BEFORE building nodes ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallArray.msg"
  "msg/Ball.msg"
  "msg/Target.msg"
  "msg/TargetStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# --- Export msg dependencies BEFORE building nodes ---
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)

# --- Executables ---
add_executable(pure_pursuit_node src/pure_pursuit.cpp)
add_executable(icp_tf_broadcaster src/icp_tf_broadcaster.cpp)
add_executable(my_pure_pursuit_node src/my_pure_pursuit.cpp)


# --- Link dependencies ---
ament_target_dependencies(pure_pursuit_node
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)

ament_target_dependencies(icp_tf_broadcaster
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)

ament_target_dependencies(my_pure_pursuit_node
  rclcpp std_msgs robomas_interfaces
)
# 生成されたメッセージヘッダーへの依存設定
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(my_pure_pursuit_node "${cpp_typesupport_target}")

include_directories(include)

# --- Install ---
install(TARGETS
  pure_pursuit_node
  icp_tf_broadcaster
  my_pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()