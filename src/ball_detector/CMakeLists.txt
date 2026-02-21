cmake_minimum_required(VERSION 3.8)
project(ball_detector)

# --- コンパイラ設定 ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- ROS2依存パッケージ ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ball.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# ament_export_dependencies(rosidl_default_runtime)


# --- OpenCV ---
find_package(OpenCV REQUIRED)

# --- 実行ファイル ---
add_executable(ball_detector_node src/main.cpp)
ament_target_dependencies(ball_detector_node rclcpp geometry_msgs)
target_link_libraries(ball_detector_node ${OpenCV_LIBS})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ball_detector_node ${cpp_typesupport_target} ${OpenCV_LIBS})

install(FILES camera_calib.yml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# --- ボール色マーカーノード ---
add_executable(ball_color_marker_node src/ball_color_marker_node.cpp)

ament_target_dependencies(ball_color_marker_node
    rclcpp
    visualization_msgs
)

install(TARGETS ball_color_marker_node
    DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(ball_color_marker_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ball_color_marker_node ${cpp_typesupport_target})

# --- インストール設定 ---
install(TARGETS
  ball_detector_node
  DESTINATION lib/${PROJECT_NAME})



ament_package()
