cmake_minimum_required(VERSION 3.8)
project(robomas_package_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorFeedback.msg"
  "msg/MotorCmd.msg"
  "msg/MotorCmdArray.msg"
  "msg/CanFrame.msg"
  "msg/Ems.msg"
)

add_executable(receiver_can0 src/receiver_node_can0.cpp)
ament_target_dependencies(receiver_can0 rclcpp std_msgs)

add_executable(sender_can0 src/set_robomas_value_node_can0.cpp)
ament_target_dependencies(sender_can0 rclcpp std_msgs)

add_executable(receiver_can1 src/receiver_node_can1.cpp)
ament_target_dependencies(receiver_can1 rclcpp std_msgs)

add_executable(sender_can1 src/set_robomas_value_node_can1.cpp)
ament_target_dependencies(sender_can1 rclcpp std_msgs)

# Use rosidl_get_typesupport_target + target_link_libraries instead of rosidl_target_interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(receiver_can0 "${cpp_typesupport_target}")
target_link_libraries(sender_can0 "${cpp_typesupport_target}")
target_link_libraries(receiver_can1 "${cpp_typesupport_target}")
target_link_libraries(sender_can1 "${cpp_typesupport_target}")

install(TARGETS
  receiver_can0
  sender_can0
  receiver_can1
  sender_can1
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()