cmake_minimum_required(VERSION 3.8)
project(odometry_plugins)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(include ${Boost_INCLUDE_DIRS})


# check if ROS_DISTRO is foxy or humble
if($ENV{ROS_DISTRO} STREQUAL "humble")
  ament_export_dependencies(rosidl_default_runtime)
endif()


add_library(odometry_bridge SHARED
src/odometry_bridge.cpp
  src/as_plugin.cpp)

# check if ROS_DISTRO is foxy or humbles
if($ENV{ROS_DISTRO} STREQUAL "humble")
  target_link_libraries(odometry_bridge "${cpp_typesupport_target}")
elseif($ENV{ROS_DISTRO} STREQUAL "foxy")
  rosidl_target_interfaces(odometry_bridge
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()


ament_target_dependencies(odometry_bridge
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
)

rclcpp_components_register_nodes(odometry_bridge "odometry_bridge::OdometryBridge")

add_executable(odometry_bridge_asnode
  src/odometry_bridge.cpp
  src/as_runnable.cpp)

# check if ROS_DISTRO is foxy or humbles
if($ENV{ROS_DISTRO} STREQUAL "humble")
  target_link_libraries(odometry_bridge_asnode "${cpp_typesupport_target}")
elseif($ENV{ROS_DISTRO} STREQUAL "foxy")
  rosidl_target_interfaces(odometry_bridge_asnode
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()


ament_target_dependencies(odometry_bridge_asnode
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

install(TARGETS
  odometry_bridge odometry_bridge_asnode
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}  
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
