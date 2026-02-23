cmake_minimum_required(VERSION 3.8)
project(lidar_imu)

# ---- C++ standard ----
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

# このパッケージ内のヘッダ (include/.hpp, include/box.data.hpp) を使うため
include_directories(
  include
)

# ================= LiDAR 数値ライブラリ =================
# my_lidar.cpp: readCSV, lidarlist, Lines2, modpi など
add_library(my_lidar_lib
  src/my_lidar.cpp
)
target_link_libraries(my_lidar_lib
  Eigen3::Eigen
)

# ================= 既存ノード（あなたの lidar_imu 側） =================

add_executable(imu_pubsub src/imu_pubsub.cpp)
ament_target_dependencies(imu_pubsub rclcpp std_msgs sensor_msgs)

add_executable(lidar_pubsub src/lidar_pubsub.cpp)
ament_target_dependencies(lidar_pubsub rclcpp std_msgs sensor_msgs)
# lidar_pubsub が .hpp を使うならリンク（不要なら削除OK）
target_link_libraries(lidar_pubsub my_lidar_lib Eigen3::Eigen)

add_executable(imu_gyro src/imu_gyro.cpp)
ament_target_dependencies(imu_gyro rclcpp std_msgs)
target_link_libraries(imu_gyro Eigen3::Eigen)


# ================= 追加ノード群 =================

# box_markers.cpp（箱Marker表示）
add_executable(box_markers src/box_markers.cpp)
ament_target_dependencies(box_markers rclcpp visualization_msgs)

# robot_points_markers.cpp（ロボットCUBE + LiDAR点群POINTS の合体ノード）
add_executable(robot_points_markers src/robot_points_markers.cpp)
ament_target_dependencies(robot_points_markers
  rclcpp
  std_msgs
  visualization_msgs
  geometry_msgs
  builtin_interfaces
)

# icp_localization_node.cpp（ICP自己位置推定）
add_executable(icp_localization_node
  src/icp_localization_node.cpp
)
ament_target_dependencies(icp_localization_node
  rclcpp
  std_msgs
  ament_index_cpp
)
target_link_libraries(icp_localization_node
  my_lidar_lib
  Eigen3::Eigen
)

# もし icp_localization_node を常に最適化したいなら（参考CMakeに合わせて任意で）
target_compile_options(icp_localization_node PRIVATE -O3)
target_link_options(icp_localization_node PRIVATE -O3)

# ================= インストール =================

install(TARGETS
  imu_pubsub
  lidar_pubsub
  imu_gyro
  box_markers
  robot_points_markers
  icp_localization_node
  
  DESTINATION lib/${PROJECT_NAME}
)

# map
install(FILES
  lidar_1211.csv
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
