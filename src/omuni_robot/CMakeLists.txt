cmake_minimum_required(VERSION 3.8)
project(omuni_robot)

# ---- コンパイラ警告 ----
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 必須パッケージ ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 自作 msg を使っているので必須
find_package(robomas_package_2 REQUIRED)

# ---- 実行ファイル ----
add_executable(omuni_3rin_node
  src/omuni_3rin.cpp
)

#コンパイルのバージョンをg++ 20に指定。これによりnumbersが使えるようになる、らしい。
target_compile_features(omuni_3rin_node PUBLIC cxx_std_20)

# ---- 依存関係 ----
ament_target_dependencies(omuni_3rin_node
  rclcpp
  sensor_msgs
  robomas_package_2
)

# ---- インストール ----
install(TARGETS
  omuni_3rin_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY tools
  DESTINATION share/${PROJECT_NAME}
)


# ---- ament ----
ament_package()
