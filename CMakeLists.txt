cmake_minimum_required(VERSION 3.5)
project(obstacle_cloud_to_scan)

# C++ 標準の設定
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ポリシーの設定
cmake_policy(SET CMP0074 NEW)

# 依存パッケージの検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(pointcloud_to_laserscan REQUIRED)
find_package(yaml-cpp REQUIRED)

# 実行ファイルの生成
add_executable(obstacle_cloud_to_scan src/obstacle_cloud_to_scan.cpp)
ament_target_dependencies(obstacle_cloud_to_scan rclcpp sensor_msgs PCL pcl_conversions pointcloud_to_laserscan)

# インクルードディレクトリの追加
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# リンカ設定
target_link_libraries(obstacle_cloud_to_scan ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} yaml-cpp)

# インストール設定
install(TARGETS
  obstacle_cloud_to_scan
  DESTINATION lib/${PROJECT_NAME})

# configディレクトリとlaunchディレクトリのインストール
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# パッケージ設定
ament_package()

