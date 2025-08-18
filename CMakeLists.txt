cmake_minimum_required(VERSION 3.5)
project(obstacle_cloud_to_scan)

# C++ 標準の設定
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ポリシーの設定
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

# 依存パッケージの検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(pointcloud_to_laserscan REQUIRED)
find_package(yaml-cpp REQUIRED)

# 実行ファイルの生成
add_library(pcl_functions src/pcl_functions.cpp)
target_link_libraries(pcl_functions ${PCL_LIBRARIES})
ament_target_dependencies(pcl_functions rclcpp PCL pointcloud_to_laserscan)

add_executable(obstacle_cloud_to_scan src/obstacle_cloud_to_scan.cpp)
target_link_libraries(obstacle_cloud_to_scan pcl_functions ${pointcloud_to_laserscan_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(obstacle_cloud_to_scan 
    rclcpp 
    tf2_ros 
    tf2_sensor_msgs 
    sensor_msgs 
    PCL 
    pcl_conversions 
    pointcloud_to_laserscan)

# インクルードディレクトリの追加
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${pointcloud_to_laserscan_INCLUDE_DIRS}
)

# リンカ設定
target_link_libraries(obstacle_cloud_to_scan ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} yaml-cpp)

# インストール設定
install(TARGETS
  obstacle_cloud_to_scan
  pcl_functions
  DESTINATION lib/${PROJECT_NAME})

# launchディレクトリのインストール
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# パッケージ設定
ament_package()

