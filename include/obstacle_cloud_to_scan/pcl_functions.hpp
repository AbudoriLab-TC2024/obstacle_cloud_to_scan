#ifndef PCL_PROCESSING_FUNCTIONS_H
#define PCL_PROCESSING_FUNCTIONS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <rclcpp/rclcpp.hpp>

// Function to downsample point cloud using voxel grid filter
pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double voxel_leaf_size,
    bool use_gpu,
    rclcpp::Logger logger);

// Function to apply passthrough filter to point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr applyPassThroughFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<double> &robot_box_size,
    rclcpp::Logger logger);

// Function to estimate normals of point cloud
pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    rclcpp::Logger logger);

// Function to filter obstacles from point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr filterObstacles(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    double max_slope_angle,
    rclcpp::Logger logger);

#endif // PCL_PROCESSING_FUNCTIONS_H
