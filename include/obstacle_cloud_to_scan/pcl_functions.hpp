#ifndef PCL_PROCESSING_FUNCTIONS_H
#define PCL_PROCESSING_FUNCTIONS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <rclcpp/rclcpp.hpp>

// 地面平面の定義（平面方程式: ax + by + cz + d = 0）
struct GroundPlane {
    double a, b, c, d;
    
    // 点から平面までの符号付き距離を計算
    double distanceToPoint(const pcl::PointXYZ &point) const {
        return (a * point.x + b * point.y + c * point.z + d) / 
               std::sqrt(a * a + b * b + c * c);
    }
};

// ダウンサンプリング
pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double voxel_leaf_size,
    rclcpp::Logger logger);

// パススルーフィルタ
pcl::PointCloud<pcl::PointXYZ>::Ptr applyPassThroughFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<double> &robot_box_size,
    rclcpp::Logger logger);

// 法線推定
pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double normal_radius,
    rclcpp::Logger logger);

// 法線から障害物を割り出し
pcl::PointCloud<pcl::PointXYZ>::Ptr filterObstacles(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    double max_slope_angle,
    rclcpp::Logger logger);

// ロボット自身のポイントクラウドを除去
pcl::PointCloud<pcl::PointXYZ>::Ptr removeRobotBody(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<double> &box_position,
    const std::vector<double> &box_size,
    rclcpp::Logger logger);

// PMFによる地面除去
pcl::PointCloud<pcl::PointXYZ>::Ptr applyProgressiveMorphologicalFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    rclcpp::Logger logger,
    int max_window_size,
    double slope,             // Changed from float to double for consistency
    double initial_distance,  // Changed from float to double
    double max_distance,      // Changed from float to double
    double cell_size);        // Changed from float to double

// 穴検知用フィルタ
pcl::PointCloud<pcl::PointXYZ>::Ptr filterHoleDetectionRange(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double range_x,
    double range_y,
    double max_height,
    rclcpp::Logger logger);

// 光線と平面の交点計算
bool rayPlaneIntersection(
    const pcl::PointXYZ &ray_start,
    const pcl::PointXYZ &ray_end,
    const GroundPlane &plane,
    pcl::PointXYZ &intersection);

// 基本穴検知
pcl::PointCloud<pcl::PointXYZ>::Ptr detectHolesBasic(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointXYZ &lidar_origin,
    const GroundPlane &ground_plane,
    double ground_tolerance,
    rclcpp::Logger logger);

#endif // PCL_PROCESSING_FUNCTIONS_H
