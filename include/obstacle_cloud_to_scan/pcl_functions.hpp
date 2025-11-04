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

// 法線推定
pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double normal_radius,
    rclcpp::Logger logger);

// 障害物フィルタリング（地面点と障害物点の両方を返す）
bool filterObstaclesWithGround(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &obstacle_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud,
    double max_slope_angle,
    rclcpp::Logger logger);

// PMFによる地面除去
pcl::PointCloud<pcl::PointXYZ>::Ptr applyProgressiveMorphologicalFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    rclcpp::Logger logger,
    int max_window_size,
    double slope,
    double initial_distance,
    double max_distance,
    double cell_size);

// PMFによる地面除去（地面点と障害物点の両方を返す）
bool applyProgressiveMorphologicalFilterWithGround(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &obstacle_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud,
    rclcpp::Logger logger,
    int max_window_size,
    double slope,
    double initial_distance,
    double max_distance,
    double cell_size);

// 光線と平面の交点計算
bool rayPlaneIntersection(
    const pcl::PointXYZ &ray_start,
    const pcl::PointXYZ &ray_end,
    const GroundPlane &plane,
    pcl::PointXYZ &intersection);

// 穴検知（統合版：シンプルな判定ロジック）
pcl::PointCloud<pcl::PointXYZ>::Ptr detectHoles(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointXYZ &lidar_origin,
    const GroundPlane &ground_plane,
    double ground_tolerance,
    rclcpp::Logger logger,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &raw_hole_points);

// ===============================================
// 動的地面平面推定関数群
// ===============================================

// Rolling window範囲フィルタ
pcl::PointCloud<pcl::PointXYZ>::Ptr filterRollingWindow(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double window_x,  // 前方距離
    double window_y,  // 横幅（±window_y/2）
    rclcpp::Logger logger);

// RANSAC地面平面推定
bool estimateGroundPlaneRANSAC(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud,
    GroundPlane &plane,
    double distance_threshold,
    int max_iterations,
    rclcpp::Logger logger);

// ===============================================
// メモリ最適化フィルタリング関数群
// ===============================================

// 統合フィルタリングパイプライン
void applyFilteringPipeline(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double voxel_leaf_size,
    double obstacle_x_min, double obstacle_x_max,
    double obstacle_y_min, double obstacle_y_max,
    double obstacle_z_min, double obstacle_z_max,
    const std::vector<double> &robot_box_position,
    const std::vector<double> &robot_box_size,
    rclcpp::Logger logger);

// ダウンサンプリング
void downsamplePointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double voxel_leaf_size,
    rclcpp::Logger logger);

// パススルーフィルタ
void applyPassThroughFilter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double x_min, double x_max,
    double y_min, double y_max,
    double z_min, double z_max,
    rclcpp::Logger logger);

// ロボット体除去
void removeRobotBody(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<double> &box_position,
    const std::vector<double> &box_size,
    rclcpp::Logger logger);

#endif // PCL_PROCESSING_FUNCTIONS_H
