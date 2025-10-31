#ifndef PCL_PROCESSING_FUNCTIONS_H
#define PCL_PROCESSING_FUNCTIONS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <rclcpp/rclcpp.hpp>
#include <future>
#include <thread>

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
    double x_min, double x_max,
    double y_min, double y_max,
    double z_min, double z_max,
    rclcpp::Logger logger);

// 法線推定
pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double normal_radius,
    rclcpp::Logger logger);

// 並列法線推定
pcl::PointCloud<pcl::Normal>::Ptr estimateNormalsParallel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double normal_radius,
    int num_threads,
    rclcpp::Logger logger);

// 法線から障害物を割り出し
pcl::PointCloud<pcl::PointXYZ>::Ptr filterObstacles(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    double max_slope_angle,
    rclcpp::Logger logger);

// 並列障害物フィルタリング
pcl::PointCloud<pcl::PointXYZ>::Ptr filterObstaclesParallel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    double max_slope_angle,
    int num_threads,
    rclcpp::Logger logger);

// 並列障害物フィルタリング（地面点と障害物点の両方を返す）
bool filterObstaclesParallelWithGround(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &obstacle_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud,
    double max_slope_angle,
    int num_threads,
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

// 高さチェック付き基本穴検知（改善版）
pcl::PointCloud<pcl::PointXYZ>::Ptr detectHolesBasicWithHeightCheck(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointXYZ &lidar_origin,
    const GroundPlane &ground_plane,
    double ground_tolerance,
    double height_buffer,  // 地面より高い点のスキップバッファ
    rclcpp::Logger logger);

// ===============================================
// Phase 1: Memory-optimized in-place functions
// ===============================================

// インプレース版 - 統合フィルタリングパイプライン
void applyInPlaceFilteringPipeline(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double voxel_leaf_size,
    double obstacle_x_min, double obstacle_x_max,
    double obstacle_y_min, double obstacle_y_max,
    double obstacle_z_min, double obstacle_z_max,
    const std::vector<double> &robot_box_position,
    const std::vector<double> &robot_box_size,
    rclcpp::Logger logger);

// インプレース版 - ダウンサンプリング
void downsamplePointCloudInPlace(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double voxel_leaf_size,
    rclcpp::Logger logger);

// インプレース版 - パススルーフィルタ
void applyPassThroughFilterInPlace(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double x_min, double x_max,
    double y_min, double y_max,
    double z_min, double z_max,
    rclcpp::Logger logger);

// インプレース版 - ロボット体除去
void removeRobotBodyInPlace(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<double> &box_position,
    const std::vector<double> &box_size,
    rclcpp::Logger logger);

// ===============================================
// Phase 2: Two-tier distance-based filtering
// ===============================================

// 点から原点までの距離計算
double calculateDistance(const pcl::PointXYZ &point, const pcl::PointXYZ &origin = pcl::PointXYZ(0,0,0));

// 2段階階層ダウンサンプリング（近距離=高精度、遠距離=低精度）
void applyTwoTierDownsampling(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double base_voxel_size,
    double collision_distance_threshold,  // デフォルト3.0m
    double far_zone_voxel_multiplier,     // デフォルト2.0 (遠方は2倍粗く)
    rclcpp::Logger logger);

// 距離ベース階層フィルタリング統合パイプライン
void applyHierarchicalFilteringPipeline(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double base_voxel_size,
    double obstacle_x_min, double obstacle_x_max,
    double obstacle_y_min, double obstacle_y_max,
    double obstacle_z_min, double obstacle_z_max,
    const std::vector<double> &robot_box_position,
    const std::vector<double> &robot_box_size,
    double collision_distance_threshold,
    double far_zone_voxel_multiplier,
    rclcpp::Logger logger);

#endif // PCL_PROCESSING_FUNCTIONS_H
