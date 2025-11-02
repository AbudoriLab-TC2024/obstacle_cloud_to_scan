#include "obstacle_cloud_to_scan/pcl_functions.hpp"
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr applyProgressiveMorphologicalFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    rclcpp::Logger logger,
    int max_window_size,
    double slope,
    double initial_distance,
    double max_distance,
    double cell_size)
{
    RCLCPP_DEBUG(logger, "PMF地面フィルタ開始");
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (cloud->empty()) {
        RCLCPP_WARN(logger, "PMFへの入力点群が空です");
        return filtered_cloud;
    }

    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(max_window_size);
    pmf.setSlope(static_cast<float>(slope)); // PMF expects float for slope
    pmf.setInitialDistance(static_cast<float>(initial_distance)); // PMF expects float
    pmf.setMaxDistance(static_cast<float>(max_distance)); // PMF expects float
    pmf.setCellSize(static_cast<float>(cell_size)); // PMF expects float for cell_size in some PCL versions, ensure compatibility or use double if available

    pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
    try {
        pmf.extract(ground_indices->indices);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Exception during PMF extract: %s", e.what());
        // Return original cloud or empty cloud on error? For now, return empty obstacle cloud.
        return filtered_cloud;
    }

    // Extract non-ground points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground_indices);
    extract.setNegative(true); // 地面以外の点を抽出
    extract.filter(*filtered_cloud);

    RCLCPP_DEBUG(logger, "PMF地面フィルタ完了: 障害物%zu点", filtered_cloud->size());
    return filtered_cloud;
}

bool applyProgressiveMorphologicalFilterWithGround(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &obstacle_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud,
    rclcpp::Logger logger,
    int max_window_size,
    double slope,
    double initial_distance,
    double max_distance,
    double cell_size)
{
    RCLCPP_DEBUG(logger, "PMF地面フィルタ開始（地面点も取得）");

    if (cloud->empty()) {
        RCLCPP_WARN(logger, "PMFへの入力点群が空です");
        return false;
    }

    // PMFで地面点のインデックスを取得
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(max_window_size);
    pmf.setSlope(static_cast<float>(slope));
    pmf.setInitialDistance(static_cast<float>(initial_distance));
    pmf.setMaxDistance(static_cast<float>(max_distance));
    pmf.setCellSize(static_cast<float>(cell_size));

    pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
    try {
        pmf.extract(ground_indices->indices);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "PMF抽出中に例外発生: %s", e.what());
        return false;
    }

    if (ground_indices->indices.empty()) {
        RCLCPP_WARN(logger, "PMFで地面点が見つかりませんでした");
        return false;
    }

    // 障害物点を抽出（地面以外）
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground_indices);
    extract.setNegative(true);  // 地面以外を抽出
    extract.filter(*obstacle_cloud);

    // 地面点を抽出
    extract.setNegative(false);  // 地面を抽出
    extract.filter(*ground_cloud);

    RCLCPP_DEBUG(logger, "PMF地面フィルタ完了: 障害物%zu点, 地面%zu点",
                obstacle_cloud->size(), ground_cloud->size());

    return true;
}

pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double normal_radius,
    rclcpp::Logger logger)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(normal_radius);
    normal_estimation.compute(*normals);
    RCLCPP_DEBUG(logger, "法線推定完了");

    return normals;
}

// 障害物フィルタリング（地面点と障害物点の両方を返す）
bool filterObstaclesWithGround(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &obstacle_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud,
    double max_slope_angle,
    rclcpp::Logger logger)
{
    // 点群と法線のサイズチェック
    if (cloud->points.size() != normals->points.size()) {
        RCLCPP_ERROR(logger, "点群と法線のサイズ不一致: %zu vs %zu",
                    cloud->points.size(), normals->points.size());
        return false;
    }

    double angle = (90 - max_slope_angle) * M_PI / 180;
    double threshold_normal_z = std::sin(angle);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto &normal = normals->points[i];
        if (normal.normal_z <= threshold_normal_z && normal.normal_z >= -threshold_normal_z) {
            obstacle_cloud->points.push_back(cloud->points[i]);
        } else {
            ground_cloud->points.push_back(cloud->points[i]);
        }
    }

    RCLCPP_DEBUG(logger, "法線ベースフィルタ完了: 障害物%zu点, 地面%zu点",
                obstacle_cloud->size(), ground_cloud->size());
    return true;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr filterHoleDetectionRange(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double range_x,
    double range_y,
    double max_height,
    rclcpp::Logger logger)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // X方向フィルタ (0 ~ range_x)
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(0.0, range_x);
    pass_x.filter(*temp_cloud);
    
    // Y方向フィルタ (-range_y/2 ~ +range_y/2)
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(temp_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-range_y/2.0, range_y/2.0);
    pass_y.filter(*temp_cloud);
    
    // Z方向フィルタ (max_height以下)
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(temp_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-10.0, max_height); // 下限は十分低く設定
    pass_z.filter(*filtered_cloud);

    RCLCPP_DEBUG(logger, "穴検知範囲フィルタ: %zu -> %zu点",
                cloud->size(), filtered_cloud->size());
    return filtered_cloud;
}

bool rayPlaneIntersection(
    const pcl::PointXYZ &ray_start,
    const pcl::PointXYZ &ray_end,
    const GroundPlane &plane,
    pcl::PointXYZ &intersection)
{
    // 光線の方向ベクトル
    double dx = ray_end.x - ray_start.x;
    double dy = ray_end.y - ray_start.y;
    double dz = ray_end.z - ray_start.z;
    
    // 光線の方向ベクトルと平面法線の内積
    double denominator = plane.a * dx + plane.b * dy + plane.c * dz;
    
    // 平行チェック（内積が0に近い場合）
    if (std::abs(denominator) < 1e-6) {
        return false; // 光線と平面が平行
    }
    
    // 交点パラメータt を計算
    double numerator = -(plane.a * ray_start.x + plane.b * ray_start.y + 
                        plane.c * ray_start.z + plane.d);
    double t = numerator / denominator;
    
    // 交点を計算
    intersection.x = ray_start.x + t * dx;
    intersection.y = ray_start.y + t * dy;
    intersection.z = ray_start.z + t * dz;
    
    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr detectHoles(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointXYZ &lidar_origin,
    const GroundPlane &ground_plane,
    double ground_tolerance,
    rclcpp::Logger logger)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr hole_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (const auto &point : cloud->points) {
        // LiDARから点への光線と地面平面の交点を計算
        pcl::PointXYZ intersection;
        if (!rayPlaneIntersection(lidar_origin, point, ground_plane, intersection)) {
            continue; // 交点計算失敗（平行など）
        }
        
        // 距離ベース穴判定（より精密な検知）
        double lidar_to_point_distance = sqrt(
            pow(point.x - lidar_origin.x, 2) + 
            pow(point.y - lidar_origin.y, 2) + 
            pow(point.z - lidar_origin.z, 2));
        double lidar_to_intersection_distance = sqrt(
            pow(intersection.x - lidar_origin.x, 2) + 
            pow(intersection.y - lidar_origin.y, 2) + 
            pow(intersection.z - lidar_origin.z, 2));

        // 実際の点が期待される地面交点より明らかに遠い場合のみ穴と判定
        if (lidar_to_point_distance > lidar_to_intersection_distance + ground_tolerance) {
            hole_cloud->points.push_back(intersection);
        }
    }

    RCLCPP_DEBUG(logger, "穴検知: %zu点中%zu点の穴を検出",
                cloud->size(), hole_cloud->size());
    return hole_cloud;
}

// ===============================================
// メモリ最適化フィルタリング関数群
// ===============================================

void applyFilteringPipeline(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double voxel_leaf_size,
    double obstacle_x_min, double obstacle_x_max,
    double obstacle_y_min, double obstacle_y_max,
    double obstacle_z_min, double obstacle_z_max,
    const std::vector<double> &robot_box_position,
    const std::vector<double> &robot_box_size,
    rclcpp::Logger logger)
{
    RCLCPP_DEBUG(logger, "フィルタリングパイプライン開始: %zu点", cloud->size());

    // Step 1: ダウンサンプリング（最も効果的な削減）
    downsamplePointCloud(cloud, voxel_leaf_size, logger);

    // Step 2: パススルーフィルタ（X,Y,Z方向障害物検知範囲制限）
    applyPassThroughFilter(cloud, obstacle_x_min, obstacle_x_max,
                          obstacle_y_min, obstacle_y_max,
                          obstacle_z_min, obstacle_z_max, logger);

    // Step 3: ロボット体除去
    removeRobotBody(cloud, robot_box_position, robot_box_size, logger);

    RCLCPP_DEBUG(logger, "フィルタリングパイプライン完了: %zu点", cloud->size());
}

void downsamplePointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double voxel_leaf_size,
    rclcpp::Logger logger)
{
    size_t original_size = cloud->size();

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

    // 同じポインタに結果を書き戻し
    voxel_filter.filter(*cloud);

    RCLCPP_DEBUG(logger, "ダウンサンプリング: %zu -> %zu点", original_size, cloud->size());
}

void applyPassThroughFilter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double x_min, double x_max,
    double y_min, double y_max,
    double z_min, double z_max,
    rclcpp::Logger logger)
{
    size_t original_size = cloud->size();

    // X方向フィルタ
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_min, x_max);
    pass_x.filter(*cloud);

    // Y方向フィルタ
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_min, y_max);
    pass_y.filter(*cloud);

    // Z方向フィルタ
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min, z_max);
    pass_z.filter(*cloud);

    RCLCPP_DEBUG(logger, "パススルーフィルタ: %zu -> %zu点 (X: %.1f~%.1f, Y: %.1f~%.1f, Z: %.1f~%.1f)",
                original_size, cloud->size(), x_min, x_max, y_min, y_max, z_min, z_max);
}

void removeRobotBody(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<double> &box_position,
    const std::vector<double> &box_size,
    rclcpp::Logger logger)
{
    size_t original_size = cloud->size();

    pcl::CropBox<pcl::PointXYZ> crop_box_filter;
    crop_box_filter.setInputCloud(cloud);

    // ロボット体のバウンディングボックス設定
    Eigen::Vector4f min_point(-(box_size[0]/2)+box_position[0],
                              -(box_size[1]/2)+box_position[1],
                              0.0+box_position[2], 1.0);
    Eigen::Vector4f max_point(box_size[0]/2+box_position[0],
                              box_size[1]/2+box_position[1],
                              box_size[2]+box_position[2], 1.0);

    crop_box_filter.setMin(min_point);
    crop_box_filter.setMax(max_point);
    crop_box_filter.setNegative(true);  // ボックス内の点群を除去

    // 同じポインタに結果を書き戻し
    crop_box_filter.filter(*cloud);

    RCLCPP_DEBUG(logger, "ロボット体除去: %zu -> %zu点", original_size, cloud->size());
}

// ===============================================
// 動的地面平面推定関数群
// ===============================================

pcl::PointCloud<pcl::PointXYZ>::Ptr filterRollingWindow(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double window_x,
    double window_y,
    rclcpp::Logger logger)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // X方向: 0 ~ window_x (前方)
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(0.0, window_x);
    pass_x.filter(*filtered_cloud);

    // Y方向: -window_y/2 ~ +window_y/2 (左右)
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(filtered_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-window_y / 2.0, window_y / 2.0);
    pass_y.filter(*filtered_cloud);

    RCLCPP_DEBUG(logger, "Rolling windowフィルタ: %zu -> %zu点 (X: 0~%.1fm, Y: ±%.1fm)",
                cloud->size(), filtered_cloud->size(), window_x, window_y / 2.0);

    return filtered_cloud;
}

bool estimateGroundPlaneRANSAC(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud,
    GroundPlane &plane,
    double distance_threshold,
    int max_iterations,
    rclcpp::Logger logger)
{
    if (!ground_cloud || ground_cloud->empty()) {
        RCLCPP_WARN(logger, "地面点群が空のため、平面推定できません");
        return false;
    }

    if (ground_cloud->size() < 3) {
        RCLCPP_WARN(logger, "地面点群が3点未満(%zu点)のため、平面推定できません",
                   ground_cloud->size());
        return false;
    }

    // RANSAC平面推定
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);
    seg.setMaxIterations(max_iterations);

    seg.setInputCloud(ground_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        RCLCPP_WARN(logger, "RANSACで平面モデルが見つかりませんでした");
        return false;
    }

    // 平面パラメータを設定 (ax + by + cz + d = 0)
    plane.a = coefficients->values[0];
    plane.b = coefficients->values[1];
    plane.c = coefficients->values[2];
    plane.d = coefficients->values[3];

    // 法線が下向き(z < 0)の場合は反転して上向きにする
    if (plane.c < 0) {
        plane.a = -plane.a;
        plane.b = -plane.b;
        plane.c = -plane.c;
        plane.d = -plane.d;
    }

    RCLCPP_DEBUG(logger, "地面平面推定完了: %.3fx + %.3fy + %.3fz + %.3f = 0 (inliers: %zu/%zu点, %.1f%%)",
                plane.a, plane.b, plane.c, plane.d,
                inliers->indices.size(), ground_cloud->size(),
                100.0 * inliers->indices.size() / ground_cloud->size());

    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr detectHolesWithHeightCheck(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointXYZ &lidar_origin,
    const GroundPlane &ground_plane,
    double ground_tolerance,
    double height_buffer,
    rclcpp::Logger logger,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &raw_hole_points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr hole_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // デバッグ用raw点群が指定されている場合は初期化
    if (raw_hole_points) {
        raw_hole_points->clear();
    }

    for (const auto &point : cloud->points) {
        // LiDARから点への光線と地面平面の交点を計算
        pcl::PointXYZ intersection;
        if (!rayPlaneIntersection(lidar_origin, point, ground_plane, intersection)) {
            continue; // 交点計算失敗（平行など）
        }

        // 【高さチェック】点が地面より明らかに高い場合はスキップ（壁や障害物の誤検知防止）
        // height_buffer: 点群のブレを考慮したバッファ（デフォルト0.1m）
        // 注意: 全点探索に変更する場合は、このif文をコメントアウトしてください
        if (point.z > intersection.z + height_buffer) {
            continue;  // 地面より高い点は穴ではなく障害物
        }

        // 距離ベース穴判定（より精密な検知）
        double lidar_to_point_distance = sqrt(
            pow(point.x - lidar_origin.x, 2) +
            pow(point.y - lidar_origin.y, 2) +
            pow(point.z - lidar_origin.z, 2));
        double lidar_to_intersection_distance = sqrt(
            pow(intersection.x - lidar_origin.x, 2) +
            pow(intersection.y - lidar_origin.y, 2) +
            pow(intersection.z - lidar_origin.z, 2));

        // 実際の点が期待される地面交点より明らかに遠い場合のみ穴と判定
        if (lidar_to_point_distance > lidar_to_intersection_distance + ground_tolerance) {
            // 地面平面との交点を穴点として登録
            hole_cloud->points.push_back(intersection);

            // デバッグ用：元の測定点（地面より低い点）も保存
            if (raw_hole_points) {
                raw_hole_points->points.push_back(point);
            }
        }
    }

    RCLCPP_DEBUG(logger, "高さチェック付き穴検知: %zu点中%zu点を穴として検知",
                cloud->size(), hole_cloud->size());
    return hole_cloud;
}
