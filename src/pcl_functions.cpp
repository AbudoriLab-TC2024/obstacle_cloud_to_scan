#include "obstacle_cloud_to_scan/pcl_functions.hpp"
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double voxel_leaf_size,
    rclcpp::Logger logger)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

    voxel_filter.filter(*downsampled_cloud);

    return downsampled_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr applyPassThroughFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<double> &robot_box_size,
    rclcpp::Logger logger)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.0, robot_box_size[2] + 0.3);
    pass.filter(*filtered_cloud);
    RCLCPP_DEBUG(logger, "Passthrough filter applied");

    return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr applyProgressiveMorphologicalFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    rclcpp::Logger logger,
    int max_window_size,
    double slope,
    double initial_distance,
    double max_distance,
    double cell_size)
{
    RCLCPP_DEBUG(logger, "Starting PMF ground filtering (in pcl_functions)");
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (cloud->empty()) {
        RCLCPP_WARN(logger, "Input cloud to PMF is empty.");
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
    extract.setNegative(true); // true = extract points NOT in indices
    extract.filter(*filtered_cloud);

    RCLCPP_DEBUG(logger, "PMF ground filtering completed (in pcl_functions). Number of obstacle points: %zu", filtered_cloud->size());
    return filtered_cloud;
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
    RCLCPP_DEBUG(logger, "Normal estimation completed");

    return normals;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filterObstacles(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    double max_slope_angle,
    rclcpp::Logger logger)
{
    RCLCPP_DEBUG(logger, "Normal cloud size: %ld", cloud->points.size());
    RCLCPP_DEBUG(logger, "Max angle slope: %f", max_slope_angle);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        const auto &normal = normals->points[i];
        //double angle = std::acos(normal.normal_z) * 180.0 / M_PI;
        double angle = (90 - max_slope_angle) * M_PI / 180;
        double threshold_normal_z = std::sin(angle);
        //RCLCPP_DEBUG(logger, "angle: %f", angle);
        //RCLCPP_DEBUG(logger, "threshold_normal_z: %f", threshold_normal_z);
        if (normal.normal_z <= threshold_normal_z && normal.normal_z >= -threshold_normal_z)
        {
            filtered_cloud->points.push_back(cloud->points[i]);
        }
    }
    RCLCPP_DEBUG(logger, "Obstacle filtering completed");

    return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeRobotBody(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<double> &box_position,
    const std::vector<double> &box_size,
    rclcpp::Logger logger)
{
    pcl::CropBox<pcl::PointXYZ> crop_box_filter;
    crop_box_filter.setInputCloud(cloud);

    // ボックスの範囲を設定
    Eigen::Vector4f min_point(-(box_size[0]/2)+box_position[0],
                              -(box_size[1]/2)+box_position[1], 
                              0.0+box_position[2], 1.0);
    Eigen::Vector4f max_point(box_size[0]/2+box_position[0], 
                              box_size[1]/2+box_position[1],
                              box_size[2]+box_position[2], 1.0);
    RCLCPP_DEBUG(logger, "min_point: %f, %f, %f", min_point[0], min_point[1], min_point[2]);
    RCLCPP_DEBUG(logger, "max_point: %f, %f, %f", max_point[0], max_point[1], max_point[2]);

    crop_box_filter.setMin(min_point);
    crop_box_filter.setMax(max_point);
    crop_box_filter.setNegative(true);  // ボックス内の点群を除去する

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop_box_filter.filter(*filtered_cloud);

    RCLCPP_DEBUG(logger, "Removed points within the robot body bounding box.");

    return filtered_cloud;
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
    
    RCLCPP_DEBUG(logger, "Hole detection range filter: %zu -> %zu points", 
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

pcl::PointCloud<pcl::PointXYZ>::Ptr detectHolesBasic(
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
    
    RCLCPP_DEBUG(logger, "Hole detection: %zu -> %zu hole points", 
                cloud->size(), hole_cloud->size());
    return hole_cloud;
}
