#include "obstacle_cloud_to_scan/pcl_functions.hpp"
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

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
    double x_min, double x_max,
    double y_min, double y_max,
    double z_min, double z_max,
    rclcpp::Logger logger)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // X方向フィルタ
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_min, x_max);
    pass_x.filter(*temp_cloud);
    
    // Y方向フィルタ
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(temp_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_min, y_max);
    pass_y.filter(*temp_cloud2);
    
    // Z方向フィルタ
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(temp_cloud2);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min, z_max);
    pass_z.filter(*filtered_cloud);
    
    RCLCPP_DEBUG(logger, "Passthrough filter applied (X: %.1f~%.1f, Y: %.1f~%.1f, Z: %.1f~%.1f)", 
                x_min, x_max, y_min, y_max, z_min, z_max);

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
    RCLCPP_DEBUG(logger, "Normal estimation completed");

    return normals;
}

// 並列法線推定実装
pcl::PointCloud<pcl::Normal>::Ptr estimateNormalsParallel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double normal_radius,
    int num_threads,
    rclcpp::Logger logger)
{
    // エラーハンドリング: CPUコア数チェック
    int effective_threads = num_threads;
    int max_threads = std::thread::hardware_concurrency();

    if (max_threads == 0) {
        RCLCPP_WARN(logger, "Cannot detect CPU core count, falling back to single thread mode");
        effective_threads = 1;
    } else if (num_threads <= 0) {
        RCLCPP_WARN(logger, "Invalid thread count %d, using single thread", num_threads);
        effective_threads = 1;
    } else if (num_threads > max_threads) {
        RCLCPP_WARN(logger, "Requested %d threads exceeds CPU cores (%d), limiting to %d threads", 
                    num_threads, max_threads, max_threads);
        effective_threads = max_threads;
    }

    if (effective_threads == 1) {
        // シングルスレッドの場合は通常の関数を使用
        return estimateNormals(cloud, normal_radius, logger);
    }
    
    RCLCPP_DEBUG(logger, "Starting parallel normal estimation with %d threads for %zu points", 
                effective_threads, cloud->size());
    
    // 結果格納用の法線点群を初期化
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normals->resize(cloud->size());
    normals->width = cloud->width;
    normals->height = cloud->height;
    normals->is_dense = cloud->is_dense;
    
    // 点群を分割して各スレッドに割り当て
    size_t points_per_thread = cloud->size() / effective_threads;
    size_t remainder = cloud->size() % effective_threads;
    
    std::vector<std::future<void>> futures;
    futures.reserve(effective_threads);
    
    for (int t = 0; t < effective_threads; ++t) {
        size_t start_idx = t * points_per_thread;
        size_t end_idx = (t == effective_threads - 1) ? start_idx + points_per_thread + remainder 
                                                      : start_idx + points_per_thread;
        
        // 各スレッドで部分点群の法線推定を実行
        auto future = std::async(std::launch::async, [=, &normals]() {
            // 部分点群作成
            pcl::PointCloud<pcl::PointXYZ>::Ptr sub_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            sub_cloud->points.assign(cloud->points.begin() + start_idx, 
                                   cloud->points.begin() + end_idx);
            sub_cloud->width = sub_cloud->points.size();
            sub_cloud->height = 1;
            sub_cloud->is_dense = true;
            
            // 部分点群の法線推定
            pcl::PointCloud<pcl::Normal>::Ptr sub_normals(new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
            normal_estimation.setInputCloud(sub_cloud);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            normal_estimation.setSearchMethod(tree);
            normal_estimation.setRadiusSearch(normal_radius);
            normal_estimation.compute(*sub_normals);
            
            // 結果をメイン配列にコピー
            for (size_t i = 0; i < sub_normals->size(); ++i) {
                normals->points[start_idx + i] = sub_normals->points[i];
            }
        });
        
        futures.push_back(std::move(future));
    }
    
    // 全スレッドの完了を待機
    for (auto& future : futures) {
        future.wait();
    }
    
    RCLCPP_DEBUG(logger, "Parallel normal estimation completed");
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

    // Validate input sizes match
    if (cloud->points.size() != normals->points.size()) {
        RCLCPP_ERROR(logger, "Cloud and normals size mismatch: %zu vs %zu",
                    cloud->points.size(), normals->points.size());
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

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

// 並列障害物フィルタリング実装
pcl::PointCloud<pcl::PointXYZ>::Ptr filterObstaclesParallel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    double max_slope_angle,
    int num_threads,
    rclcpp::Logger logger)
{
    // Validate input sizes match
    if (cloud->points.size() != normals->points.size()) {
        RCLCPP_ERROR(logger, "Cloud and normals size mismatch: %zu vs %zu",
                    cloud->points.size(), normals->points.size());
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    // エラーハンドリング: CPUコア数チェック
    int effective_threads = num_threads;
    int max_threads = std::thread::hardware_concurrency();

    if (max_threads == 0) {
        RCLCPP_WARN(logger, "Cannot detect CPU core count, falling back to single thread mode");
        effective_threads = 1;
    } else if (num_threads <= 0) {
        RCLCPP_WARN(logger, "Invalid thread count %d, using single thread", num_threads);
        effective_threads = 1;
    } else if (num_threads > max_threads) {
        RCLCPP_WARN(logger, "Requested %d threads exceeds CPU cores (%d), limiting to %d threads", 
                    num_threads, max_threads, max_threads);
        effective_threads = max_threads;
    }

    if (effective_threads == 1) {
        // シングルスレッドの場合は通常の関数を使用
        return filterObstacles(cloud, normals, max_slope_angle, logger);
    }
    
    RCLCPP_DEBUG(logger, "Starting parallel obstacle filtering with %d threads for %zu points", 
                effective_threads, cloud->size());
    
    // スレッドセーフな結果収集用
    std::vector<std::vector<size_t>> thread_results(effective_threads);
    
    // 点群を分割して各スレッドに割り当て
    size_t points_per_thread = cloud->size() / effective_threads;
    size_t remainder = cloud->size() % effective_threads;
    
    std::vector<std::future<void>> futures;
    futures.reserve(effective_threads);
    
    for (int t = 0; t < effective_threads; ++t) {
        size_t start_idx = t * points_per_thread;
        size_t end_idx = (t == effective_threads - 1) ? start_idx + points_per_thread + remainder 
                                                      : start_idx + points_per_thread;
        
        // 各スレッドで障害物フィルタリングを実行
        auto future = std::async(std::launch::async, [=, &thread_results]() {
            double angle = (90 - max_slope_angle) * M_PI / 180;
            double threshold_normal_z = std::sin(angle);
            
            // このスレッドの処理範囲で障害物点のインデックスを収集
            for (size_t i = start_idx; i < end_idx; ++i) {
                const auto &normal = normals->points[i];
                if (normal.normal_z <= threshold_normal_z && normal.normal_z >= -threshold_normal_z) {
                    thread_results[t].push_back(i);
                }
            }
        });
        
        futures.push_back(std::move(future));
    }
    
    // 全スレッドの完了を待機
    for (auto& future : futures) {
        future.wait();
    }
    
    // 結果を統合
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    size_t total_obstacles = 0;
    for (const auto& result : thread_results) {
        total_obstacles += result.size();
    }
    
    filtered_cloud->reserve(total_obstacles);
    for (const auto& result : thread_results) {
        for (size_t idx : result) {
            filtered_cloud->points.push_back(cloud->points[idx]);
        }
    }
    
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    
    RCLCPP_DEBUG(logger, "Parallel obstacle filtering completed");
    return filtered_cloud;
}

bool filterObstaclesParallelWithGround(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &obstacle_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud,
    double max_slope_angle,
    int num_threads,
    rclcpp::Logger logger)
{
    // 点群と法線のサイズチェック
    if (cloud->points.size() != normals->points.size()) {
        RCLCPP_ERROR(logger, "点群と法線のサイズ不一致: %zu vs %zu",
                    cloud->points.size(), normals->points.size());
        return false;
    }

    // CPUコア数チェック
    int effective_threads = num_threads;
    int max_threads = std::thread::hardware_concurrency();

    if (max_threads == 0) {
        RCLCPP_WARN(logger, "CPUコア数を検出できません、シングルスレッドにフォールバック");
        effective_threads = 1;
    } else if (num_threads <= 0) {
        RCLCPP_WARN(logger, "無効なスレッド数%d、シングルスレッドを使用", num_threads);
        effective_threads = 1;
    } else if (num_threads > max_threads) {
        RCLCPP_WARN(logger, "要求されたスレッド数%dがCPUコア数(%d)を超えています、%dスレッドに制限",
                    num_threads, max_threads, max_threads);
        effective_threads = max_threads;
    }

    if (effective_threads == 1) {
        // シングルスレッドの場合は通常の処理
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

        RCLCPP_DEBUG(logger, "法線ベースフィルタ完了（シングルスレッド）: 障害物%zu点, 地面%zu点",
                    obstacle_cloud->size(), ground_cloud->size());
        return true;
    }

    RCLCPP_DEBUG(logger, "%dスレッドで並列障害物/地面フィルタ開始（%zu点）",
                effective_threads, cloud->size());

    // スレッドセーフな結果収集用
    std::vector<std::vector<size_t>> obstacle_indices(effective_threads);
    std::vector<std::vector<size_t>> ground_indices(effective_threads);

    // 点群を分割して各スレッドに割り当て
    size_t points_per_thread = cloud->size() / effective_threads;
    size_t remainder = cloud->size() % effective_threads;

    std::vector<std::future<void>> futures;
    futures.reserve(effective_threads);

    for (int t = 0; t < effective_threads; ++t) {
        size_t start_idx = t * points_per_thread;
        size_t end_idx = (t == effective_threads - 1) ? start_idx + points_per_thread + remainder
                                                      : start_idx + points_per_thread;

        // 各スレッドで障害物と地面を分類
        auto future = std::async(std::launch::async, [=, &obstacle_indices, &ground_indices]() {
            double angle = (90 - max_slope_angle) * M_PI / 180;
            double threshold_normal_z = std::sin(angle);

            // このスレッドの処理範囲で障害物と地面のインデックスを収集
            for (size_t i = start_idx; i < end_idx; ++i) {
                const auto &normal = normals->points[i];
                if (normal.normal_z <= threshold_normal_z && normal.normal_z >= -threshold_normal_z) {
                    obstacle_indices[t].push_back(i);
                } else {
                    ground_indices[t].push_back(i);
                }
            }
        });

        futures.push_back(std::move(future));
    }

    // 全スレッドの完了を待機
    for (auto& future : futures) {
        future.wait();
    }

    // 障害物点を統合
    size_t total_obstacles = 0;
    for (const auto& result : obstacle_indices) {
        total_obstacles += result.size();
    }
    obstacle_cloud->reserve(total_obstacles);
    for (const auto& result : obstacle_indices) {
        for (size_t idx : result) {
            obstacle_cloud->points.push_back(cloud->points[idx]);
        }
    }

    // 地面点を統合
    size_t total_ground = 0;
    for (const auto& result : ground_indices) {
        total_ground += result.size();
    }
    ground_cloud->reserve(total_ground);
    for (const auto& result : ground_indices) {
        for (size_t idx : result) {
            ground_cloud->points.push_back(cloud->points[idx]);
        }
    }

    obstacle_cloud->width = obstacle_cloud->points.size();
    obstacle_cloud->height = 1;
    obstacle_cloud->is_dense = true;

    ground_cloud->width = ground_cloud->points.size();
    ground_cloud->height = 1;
    ground_cloud->is_dense = true;

    RCLCPP_DEBUG(logger, "並列障害物/地面フィルタ完了: 障害物%zu点, 地面%zu点",
                obstacle_cloud->size(), ground_cloud->size());
    return true;
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

// ===============================================
// Phase 1: Memory-optimized in-place functions
// ===============================================

void applyInPlaceFilteringPipeline(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double voxel_leaf_size,
    double obstacle_x_min, double obstacle_x_max,
    double obstacle_y_min, double obstacle_y_max,
    double obstacle_z_min, double obstacle_z_max,
    const std::vector<double> &robot_box_position,
    const std::vector<double> &robot_box_size,
    rclcpp::Logger logger)
{
    RCLCPP_DEBUG(logger, "Starting in-place filtering pipeline with %zu points", cloud->size());
    
    // Step 1: ダウンサンプリング (最も効果的な削減)
    downsamplePointCloudInPlace(cloud, voxel_leaf_size, logger);
    
    // Step 2: パススルーフィルタ (X,Y,Z方向障害物検知範囲制限)
    applyPassThroughFilterInPlace(cloud, obstacle_x_min, obstacle_x_max, 
                                  obstacle_y_min, obstacle_y_max,
                                  obstacle_z_min, obstacle_z_max, logger);
    
    // Step 3: ロボット体除去
    removeRobotBodyInPlace(cloud, robot_box_position, robot_box_size, logger);
    
    RCLCPP_DEBUG(logger, "In-place filtering pipeline completed with %zu points", cloud->size());
}

void downsamplePointCloudInPlace(
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
    
    RCLCPP_DEBUG(logger, "In-place downsampling: %zu -> %zu points", original_size, cloud->size());
}

void applyPassThroughFilterInPlace(
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
    
    RCLCPP_DEBUG(logger, "In-place passthrough: %zu -> %zu points (X: %.1f~%.1f, Y: %.1f~%.1f, Z: %.1f~%.1f)", 
                original_size, cloud->size(), x_min, x_max, y_min, y_max, z_min, z_max);
}

void removeRobotBodyInPlace(
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
    
    RCLCPP_DEBUG(logger, "In-place robot body removal: %zu -> %zu points", original_size, cloud->size());
}

// ===============================================
// Two-tier distance-based hierarchical filtering
// ===============================================

double calculateDistance(const pcl::PointXYZ &point, const pcl::PointXYZ &origin)
{
    double dx = point.x - origin.x;
    double dy = point.y - origin.y;
    double dz = point.z - origin.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void applyTwoTierDownsampling(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double base_voxel_size,
    double collision_distance_threshold,
    double far_zone_voxel_multiplier,
    rclcpp::Logger logger)
{
    RCLCPP_DEBUG(logger, "Starting two-tier downsampling with %zu points", cloud->size());
    
    // 近距離・遠距離点群を分離
    pcl::PointCloud<pcl::PointXYZ>::Ptr near_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr far_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::PointXYZ origin(0.0, 0.0, 0.0);
    
    for (const auto &point : cloud->points) {
        double distance = calculateDistance(point, origin);
        if (distance <= collision_distance_threshold) {
            near_cloud->points.push_back(point);
        } else {
            far_cloud->points.push_back(point);
        }
    }
    
    RCLCPP_DEBUG(logger, "Distance separation: Near(%zu) / Far(%zu) points", 
                near_cloud->size(), far_cloud->size());
    
    // 近距離: 高精度ダウンサンプリング
    if (!near_cloud->empty()) {
        pcl::VoxelGrid<pcl::PointXYZ> near_filter;
        near_filter.setInputCloud(near_cloud);
        near_filter.setLeafSize(base_voxel_size, base_voxel_size, base_voxel_size);
        near_filter.filter(*near_cloud);
    }
    
    // 遠距離: 低精度ダウンサンプリング
    if (!far_cloud->empty()) {
        double far_voxel_size = base_voxel_size * far_zone_voxel_multiplier;
        pcl::VoxelGrid<pcl::PointXYZ> far_filter;
        far_filter.setInputCloud(far_cloud);
        far_filter.setLeafSize(far_voxel_size, far_voxel_size, far_voxel_size);
        far_filter.filter(*far_cloud);
    }
    
    // 結果を統合
    cloud->clear();
    *cloud += *near_cloud;
    *cloud += *far_cloud;
    
    RCLCPP_DEBUG(logger, "Two-tier downsampling completed: Near(%zu) + Far(%zu) = Total(%zu) points", 
                near_cloud->size(), far_cloud->size(), cloud->size());
}

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
    rclcpp::Logger logger)
{
    RCLCPP_DEBUG(logger, "Starting hierarchical filtering pipeline with %zu points", cloud->size());
    
    // Step 1: 2段階階層ダウンサンプリング（最も効果的）
    applyTwoTierDownsampling(cloud, base_voxel_size, collision_distance_threshold, 
                            far_zone_voxel_multiplier, logger);
    
    // Step 2: パススルーフィルタ (X,Y,Z方向障害物検知範囲制限)
    applyPassThroughFilterInPlace(cloud, obstacle_x_min, obstacle_x_max, 
                                  obstacle_y_min, obstacle_y_max,
                                  obstacle_z_min, obstacle_z_max, logger);
    
    // Step 3: ロボット体除去
    removeRobotBodyInPlace(cloud, robot_box_position, robot_box_size, logger);
    
    RCLCPP_DEBUG(logger, "Hierarchical filtering pipeline completed with %zu points", cloud->size());
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

pcl::PointCloud<pcl::PointXYZ>::Ptr detectHolesBasicWithHeightCheck(
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
