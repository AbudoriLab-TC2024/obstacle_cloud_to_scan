#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "obstacle_cloud_to_scan/pcl_functions.hpp"

#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <chrono>
#include <numeric>
#include <functional>
#include "obstacle_cloud_to_scan/obstacle_cloud_to_scan.hpp"


    ObstacleCloudToScanNode::ObstacleCloudToScanNode() : Node("obstacle_cloud_to_scan")
    {
        RCLCPP_DEBUG(this->get_logger(), "Initializing ObstacleCloudToScanNode");

        last_log_time_ = this->get_clock()->now();
        logging_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ObstacleCloudToScanNode::logPerformance, this));

        ground_plane_initialized_ = false;

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        declare_parameters();
        get_parameters();

        // LiDAR原点をパラメータから初期化（動的モードでも必須）
        lidar_origin_.x = lidar_origin_x_;
        lidar_origin_.y = lidar_origin_y_;
        lidar_origin_.z = lidar_origin_z_;
        RCLCPP_INFO(this->get_logger(),
                   "LiDAR origin initialized from parameters: (%.3f, %.3f, %.3f)",
                   lidar_origin_.x, lidar_origin_.y, lidar_origin_.z);

        auto sensor_qos = rclcpp::SensorDataQoS();
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, sensor_qos, std::bind(&ObstacleCloudToScanNode::pointCloudCallback, this, std::placeholders::_1));
        RCLCPP_DEBUG(this->get_logger(), "Subscribed to topic: %s", input_topic_.c_str());

        filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, sensor_qos);
        RCLCPP_DEBUG(this->get_logger(), "Publisher created for topic: %s", output_topic_.c_str());

        hole_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(hole_output_topic_, sensor_qos);
        RCLCPP_DEBUG(this->get_logger(), "Hole cloud publisher created for topic: %s", hole_output_topic_.c_str());

        hole_raw_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(hole_raw_output_topic_, sensor_qos);
        RCLCPP_DEBUG(this->get_logger(), "Hole raw cloud publisher created for topic: %s", hole_raw_output_topic_.c_str());

        ground_plane_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(ground_plane_visualization_topic_, 10);
        RCLCPP_DEBUG(this->get_logger(), "Ground plane marker publisher created for topic: %s", ground_plane_visualization_topic_.c_str());
    }

    void ObstacleCloudToScanNode::declare_parameters()
    {
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<std::string>("input_topic", "/livox/lidar");
        this->declare_parameter<std::string>("output_topic", "/obstacle_cloud/cloud");
        this->declare_parameter<std::string>("ground_remove_algorithm", "NORMAL");
        this->declare_parameter<double>("voxel_leaf_size", 0.1);
        this->declare_parameter<std::vector<double>>("robot_box_size", {0.6, 0.6, 1.0});
        this->declare_parameter<std::vector<double>>("robot_box_position", {0.0, 0.0, 0.0});
        
        // Obstacle detection range parameters (X, Y, Z PassThrough filter)
        this->declare_parameter<double>("obstacle_detection_range_x_min", -3.0);
        this->declare_parameter<double>("obstacle_detection_range_x_max", 3.0);
        this->declare_parameter<double>("obstacle_detection_range_y_min", -3.0);
        this->declare_parameter<double>("obstacle_detection_range_y_max", 3.0);
        this->declare_parameter<double>("obstacle_detection_range_z_min", -1.0);
        this->declare_parameter<double>("obstacle_detection_range_z_max", 1.3);
        this->declare_parameter<double>("normal_max_slope_angle", 5.0);
        this->declare_parameter<double>("normal_radius", 0.6);
        this->declare_parameter<int>("pmf_max_window_size", 33);
        this->declare_parameter<double>("pmf_slope", 1.0);
        this->declare_parameter<double>("pmf_initial_distance", 0.15);
        this->declare_parameter<double>("pmf_max_distance", 3.0);
        this->declare_parameter<double>("pmf_cell_size", 0.5);

        // Hole detection parameters
        this->declare_parameter<bool>("hole_detection_enabled", false);
        this->declare_parameter<std::string>("hole_detection_algorithm", "BASIC");
        this->declare_parameter<std::string>("hole_output_topic", "/hole_cloud/cloud");
        this->declare_parameter<std::string>("hole_raw_output_topic", "/hole_cloud/raw");  // デバッグ用
        this->declare_parameter<std::string>("lidar_frame", "livox_frame");
        this->declare_parameter<double>("hole_detection_range_x", 3.0);
        this->declare_parameter<double>("hole_detection_range_y", 5.0);
        this->declare_parameter<double>("hole_detection_max_height", 0.3);
        this->declare_parameter<double>("hole_ground_tolerance", 0.05);

        // Dynamic ground plane estimation parameters
        this->declare_parameter<bool>("use_dynamic_ground_plane", true);
        this->declare_parameter<double>("ground_plane_rolling_window_x", 4.0);
        this->declare_parameter<double>("ground_plane_rolling_window_y", 6.0);
        this->declare_parameter<double>("ground_plane_ransac_distance_threshold", 0.05);
        this->declare_parameter<int>("ground_plane_ransac_max_iterations", 100);
        this->declare_parameter<double>("hole_detection_height_buffer", 0.1);

        // Ground plane visualization parameters
        this->declare_parameter<bool>("visualize_ground_plane", false);
        this->declare_parameter<std::string>("ground_plane_visualization_topic", "/ground_plane_marker");
        this->declare_parameter<double>("ground_plane_visualization_size", 5.0);

        // LiDAR origin parameters (fallback when TF is unavailable)
        this->declare_parameter<double>("lidar_origin_x", 0.0);
        this->declare_parameter<double>("lidar_origin_y", 0.0);
        this->declare_parameter<double>("lidar_origin_z", 0.0);

    }

    void ObstacleCloudToScanNode::get_parameters()
    {
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("ground_remove_algorithm", ground_remove_algorithm_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->get_parameter("robot_box_size", robot_box_size_);
        this->get_parameter("robot_box_position", robot_box_position_);
        
        // Obstacle detection range parameters
        this->get_parameter("obstacle_detection_range_x_min", obstacle_detection_range_x_min_);
        this->get_parameter("obstacle_detection_range_x_max", obstacle_detection_range_x_max_);
        this->get_parameter("obstacle_detection_range_y_min", obstacle_detection_range_y_min_);
        this->get_parameter("obstacle_detection_range_y_max", obstacle_detection_range_y_max_);
        this->get_parameter("obstacle_detection_range_z_min", obstacle_detection_range_z_min_);
        this->get_parameter("obstacle_detection_range_z_max", obstacle_detection_range_z_max_);
        this->get_parameter("normal_max_slope_angle", normal_max_slope_angle_);
        this->get_parameter("normal_radius", normal_radius_);
        this->get_parameter("pmf_max_window_size", pmf_max_window_size_);
        this->get_parameter("pmf_slope", pmf_slope_);
        this->get_parameter("pmf_initial_distance", pmf_initial_distance_);
        this->get_parameter("pmf_max_distance", pmf_max_distance_);
        this->get_parameter("pmf_cell_size", pmf_cell_size_);

        // Hole detection parameters
        this->get_parameter("hole_detection_enabled", hole_detection_enabled_);
        this->get_parameter("hole_detection_algorithm", hole_detection_algorithm_);
        this->get_parameter("hole_output_topic", hole_output_topic_);
        this->get_parameter("hole_raw_output_topic", hole_raw_output_topic_);
        this->get_parameter("lidar_frame", lidar_frame_);
        this->get_parameter("hole_detection_range_x", hole_detection_range_x_);
        this->get_parameter("hole_detection_range_y", hole_detection_range_y_);
        this->get_parameter("hole_detection_max_height", hole_detection_max_height_);
        this->get_parameter("hole_ground_tolerance", hole_ground_tolerance_);

        // Dynamic ground plane estimation parameters
        this->get_parameter("use_dynamic_ground_plane", use_dynamic_ground_plane_);
        this->get_parameter("ground_plane_rolling_window_x", ground_plane_rolling_window_x_);
        this->get_parameter("ground_plane_rolling_window_y", ground_plane_rolling_window_y_);
        this->get_parameter("ground_plane_ransac_distance_threshold", ground_plane_ransac_distance_threshold_);
        this->get_parameter("ground_plane_ransac_max_iterations", ground_plane_ransac_max_iterations_);
        this->get_parameter("hole_detection_height_buffer", hole_detection_height_buffer_);

        // Parameter validation
        if (robot_box_size_.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "robot_box_size must have exactly 3 elements (x, y, z). Got %zu elements.", robot_box_size_.size());
            throw std::runtime_error("Invalid robot_box_size parameter");
        }

        if (robot_box_position_.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "robot_box_position must have exactly 3 elements (x, y, z). Got %zu elements.", robot_box_position_.size());
            throw std::runtime_error("Invalid robot_box_position parameter");
        }

        if (voxel_leaf_size_ <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "voxel_leaf_size must be positive. Got %.3f", voxel_leaf_size_);
            throw std::runtime_error("Invalid voxel_leaf_size parameter");
        }

        if (normal_radius_ <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "normal_radius must be positive. Got %.3f", normal_radius_);
            throw std::runtime_error("Invalid normal_radius parameter");
        }

        if (ground_remove_algorithm_ != "NORMAL" && ground_remove_algorithm_ != "PMF") {
            RCLCPP_WARN(this->get_logger(),
                "ground_remove_algorithm must be 'NORMAL' or 'PMF'; using 'NORMAL' (got: '%s').",
                ground_remove_algorithm_.c_str());
            ground_remove_algorithm_ = "NORMAL";
        }

        if (hole_detection_algorithm_ != "BASIC" && hole_detection_algorithm_ != "GRID") {
            RCLCPP_WARN(this->get_logger(),
                "hole_detection_algorithm must be 'BASIC' or 'GRID'; using 'BASIC' (got: '%s').",
                hole_detection_algorithm_.c_str());
            hole_detection_algorithm_ = "BASIC";
        }

        RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
        RCLCPP_INFO(this->get_logger(), "target_frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "ground_remove_algorithm: %s", ground_remove_algorithm_.c_str());
        RCLCPP_INFO(this->get_logger(), "voxel_leaf_size: %f", voxel_leaf_size_);
        RCLCPP_INFO(this->get_logger(), "normal_max_slope_angle: %f", normal_max_slope_angle_);
        RCLCPP_INFO(this->get_logger(), "normal_radius: %f", normal_radius_);
        RCLCPP_INFO(this->get_logger(), "pmf_max_window_size: %d", pmf_max_window_size_);
        RCLCPP_INFO(this->get_logger(), "pmf_slope: %f", pmf_slope_);
        RCLCPP_INFO(this->get_logger(), "pmf_initial_distance: %f", pmf_initial_distance_);
        RCLCPP_INFO(this->get_logger(), "pmf_max_distance: %f", pmf_max_distance_);
        RCLCPP_INFO(this->get_logger(), "pmf_cell_size: %f", pmf_cell_size_);

        // Hole detection parameters log
        RCLCPP_INFO(this->get_logger(), "hole_detection_enabled: %s", hole_detection_enabled_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "hole_detection_algorithm: %s", hole_detection_algorithm_.c_str());
        RCLCPP_INFO(this->get_logger(), "hole_output_topic: %s", hole_output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "hole_raw_output_topic: %s", hole_raw_output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "lidar_frame: %s", lidar_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "hole_detection_range_x: %f", hole_detection_range_x_);
        RCLCPP_INFO(this->get_logger(), "hole_detection_range_y: %f", hole_detection_range_y_);
        RCLCPP_INFO(this->get_logger(), "hole_detection_max_height: %f", hole_detection_max_height_);
        RCLCPP_INFO(this->get_logger(), "hole_ground_tolerance: %f", hole_ground_tolerance_);

        // Dynamic ground plane estimation parameters log
        RCLCPP_INFO(this->get_logger(), "use_dynamic_ground_plane: %s", use_dynamic_ground_plane_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "ground_plane_rolling_window_x: %f", ground_plane_rolling_window_x_);
        RCLCPP_INFO(this->get_logger(), "ground_plane_rolling_window_y: %f", ground_plane_rolling_window_y_);
        RCLCPP_INFO(this->get_logger(), "ground_plane_ransac_distance_threshold: %f", ground_plane_ransac_distance_threshold_);
        RCLCPP_INFO(this->get_logger(), "ground_plane_ransac_max_iterations: %d", ground_plane_ransac_max_iterations_);
        RCLCPP_INFO(this->get_logger(), "hole_detection_height_buffer: %f", hole_detection_height_buffer_);

        // Ground plane visualization parameters
        this->get_parameter("visualize_ground_plane", visualize_ground_plane_);
        this->get_parameter("ground_plane_visualization_topic", ground_plane_visualization_topic_);
        this->get_parameter("ground_plane_visualization_size", ground_plane_visualization_size_);

        // Ground plane visualization parameters log
        RCLCPP_INFO(this->get_logger(), "visualize_ground_plane: %s", visualize_ground_plane_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "ground_plane_visualization_topic: %s", ground_plane_visualization_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "ground_plane_visualization_size: %f", ground_plane_visualization_size_);

        // LiDAR origin parameters
        this->get_parameter("lidar_origin_x", lidar_origin_x_);
        this->get_parameter("lidar_origin_y", lidar_origin_y_);
        this->get_parameter("lidar_origin_z", lidar_origin_z_);

        // LiDAR origin parameters log
        RCLCPP_INFO(this->get_logger(), "lidar_origin (fallback): (%.3f, %.3f, %.3f)",
                   lidar_origin_x_, lidar_origin_y_, lidar_origin_z_);
    }

    void ObstacleCloudToScanNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto callback_start_time = std::chrono::high_resolution_clock::now();
        RCLCPP_DEBUG(this->get_logger(), "=== Processing pipeline started ===");
        RCLCPP_DEBUG(this->get_logger(), "Input cloud size: %u points", msg->width * msg->height);

        // 時間計測変数の宣言
        double tf_time_ms = 0.0;
        double filtering_time_ms = 0.0;
        auto filtering_start_time = callback_start_time;
        auto filtering_end_time = callback_start_time;

        // TF変換処理開始
        auto tf_start_time = std::chrono::high_resolution_clock::now();
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped 
            = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
            return;
        }
        Eigen::Affine3d transform = tf2::transformToEigen(transform_stamped.transform);

        // メモリ最適化パイプライン
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // TF変換を直接cloudに適用（余計なコピーを削除）
        pcl::transformPointCloud(*cloud, *cloud, transform);

        auto tf_end_time = std::chrono::high_resolution_clock::now();
        tf_time_ms = std::chrono::duration<double, std::milli>(tf_end_time - tf_start_time).count();
        RCLCPP_DEBUG(this->get_logger(), "TF Transform: %.3f ms (%zu points)", tf_time_ms, cloud->size());
          
        // フィルタリングパイプライン
        filtering_start_time = std::chrono::high_resolution_clock::now();
        size_t original_points = cloud->size();

        applyFilteringPipeline(cloud, voxel_leaf_size_,
                              obstacle_detection_range_x_min_, obstacle_detection_range_x_max_,
                              obstacle_detection_range_y_min_, obstacle_detection_range_y_max_,
                              obstacle_detection_range_z_min_, obstacle_detection_range_z_max_,
                              robot_box_position_, robot_box_size_, this->get_logger());
        
        filtering_end_time = std::chrono::high_resolution_clock::now();
        filtering_time_ms = std::chrono::duration<double, std::milli>(filtering_end_time - filtering_start_time).count();
        size_t filtered_points = cloud->size();
        
        RCLCPP_DEBUG(this->get_logger(), "Filtering pipeline: %.3f ms (%zu -> %zu points, %.1f%% reduction)", 
                    filtering_time_ms, original_points, filtered_points, 
                    100.0 * (1.0 - static_cast<double>(filtered_points) / original_points));

        // body_removed_cloudとして使用 (変数名互換性のため)
        pcl::PointCloud<pcl::PointXYZ>::Ptr body_removed_cloud = cloud;
        
        // 統計情報収集用
        size_t num_downsampled_points = filtered_points;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 地面除去処理時間計測開始
        auto ground_removal_start_time = std::chrono::high_resolution_clock::now();

        if (ground_remove_algorithm_ == "PMF") {
            RCLCPP_DEBUG(this->get_logger(), "Using PMF filter for ground segmentation.");

            // PMFで障害物点と地面点の両方を取得
            bool pmf_success = applyProgressiveMorphologicalFilterWithGround(
                body_removed_cloud,
                filtered_cloud,
                ground_cloud,
                this->get_logger(),
                pmf_max_window_size_,
                pmf_slope_,
                pmf_initial_distance_,
                pmf_max_distance_,
                pmf_cell_size_);

            // PMF失敗時の処理
            if (!pmf_success || filtered_cloud->empty()) {
                RCLCPP_WARN(this->get_logger(), "PMFフィルタが失敗、元の点群を使用");
                filtered_cloud = body_removed_cloud;
                ground_cloud->clear();  // 地面点も空にする
            }

            auto ground_removal_end_time = std::chrono::high_resolution_clock::now();
            double ground_removal_time_ms = std::chrono::duration<double, std::milli>(ground_removal_end_time - ground_removal_start_time).count();
            RCLCPP_DEBUG(this->get_logger(), "PMF地面除去: %.3f ms (%zu -> 障害物%zu点, 地面%zu点)",
                        ground_removal_time_ms, body_removed_cloud->size(),
                        filtered_cloud->size(), ground_cloud->size());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Using normal-based filter for ground segmentation.");

            // 並列法線推定の時間計測
            auto normal_start_time = std::chrono::high_resolution_clock::now();
            pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormals(body_removed_cloud, normal_radius_, this->get_logger());
            auto normal_end_time = std::chrono::high_resolution_clock::now();
            double normal_time_ms = std::chrono::duration<double, std::milli>(normal_end_time - normal_start_time).count();
            RCLCPP_DEBUG(this->get_logger(), "Normal estimation: %.3f ms (%zu points)", normal_time_ms, body_removed_cloud->size());

            // 障害物/地面フィルタリング（両方を同時に取得）
            auto filter_start_time = std::chrono::high_resolution_clock::now();
            bool filter_success = filterObstaclesWithGround(
                body_removed_cloud,
                normals,
                filtered_cloud,
                ground_cloud,
                normal_max_slope_angle_,
                this->get_logger());

            if (!filter_success || filtered_cloud->empty()) {
                RCLCPP_WARN(this->get_logger(), "法線ベースフィルタが失敗、元の点群を使用");
                filtered_cloud = body_removed_cloud;
                ground_cloud->clear();  // 地面点も空にする
            }

            auto filter_end_time = std::chrono::high_resolution_clock::now();
            double filter_time_ms = std::chrono::duration<double, std::milli>(filter_end_time - filter_start_time).count();

            auto ground_removal_end_time = std::chrono::high_resolution_clock::now();
            double ground_removal_time_ms = std::chrono::duration<double, std::milli>(ground_removal_end_time - ground_removal_start_time).count();
            RCLCPP_DEBUG(this->get_logger(), "Total normal-based ground removal: %.3f ms (Normal: %.3f ms + Filter: %.3f ms, 障害物%zu点, 地面%zu点)",
                        ground_removal_time_ms, normal_time_ms, filter_time_ms,
                        filtered_cloud->size(), ground_cloud->size());
        }

        // 障害物検知処理時間計測終了（互換性のため変数名維持）
        auto obstacle_end_time = std::chrono::high_resolution_clock::now();
        double obstacle_processing_time_ms = std::chrono::duration<double, std::milli>(obstacle_end_time - ground_removal_start_time).count();

        // 地面平面推定（穴検知が有効な場合のみ）
        if (hole_detection_enabled_) {
            if (use_dynamic_ground_plane_) {
                // 動的地面平面推定（RANSAC）
                if (!ground_cloud->empty()) {
                    // Rolling window内の地面点をフィルタ
                    auto window_ground = filterRollingWindow(
                        ground_cloud,
                        ground_plane_rolling_window_x_,
                        ground_plane_rolling_window_y_,
                        this->get_logger());

                    // RANSAC平面推定
                    if (window_ground && !window_ground->empty()) {
                        bool ransac_success = estimateGroundPlaneRANSAC(
                            window_ground,
                            ground_plane_,
                            ground_plane_ransac_distance_threshold_,
                            ground_plane_ransac_max_iterations_,
                            this->get_logger());

                        if (ransac_success) {
                            ground_plane_initialized_ = true;
                            RCLCPP_DEBUG(this->get_logger(), "動的地面平面推定成功");

                            // 地面平面を可視化
                            publishGroundPlaneVisualization(ground_plane_, msg->header.stamp);
                        } else {
                            RCLCPP_WARN(this->get_logger(), "RANSAC平面推定失敗、静的平面にフォールバック");
                            if (!ground_plane_initialized_) {
                                initializeGroundPlane();
                            }
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Rolling window内の地面点が空、静的平面にフォールバック");
                        if (!ground_plane_initialized_) {
                            initializeGroundPlane();
                        }
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "地面点が空、静的平面にフォールバック");
                    if (!ground_plane_initialized_) {
                        initializeGroundPlane();
                    }
                }
            } else {
                // 静的地面平面推定（z=0平面、初回のみ）
                if (!ground_plane_initialized_) {
                    initializeGroundPlane();
                }
            }
        }

        // 穴検知処理
        pcl::PointCloud<pcl::PointXYZ>::Ptr hole_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_hole_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        double hole_processing_time_ms = 0.0;

        auto hole_start_time = std::chrono::high_resolution_clock::now();
        hole_cloud = detectHoles(body_removed_cloud, raw_hole_cloud);
        auto hole_end_time = std::chrono::high_resolution_clock::now();
        hole_processing_time_ms = std::chrono::duration<double, std::milli>(hole_end_time - hole_start_time).count();

        // パブリッシュ処理時間計測開始
        auto publish_start_time = std::chrono::high_resolution_clock::now();
        
        // 障害物点群をパブリッシュ
        RCLCPP_DEBUG(this->get_logger(), "Publishing filtered point cloud");
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);

        filtered_msg.header.frame_id = target_frame_;
        filtered_msg.header.stamp = msg->header.stamp;
        filtered_cloud_publisher_->publish(filtered_msg);

        // 穴点群をパブリッシュ（空でも常にパブリッシュしてRViz表示を更新）
        if (hole_detection_enabled_) {
            sensor_msgs::msg::PointCloud2 hole_msg;
            pcl::toROSMsg(*hole_cloud, hole_msg);

            hole_msg.header.frame_id = target_frame_;
            hole_msg.header.stamp = msg->header.stamp;
            hole_cloud_publisher_->publish(hole_msg);

            if (hole_cloud->size() > 0) {
                RCLCPP_DEBUG(this->get_logger(), "Publishing hole point cloud with %zu points", hole_cloud->size());
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Publishing empty hole cloud to update RViz display");
            }

            // デバッグ用：穴検知前の元の点群をパブリッシュ
            sensor_msgs::msg::PointCloud2 raw_hole_msg;
            pcl::toROSMsg(*raw_hole_cloud, raw_hole_msg);

            raw_hole_msg.header.frame_id = target_frame_;
            raw_hole_msg.header.stamp = msg->header.stamp;
            hole_raw_cloud_publisher_->publish(raw_hole_msg);

            if (raw_hole_cloud->size() > 0) {
                RCLCPP_DEBUG(this->get_logger(), "デバッグ用：穴検知前の元の点群をパブリッシュ (%zu点)", raw_hole_cloud->size());
            }
        }
        
        auto publish_end_time = std::chrono::high_resolution_clock::now();
        double publish_time_ms = std::chrono::duration<double, std::milli>(publish_end_time - publish_start_time).count();
        RCLCPP_DEBUG(this->get_logger(), "Publishing: %.3f ms", publish_time_ms);
        
        auto callback_end_time = std::chrono::high_resolution_clock::now();
        double total_processing_time_ms = std::chrono::duration<double, std::milli>(callback_end_time - callback_start_time).count();

        // フィルタリング時間は既に計算済み
        
        // 総合計測結果表示
        RCLCPP_DEBUG(this->get_logger(), "=== Processing pipeline completed ===");
        RCLCPP_DEBUG(this->get_logger(), "Total processing: %.3f ms", total_processing_time_ms);
        RCLCPP_DEBUG(this->get_logger(), "Pipeline breakdown: TF+Filter: %.3f ms | Ground: %.3f ms | Hole: %.3f ms | Publish: %.3f ms", 
                    tf_time_ms + filtering_time_ms, obstacle_processing_time_ms, hole_processing_time_ms, publish_time_ms);

        { // Scope for lock guard
            std::lock_guard<std::mutex> lock(data_mutex_);
            processing_times_.push_back(total_processing_time_ms);
            downsampled_points_counts_.push_back(num_downsampled_points);
            
            // 分離されたパフォーマンス計測データを収集
            obstacle_processing_times_.push_back(obstacle_processing_time_ms);
            hole_processing_times_.push_back(hole_processing_time_ms);
        }
    }

void ObstacleCloudToScanNode::logPerformance()
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (processing_times_.empty() || downsampled_points_counts_.empty())
    {
        return; // Nothing to log
    }

    // 総処理時間の平均
    double sum_processing_time = 0.0;
    for (double time : processing_times_) {
        sum_processing_time += time;
    }
    double avg_processing_time = sum_processing_time / processing_times_.size();

    // ダウンサンプル点数の平均
    size_t sum_points = 0;
    for (size_t count : downsampled_points_counts_) {
        sum_points += count;
    }
    double avg_downsampled_points = static_cast<double>(sum_points) / downsampled_points_counts_.size();

    // 障害物検知処理時間の平均
    double avg_obstacle_time = 0.0;
    if (!obstacle_processing_times_.empty()) {
        double sum_obstacle_time = 0.0;
        for (double time : obstacle_processing_times_) {
            sum_obstacle_time += time;
        }
        avg_obstacle_time = sum_obstacle_time / obstacle_processing_times_.size();
    }

    // 穴検知処理時間の平均
    double avg_hole_time = 0.0;
    if (!hole_processing_times_.empty()) {
        double sum_hole_time = 0.0;
        for (double time : hole_processing_times_) {
            sum_hole_time += time;
        }
        avg_hole_time = sum_hole_time / hole_processing_times_.size();
    }

    // ログ出力
    RCLCPP_INFO(this->get_logger(), "Avg Total Time: %.3f ms | Obstacle: %.3f ms | Hole: %.3f ms", 
               avg_processing_time, avg_obstacle_time, avg_hole_time);
    RCLCPP_INFO(this->get_logger(), "Avg Downsampled Points: %.0f", avg_downsampled_points);

    // データクリア
    processing_times_.clear();
    downsampled_points_counts_.clear();
    obstacle_processing_times_.clear();
    hole_processing_times_.clear();
    last_log_time_ = this->get_clock()->now();
}

void ObstacleCloudToScanNode::initializeGroundPlane()
{
    // 静的地面平面を初期化（z=0の水平平面）
    // 注：LiDAR原点は既にコンストラクタで初期化済み

    // target_frame座標系でのz=0平面（水平地面）
    ground_plane_.a = 0.0;  // x係数
    ground_plane_.b = 0.0;  // y係数
    ground_plane_.c = 1.0;  // z係数（上向き法線）
    ground_plane_.d = 0.0;  // 定数項（z=0平面）

    ground_plane_initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "Static ground plane initialized (z=0 plane)");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObstacleCloudToScanNode::detectHoles(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &raw_hole_points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr hole_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // raw_hole_pointsを初期化
    if (!raw_hole_points) {
        raw_hole_points = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    }
    raw_hole_points->clear();

    if (!hole_detection_enabled_) {
        return hole_cloud;
    }

    if (!ground_plane_initialized_) {
        RCLCPP_DEBUG(this->get_logger(), "Ground plane not initialized");
        return hole_cloud;
    }

    // 穴検知を実行（フィルタ削除、統合版detectHolesを使用）
    RCLCPP_DEBUG(this->get_logger(), "穴検知実行（シンプル判定ロジック）");
    hole_cloud = ::detectHoles(
        cloud,  // body_removed_cloudをそのまま使用（フィルタ削除）
        lidar_origin_,
        ground_plane_,
        hole_ground_tolerance_,
        this->get_logger(),
        raw_hole_points);  // RGB色付き点群

    return hole_cloud;
}

void ObstacleCloudToScanNode::publishGroundPlaneVisualization(const GroundPlane &plane, const rclcpp::Time &stamp)
{
    if (!visualize_ground_plane_) {
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = target_frame_;
    marker.header.stamp = stamp;
    marker.ns = "ground_plane";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 平面方程式: ax + by + cz + d = 0
    // 法線ベクトル: (a, b, c)
    double a = plane.a;
    double b = plane.b;
    double c = plane.c;
    double d = plane.d;

    // 法線ベクトルの正規化
    double norm = std::sqrt(a*a + b*b + c*c);
    if (norm < 1e-6) {
        RCLCPP_WARN(this->get_logger(), "地面平面の法線ベクトルがゼロに近い、可視化をスキップ");
        return;
    }
    a /= norm;
    b /= norm;
    c /= norm;
    d /= norm;

    // 原点から平面への最短距離の点を計算（平面上の基準点）
    // P0 = -d * (a, b, c)
    double p0_x = -d * a;
    double p0_y = -d * b;
    double p0_z = -d * c;

    // 平面上の2つの直交する方向ベクトルを計算
    // v1は法線に垂直なベクトル
    double v1_x, v1_y, v1_z;
    if (std::abs(c) > 0.1) {
        // z成分が大きい場合、x軸方向のベクトルを基準にする
        v1_x = 1.0;
        v1_y = 0.0;
        v1_z = -(a * v1_x + b * v1_y) / c;
    } else if (std::abs(b) > 0.1) {
        // y成分が大きい場合、x軸方向のベクトルを基準にする
        v1_x = 1.0;
        v1_y = -(a * v1_x + c * 0.0) / b;
        v1_z = 0.0;
    } else {
        // x成分が大きい場合、y軸方向のベクトルを基準にする
        v1_x = 0.0;
        v1_y = 1.0;
        v1_z = -(b * v1_y) / a;
    }

    // v1を正規化
    double v1_norm = std::sqrt(v1_x*v1_x + v1_y*v1_y + v1_z*v1_z);
    v1_x /= v1_norm;
    v1_y /= v1_norm;
    v1_z /= v1_norm;

    // v2 = 法線 × v1 (外積)
    double v2_x = b * v1_z - c * v1_y;
    double v2_y = c * v1_x - a * v1_z;
    double v2_z = a * v1_y - b * v1_x;

    // 矩形の4つの頂点を計算
    double size = ground_plane_visualization_size_;
    double half_size = size / 2.0;

    geometry_msgs::msg::Point p1, p2, p3, p4;

    // 頂点1: p0 + half_size*v1 + half_size*v2
    p1.x = p0_x + half_size * v1_x + half_size * v2_x;
    p1.y = p0_y + half_size * v1_y + half_size * v2_y;
    p1.z = p0_z + half_size * v1_z + half_size * v2_z;

    // 頂点2: p0 - half_size*v1 + half_size*v2
    p2.x = p0_x - half_size * v1_x + half_size * v2_x;
    p2.y = p0_y - half_size * v1_y + half_size * v2_y;
    p2.z = p0_z - half_size * v1_z + half_size * v2_z;

    // 頂点3: p0 - half_size*v1 - half_size*v2
    p3.x = p0_x - half_size * v1_x - half_size * v2_x;
    p3.y = p0_y - half_size * v1_y - half_size * v2_y;
    p3.z = p0_z - half_size * v1_z - half_size * v2_z;

    // 頂点4: p0 + half_size*v1 - half_size*v2
    p4.x = p0_x + half_size * v1_x - half_size * v2_x;
    p4.y = p0_y + half_size * v1_y - half_size * v2_y;
    p4.z = p0_z + half_size * v1_z - half_size * v2_z;

    // 矩形を2つの三角形で表現
    // 三角形1: p1, p2, p3
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);

    // 三角形2: p1, p3, p4
    marker.points.push_back(p1);
    marker.points.push_back(p3);
    marker.points.push_back(p4);

    // 色設定（半透明の緑色）
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;  // 半透明

    marker.lifetime = rclcpp::Duration::from_seconds(0.5);  // 0.5秒で消える

    ground_plane_marker_publisher_->publish(marker);
    RCLCPP_DEBUG(this->get_logger(), "地面平面可視化マーカーをパブリッシュ");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleCloudToScanNode>());
    rclcpp::shutdown();
    return 0;
}
