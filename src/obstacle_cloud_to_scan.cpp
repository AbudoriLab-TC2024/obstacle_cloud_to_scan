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

        auto sensor_qos = rclcpp::SensorDataQoS();
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, sensor_qos, std::bind(&ObstacleCloudToScanNode::pointCloudCallback, this, std::placeholders::_1));
        RCLCPP_DEBUG(this->get_logger(), "Subscribed to topic: %s", input_topic_.c_str());

        filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, sensor_qos);
        RCLCPP_DEBUG(this->get_logger(), "Publisher created for topic: %s", output_topic_.c_str());

        hole_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(hole_output_topic_, sensor_qos);
        RCLCPP_DEBUG(this->get_logger(), "Hole cloud publisher created for topic: %s", hole_output_topic_.c_str());
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
        this->declare_parameter<std::string>("lidar_frame", "livox_frame");
        this->declare_parameter<double>("hole_detection_range_x", 3.0);
        this->declare_parameter<double>("hole_detection_range_y", 5.0);
        this->declare_parameter<double>("hole_detection_max_height", 0.3);
        this->declare_parameter<double>("hole_ground_tolerance", 0.05);

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
        this->get_parameter("lidar_frame", lidar_frame_);
        this->get_parameter("hole_detection_range_x", hole_detection_range_x_);
        this->get_parameter("hole_detection_range_y", hole_detection_range_y_);
        this->get_parameter("hole_detection_max_height", hole_detection_max_height_);
        this->get_parameter("hole_ground_tolerance", hole_ground_tolerance_);

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
        RCLCPP_INFO(this->get_logger(), "lidar_frame: %s", lidar_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "hole_detection_range_x: %f", hole_detection_range_x_);
        RCLCPP_INFO(this->get_logger(), "hole_detection_range_y: %f", hole_detection_range_y_);
        RCLCPP_INFO(this->get_logger(), "hole_detection_max_height: %f", hole_detection_max_height_);
        RCLCPP_INFO(this->get_logger(), "hole_ground_tolerance: %f", hole_ground_tolerance_);
    }

    void ObstacleCloudToScanNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto callback_start_time = std::chrono::high_resolution_clock::now();
        RCLCPP_DEBUG(this->get_logger(), "Received point cloud message");

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

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

        auto tf_transforme_end_time = std::chrono::high_resolution_clock::now(); // 計測終了
          
        // ダウンサンプリング処理
        RCLCPP_DEBUG(this->get_logger(), "Starting downsampling");
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud 
            = downsamplePointCloud(transformed_cloud, voxel_leaf_size_, this->get_logger());
        size_t num_downsampled_points = downsampled_cloud->points.size();
        RCLCPP_DEBUG(this->get_logger(), "Downsampling completed");

        // パススルーフィルタの適用
        RCLCPP_DEBUG(this->get_logger(), "Starting passthrough filter");
        pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud 
            = applyPassThroughFilter(downsampled_cloud, robot_box_size_, this->get_logger());
        RCLCPP_DEBUG(this->get_logger(), "Passthrough filter completed");

        // ロボットの体の除去
        RCLCPP_DEBUG(this->get_logger(), "Removing robot body from point cloud");
        pcl::PointCloud<pcl::PointXYZ>::Ptr body_removed_cloud 
            = removeRobotBody(passthrough_cloud, robot_box_position_, robot_box_size_, this->get_logger());
        RCLCPP_DEBUG(this->get_logger(), "Robot body removal completed");

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 障害物検知処理時間計測開始
        auto obstacle_start_time = std::chrono::high_resolution_clock::now();

        if (ground_remove_algorithm_ == "PMF") {
            RCLCPP_DEBUG(this->get_logger(), "Using PMF filter for ground segmentation.");
            filtered_cloud = applyProgressiveMorphologicalFilter(
                body_removed_cloud,
                this->get_logger(),
                pmf_max_window_size_,
                pmf_slope_,
                pmf_initial_distance_,
                pmf_max_distance_,
                pmf_cell_size_);
            RCLCPP_DEBUG(this->get_logger(), "PMF filtering returned. Number of obstacle points: %zu", filtered_cloud ? filtered_cloud->size() : 0);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Using original normal-based filter for ground segmentation.");
            RCLCPP_DEBUG(this->get_logger(), "Starting normal estimation");
            pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormals(body_removed_cloud, normal_radius_, this->get_logger());
            RCLCPP_DEBUG(this->get_logger(), "Normal estimation completed");

            RCLCPP_DEBUG(this->get_logger(), "Starting obstacle filtering");
            filtered_cloud = filterObstacles(body_removed_cloud, normals, normal_max_slope_angle_, this->get_logger());
            RCLCPP_DEBUG(this->get_logger(), "Obstacle filtering completed. Number of obstacle points: %zu", filtered_cloud ? filtered_cloud->size() : 0);
        }

        // 障害物検知処理時間計測終了
        auto obstacle_end_time = std::chrono::high_resolution_clock::now();
        double obstacle_processing_time_ms = std::chrono::duration<double, std::milli>(obstacle_end_time - obstacle_start_time).count();

        // 地面平面初期化（初回のみ）
        if (!ground_plane_initialized_ && hole_detection_enabled_) {
            initializeGroundPlane();
        }

        // 穴検知処理時間計測開始
        auto hole_start_time = std::chrono::high_resolution_clock::now();
        
        // 穴検知処理
        pcl::PointCloud<pcl::PointXYZ>::Ptr hole_cloud = detectHoles(body_removed_cloud);
        
        // 穴検知処理時間計測終了
        auto hole_end_time = std::chrono::high_resolution_clock::now();
        double hole_processing_time_ms = std::chrono::duration<double, std::milli>(hole_end_time - hole_start_time).count();

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
        }
        
        auto callback_end_time = std::chrono::high_resolution_clock::now();
        double total_processing_time_ms = std::chrono::duration<double, std::milli>(callback_end_time - callback_start_time).count();

        { // Scope for lock guard
            std::lock_guard<std::mutex> lock(data_mutex_);
            processing_times_.push_back(total_processing_time_ms);
            downsampled_points_counts_.push_back(num_downsampled_points);
            
            // 分離されたパフォーマンス計測データを収集
            obstacle_processing_times_.push_back(obstacle_processing_time_ms);
            hole_processing_times_.push_back(hole_processing_time_ms);
        }
        RCLCPP_DEBUG(this->get_logger(), "PointCloud callback finished processing.");
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
    try {
        // LiDAR原点をtarget_frame座標系で取得
        auto transform_stamped = tf_buffer_->lookupTransform(
            target_frame_, lidar_frame_, tf2::TimePointZero);
        
        lidar_origin_.x = transform_stamped.transform.translation.x;
        lidar_origin_.y = transform_stamped.transform.translation.y;
        lidar_origin_.z = transform_stamped.transform.translation.z;
        
        // target_frame座標系でのz=0平面（水平地面）
        ground_plane_.a = 0.0;  // x係数
        ground_plane_.b = 0.0;  // y係数  
        ground_plane_.c = 1.0;  // z係数（上向き法線）
        ground_plane_.d = 0.0;  // 定数項（z=0平面）
        
        ground_plane_initialized_ = true;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Ground plane initialized. LiDAR origin in %s: (%.3f, %.3f, %.3f)",
                   target_frame_.c_str(), lidar_origin_.x, lidar_origin_.y, lidar_origin_.z);
                   
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), 
                   "Could not initialize ground plane: %s", ex.what());
        ground_plane_initialized_ = false;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObstacleCloudToScanNode::detectHoles(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr hole_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (!hole_detection_enabled_) {
        return hole_cloud;
    }
    
    if (!ground_plane_initialized_) {
        RCLCPP_DEBUG(this->get_logger(), "Ground plane not initialized, skipping hole detection");
        return hole_cloud;
    }
    
    // 穴検知範囲フィルタを適用
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filterHoleDetectionRange(
        cloud, 
        hole_detection_range_x_, 
        hole_detection_range_y_, 
        hole_detection_max_height_,
        this->get_logger());
    
    // 穴検知アルゴリズムを実行
    if (hole_detection_algorithm_ == "BASIC") {
        RCLCPP_DEBUG(this->get_logger(), "Using BASIC hole detection algorithm");
        hole_cloud = detectHolesBasic(
            filtered_cloud,
            lidar_origin_,
            ground_plane_,
            hole_ground_tolerance_,
            this->get_logger());
    } else if (hole_detection_algorithm_ == "GRID") {
        RCLCPP_DEBUG(this->get_logger(), "GRID algorithm not yet implemented, using BASIC");
        hole_cloud = detectHolesBasic(
            filtered_cloud,
            lidar_origin_,
            ground_plane_,
            hole_ground_tolerance_,
            this->get_logger());
    } else {
        // 不正な値の場合はBASICをデフォルトとして使用
        RCLCPP_DEBUG(this->get_logger(), "Unknown algorithm '%s', using BASIC as default", 
                    hole_detection_algorithm_.c_str());
        hole_cloud = detectHolesBasic(
            filtered_cloud,
            lidar_origin_,
            ground_plane_,
            hole_ground_tolerance_,
            this->get_logger());
    }
    
    return hole_cloud;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleCloudToScanNode>());
    rclcpp::shutdown();
    return 0;
}
