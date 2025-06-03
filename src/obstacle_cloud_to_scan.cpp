#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
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

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        declare_parameters();
        get_parameters();

        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 1, std::bind(&ObstacleCloudToScanNode::pointCloudCallback, this, std::placeholders::_1));
        RCLCPP_DEBUG(this->get_logger(), "Subscribed to topic: %s", input_topic_.c_str());

        auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_,rclcpp::QoS(rclcpp::KeepLast(10)));
        RCLCPP_DEBUG(this->get_logger(), "Publisher created for topic: %s", output_topic_.c_str());
    }

    void ObstacleCloudToScanNode::declare_parameters()
    {
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<std::string>("input_topic", "/input_cloud");
        this->declare_parameter<std::string>("output_topic", "/filtered_point_cloud");
        this->declare_parameter<std::string>("laser_scan_topic", "/scan");
        this->declare_parameter<double>("voxel_leaf_size", 0.1);
        this->declare_parameter<double>("min_distance", 0.1);
        this->declare_parameter<std::vector<double>>("robot_box_size", {0.6, 0.6, 1.0});
        this->declare_parameter<std::vector<double>>("robot_box_position", {0.0, 0.0, 0.0});
        this->declare_parameter<double>("max_slope_angle", 5.0);
        this->declare_parameter<bool>("use_gpu", false);
        this->declare_parameter<bool>("use_pmf_filter", true);
        this->declare_parameter<int>("pmf_max_window_size", 33);
        this->declare_parameter<double>("pmf_slope", 1.0);
        this->declare_parameter<double>("pmf_initial_distance", 0.15);
        this->declare_parameter<double>("pmf_max_distance", 3.0);
        this->declare_parameter<double>("pmf_cell_size", 0.5);

    }

    void ObstacleCloudToScanNode::get_parameters()
    {
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->get_parameter("robot_box_size", robot_box_size_);
        this->get_parameter("robot_box_position", robot_box_position_);
        this->get_parameter("max_slope_angle", max_slope_angle_);
        this->get_parameter("use_gpu", use_gpu_);
        this->get_parameter("use_pmf_filter", use_pmf_filter_);
        this->get_parameter("pmf_max_window_size", pmf_max_window_size_);
        this->get_parameter("pmf_slope", pmf_slope_);
        this->get_parameter("pmf_initial_distance", pmf_initial_distance_);
        this->get_parameter("pmf_max_distance", pmf_max_distance_);
        this->get_parameter("pmf_cell_size", pmf_cell_size_);

        RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
        RCLCPP_INFO(this->get_logger(), "target_frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "voxel_leaf_size: %f", voxel_leaf_size_);
        RCLCPP_INFO(this->get_logger(), "max_slope_angle: %f", max_slope_angle_);
        RCLCPP_INFO(this->get_logger(), "use_gpu: %s", use_gpu_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "use_pmf_filter: %s", use_pmf_filter_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "pmf_max_window_size: %d", pmf_max_window_size_);
        RCLCPP_INFO(this->get_logger(), "pmf_slope: %f", pmf_slope_);
        RCLCPP_INFO(this->get_logger(), "pmf_initial_distance: %f", pmf_initial_distance_);
        RCLCPP_INFO(this->get_logger(), "pmf_max_distance: %f", pmf_max_distance_);
        RCLCPP_INFO(this->get_logger(), "pmf_cell_size: %f", pmf_cell_size_);
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
            = downsamplePointCloud(transformed_cloud, voxel_leaf_size_, use_gpu_, this->get_logger());
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

        if (use_pmf_filter_) {
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
            RCLCPP_DEBUG(this->get_logger(), "Starting normal estimation (original method)");
            pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormals(body_removed_cloud, this->get_logger());
            RCLCPP_DEBUG(this->get_logger(), "Normal estimation completed (original method)");

            RCLCPP_DEBUG(this->get_logger(), "Starting obstacle filtering (original method)");
            filtered_cloud = filterObstacles(body_removed_cloud, normals, max_slope_angle_, this->get_logger());
            RCLCPP_DEBUG(this->get_logger(), "Obstacle filtering completed (original method). Number of obstacle points: %zu", filtered_cloud ? filtered_cloud->size() : 0);
        }

        // フィルタリング後の点群をパブリッシュ
        RCLCPP_DEBUG(this->get_logger(), "Publishing filtered point cloud");
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);

        filtered_msg.header.frame_id = "base_link"; // Should be target_frame_
        filtered_msg.header.stamp = this->get_clock()->now();
        filtered_cloud_publisher_->publish(filtered_msg);
        
        auto callback_end_time = std::chrono::high_resolution_clock::now();
        double total_processing_time_ms = std::chrono::duration<double, std::milli>(callback_end_time - callback_start_time).count();

        { // Scope for lock guard
            std::lock_guard<std::mutex> lock(data_mutex_);
            processing_times_.push_back(total_processing_time_ms);
            downsampled_points_counts_.push_back(num_downsampled_points);
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

    double sum_processing_time = 0.0;
    for (double time : processing_times_) {
        sum_processing_time += time;
    }
    double avg_processing_time = sum_processing_time / processing_times_.size();

    size_t sum_points = 0;
    for (size_t count : downsampled_points_counts_) {
        sum_points += count;
    }
    double avg_downsampled_points = static_cast<double>(sum_points) / downsampled_points_counts_.size();

    RCLCPP_INFO(this->get_logger(), "Avg Process Time (last 1s): %.3f ms", avg_processing_time);
    RCLCPP_INFO(this->get_logger(), "Avg Downsampled Points (last 1s): %.0f", avg_downsampled_points);

    processing_times_.clear();
    downsampled_points_counts_.clear();
    last_log_time_ = this->get_clock()->now();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleCloudToScanNode>());
    rclcpp::shutdown();
    return 0;
}
