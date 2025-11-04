#ifndef OBSTACLE_CLOUD_TO_SCAN_NODE_HPP
#define OBSTACLE_CLOUD_TO_SCAN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <vector>
#include <mutex>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <memory>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "obstacle_cloud_to_scan/pcl_functions.hpp"

class ObstacleCloudToScanNode : public rclcpp::Node
{
public:
    ObstacleCloudToScanNode();

private:
    void declare_parameters();
    void get_parameters();
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void logPerformance();
    
    // Hole detection functions
    pcl::PointCloud<pcl::PointXYZ>::Ptr detectHoles(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &raw_hole_points);

    // Ground plane initialization
    void initializeGroundPlane();

    // Ground plane visualization
    void publishGroundPlaneVisualization(const GroundPlane &plane, const rclcpp::Time &stamp);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hole_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hole_raw_cloud_publisher_;  // デバッグ用：穴検知前の元の点群
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ground_plane_marker_publisher_;

    // TF2 members
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    std::string target_frame_;
    std::string input_topic_;
    std::string output_topic_;
    std::string ground_remove_algorithm_;
    double voxel_leaf_size_;
    std::vector<double> robot_box_size_;
    std::vector<double> robot_box_position_;
    double normal_max_slope_angle_;
    double normal_radius_;

    // PMF Parameters
    int pmf_max_window_size_;
    double pmf_slope_;
    double pmf_initial_distance_;
    double pmf_max_distance_;
    double pmf_cell_size_;

    // Hole Detection Parameters
    bool hole_detection_enabled_;
    std::string hole_detection_algorithm_;
    std::string hole_output_topic_;
    std::string hole_raw_output_topic_;  // デバッグ用：穴検知前の元の点群トピック
    std::string lidar_frame_;
    double hole_detection_range_x_;
    double hole_detection_range_y_;
    double hole_detection_max_height_;
    double hole_ground_tolerance_;

    // Dynamic Ground Plane Estimation Parameters
    bool use_dynamic_ground_plane_;
    double ground_plane_rolling_window_x_;
    double ground_plane_rolling_window_y_;
    double ground_plane_ransac_distance_threshold_;
    int ground_plane_ransac_max_iterations_;
    double hole_detection_height_buffer_;

    // Ground Plane Visualization Parameters
    bool visualize_ground_plane_;
    std::string ground_plane_visualization_topic_;
    double ground_plane_visualization_size_;

    // Obstacle detection range parameters (X, Y, Z PassThrough filter)
    double obstacle_detection_range_x_min_;
    double obstacle_detection_range_x_max_;
    double obstacle_detection_range_y_min_;
    double obstacle_detection_range_y_max_;
    double obstacle_detection_range_z_min_;
    double obstacle_detection_range_z_max_;

    // Ground plane and LiDAR origin for hole detection
    GroundPlane ground_plane_;
    pcl::PointXYZ lidar_origin_;
    bool ground_plane_initialized_;

    // LiDAR origin parameters (fallback when TF is unavailable)
    double lidar_origin_x_;
    double lidar_origin_y_;
    double lidar_origin_z_;

    std::vector<double> processing_times_;
    std::vector<size_t> downsampled_points_counts_;
    
    // Separated performance metrics for obstacle detection and hole detection
    std::vector<double> obstacle_processing_times_;
    std::vector<double> hole_processing_times_;
    
    rclcpp::Time last_log_time_;
    rclcpp::TimerBase::SharedPtr logging_timer_;
    std::mutex data_mutex_;
};

#endif // OBSTACLE_CLOUD_TO_SCAN_NODE_HPP

