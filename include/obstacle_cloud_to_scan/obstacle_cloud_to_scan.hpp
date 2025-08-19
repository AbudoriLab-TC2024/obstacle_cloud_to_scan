#ifndef OBSTACLE_CLOUD_TO_SCAN_NODE_HPP
#define OBSTACLE_CLOUD_TO_SCAN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    
    // Ground plane initialization
    void initializeGroundPlane();
  
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hole_cloud_publisher_;

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
    double scan_angle_min_;
    double scan_angle_max_;
    double scan_range_min_;
    double scan_range_max_;

    // PMF Parameters
    bool use_pmf_filter_;
    int pmf_max_window_size_;
    double pmf_slope_;
    double pmf_initial_distance_;
    double pmf_max_distance_;
    double pmf_cell_size_;

    // Hole Detection Parameters
    bool hole_detection_enabled_;
    std::string hole_detection_algorithm_;
    std::string hole_output_topic_;
    std::string lidar_frame_;
    double hole_detection_range_x_;
    double hole_detection_range_y_;
    double hole_detection_max_height_;
    double hole_ground_tolerance_;

    // Ground plane and LiDAR origin for hole detection
    GroundPlane ground_plane_;
    pcl::PointXYZ lidar_origin_;
    bool ground_plane_initialized_;

    std::vector<double> processing_times_;
    std::vector<size_t> downsampled_points_counts_;
    rclcpp::Time last_log_time_;
    rclcpp::TimerBase::SharedPtr logging_timer_;
    std::mutex data_mutex_;
};

#endif // OBSTACLE_CLOUD_TO_SCAN_NODE_HPP

