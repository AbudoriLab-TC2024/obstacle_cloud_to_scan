#ifndef OBSTACLE_CLOUD_TO_SCAN_NODE_HPP
#define OBSTACLE_CLOUD_TO_SCAN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>

class ObstacleCloudToScanNode : public rclcpp::Node
{
public:
    ObstacleCloudToScanNode();

private:
    void declare_parameters();
    void get_parameters();
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals);
    void publishLaserScan(const pcl::PointCloud<pcl::PointXYZ>::Ptr &points, const std_msgs::msg::Header &header);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;

    std::string input_topic_;
    std::string output_topic_;
    std::string laser_scan_topic_;
    double voxel_leaf_size_;
    double max_distance_;
    double min_distance_;
    std::vector<double> robot_box_size_;
    std::vector<double> robot_box_position_;
    double max_slope_angle_;
    bool use_gpu_;
    double scan_angle_min_;
    double scan_angle_max_;
    double scan_range_min_;
    double scan_range_max_;
};

#endif // OBSTACLE_CLOUD_TO_SCAN_NODE_HPP

