#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp>
#include "obstacle_cloud_to_scan/pcl_functions.hpp"

class ObstacleCloudToScanNode : public rclcpp::Node
{
public:
    ObstacleCloudToScanNode() : Node("obstacle_cloud_to_scan")
    {
        RCLCPP_DEBUG(this->get_logger(), "Initializing ObstacleCloudToScanNode");

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        declare_parameters();
        get_parameters();

        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, std::bind(&ObstacleCloudToScanNode::pointCloudCallback, this, std::placeholders::_1));
        RCLCPP_DEBUG(this->get_logger(), "Subscribed to topic: %s", input_topic_.c_str());

        filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
        RCLCPP_DEBUG(this->get_logger(), "Publisher created for topic: %s", output_topic_.c_str());
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;

    std::string input_topic_;
    std::string output_topic_;
    double voxel_leaf_size_;
    std::vector<double> robot_box_size_;
    std::vector<double> robot_box_position_;
    double max_slope_angle_;
    bool use_gpu_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void declare_parameters()
    {
        this->declare_parameter<std::string>("input_topic", "/input_cloud");
        this->declare_parameter<std::string>("output_topic", "/filtered_point_cloud");
        this->declare_parameter<std::string>("laser_scan_topic", "/scan");
        this->declare_parameter<double>("voxel_leaf_size", 0.1);
        this->declare_parameter<double>("min_distance", 0.1);
        this->declare_parameter<std::vector<double>>("robot_box_size", {0.6, 0.6, 1.0});
        this->declare_parameter<std::vector<double>>("robot_box_position", {0.0, 0.0, 0.0});
        this->declare_parameter<double>("max_slope_angle", 5.0);
        this->declare_parameter<bool>("use_gpu", false);

    }

    void get_parameters()
    {
        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->get_parameter("robot_box_size", robot_box_size_);
        this->get_parameter("robot_box_position", robot_box_position_);
        this->get_parameter("max_slope_angle", max_slope_angle_);
        this->get_parameter("use_gpu", use_gpu_);

        RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
        RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "voxel_leaf_size: %f", voxel_leaf_size_);
        RCLCPP_INFO(this->get_logger(), "max_slope_angle: %f", max_slope_angle_);
        RCLCPP_INFO(this->get_logger(), "use_gpu: %s", use_gpu_ ? "true" : "false");
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received point cloud message");

        // 指定のTFフレームに転写
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        try
        {
            geometry_msgs::msg::TransformStamped transform_stamped 
                = tf_buffer_->lookupTransform("base_link", msg->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(*msg, transformed_cloud, transform_stamped);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_cloud, *cloud);

        if (cloud->points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Could not read point cloud data cloud size: %lu", cloud->points.size());
            return;
        }

        // ダウンサンプリング処理
        RCLCPP_DEBUG(this->get_logger(), "Starting downsampling");
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud 
            = downsamplePointCloud(cloud, voxel_leaf_size_, use_gpu_, this->get_logger());
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

        // 法線推定と傾斜の判定
        RCLCPP_DEBUG(this->get_logger(), "Starting normal estimation");
        pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormals(body_removed_cloud, this->get_logger());
        RCLCPP_DEBUG(this->get_logger(), "Normal estimation completed");

        RCLCPP_DEBUG(this->get_logger(), "Starting obstacle filtering");
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud 
            = filterObstacles(body_removed_cloud, normals, max_slope_angle_, this->get_logger());
        RCLCPP_DEBUG(this->get_logger(), "Obstacle filtering completed");

        // フィルタリング後の点群をパブリッシュ
        RCLCPP_DEBUG(this->get_logger(), "Publishing filtered point cloud");
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);

        filtered_msg.header.frame_id = "base_link";
        filtered_cloud_publisher_->publish(filtered_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleCloudToScanNode>());
    rclcpp::shutdown();
    return 0;
}

