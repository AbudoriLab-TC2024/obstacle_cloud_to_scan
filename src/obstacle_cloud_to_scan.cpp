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

        // Best Effort QoS設定
        //auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
        auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_,rclcpp::QoS(rclcpp::KeepLast(10)));
        //filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, reliable_qos);
        //filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, best_effort_qos);
        RCLCPP_DEBUG(this->get_logger(), "Publisher created for topic: %s", output_topic_.c_str());
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;

    std::string target_frame_;
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

    }

    void get_parameters()
    {
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->get_parameter("robot_box_size", robot_box_size_);
        this->get_parameter("robot_box_position", robot_box_position_);
        this->get_parameter("max_slope_angle", max_slope_angle_);
        this->get_parameter("use_gpu", use_gpu_);

        RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
        RCLCPP_INFO(this->get_logger(), "target_frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "voxel_leaf_size: %f", voxel_leaf_size_);
        RCLCPP_INFO(this->get_logger(), "max_slope_angle: %f", max_slope_angle_);
        RCLCPP_INFO(this->get_logger(), "use_gpu: %s", use_gpu_ ? "true" : "false");
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto callback_start_time = std::chrono::high_resolution_clock::now(); // 計測開始
        RCLCPP_DEBUG(this->get_logger(), "Received point cloud message");

        // 指定のTFフレームに転写
        //sensor_msgs::msg::PointCloud2 transformed_cloud;
        // PCLでTransformした場合
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
        // トランスフォームをEigenに変換
        Eigen::Affine3d transform = tf2::transformToEigen(transform_stamped.transform);

        // sensor_msgs::PointCloud2 を pcl::PointCloud に変換
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // PCLで点群を変換
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

        // doTransformでした場合
        /*
        try
        {
            geometry_msgs::msg::TransformStamped transform_stamped 
                = tf_buffer_->lookupTransform("base_link", msg->header.frame_id, tf2::TimePointZero);

            auto tf_receive_end_time = std::chrono::high_resolution_clock::now(); // 計測終了
            std::chrono::duration<double, std::milli> tf_receive_elapsed 
                        = tf_receive_end_time - callback_start_time;
            RCLCPP_INFO(this->get_logger(), "tf receive took %.2f ms", tf_receive_elapsed.count());

            tf2::doTransform(*msg, transformed_cloud, transform_stamped);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
            return;
        }
        */

        auto tf_transforme_end_time = std::chrono::high_resolution_clock::now(); // 計測終了
        std::chrono::duration<double, std::milli> tf_transforme_elapsed 
                        = tf_transforme_end_time - callback_start_time;
        RCLCPP_INFO(this->get_logger(), "tf fransforme took %.2f ms", tf_transforme_elapsed.count());

/*
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_cloud, *cloud);

        if (cloud->points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Could not read point cloud data cloud size: %lu", cloud->points.size());
            return;
        }
        */

        // ダウンサンプリング処理
        RCLCPP_DEBUG(this->get_logger(), "Starting downsampling");
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud 
            = downsamplePointCloud(transformed_cloud, voxel_leaf_size_, use_gpu_, this->get_logger());
        RCLCPP_DEBUG(this->get_logger(), "Downsampling completed");

        auto downsampling_end_time = std::chrono::high_resolution_clock::now(); // 計測終了
        std::chrono::duration<double, std::milli> downsampling_elapsed 
                        = downsampling_end_time - callback_start_time;
        RCLCPP_INFO(this->get_logger(), "downsampling took %.2f ms", downsampling_elapsed.count());

        // パススルーフィルタの適用
        RCLCPP_DEBUG(this->get_logger(), "Starting passthrough filter");
        pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud 
            = applyPassThroughFilter(downsampled_cloud, robot_box_size_, this->get_logger());
        RCLCPP_DEBUG(this->get_logger(), "Passthrough filter completed");
        
        auto passthrough_end_time = std::chrono::high_resolution_clock::now(); // 計測終了
        std::chrono::duration<double, std::milli> passthrough_elapsed 
                        = passthrough_end_time - callback_start_time;
        RCLCPP_INFO(this->get_logger(), "passthrough took %.2f ms", passthrough_elapsed.count());


        // ロボットの体の除去
        RCLCPP_DEBUG(this->get_logger(), "Removing robot body from point cloud");
        pcl::PointCloud<pcl::PointXYZ>::Ptr body_removed_cloud 
            = removeRobotBody(passthrough_cloud, robot_box_position_, robot_box_size_, this->get_logger());
        RCLCPP_DEBUG(this->get_logger(), "Robot body removal completed");

        auto removeRobotBody_end_time = std::chrono::high_resolution_clock::now(); // 計測終了
        std::chrono::duration<double, std::milli> removeRobotBody_elapsed 
                        = removeRobotBody_end_time - callback_start_time;
        RCLCPP_INFO(this->get_logger(), "removeRobotBody took %.2f ms", removeRobotBody_elapsed.count());

        // 法線推定と傾斜の判定
        RCLCPP_DEBUG(this->get_logger(), "Starting normal estimation");
        pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormals(body_removed_cloud, this->get_logger());
        RCLCPP_DEBUG(this->get_logger(), "Normal estimation completed");

        auto normals_end_time = std::chrono::high_resolution_clock::now(); // 計測終了
        std::chrono::duration<double, std::milli> normals_elapsed 
                        = normals_end_time - callback_start_time;
        RCLCPP_INFO(this->get_logger(), "normals took %.2f ms", normals_elapsed.count());

        // 法線から不要な点群を除去
        RCLCPP_DEBUG(this->get_logger(), "Starting obstacle filtering");
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud 
            = filterObstacles(body_removed_cloud, normals, max_slope_angle_, this->get_logger());
        RCLCPP_DEBUG(this->get_logger(), "Obstacle filtering completed");

        auto filterObstacles_end_time = std::chrono::high_resolution_clock::now(); // 計測終了
        std::chrono::duration<double, std::milli> filterObstacles_elapsed 
                        = filterObstacles_end_time - callback_start_time;
        RCLCPP_INFO(this->get_logger(), "filterObstacles took %.2f ms", filterObstacles_elapsed.count());


        // フィルタリング後の点群をパブリッシュ
        RCLCPP_DEBUG(this->get_logger(), "Publishing filtered point cloud");
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);

        filtered_msg.header.frame_id = "base_link";
        filtered_msg.header.stamp = this->get_clock()->now();
        filtered_cloud_publisher_->publish(filtered_msg);
        
        auto callback_end_time = std::chrono::high_resolution_clock::now(); // 計測終了
        std::chrono::duration<double, std::milli> callback_elapsed 
                        = callback_end_time - callback_start_time;
        RCLCPP_INFO(this->get_logger(), "publish took %.2f ms", callback_elapsed.count());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleCloudToScanNode>());
    rclcpp::shutdown();
    return 0;
}

