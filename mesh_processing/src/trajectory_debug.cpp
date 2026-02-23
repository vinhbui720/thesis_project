#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <iostream>

class TrajectoryDebug : public rclcpp::Node
{
public:
    TrajectoryDebug() : Node("trajectory_debug")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Subscribe to aligned_model and processed_scene to debug frames
        aligned_model_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/aligned_model", 1,
            std::bind(&TrajectoryDebug::aligned_model_callback, this, std::placeholders::_1));

        processed_scene_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/processed_scene", 1,
            std::bind(&TrajectoryDebug::processed_scene_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Trajectory Debug Node Started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_model_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr processed_scene_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    int aligned_count_ = 0;
    int scene_count_ = 0;

    void aligned_model_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (aligned_count_ % 30 == 0) // Print every 30 frames
        {
            RCLCPP_INFO(this->get_logger(),
                        "[ALIGNED_MODEL] Frame: %s | Points: %u | Timestamp: %ld.%ld",
                        msg->header.frame_id.c_str(),
                        (unsigned)(msg->width * msg->height),
                        msg->header.stamp.sec,
                        msg->header.stamp.nanosec);

            // Get first point
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(*msg, cloud);
            if (!cloud.empty())
            {
                auto p = cloud[0];
                RCLCPP_INFO(this->get_logger(), "  First point: [%.4f, %.4f, %.4f]", p.x, p.y, p.z);
            }

            // List all available TF frames
            print_all_frames();
        }
        aligned_count_++;
    }

    void processed_scene_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (scene_count_ % 30 == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[SCENE] Frame: %s | Points: %lu",
                        msg->header.frame_id.c_str(),
                        msg->width * msg->height);
        }
        scene_count_++;
    }

    void print_all_frames()
    {
        try
        {
            auto frame_names = tf_buffer_->getAllFrameNames();
            RCLCPP_INFO(this->get_logger(), "=== TF FRAMES ===");
            for (const auto &frame : frame_names)
            {
                RCLCPP_INFO(this->get_logger(), "  - %s", frame.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get frames: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryDebug>());
    rclcpp::shutdown();
    return 0;
}
