#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.hpp>

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <string>

using namespace std::chrono_literals;

class EnvTFBroadcaster : public rclcpp::Node
{
public:
    EnvTFBroadcaster()
        : Node("env_tf_broadcaster")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        load_environment_yaml();

        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&EnvTFBroadcaster::publish_transforms, this));

        RCLCPP_INFO(this->get_logger(), "Environment TF broadcaster started");
    }

private:
    struct FrameData
    {
        std::string parent;
        std::string child;

        double x, y, z;
        double roll, pitch, yaw;
    };

    std::vector<FrameData> frames_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    void load_environment_yaml()
    {
        std::string pkg_path =
            ament_index_cpp::get_package_share_directory("motomini");

        std::string yaml_path =
            pkg_path + "/config/planning_env.yaml";

        YAML::Node config = YAML::LoadFile(yaml_path);

        auto frames = config["environment"]["frames"];

        for (auto it : frames)
        {
            FrameData f;

            std::string frame_name = it.first.as<std::string>();

            f.child = frame_name;

            f.parent = it.second["parent"].as<std::string>();

            auto xyz = it.second["pose"]["xyz"];
            auto rpy = it.second["pose"]["rpy"];

            f.x = xyz[0].as<double>();
            f.y = xyz[1].as<double>();
            f.z = xyz[2].as<double>();

            f.roll = rpy[0].as<double>();
            f.pitch = rpy[1].as<double>();
            f.yaw = rpy[2].as<double>();

            frames_.push_back(f);

            RCLCPP_INFO(
                this->get_logger(),
                "Loaded frame: %s -> %s",
                f.parent.c_str(),
                f.child.c_str());
        }
    }

    void publish_transforms()
    {
        auto now = this->get_clock()->now();

        for (auto &f : frames_)
        {
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = now;
            t.header.frame_id = f.parent;
            t.child_frame_id = f.child;

            t.transform.translation.x = f.x;
            t.transform.translation.y = f.y;
            t.transform.translation.z = f.z;

            tf2::Quaternion q;
            q.setRPY(f.roll, f.pitch, f.yaw);

            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(t);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<EnvTFBroadcaster>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}