#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ObjectTfBroadcaster : public rclcpp::Node
{
public:
    ObjectTfBroadcaster() : Node("object_tf_broadcaster"), is_attached_(false)
    {
        this->declare_parameter<std::string>("ee_link", "gripper_link");
        this->declare_parameter<bool>("real_robot", false);

        ee_link_ = this->get_parameter("ee_link").as_string();
        real_robot_ = this->get_parameter("real_robot").as_bool();

        enable_client_ = this->create_client<std_srvs::srv::Trigger>("/tool_enable");
        disable_client_ = this->create_client<std_srvs::srv::Trigger>("/tool_disable");

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Initialize TF Buffer and Listener to read the tool's position
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        current_pose_.pose.position.x = 0.5;
        current_pose_.pose.position.y = 0.0;
        current_pose_.pose.position.z = 0.1;
        current_pose_.pose.orientation.w = 1.0;

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_object_pose", 10,
            std::bind(&ObjectTfBroadcaster::pose_callback, this, std::placeholders::_1));

        signal_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/object_attach_signal", 10,
            std::bind(&ObjectTfBroadcaster::signal_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&ObjectTfBroadcaster::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Object TF Broadcaster Started.");
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
    }

    void signal_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !is_attached_)
        {
            RCLCPP_INFO(this->get_logger(), "Calculating relative offset to %s...", ee_link_.c_str());

            try
            {
                // Look up transform from the Tool to the World
                geometry_msgs::msg::TransformStamped t_ee_to_world =
                    tf_buffer_->lookupTransform(ee_link_, "world", tf2::TimePointZero);

                // Convert ROS messages to TF2 math objects
                tf2::Transform tf_ee_to_world;
                tf2::fromMsg(t_ee_to_world.transform, tf_ee_to_world);

                tf2::Transform tf_world_to_obj;
                tf_world_to_obj.setOrigin(tf2::Vector3(
                    current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z));
                tf_world_to_obj.setRotation(tf2::Quaternion(
                    current_pose_.pose.orientation.x, current_pose_.pose.orientation.y,
                    current_pose_.pose.orientation.z, current_pose_.pose.orientation.w));

                // MATH: Multiply to get the relative offset
                tf2::Transform tf_ee_to_obj = tf_ee_to_world * tf_world_to_obj;

                // Save this offset for the timer callback
                relative_offset_.translation.x = tf_ee_to_obj.getOrigin().x();
                relative_offset_.translation.y = tf_ee_to_obj.getOrigin().y();
                relative_offset_.translation.z = tf_ee_to_obj.getOrigin().z();
                relative_offset_.rotation = tf2::toMsg(tf_ee_to_obj.getRotation());

                is_attached_ = true;
                RCLCPP_INFO(this->get_logger(), "Successfully attached with maintained distance.");

                if (real_robot_)
                {
                    call_service(enable_client_, "Enable");
                }
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Could not attach: %s", ex.what());
            }
        }
        else if (!msg->data && is_attached_)
        {
            is_attached_ = false;
            RCLCPP_INFO(this->get_logger(), "Detach signal received. Returning object to world frame.");
            if (real_robot_)
            {
                call_service(disable_client_, "Disable");
            }
        }
    }

    void call_service(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, const std::string &action_name)
    {
        if (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "/robot_%s service not available.", action_name.c_str());
            return;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        client->async_send_request(request);
    }

    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.child_frame_id = "object_link";

        if (is_attached_)
        {
            // Publish using the saved relative distance
            t.header.frame_id = ee_link_;
            t.transform = relative_offset_;
        }
        else
        {
            // Publish normal world pose
            t.header.frame_id = "world";
            t.transform.translation.x = current_pose_.pose.position.x;
            t.transform.translation.y = current_pose_.pose.position.y;
            t.transform.translation.z = current_pose_.pose.position.z;
            t.transform.rotation = current_pose_.pose.orientation;
        }

        tf_broadcaster_->sendTransform(t);
    }

    std::string ee_link_;
    bool real_robot_;
    bool is_attached_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr signal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enable_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr disable_client_;

    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Transform relative_offset_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}