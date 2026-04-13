#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cstdlib>
#include <string>
#include <cmath>

class ObjectController : public rclcpp::Node
{
public:
    ObjectController() : Node("object_controller")
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/object/set_pose", 10,
            std::bind(&ObjectController::pose_callback, this, std::placeholders::_1));

        mesh_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/object/set_mesh", 10,
            std::bind(&ObjectController::mesh_callback, this, std::placeholders::_1));

        gravity_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/object/set_gravity", 10,
            std::bind(&ObjectController::gravity_callback, this, std::placeholders::_1));

        // Default position
        // Default position
        current_pose_.position.x = -0.21;
        current_pose_.position.y = -0.21;
        current_pose_.position.z = 1.06;

        // Convert Euler → Quaternion
        tf2::Quaternion q;
        q.setRPY(1.57, 0.0, 0.0); // roll=1.57, pitch=0, yaw=0
        q.normalize();

        current_pose_.orientation = tf2::toMsg(q);

        respawn_object();

        RCLCPP_INFO(this->get_logger(), "Object Controller Ready (Euler Control Enabled)");
    }

private:
    std::string current_mesh_ = "package://motomini/meshes/puzzel.stl";
    bool gravity_off_ = true;
    geometry_msgs::msg::Pose current_pose_;

    void respawn_object()
    {
        RCLCPP_INFO(this->get_logger(), "Respawning object...");

        system("ign service -s /world/empty/remove "
               "--reqtype ignition.msgs.Entity "
               "--reptype ignition.msgs.Boolean "
               "--timeout 1000 "
               "--req 'name: \"target_object\" type: MODEL' "
               "> /dev/null 2>&1");

        std::string grav_str = gravity_off_ ? "true" : "false";

        std::string cmd =
            "xacro $(ros2 pkg prefix motomini)/share/motomini/urdf/custom_object.urdf.xacro "
            "mesh_path:=" +
            current_mesh_ +
            " gravity_off:=" + grav_str +
            " > /tmp/target_object.urdf && "
            "ros2 run ros_gz_sim create -name target_object "
            "-file /tmp/target_object.urdf "
            "-x " +
            std::to_string(current_pose_.position.x) +
            " -y " + std::to_string(current_pose_.position.y) +
            " -z " + std::to_string(current_pose_.position.z);

        system(cmd.c_str());
    }

    void mesh_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_mesh_ = msg->data;
        respawn_object();
    }

    void gravity_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        gravity_off_ = !(msg->data);
        respawn_object();
    }

    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;

        std::string req =
            "'name: \"target_object\", "
            "position: {x: " +
            std::to_string(msg->position.x) +
            ", y: " + std::to_string(msg->position.y) +
            ", z: " + std::to_string(msg->position.z) +
            "}, orientation: {x: " + std::to_string(msg->orientation.x) +
            ", y: " + std::to_string(msg->orientation.y) +
            ", z: " + std::to_string(msg->orientation.z) +
            ", w: " + std::to_string(msg->orientation.w) + "}'";

        std::string cmd =
            "ign service -s /world/empty/set_pose "
            "--reqtype ignition.msgs.Pose "
            "--reptype ignition.msgs.Boolean "
            "--timeout 1000 --req " +
            req +
            " > /dev/null 2>&1";

        system(cmd.c_str());
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mesh_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gravity_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectController>());
    rclcpp::shutdown();
    return 0;
}