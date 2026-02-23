#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Ignition Gazebo includes
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

class GzPoseToRosBridge : public rclcpp::Node
{
public:
    GzPoseToRosBridge() : Node("gz_pose_to_ros_bridge")
    {
        // ROS 2 Publishers for Target Object
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_object_pose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/target_object_marker", 10);

        // ROS 2 Publisher for the Robot Visualization
        robot_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/robot_marker", 10);

        if (!gz_node_.Subscribe("/world/empty/dynamic_pose/info", &GzPoseToRosBridge::GzCallback, this))
        {
            RCLCPP_ERROR(this->get_logger(), "Error subscribing to Gazebo topic.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Listening for poses from Gazebo and bridging to ROS 2...");
        }
    }

private:
    void GzCallback(const gz::msgs::Pose_V &msg)
    {
        for (int i = 0; i < msg.pose_size(); ++i)
        {
            const auto &pose = msg.pose(i);

            // Extract position and orientation formulas
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "world";

            pose_msg.pose.position.x = pose.position().x();
            pose_msg.pose.position.y = pose.position().y();
            pose_msg.pose.position.z = pose.position().z();
            pose_msg.pose.orientation.x = pose.orientation().x();
            pose_msg.pose.orientation.y = pose.orientation().y();
            pose_msg.pose.orientation.z = pose.orientation().z();
            pose_msg.pose.orientation.w = pose.orientation().w();

            if (pose.name() == "target_object")
            {
                pose_pub_->publish(pose_msg);
                publish_marker(pose_msg, "target_object", "package://motomini/meshes/puzzel.stl", marker_pub_);
            }
            else if (pose.name() == "motomini" || pose.name() == "robot") // Adjust name to match your Gazebo model
            {
                // Visualize the robot base/arm in RViz
                // If you have a robot mesh, replace CUBE with MESH_RESOURCE and provide the path
                publish_marker(pose_msg, "robot_base", "", robot_marker_pub_, visualization_msgs::msg::Marker::CUBE);
            }
        }
    }

    void publish_marker(const geometry_msgs::msg::PoseStamped &pose_msg,
                        const std::string &ns,
                        const std::string &mesh_path,
                        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub,
                        int type = visualization_msgs::msg::Marker::MESH_RESOURCE)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = pose_msg.header;
        marker.ns = ns;
        marker.id = 0;
        marker.type = type;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose_msg.pose;

        // Apply the 0.001 scale for the mesh to match the URDF
        // Keep the cube scale at 0.2 meters for the robot placeholder
        marker.scale.x = (type == visualization_msgs::msg::Marker::CUBE) ? 0.2 : 0.001;
        marker.scale.y = (type == visualization_msgs::msg::Marker::CUBE) ? 0.2 : 0.001;
        marker.scale.z = (type == visualization_msgs::msg::Marker::CUBE) ? 0.2 : 0.001;

        if (type == visualization_msgs::msg::Marker::MESH_RESOURCE)
        {
            marker.mesh_resource = mesh_path;
            marker.mesh_use_embedded_materials = true;
        }

        // Color (White/Grey)
        marker.color.r = 0.8;
        marker.color.g = 0.8;
        marker.color.b = 0.8;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0);
        pub->publish(marker);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_marker_pub_;

    gz::transport::Node gz_node_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GzPoseToRosBridge>());
    rclcpp::shutdown();
    return 0;
}