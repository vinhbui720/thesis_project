#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <json/json.h>
#include <fstream>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

class TrajectoryVisualizer : public rclcpp::Node
{
public:
    TrajectoryVisualizer() : Node("trajectory_visualizer"),
                             tf_buffer_(this->get_clock()),
                             tf_listener_(tf_buffer_)
    {
        // Declare parameters
        this->declare_parameter("json_file_path", "surface_plan.json");
        this->declare_parameter("publish_rate", 1.0); // Hz

        // Get the mesh_processing package config directory
        std::string pkg_share = ament_index_cpp::get_package_share_directory("mesh_processing");
        std::string config_dir = pkg_share + "/config";
        std::string json_path = config_dir + "/surface_plan.json";

        // Publishers
        pick_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/trajectory/pick_point", 10);

        trajectory_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/trajectory/points", 10);

        trajectory_line_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/trajectory/line", 10);

        // Subscribe to aligned_model to get the correct frame_id and timestamp
        aligned_model_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/aligned_model", 10,
            std::bind(&TrajectoryVisualizer::aligned_model_callback, this, std::placeholders::_1));

        // Timer for periodic publishing
        double rate = this->get_parameter("publish_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&TrajectoryVisualizer::publish_callback, this));

        // Load trajectory from JSON
        load_trajectory_from_json(json_path);

        RCLCPP_INFO(this->get_logger(),
                    "Trajectory Visualizer Node Started. Loading from: %s", json_path.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "Waiting for /aligned_model and TF to transform coordinates...");
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pick_point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_line_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_model_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF2 members for coordinate transformation
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Original trajectory in camera frame
    std::vector<std::array<double, 3>> trajectory_waypoints_original_;
    std::array<double, 3> pick_point_original_;

    // Transformed trajectory in estimated_object_frame
    std::vector<std::array<double, 3>> trajectory_waypoints_transformed_;
    std::array<double, 3> pick_point_transformed_;

    bool has_pick_point_ = false;
    bool has_trajectory_ = false;
    bool has_transform_ = false;

    std::string frame_id_ = "estimated_object_frame"; // Frame where aligned model is
    std::string camera_frame_ = "model_frame";        // Original frame (trajectory is in model_frame)
    rclcpp::Time latest_stamp_;
    std::mutex frame_mutex_;
    int frame_callback_count_ = 0;

    // ============================================================
    // Callback to get frame_id and timestamp from aligned_model
    // ============================================================
    void aligned_model_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        frame_id_ = msg->header.frame_id;
        latest_stamp_ = msg->header.stamp;

        // Debug info every 30 frames
        if (frame_callback_count_ % 30 == 0)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "[ALIGNED_MODEL] frame_id: %s | points: %u",
                         frame_id_.c_str(), (unsigned)(msg->width * msg->height));
        }
        frame_callback_count_++;

        // Try to update transform
        if (!has_transform_)
        {
            update_trajectory_transform();
        }
    }

    // ============================================================
    // Transform trajectory coordinates from camera to object frame
    // ============================================================
    void update_trajectory_transform()
    {
        try
        {
            // Look up the transform from camera_frame to estimated_object_frame
            auto transform = tf_buffer_.lookupTransform(
                frame_id_,         // target frame (estimated_object_frame)
                camera_frame_,     // source frame (camera_depth_optical_frame)
                tf2::TimePointZero // get most recent transform
            );

            RCLCPP_INFO(this->get_logger(),
                        "Transform found: %s -> %s", camera_frame_.c_str(), frame_id_.c_str());

            // Extract rotation and translation
            Eigen::Quaterniond quat(
                transform.transform.rotation.w,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z);
            Eigen::Matrix3d rotation = quat.toRotationMatrix();
            Eigen::Vector3d translation(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z);

            // Transform pick point
            if (has_pick_point_)
            {
                Eigen::Vector3d pick_cam(
                    pick_point_original_[0],
                    pick_point_original_[1],
                    pick_point_original_[2]);
                Eigen::Vector3d pick_obj = rotation * pick_cam + translation;
                pick_point_transformed_[0] = pick_obj(0);
                pick_point_transformed_[1] = pick_obj(1);
                pick_point_transformed_[2] = pick_obj(2);

                RCLCPP_INFO(this->get_logger(),
                            "Pick point transformed: [%.4f, %.4f, %.4f] -> [%.4f, %.4f, %.4f]",
                            pick_point_original_[0], pick_point_original_[1], pick_point_original_[2],
                            pick_point_transformed_[0], pick_point_transformed_[1], pick_point_transformed_[2]);
            }

            // Transform trajectory waypoints
            if (has_trajectory_)
            {
                trajectory_waypoints_transformed_.clear();
                for (const auto &wp_cam : trajectory_waypoints_original_)
                {
                    Eigen::Vector3d wp_eigen(wp_cam[0], wp_cam[1], wp_cam[2]);
                    Eigen::Vector3d wp_obj = rotation * wp_eigen + translation;
                    trajectory_waypoints_transformed_.push_back({wp_obj(0), wp_obj(1), wp_obj(2)});
                }

                RCLCPP_INFO(this->get_logger(),
                            "Transformed %lu trajectory waypoints",
                            trajectory_waypoints_transformed_.size());
            }

            has_transform_ = true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Transform not yet available: %s", ex.what());
        }
    }

    // ============================================================
    // Load trajectory from JSON file
    // ============================================================
    void load_trajectory_from_json(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            RCLCPP_WARN(this->get_logger(),
                        "Could not open JSON file: %s", filename.c_str());
            return;
        }

        Json::Value root;
        file >> root;
        file.close();

        // Load frame_id from JSON (defaults to model_frame if not specified)
        if (root.isMember("frame_id") && root["frame_id"].isString())
        {
            camera_frame_ = root["frame_id"].asString();
            RCLCPP_INFO(this->get_logger(),
                        "Loaded frame_id from JSON: %s", camera_frame_.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "No frame_id in JSON, using default: %s", camera_frame_.c_str());
        }

        // Load pick point (store original in model frame)
        if (root.isMember("pick_point") && root["pick_point"].isArray() &&
            root["pick_point"].size() == 3)
        {
            pick_point_original_[0] = root["pick_point"][0].asDouble();
            pick_point_original_[1] = root["pick_point"][1].asDouble();
            pick_point_original_[2] = root["pick_point"][2].asDouble();
            pick_point_transformed_ = pick_point_original_; // will be updated after transform
            has_pick_point_ = true;

            RCLCPP_INFO(this->get_logger(),
                        "Loaded pick point (%s frame): [%.4f, %.4f, %.4f]",
                        camera_frame_.c_str(),
                        pick_point_original_[0], pick_point_original_[1], pick_point_original_[2]);
        }

        // Load trajectory (store original in model frame)
        if (root.isMember("trajectory") && root["trajectory"].isArray())
        {
            trajectory_waypoints_original_.clear();
            for (const auto &pt : root["trajectory"])
            {
                if (pt.isArray() && pt.size() == 3)
                {
                    std::array<double, 3> waypoint = {
                        pt[0].asDouble(),
                        pt[1].asDouble(),
                        pt[2].asDouble()};
                    trajectory_waypoints_original_.push_back(waypoint);
                }
            }
            trajectory_waypoints_transformed_ = trajectory_waypoints_original_; // will be updated after transform
            has_trajectory_ = (trajectory_waypoints_original_.size() > 0);

            RCLCPP_INFO(this->get_logger(),
                        "Loaded %lu trajectory waypoints (%s frame)",
                        trajectory_waypoints_original_.size(),
                        camera_frame_.c_str());
        }
    }

    // ============================================================
    // Publish callback
    // ============================================================
    void publish_callback()
    {
        // Publish pick point as large RED sphere
        if (has_pick_point_)
        {
            publish_pick_point();
        }

        // Publish trajectory waypoints as small GREEN spheres
        if (has_trajectory_)
        {
            publish_trajectory_points();
            publish_trajectory_line();
        }
    }

    // ============================================================
    // Publish pick point marker (large red sphere)
    // ============================================================
    void publish_pick_point()
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = latest_stamp_;
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position (use transformed coordinates)
        marker.pose.position.x = pick_point_transformed_[0];
        marker.pose.position.y = pick_point_transformed_[1];
        marker.pose.position.z = pick_point_transformed_[2];
        marker.pose.orientation.w = 1.0;

        // Scale (large sphere)
        marker.scale.x = 0.05; // 5 cm diameter
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        // Color (RED)
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;

        marker.lifetime = rclcpp::Duration(0, 0); // Persistent

        static int pick_publish_count = 0;
        if (pick_publish_count % 30 == 0)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "[PICK POINT] frame_id: %s | pos: [%.4f, %.4f, %.4f] | has_transform: %s",
                         frame_id_.c_str(),
                         pick_point_transformed_[0], pick_point_transformed_[1], pick_point_transformed_[2],
                         has_transform_ ? "true" : "false");
        }
        pick_publish_count++;

        pick_point_pub_->publish(marker);
    }

    // ============================================================
    // Publish trajectory waypoints (small green spheres)
    // ============================================================
    void publish_trajectory_points()
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);

        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < trajectory_waypoints_transformed_.size(); ++i)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = latest_stamp_;
            marker.id = i + 1; // ID 0 is reserved for pick point
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Position (use transformed coordinates)
            marker.pose.position.x = trajectory_waypoints_transformed_[i][0];
            marker.pose.position.y = trajectory_waypoints_transformed_[i][1];
            marker.pose.position.z = trajectory_waypoints_transformed_[i][2];
            marker.pose.orientation.w = 1.0;

            // Scale (small spheres)
            marker.scale.x = 0.01; // 1 cm diameter
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;

            // Color (GREEN)
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.8f;

            marker.lifetime = rclcpp::Duration(0, 0);

            marker_array.markers.push_back(marker);
        }

        static int traj_publish_count = 0;
        if (traj_publish_count % 30 == 0)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "[TRAJECTORY] frame_id: %s | waypoints: %lu | has_transform: %s",
                         frame_id_.c_str(),
                         trajectory_waypoints_transformed_.size(),
                         has_transform_ ? "true" : "false");
            if (trajectory_waypoints_transformed_.size() > 0)
            {
                RCLCPP_DEBUG(this->get_logger(),
                             "[TRAJECTORY] first waypoint transformed: [%.4f, %.4f, %.4f]",
                             trajectory_waypoints_transformed_[0][0],
                             trajectory_waypoints_transformed_[0][1],
                             trajectory_waypoints_transformed_[0][2]);
            }
        }
        traj_publish_count++;

        trajectory_pub_->publish(marker_array);
    }

    // ============================================================
    // Publish trajectory as a line
    // ============================================================
    void publish_trajectory_line()
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);

        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = frame_id_;
        line_marker.header.stamp = latest_stamp_;
        line_marker.id = 9999; // Special ID for line
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;

        line_marker.pose.orientation.w = 1.0;

        // Scale (line width)
        line_marker.scale.x = 0.002; // 2 mm line width

        // Color (GREEN)
        line_marker.color.r = 0.0f;
        line_marker.color.g = 1.0f;
        line_marker.color.b = 0.0f;
        line_marker.color.a = 1.0f;

        // Add all waypoints to the line (use transformed coordinates)
        for (const auto &waypoint : trajectory_waypoints_transformed_)
        {
            geometry_msgs::msg::Point p;
            p.x = waypoint[0];
            p.y = waypoint[1];
            p.z = waypoint[2];
            line_marker.points.push_back(p);
        }

        line_marker.lifetime = rclcpp::Duration(0, 0);

        trajectory_line_pub_->publish(line_marker);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryVisualizer>());
    rclcpp::shutdown();
    return 0;
}
