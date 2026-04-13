#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

class MainCommandNode : public rclcpp::Node
{
public:
    MainCommandNode() : Node("main_command_node")
    {
        // 1. Declare YAML parameters for the home pose
        this->declare_parameter<std::vector<double>>("home_pose.xyz", {0.185, 0.0, 0.245});
        this->declare_parameter<std::vector<double>>("home_pose.rpy", {0.0, 3.14159, 0.0});

        // 2. Initialize Publishers
        pub_target_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/target_poses", 10);
        pub_start_ = this->create_publisher<std_msgs::msg::Bool>("/start", 10);
        pub_clear_targets_ = this->create_publisher<std_msgs::msg::Bool>("/clear_targets", 10);
        pub_attach_signal_ = this->create_publisher<std_msgs::msg::Bool>("/object_attach_signal", 10);
        pub_get_trajectory_ = this->create_publisher<std_msgs::msg::Empty>("/get_trajectory", 10);

        // 3. Initialize Subscriptions
        sub_main_cmd_ = this->create_subscription<std_msgs::msg::String>(
            "/main_cmd", 10, std::bind(&MainCommandNode::mainCmdCallback, this, std::placeholders::_1));

        sub_pick_point_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/pick_point", 10, std::bind(&MainCommandNode::pickPointCallback, this, std::placeholders::_1));

        sub_opt_status_ = this->create_subscription<std_msgs::msg::String>(
            "/optimization_status", 10, std::bind(&MainCommandNode::optimizationStatusCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Advanced Main Command Node is ready.");
    }

    ~MainCommandNode()
    {
        // Clean up thread on shutdown
        if (task_thread_.joinable())
        {
            task_thread_.join();
        }
    }

private:
    // --- PUBLISHERS & SUBSCRIBERS ---
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_target_poses_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_start_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_clear_targets_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_attach_signal_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_get_trajectory_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_main_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_pick_point_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_opt_status_;

    // --- THREADING & SYNCHRONIZATION VARIABLES ---
    std::thread task_thread_;
    std::atomic<bool> is_task_running_{false};

    std::mutex data_mutex_;
    std::condition_variable cv_;

    // Flags for our waiting functions
    bool flag_pick_point_received_ = false;
    bool flag_optimization_finished_ = false;
    bool last_optimization_success_ = false; // Added to track actual success/failure

    geometry_msgs::msg::Point current_pick_point_;

    // =========================================================================
    // CALLBACKS (These run in the ROS Executor thread)
    // =========================================================================

    void mainCmdCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (is_task_running_)
        {
            RCLCPP_WARN(this->get_logger(), "A task is already running. Ignoring command: %s", msg->data.c_str());
            return;
        }

        // TASK ROUTER
        if (msg->data == "picking task")
        {
            startTaskThread(&MainCommandNode::pickAndPlaceTask);
        }
        else if (msg->data == "home task")
        {
            startTaskThread(&MainCommandNode::goHomeTask);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", msg->data.c_str());
        }
    }

    void pickPointCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_pick_point_ = *msg;
        flag_pick_point_received_ = true;
        cv_.notify_all(); // Wake up any sleeping task threads
    }

    void optimizationStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string status = msg->data;

        // Only trigger the pipeline to continue if we get a terminal state (Success or Failed)
        if (status == "Success" || status.find("Failed") != std::string::npos)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            flag_optimization_finished_ = true;
            last_optimization_success_ = (status == "Success"); // Store true if Success, false if Failed
            cv_.notify_all();                                   // Wake up any sleeping task threads
        }
        else
        {
            // Log intermediate states but DO NOT wake up the thread
            RCLCPP_INFO(this->get_logger(), "Planner is busy: %s", status.c_str());
        }
    }

    // =========================================================================
    // HELPER PIPELINE FUNCTIONS (Famous Functions)
    // =========================================================================

    // Helper 1: Trigger Trajectory Calculation and Wait for Pick Point
    bool waitForTargetPoint(geometry_msgs::msg::Point &out_point)
    {
        std::unique_lock<std::mutex> lock(data_mutex_);

        // Reset flag before waiting
        flag_pick_point_received_ = false;

        // 1. Send the trigger to the Python node
        RCLCPP_INFO(this->get_logger(), "Publishing trigger to /get_trajectory...");
        pub_get_trajectory_->publish(std_msgs::msg::Empty());

        // 2. Wait for the Python node to compute and reply on /pick_point
        RCLCPP_INFO(this->get_logger(), "Waiting for /pick_point response...");
        cv_.wait(lock, [this]()
                 { return flag_pick_point_received_ || !rclcpp::ok(); });

        if (!rclcpp::ok())
            return false;

        out_point = current_pick_point_;
        return true;
    }

    // Helper 2: The famous Motion Pipeline (Send -> Start -> Wait -> Clear)
    bool executeMotionPipeline(const geometry_msgs::msg::PoseArray &target_poses)
    {
        // 1. Send Poses
        pub_target_poses_->publish(target_poses);

        // 2. Start
        std_msgs::msg::Bool true_msg;
        true_msg.data = true;
        pub_start_->publish(true_msg);

        // 3. Wait for finish
        std::unique_lock<std::mutex> lock(data_mutex_);
        flag_optimization_finished_ = false; // Reset flag
        last_optimization_success_ = false;  // Reset success state

        RCLCPP_INFO(this->get_logger(), "Waiting for terminal status (/optimization_status)...");
        cv_.wait(lock, [this]()
                 { return flag_optimization_finished_ || !rclcpp::ok(); });

        if (!rclcpp::ok())
            return false;

        // Check if the planner actually succeeded
        if (!last_optimization_success_)
        {
            RCLCPP_ERROR(this->get_logger(), "Motion pipeline aborted due to planner failure.");
            // We should still clear the bad targets from the planner's buffer!
            pub_clear_targets_->publish(true_msg);
            return false;
        }

        // 4. Clear
        pub_clear_targets_->publish(true_msg);
        return true;
    }

    // Helper 3: Tool Attachment
    void setToolAttachment(bool attach)
    {
        std_msgs::msg::Bool msg;
        msg.data = attach;
        pub_attach_signal_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Tool attachment set to: %s", attach ? "TRUE" : "FALSE");
    }

    // Helper 4: Generate a single PoseArray from a Point
    geometry_msgs::msg::PoseArray createPoseArrayFromPoint(const geometry_msgs::msg::Point &pt, double z_offset)
    {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = this->now();
        pose_array.header.frame_id = "world";

        geometry_msgs::msg::Pose pose;
        pose.position.x = pt.x;
        pose.position.y = pt.y;
        pose.position.z = pt.z + z_offset;

        // Default pointing straight down
        pose.orientation.x = 0.0;
        pose.orientation.y = 1.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;

        pose_array.poses.push_back(pose);
        return pose_array;
    }

    // Helper 5: Load Home Pose from YAML
    geometry_msgs::msg::PoseArray getHomePose()
    {
        std::vector<double> home_xyz = this->get_parameter("home_pose.xyz").as_double_array();
        std::vector<double> home_rpy = this->get_parameter("home_pose.rpy").as_double_array();

        geometry_msgs::msg::PoseArray home_pose_array;
        home_pose_array.header.stamp = this->now();
        home_pose_array.header.frame_id = "world";

        geometry_msgs::msg::Pose home_pose;
        home_pose.position.x = home_xyz[0];
        home_pose.position.y = home_xyz[1];
        home_pose.position.z = home_xyz[2];

        tf2::Quaternion q;
        q.setRPY(home_rpy[0], home_rpy[1], home_rpy[2]);
        home_pose.orientation.x = q.x();
        home_pose.orientation.y = q.y();
        home_pose.orientation.z = q.z();
        home_pose.orientation.w = q.w();

        home_pose_array.poses.push_back(home_pose);
        return home_pose_array;
    }

    // =========================================================================
    // MAIN TASK FUNCTIONS
    // =========================================================================

    // Utility to launch tasks safely
    void startTaskThread(void (MainCommandNode::*taskFunction)())
    {
        if (task_thread_.joinable())
        {
            task_thread_.join();
        }
        is_task_running_ = true;
        task_thread_ = std::thread([this, taskFunction]()
                                   {
            (this->*taskFunction)(); // Execute the specific task
            is_task_running_ = false;
            RCLCPP_INFO(this->get_logger(), "Task finished. Back to IDLE."); });
    }

    // TASK 1: The Pick and Place Pipeline
    void pickAndPlaceTask()
    {
        RCLCPP_INFO(this->get_logger(), "--- Starting Pick and Place Task ---");

        // 1. Wait for trajectory and point
        geometry_msgs::msg::Point target_point;
        if (!waitForTargetPoint(target_point))
            return;

        // 2. Move to Pick Point (with 0.001 Z offset)
        RCLCPP_INFO(this->get_logger(), "Executing motion to Pick Point");
        geometry_msgs::msg::PoseArray pick_poses = createPoseArrayFromPoint(target_point, 0.001);
        if (!executeMotionPipeline(pick_poses))
            return; // If it failed, abort the task!

        // 3. Attach Tool (e.g., turn on suction/gripper)
        setToolAttachment(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Optional small delay

        // 4. Move Home
        RCLCPP_INFO(this->get_logger(), "Executing motion to Home");
        geometry_msgs::msg::PoseArray home_poses = getHomePose();
        if (!executeMotionPipeline(home_poses))
            return; // If it failed, abort the task!

        // 5. Release Tool
        setToolAttachment(false);
    }

    // TASK 2: Simple Go Home
    void goHomeTask()
    {
        RCLCPP_INFO(this->get_logger(), "--- Starting Go Home Task ---");
        geometry_msgs::msg::PoseArray home_poses = getHomePose();
        executeMotionPipeline(home_poses);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainCommandNode>());
    rclcpp::shutdown();
    return 0;
}