#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <Eigen/Dense>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_rosutils/utils.h>

// Maximum allowed lead of command vs actual (rad). Prevents buffer runaway.
static constexpr double MAX_LEAD_RAD = 0.10;

// Seconds to wait before sending arm trigger (let roscore/bridge settle).
static constexpr double ARM_PRE_DELAY_S = 0.5;
// Seconds to wait after arm trigger before beginning stream.
static constexpr double ARM_POST_DELAY_S = 1.0;

class MotoMiniVelTrackingNode : public rclcpp::Node
{
public:
    MotoMiniVelTrackingNode()
        : rclcpp::Node("motomini_vel_tracking")
    {
        this->declare_parameter<std::string>("robot_description", "");
        this->declare_parameter<std::string>("robot_description_semantic", "");
        this->declare_parameter<std::string>("manipulator_group", "manipulator");
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("ee_link", "tool0");
        this->declare_parameter<double>("rate_hz", 25.0);
        this->declare_parameter<double>("theta_d_lim", 3.14);
        this->declare_parameter<double>("w0", 0.1);
        this->declare_parameter<double>("k0", 0.001);

        this->get_parameter("robot_description", urdf_xml_);
        this->get_parameter("robot_description_semantic", srdf_xml_);
        manipulator_group_ = this->get_parameter("manipulator_group").as_string();
        base_link_ = this->get_parameter("base_link").as_string();
        ee_link_ = this->get_parameter("ee_link").as_string();
        rate_hz_ = this->get_parameter("rate_hz").as_double();
        theta_d_limit_ = this->get_parameter("theta_d_lim").as_double();
        w0_ = this->get_parameter("w0").as_double();
        k0_ = this->get_parameter("k0").as_double();
        dt_ = 1.0 / std::max(1.0, rate_hz_);

        if (!initializeKinematics())
            throw std::runtime_error("Failed to initialize Tesseract kinematics");

        // Phase 1 publisher — arm trigger only.
        pub_arm_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_path_command", 10);

        // Phase 2 publisher — real-time streaming.
        pub_stream_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_command", 10);

        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/pose_following/cmd_vel", 10,
            std::bind(&MotoMiniVelTrackingNode::cmdVelCallback, this, std::placeholders::_1));

        // Subscribe to the MotoPlus joint state topic.
        sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 20,
            std::bind(&MotoMiniVelTrackingNode::jointStateCallback, this, std::placeholders::_1));

        srv_start_ = this->create_service<std_srvs::srv::Trigger>(
            "/pose_following/start",
            std::bind(&MotoMiniVelTrackingNode::startCallback, this, std::placeholders::_1, std::placeholders::_2));

        srv_stop_ = this->create_service<std_srvs::srv::Trigger>(
            "/pose_following/stop",
            std::bind(&MotoMiniVelTrackingNode::stopCallback, this, std::placeholders::_1, std::placeholders::_2));

        latest_cart_vel_.setZero();

        auto period_ns = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / std::max(1.0, rate_hz_)));
        timer_ = this->create_wall_timer(
            period_ns,
            std::bind(&MotoMiniVelTrackingNode::tick, this));

        RCLCPP_INFO(this->get_logger(),
                    "Node initialized (%.0f Hz). Waiting for joint state on /motomini_joint_states ...",
                    rate_hz_);
    }

private:
    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------
    enum State
    {
        STATE_WAIT_JOINT, // Phase 0 — waiting for first valid joint state.
        STATE_ARMING,     // Phase 1 — arm trigger sent; waiting settle time.
        STATE_STREAMING,  // Phase 2 — continuous stream to /joint_command.
        STATE_STOPPED     // Permanently halted; only /start can restart.
    };

    // -----------------------------------------------------------------------
    // Kinematics
    // -----------------------------------------------------------------------
    bool initializeKinematics()
    {
        auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
        env_ = std::make_shared<tesseract_environment::Environment>();
        if (!env_->init(urdf_xml_, srdf_xml_, locator))
            return false;

        manip_ = env_->getKinematicGroup(manipulator_group_);
        if (!manip_)
            return false;

        joint_names_ = manip_->getJointNames();
        return !joint_names_.empty();
    }

    static Eigen::MatrixXd calcSrInverse(const Eigen::MatrixXd &J, double w, double w0, double k0)
    {
        double k = (w < w0) ? k0 * std::pow(1.0 - w / w0, 2.0) : 0.0;
        const Eigen::Index rows = J.rows();
        const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(rows, rows);
        return J.transpose() * (J * J.transpose() + k * I).inverse();
    }

    bool currentManipulatorJointVector(Eigen::VectorXd &q) const
    {
        if (!last_joint_state_)
            return false;
        q.resize(joint_names_.size());
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            auto it = std::find(last_joint_state_->name.begin(),
                                last_joint_state_->name.end(), joint_names_[i]);
            if (it == last_joint_state_->name.end())
                return false;
            q[static_cast<Eigen::Index>(i)] =
                last_joint_state_->position[std::distance(last_joint_state_->name.begin(), it)];
        }
        return true;
    }

    bool initTrackedPositions()
    {
        Eigen::VectorXd q;
        if (!currentManipulatorJointVector(q))
            return false;
        tracked_positions_.assign(q.data(), q.data() + q.size());
        return true;
    }

    // -----------------------------------------------------------------------
    // Phase 1 — arm trigger
    // -----------------------------------------------------------------------
    void sendArmTrigger()
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now();
        traj.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions = tracked_positions_;
        pt.velocities.assign(joint_names_.size(), 0.0);
        pt.time_from_start = rclcpp::Duration::from_seconds(0.5);

        traj.points.push_back(pt);
        pub_arm_->publish(traj);
        RCLCPP_INFO(this->get_logger(), "Phase 1: arm trigger sent to /joint_path_command");
    }

    // -----------------------------------------------------------------------
    // Phase 2 — streaming helpers
    // -----------------------------------------------------------------------

    // Publish exactly ONE point to /joint_command.
    void publishStreamPoint(const std::vector<double> &positions,
                            const std::vector<double> &velocities,
                            double time_from_start)
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now();
        traj.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions = positions;
        pt.velocities = velocities;
        pt.time_from_start = rclcpp::Duration::from_seconds(time_from_start);

        traj.points.push_back(pt); // EXACTLY one point per spec.
        pub_stream_->publish(traj);
    }

    // Seed — first message of every streaming session (time = 0, vel = 0).
    void sendSeed()
    {
        std::vector<double> zero_vel(joint_names_.size(), 0.0);
        publishStreamPoint(tracked_positions_, zero_vel, 0.0);
        stream_time_ = 0.0;
        RCLCPP_INFO(this->get_logger(),
                    "Phase 2: seed sent to /joint_command — streaming started");
    }

    // -----------------------------------------------------------------------
    // Callbacks
    // -----------------------------------------------------------------------
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_joint_state_ = msg;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Only update velocity; state transitions happen in tick().
        latest_cart_vel_ << msg->linear.x, msg->linear.y, msg->linear.z,
            msg->angular.x, msg->angular.y, msg->angular.z;
    }

    // /pose_following/start — re-arm from any stopped / initial state.
    void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (state_ == STATE_ARMING || state_ == STATE_STREAMING)
        {
            res->success = false;
            res->message = "Already active. Call /pose_following/stop first.";
            return;
        }
        tracked_positions_.clear();
        latest_cart_vel_.setZero();
        arm_trigger_sent_ = false;
        state_ = STATE_WAIT_JOINT;
        res->success = true;
        res->message = "Re-arming: waiting for joint state.";
        RCLCPP_INFO(this->get_logger(), "Re-arm requested via /pose_following/start");
    }

    // /pose_following/stop — the ONLY way to stop streaming.
    void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        latest_cart_vel_.setZero();
        state_ = STATE_STOPPED;
        res->success = true;
        res->message = "Streaming stopped.";
        RCLCPP_INFO(this->get_logger(), "Streaming stopped via /pose_following/stop");
    }

    // -----------------------------------------------------------------------
    // Main timer tick
    // -----------------------------------------------------------------------
    void tick()
    {
        switch (state_)
        {
        // ---- Completely stopped — do nothing. ----
        case STATE_STOPPED:
            return;

        // ---- Phase 0: wait for first valid joint state. ----
        case STATE_WAIT_JOINT:
        {
            if (!last_joint_state_)
                return;
            if (!initTrackedPositions())
                return;
            arm_entry_time_ = this->now();
            arm_trigger_sent_ = false;
            state_ = STATE_ARMING;
            RCLCPP_INFO(this->get_logger(),
                        "Joint state acquired. Entering arming phase (pre-delay %.1fs).",
                        ARM_PRE_DELAY_S);
            return;
        }

        // ---- Phase 1: send arm trigger after pre-delay; wait post-delay. ----
        case STATE_ARMING:
        {
            double elapsed = (this->now() - arm_entry_time_).seconds();
            if (!arm_trigger_sent_)
            {
                if (elapsed < ARM_PRE_DELAY_S)
                    return;
                sendArmTrigger();
                arm_trigger_sent_ = true;
                return;
            }
            // Wait post-delay after trigger was sent.
            if (elapsed < ARM_PRE_DELAY_S + ARM_POST_DELAY_S)
                return;
            // Re-sync tracked positions to actual before seeding.
            if (!initTrackedPositions())
                return;
            sendSeed();
            state_ = STATE_STREAMING;
            return;
        }

        // ---- Phase 2: continuous streaming — NEVER stop. ----
        case STATE_STREAMING:
            doStream();
            return;
        }
    }

    // -----------------------------------------------------------------------
    // Phase 2 streaming logic
    // -----------------------------------------------------------------------
    void doStream()
    {
        if (!last_joint_state_)
            return;

        Eigen::VectorXd q;
        if (!currentManipulatorJointVector(q))
            return;

        // Compute desired joint velocities from Cartesian cmd_vel.
        // If no cmd_vel is active, hold current position (zero velocity).
        Eigen::VectorXd theta_d(static_cast<Eigen::Index>(joint_names_.size()));
        theta_d.setZero();

        if (latest_cart_vel_.norm() > 0.0)
        {
            Eigen::MatrixXd J = manip_->calcJacobian(q, base_link_, ee_link_);
            double w = std::sqrt(std::max(0.0, (J * J.transpose()).determinant()));
            theta_d = calcSrInverse(J, w, w0_, k0_) * latest_cart_vel_;

            // Hard-stop if any joint velocity exceeds limit.
            for (Eigen::Index i = 0; i < theta_d.size(); ++i)
            {
                if (std::abs(theta_d[i]) > theta_d_limit_)
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Joint %ld velocity %.3f exceeds limit %.3f — STOPPING.",
                                 static_cast<long>(i), theta_d[i], theta_d_limit_);
                    latest_cart_vel_.setZero();
                    state_ = STATE_STOPPED;
                    return;
                }
            }
        }

        // Integrate command positions.
        for (size_t i = 0; i < joint_names_.size(); ++i)
            tracked_positions_[i] += theta_d[static_cast<Eigen::Index>(i)] * dt_;

        // Latency safety: clamp lead against actual position.
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            double actual = q[static_cast<Eigen::Index>(i)];
            double lead = tracked_positions_[i] - actual;
            if (std::abs(lead) > MAX_LEAD_RAD)
            {
                tracked_positions_[i] = actual + std::copysign(MAX_LEAD_RAD, lead);
                // Recompute velocity to reflect clamped position.
                theta_d[static_cast<Eigen::Index>(i)] =
                    (tracked_positions_[i] - (actual - std::copysign(MAX_LEAD_RAD, lead))) / dt_;
            }
        }

        // Advance stream time (strictly monotonic).
        stream_time_ += dt_;

        std::vector<double> velocities(theta_d.data(), theta_d.data() + theta_d.size());
        publishStreamPoint(tracked_positions_, velocities, stream_time_);
    }

    // -----------------------------------------------------------------------
    // Members
    // -----------------------------------------------------------------------
    std::string urdf_xml_, srdf_xml_, manipulator_group_, base_link_, ee_link_;
    std::vector<std::string> joint_names_;
    double rate_hz_{25.0};
    double theta_d_limit_{3.14};
    double w0_{0.1};
    double k0_{0.001};
    double dt_{0.04};

    State state_{STATE_WAIT_JOINT};
    Eigen::Matrix<double, 6, 1> latest_cart_vel_;

    rclcpp::Time arm_entry_time_{0, 0, RCL_ROS_TIME};
    bool arm_trigger_sent_{false};
    double stream_time_{0.0};

    tesseract_environment::Environment::Ptr env_;
    tesseract_kinematics::KinematicGroup::ConstPtr manip_;

    std::vector<double> tracked_positions_;
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_stream_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_, srv_stop_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotoMiniVelTrackingNode>());
    rclcpp::shutdown();
    return 0;
}