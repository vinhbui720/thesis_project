#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_srvs/srv/trigger.hpp>

// Limits
#define JOINT_1_S_UPPER_LIMIT_RAD (170.0 * M_PI / 180.0)
#define JOINT_1_S_LOWER_LIMIT_RAD (-170.0 * M_PI / 180.0)
#define JOINT_2_L_UPPER_LIMIT_RAD (90.0 * M_PI / 180.0)
#define JOINT_2_L_LOWER_LIMIT_RAD (-85.0 * M_PI / 180.0)
#define JOINT_3_U_UPPER_LIMIT_RAD (120.0 * M_PI / 180.0)
#define JOINT_3_U_LOWER_LIMIT_RAD (-175.0 * M_PI / 180.0)
#define JOINT_4_R_UPPER_LIMIT_RAD (140.0 * M_PI / 180.0)
#define JOINT_4_R_LOWER_LIMIT_RAD (-140.0 * M_PI / 180.0)
#define JOINT_5_B_UPPER_LIMIT_RAD (210.0 * M_PI / 180.0)
#define JOINT_5_B_LOWER_LIMIT_RAD (-30.0 * M_PI / 180.0)
#define JOINT_6_T_UPPER_LIMIT_RAD (360.0 * M_PI / 180.0)
#define JOINT_6_T_LOWER_LIMIT_RAD (-360.0 * M_PI / 180.0)

#define JOINT_1_S_VEL_LIMIT_RADSEC (M_PI * 7.0 / 4.0)
#define JOINT_2_L_VEL_LIMIT_RADSEC (M_PI * 7.0 / 4.0)
#define JOINT_3_U_VEL_LIMIT_RADSEC (M_PI * 7.0 / 3.0)
#define JOINT_4_R_VEL_LIMIT_RADSEC (M_PI * 10.0 / 3.0)
#define JOINT_5_B_VEL_LIMIT_RADSEC (M_PI * 10.0 / 3.0)
#define JOINT_6_T_VEL_LIMIT_RADSEC (M_PI * 10.0 / 3.0)

#define SAFETY_VELOCITY_ALPHA 0.85
#define SAFETY_JOINT_PADDING_RAD (5.0 * M_PI / 180.0)
#define MAX_LEAD_RAD 0.10

class MotoMiniTrajStreamer : public rclcpp::Node
{
public:
    MotoMiniTrajStreamer() : rclcpp::Node("motomini_traj_streamer")
    {
        this->declare_parameter<double>("rate_hz", 50.0);
        rate_hz_ = this->get_parameter("rate_hz").as_double();
        dt_ = 1.0 / std::max(1.0, rate_hz_);

        // Ensure these exact joint names for Motomini
        joint_names_ = {"joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};

        pub_arm_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_path_command", 10);
        pub_stream_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_command", 10);

        sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 20, std::bind(&MotoMiniTrajStreamer::jointStateCallback, this, std::placeholders::_1));

        sub_traj_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/joint_path_command", 10, std::bind(&MotoMiniTrajStreamer::trajectoryCallback, this, std::placeholders::_1));

        srv_start_ = this->create_service<std_srvs::srv::Trigger>(
            "/pose_following/start", std::bind(&MotoMiniTrajStreamer::startCallback, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_stop_ = this->create_service<std_srvs::srv::Trigger>(
            "/pose_following/stop", std::bind(&MotoMiniTrajStreamer::stopCallback, this, std::placeholders::_1, std::placeholders::_2));

        auto period_ns = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / std::max(1.0, rate_hz_)));
        timer_ = this->create_wall_timer(period_ns, std::bind(&MotoMiniTrajStreamer::tick, this));

        RCLCPP_INFO(this->get_logger(), "Trajectory Streamer initialized (%.0f Hz)", rate_hz_);
    }

private:
    enum State {
        STATE_WAIT_JOINT,
        STATE_ARMING,
        STATE_STREAMING,
        STATE_STOPPED
    };

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_joint_state_ = msg;
    }

    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        // Ignore single-point arming messages (like the ones we send)
        if (msg->points.size() <= 1) return;

        RCLCPP_INFO(this->get_logger(), "Received new trajectory with %zu points", msg->points.size());
        cached_traj_ = msg;
        traj_start_time_ = this->now();
    }

    void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        state_ = STATE_WAIT_JOINT;
        tracked_positions_.clear();
        cached_traj_ = nullptr;
        arm_trigger_sent_ = false;
        res->success = true;
        res->message = "Re-arming: waiting for joint state.";
    }

    void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        state_ = STATE_STOPPED;
        res->success = true;
        res->message = "Streaming stopped.";
    }

    bool initTrackedPositions()
    {
        if (!last_joint_state_) return false;
        tracked_positions_.resize(joint_names_.size(), 0.0);
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            auto it = std::find(last_joint_state_->name.begin(), last_joint_state_->name.end(), joint_names_[i]);
            if (it == last_joint_state_->name.end()) return false;
            tracked_positions_[i] = last_joint_state_->position[std::distance(last_joint_state_->name.begin(), it)];
        }
        return true;
    }

    bool checkLimits(const std::vector<double>& pos) const
    {
        static const double lower[] = {
            JOINT_1_S_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD, JOINT_2_L_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_3_U_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD, JOINT_4_R_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_5_B_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD, JOINT_6_T_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD
        };
        static const double upper[] = {
            JOINT_1_S_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD, JOINT_2_L_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_3_U_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD, JOINT_4_R_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_5_B_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD, JOINT_6_T_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD
        };
        for (size_t i = 0; i < pos.size() && i < 6; ++i) {
            if (pos[i] <= lower[i] || pos[i] >= upper[i]) return false;
        }
        return true;
    }

    void tick()
    {
        switch (state_) {
            case STATE_STOPPED: return;
            case STATE_WAIT_JOINT: {
                if (!initTrackedPositions()) return;
                arm_entry_time_ = this->now();
                arm_trigger_sent_ = false;
                state_ = STATE_ARMING;
                return;
            }
            case STATE_ARMING: {
                double elapsed = (this->now() - arm_entry_time_).seconds();
                if (!arm_trigger_sent_) {
                    if (elapsed < 0.5) return;
                    trajectory_msgs::msg::JointTrajectory traj;
                    traj.header.stamp = this->now();
                    traj.joint_names = joint_names_;
                    trajectory_msgs::msg::JointTrajectoryPoint pt;
                    pt.positions = tracked_positions_;
                    pt.velocities.assign(joint_names_.size(), 0.0);
                    pt.time_from_start = rclcpp::Duration::from_seconds(0.5);
                    traj.points.push_back(pt);
                    pub_arm_->publish(traj);
                    arm_trigger_sent_ = true;
                    return;
                }
                if (elapsed < 1.5) return;
                if (!initTrackedPositions()) return;
                stream_time_ = 0.0;
                publishStreamPoint(tracked_positions_, std::vector<double>(joint_names_.size(), 0.0));
                state_ = STATE_STREAMING;
                RCLCPP_INFO(this->get_logger(), "Streaming Started");
                return;
            }
            case STATE_STREAMING: {
                doStream();
                return;
            }
        }
    }

    void publishStreamPoint(const std::vector<double>& pos, const std::vector<double>& vel)
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now();
        traj.joint_names = joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions = pos;
        pt.velocities = vel;
        pt.time_from_start = rclcpp::Duration::from_seconds(stream_time_);
        traj.points.push_back(pt);
        pub_stream_->publish(traj);
    }

    void doStream()
    {
        stream_time_ += dt_;
        std::vector<double> target_pos = tracked_positions_;
        std::vector<double> target_vel(joint_names_.size(), 0.0);

        if (cached_traj_) {
            double elapsed = (this->now() - traj_start_time_).seconds();
            size_t n = cached_traj_->points.size();
            double t_end = rclcpp::Duration(cached_traj_->points.back().time_from_start).seconds();

            if (elapsed >= t_end) {
                target_pos = cached_traj_->points.back().positions;
            } else {
                for (size_t i = 0; i < n - 1; ++i) {
                    double t0 = rclcpp::Duration(cached_traj_->points[i].time_from_start).seconds();
                    double t1 = rclcpp::Duration(cached_traj_->points[i+1].time_from_start).seconds();
                    if (elapsed >= t0 && elapsed <= t1) {
                        double alpha = (elapsed - t0) / (t1 - t0);
                        for (size_t j = 0; j < joint_names_.size(); ++j) {
                            target_pos[j] = cached_traj_->points[i].positions[j] + alpha * (cached_traj_->points[i+1].positions[j] - cached_traj_->points[i].positions[j]);
                            target_vel[j] = cached_traj_->points[i].velocities.empty() ? 0.0 : 
                                            cached_traj_->points[i].velocities[j] + alpha * (cached_traj_->points[i+1].velocities[j] - cached_traj_->points[i].velocities[j]);
                        }
                        break;
                    }
                }
            }
        }

        if (checkLimits(target_pos)) {
            tracked_positions_ = target_pos;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Position limits breached! Holding last safe posture.");
            target_vel.assign(joint_names_.size(), 0.0);
        }

        // Clamp lead to actual position
        if (last_joint_state_) {
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                auto it = std::find(last_joint_state_->name.begin(), last_joint_state_->name.end(), joint_names_[i]);
                if (it != last_joint_state_->name.end()) {
                    double actual = last_joint_state_->position[std::distance(last_joint_state_->name.begin(), it)];
                    double lead = tracked_positions_[i] - actual;
                    if (std::abs(lead) > MAX_LEAD_RAD) {
                        tracked_positions_[i] = actual + std::copysign(MAX_LEAD_RAD, lead);
                    }
                }
            }
        }

        publishStreamPoint(tracked_positions_, target_vel);
    }

    std::vector<std::string> joint_names_;
    double rate_hz_{50.0}, dt_{0.02};
    State state_{STATE_WAIT_JOINT};

    rclcpp::Time arm_entry_time_{0, 0, RCL_ROS_TIME};
    bool arm_trigger_sent_{false};
    double stream_time_{0.0};

    std::vector<double> tracked_positions_;
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

    trajectory_msgs::msg::JointTrajectory::SharedPtr cached_traj_;
    rclcpp::Time traj_start_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_stream_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_traj_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_, srv_stop_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotoMiniTrajStreamer>());
    rclcpp::shutdown();
    return 0;
}
