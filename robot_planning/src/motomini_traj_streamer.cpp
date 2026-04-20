/**
 * @file motomini_traj_streamer.cpp
 * @brief Motomini trajectory streamer — smooth splice-in of incoming trajectories.
 *
 * Architecture:
 *   - Receives full JointTrajectory on /joint_path_command (from planner or tracker)
 *   - On new trajectory: finds the nearest point to the current robot position
 *     and re-bases time from that splice point → the robot smoothly merges from
 *     wherever it is right now into the new trajectory without any back-step jerk
 *   - Interpolates at 50 Hz and publishes single-point /joint_command continuously
 *
 * @author Bùi Quang Vinh
 */

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

// Joint limits
#define JOINT_1_S_UPPER_LIMIT_RAD    (170.0 * M_PI / 180.0)
#define JOINT_1_S_LOWER_LIMIT_RAD   (-170.0 * M_PI / 180.0)
#define JOINT_2_L_UPPER_LIMIT_RAD    (90.0  * M_PI / 180.0)
#define JOINT_2_L_LOWER_LIMIT_RAD   (-85.0  * M_PI / 180.0)
#define JOINT_3_U_UPPER_LIMIT_RAD    (120.0 * M_PI / 180.0)
#define JOINT_3_U_LOWER_LIMIT_RAD   (-175.0 * M_PI / 180.0)
#define JOINT_4_R_UPPER_LIMIT_RAD    (140.0 * M_PI / 180.0)
#define JOINT_4_R_LOWER_LIMIT_RAD   (-140.0 * M_PI / 180.0)
#define JOINT_5_B_UPPER_LIMIT_RAD    (210.0 * M_PI / 180.0)
#define JOINT_5_B_LOWER_LIMIT_RAD   (-30.0  * M_PI / 180.0)
#define JOINT_6_T_UPPER_LIMIT_RAD    (360.0 * M_PI / 180.0)
#define JOINT_6_T_LOWER_LIMIT_RAD   (-360.0 * M_PI / 180.0)

#define SAFETY_JOINT_PADDING_RAD     (5.0 * M_PI / 180.0)
#define MAX_LEAD_RAD                 0.10   // max commanded-vs-actual error tolerated

class MotoMiniTrajStreamer : public rclcpp::Node
{
public:
    MotoMiniTrajStreamer() : rclcpp::Node("motomini_traj_streamer")
    {
        this->declare_parameter<double>("rate_hz", 50.0);
        rate_hz_ = this->get_parameter("rate_hz").as_double();
        dt_ = 1.0 / std::max(1.0, rate_hz_);

        joint_names_ = {"joint_1_s", "joint_2_l", "joint_3_u",
                        "joint_4_r", "joint_5_b", "joint_6_t"};
        n_joints_ = joint_names_.size();

        pub_arm_    = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_path_command", 10);
        pub_stream_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_command", 10);

        sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 20,
            std::bind(&MotoMiniTrajStreamer::jointStateCallback, this, std::placeholders::_1));

        sub_traj_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/joint_path_command", 10,
            std::bind(&MotoMiniTrajStreamer::trajectoryCallback, this, std::placeholders::_1));

        srv_start_ = this->create_service<std_srvs::srv::Trigger>(
            "/pose_following/start",
            std::bind(&MotoMiniTrajStreamer::startCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        srv_stop_ = this->create_service<std_srvs::srv::Trigger>(
            "/pose_following/stop",
            std::bind(&MotoMiniTrajStreamer::stopCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        auto period_ns = std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / std::max(1.0, rate_hz_)));
        timer_ = this->create_wall_timer(
            period_ns, std::bind(&MotoMiniTrajStreamer::tick, this));

        RCLCPP_INFO(this->get_logger(), "Trajectory Streamer initialized (%.0f Hz)", rate_hz_);
    }

private:
    // =========================================================================
    // State machine
    // =========================================================================
    enum State { STATE_WAIT_JOINT, STATE_ARMING, STATE_STREAMING, STATE_STOPPED };

    // =========================================================================
    // Callbacks
    // =========================================================================
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_joint_state_ = msg;
    }

    /**
     * trajectoryCallback — splice-in logic
     *
     * When a new trajectory arrives:
     *  1. Ignore single-point arm messages.
     *  2. Find the trajectory index nearest to the current robot position.
     *  3. Re-base the trajectory time so t=0 at that splice index.
     *  4. Truncate any points already behind the splice index.
     *  5. Replace cached_traj_ and reset traj_start_time_ to NOW.
     *
     * This guarantees the controller always receives a trajectory that starts
     * from (≈) where the robot actually is, preventing back-step jerks.
     */
    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (msg->points.size() <= 1) return;  // arm-trigger messages — ignore

        if (!last_joint_state_)
        {
            // No joint state yet: just cache verbatim
            cached_traj_ = msg;
            traj_start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "New traj (%zu pts) accepted (no joint state yet).",
                        msg->points.size());
            return;
        }

        // Get current robot joint positions mapped to our joint_names_ order
        std::vector<double> current_pos(n_joints_, 0.0);
        for (size_t i = 0; i < n_joints_; ++i)
        {
            auto it = std::find(last_joint_state_->name.begin(),
                                last_joint_state_->name.end(),
                                joint_names_[i]);
            if (it != last_joint_state_->name.end())
            {
                size_t idx = static_cast<size_t>(
                    std::distance(last_joint_state_->name.begin(), it));
                current_pos[i] = last_joint_state_->position[idx];
            }
        }

        // Map incoming trajectory joint names → our joint order
        std::vector<int> src_idx(n_joints_, -1);
        for (size_t i = 0; i < n_joints_; ++i)
        {
            auto it = std::find(msg->joint_names.begin(),
                                msg->joint_names.end(),
                                joint_names_[i]);
            if (it != msg->joint_names.end())
                src_idx[i] = static_cast<int>(
                    std::distance(msg->joint_names.begin(), it));
        }

        // Find the trajectory point nearest (L2 in joint space) to current_pos
        size_t splice_idx = 0;
        double best_dist = std::numeric_limits<double>::max();
        for (size_t p = 0; p < msg->points.size(); ++p)
        {
            double dist = 0.0;
            for (size_t i = 0; i < n_joints_; ++i)
            {
                int s = src_idx[i];
                if (s < 0 || s >= static_cast<int>(msg->points[p].positions.size()))
                    continue;
                double d = msg->points[p].positions[static_cast<size_t>(s)] - current_pos[i];
                dist += d * d;
            }
            if (dist < best_dist)
            {
                best_dist = dist;
                splice_idx = p;
            }
        }

        // Re-base time so t=0 at splice_idx
        double t_offset = rclcpp::Duration(msg->points[splice_idx].time_from_start).seconds();

        auto spliced = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
        spliced->joint_names = msg->joint_names;
        spliced->header      = msg->header;

        for (size_t p = splice_idx; p < msg->points.size(); ++p)
        {
            auto pt = msg->points[p];
            double t_rebased = rclcpp::Duration(pt.time_from_start).seconds() - t_offset;
            if (t_rebased < 0.0) t_rebased = 0.0;
            pt.time_from_start = rclcpp::Duration::from_seconds(t_rebased);
            spliced->points.push_back(pt);
        }

        cached_traj_     = spliced;
        traj_start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
                    "Traj spliced at idx %zu/%zu (dist=%.3f rad), %zu pts remain, "
                    "t_end=%.3f s",
                    splice_idx, msg->points.size(), std::sqrt(best_dist),
                    spliced->points.size(),
                    spliced->points.empty() ? 0.0
                        : rclcpp::Duration(spliced->points.back().time_from_start).seconds());
    }

    void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        state_           = STATE_WAIT_JOINT;
        tracked_pos_.clear();
        cached_traj_     = nullptr;
        arm_trigger_sent_ = false;
        res->success     = true;
        res->message     = "Re-arming: waiting for joint state.";
    }

    void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        state_       = STATE_STOPPED;
        res->success = true;
        res->message = "Streaming stopped.";
    }

    // =========================================================================
    // Helpers
    // =========================================================================
    bool initTrackedPositions()
    {
        if (!last_joint_state_) return false;
        tracked_pos_.assign(n_joints_, 0.0);
        for (size_t i = 0; i < n_joints_; ++i)
        {
            auto it = std::find(last_joint_state_->name.begin(),
                                last_joint_state_->name.end(),
                                joint_names_[i]);
            if (it == last_joint_state_->name.end()) return false;
            tracked_pos_[i] = last_joint_state_->position[
                static_cast<size_t>(std::distance(last_joint_state_->name.begin(), it))];
        }
        return true;
    }

    bool checkLimits(const std::vector<double>& pos) const
    {
        static const double lower[] = {
            JOINT_1_S_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_2_L_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_3_U_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_4_R_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_5_B_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_6_T_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD
        };
        static const double upper[] = {
            JOINT_1_S_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_2_L_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_3_U_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_4_R_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_5_B_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_6_T_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD
        };
        for (size_t i = 0; i < pos.size() && i < 6; ++i)
            if (pos[i] <= lower[i] || pos[i] >= upper[i]) return false;
        return true;
    }

    void publishStreamPoint(const std::vector<double>& pos,
                            const std::vector<double>& vel)
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp  = this->now();
        traj.joint_names   = joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions       = pos;
        pt.velocities      = vel;
        pt.time_from_start = rclcpp::Duration::from_seconds(dt_); // always 1 tick ahead
        traj.points.push_back(pt);
        pub_stream_->publish(traj);
    }

    // =========================================================================
    // Main streaming logic — interpolate cached_traj_ at current elapsed time
    // =========================================================================
    void doStream()
    {
        std::vector<double> target_pos = tracked_pos_;
        std::vector<double> target_vel(n_joints_, 0.0);

        if (cached_traj_ && !cached_traj_->points.empty())
        {
            double elapsed = (this->now() - traj_start_time_).seconds();
            const auto& pts = cached_traj_->points;
            const size_t n  = pts.size();
            double t_end    = rclcpp::Duration(pts.back().time_from_start).seconds();

            // Map traj joint names → our canonical joint order
            std::vector<int> src(n_joints_, -1);
            for (size_t i = 0; i < n_joints_; ++i)
            {
                auto it = std::find(cached_traj_->joint_names.begin(),
                                    cached_traj_->joint_names.end(),
                                    joint_names_[i]);
                if (it != cached_traj_->joint_names.end())
                    src[i] = static_cast<int>(
                        std::distance(cached_traj_->joint_names.begin(), it));
            }

            if (elapsed >= t_end)
            {
                // Hold last point with zero velocity
                const auto& last = pts.back();
                for (size_t i = 0; i < n_joints_; ++i)
                {
                    int s = src[i];
                    if (s >= 0 && s < static_cast<int>(last.positions.size()))
                        target_pos[i] = last.positions[static_cast<size_t>(s)];
                }
                // Zero velocity already
            }
            else
            {
                // Binary-search for the right segment and interpolate
                for (size_t k = 0; k < n - 1; ++k)
                {
                    double t0 = rclcpp::Duration(pts[k    ].time_from_start).seconds();
                    double t1 = rclcpp::Duration(pts[k + 1].time_from_start).seconds();
                    if (elapsed >= t0 && elapsed <= t1)
                    {
                        double alpha = (elapsed - t0) / std::max(1e-9, t1 - t0);
                        for (size_t i = 0; i < n_joints_; ++i)
                        {
                            int s = src[i];
                            if (s < 0) continue;
                            size_t si = static_cast<size_t>(s);

                            auto getPos = [&](const trajectory_msgs::msg::JointTrajectoryPoint& p) {
                                return (si < p.positions.size()) ? p.positions[si] : 0.0;
                            };
                            auto getVel = [&](const trajectory_msgs::msg::JointTrajectoryPoint& p) {
                                return (si < p.velocities.size()) ? p.velocities[si] : 0.0;
                            };

                            target_pos[i] = getPos(pts[k]) + alpha * (getPos(pts[k+1]) - getPos(pts[k]));
                            target_vel[i] = getVel(pts[k]) + alpha * (getVel(pts[k+1]) - getVel(pts[k]));
                        }
                        break;
                    }
                }
            }
        }

        // Safety: clamp commanded lead vs actual to prevent runaway
        if (last_joint_state_)
        {
            double total_err = 0.0;
            for (size_t i = 0; i < n_joints_; ++i)
            {
                auto it = std::find(last_joint_state_->name.begin(),
                                    last_joint_state_->name.end(),
                                    joint_names_[i]);
                if (it == last_joint_state_->name.end()) continue;
                double actual = last_joint_state_->position[
                    static_cast<size_t>(std::distance(last_joint_state_->name.begin(), it))];
                double lead = target_pos[i] - actual;
                total_err  += std::abs(lead);
                if (std::abs(lead) > MAX_LEAD_RAD)
                    target_pos[i] = actual + std::copysign(MAX_LEAD_RAD, lead);
            }
            (void)total_err; // could log if needed
        }

        // Limit check
        if (!checkLimits(target_pos))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Position limits breached! Holding last safe posture.");
            target_vel.assign(n_joints_, 0.0);
        }
        else
        {
            tracked_pos_ = target_pos;
        }

        publishStreamPoint(tracked_pos_, target_vel);
    }

    // =========================================================================
    // State-machine tick
    // =========================================================================
    void tick()
    {
        switch (state_)
        {
            case STATE_STOPPED: return;

            case STATE_WAIT_JOINT:
                if (!initTrackedPositions()) return;
                arm_entry_time_   = this->now();
                arm_trigger_sent_ = false;
                state_            = STATE_ARMING;
                return;

            case STATE_ARMING:
            {
                double elapsed = (this->now() - arm_entry_time_).seconds();
                if (!arm_trigger_sent_)
                {
                    if (elapsed < 0.5) return;
                    // Send a 1-point arming message to "prime" the controller
                    trajectory_msgs::msg::JointTrajectory traj;
                    traj.header.stamp  = this->now();
                    traj.joint_names   = joint_names_;
                    trajectory_msgs::msg::JointTrajectoryPoint pt;
                    pt.positions       = tracked_pos_;
                    pt.velocities.assign(n_joints_, 0.0);
                    pt.time_from_start = rclcpp::Duration::from_seconds(0.5);
                    traj.points.push_back(pt);
                    pub_arm_->publish(traj);
                    arm_trigger_sent_ = true;
                    return;
                }
                if (elapsed < 1.5) return;
                if (!initTrackedPositions()) return;
                publishStreamPoint(tracked_pos_, std::vector<double>(n_joints_, 0.0));
                state_ = STATE_STREAMING;
                RCLCPP_INFO(this->get_logger(), "Streaming Started");
                return;
            }

            case STATE_STREAMING:
                doStream();
                return;
        }
    }

    // =========================================================================
    // Member variables
    // =========================================================================
    std::vector<std::string> joint_names_;
    size_t n_joints_{6};
    double rate_hz_{50.0};
    double dt_{0.02};

    State state_{STATE_WAIT_JOINT};

    rclcpp::Time arm_entry_time_{0, 0, RCL_ROS_TIME};
    bool arm_trigger_sent_{false};

    std::vector<double> tracked_pos_;                       // streamer's running commanded position
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
