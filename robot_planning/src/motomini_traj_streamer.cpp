/**
 * @file motomini_traj_streamer.cpp
 * @brief Motomini trajectory streamer — MPC-aware trajectory executor.
 *
 * Design rationale (v3):
 * ─────────────────────────────────────────────────────────────────────────────
 *  The upstream planner (motomini_planning_tracking.cpp) is an MPC loop that:
 *   - Re-plans from the CURRENT robot state every tick
 *   - Publishes a short trajectory (5 waypoints, ~100–300 ms horizon)
 *
 *  Therefore the streamer MUST:
 *   1. Always execute the trajectory starting from t=0 (the planner already
 *      seeds from current state, so point[0] ≈ robot's actual position).
 *   2. Replace the active trajectory immediately when a new one arrives.
 *   3. On replace, do a SHORT Hermite velocity-blend (≈ 1–2 control ticks)
 *      just to avoid a velocity discontinuity — NOT a long geometric blend.
 *   4. If no new trajectory arrives before the current one ends, HOLD the
 *      last waypoint (do not extrapolate).
 *
 *  Key corrections vs v2:
 *   - traj_start_time_ resets to NOW every time a new trajectory is accepted
 *     (correct for MPC: each new plan is relative to now).
 *   - Splice is always at index 0 (the planner sends "start = current state").
 *   - blend_dur_ is very short (1–2 ticks = 20–40 ms). Long blends caused
 *     the streamer to be perpetually blending, never tracking.
 *   - Debounce still guards against same-millisecond republishes.
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

// ============================================================================
// Joint limits (MotoMini hardware)
// ============================================================================
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
#define MAX_LEAD_RAD                 0.10   // max commanded-vs-actual gap tolerated

// Minimum interval between accepted trajectories (prevents same-ms spam)
static constexpr double MIN_TRAJ_UPDATE_INTERVAL_S = 0.02;  // 20 ms

// ============================================================================
// Cubic Hermite Blend — C1-continuous velocity merge over a short window
// ============================================================================
/**
 * Per-joint cubic Hermite segment.
 *   h00 = 2t³ − 3t² + 1   h10 = t³  − 2t² + t
 *   h01 = −2t³ + 3t²       h11 = t³  − t²
 * Guarantees vel continuity at t=0 (v0) and t=T (v1).
 */
struct HermiteBlend
{
    double p0{0.0}, v0{0.0};  // start: commanded pos + estimated vel
    double p1{0.0}, v1{0.0};  // end: first waypoint of new traj + its velocity
    double T{0.04};            // blend duration [s], default = 2 ticks

    double pos(double t) const
    {
        const double s  = std::min(std::max(t / std::max(T, 1e-9), 0.0), 1.0);
        const double s2 = s * s, s3 = s2 * s;
        return ( 2*s3 - 3*s2 + 1)*p0 + (s3 - 2*s2 + s)*T*v0
             + (-2*s3 + 3*s2    )*p1 + (s3 -   s2    )*T*v1;
    }

    double vel(double t) const
    {
        const double s  = std::min(std::max(t / std::max(T, 1e-9), 0.0), 1.0);
        const double s2 = s * s;
        return ((6*s2 - 6*s)*p0 + (3*s2 - 4*s + 1)*T*v0
              + (-6*s2 + 6*s)*p1 + (3*s2 - 2*s    )*T*v1) / std::max(T, 1e-9);
    }
};

// ============================================================================
// Main node
// ============================================================================
class MotoMiniTrajStreamer : public rclcpp::Node
{
public:
    MotoMiniTrajStreamer() : rclcpp::Node("motomini_traj_streamer")
    {
        this->declare_parameter<double>("rate_hz",        50.0);
        // blend_ticks: how many control ticks the Hermite velocity blend lasts
        // (2 ticks = 40 ms at 50 Hz — just enough for a smooth vel merge)
        this->declare_parameter<int>   ("blend_ticks",    2);

        rate_hz_    = this->get_parameter("rate_hz").as_double();
        dt_         = 1.0 / std::max(1.0, rate_hz_);
        blend_dur_  = dt_ * std::max(1, static_cast<int>(this->get_parameter("blend_ticks").as_int()));

        joint_names_ = {"joint_1_s", "joint_2_l", "joint_3_u",
                        "joint_4_r", "joint_5_b", "joint_6_t"};
        n_joints_ = joint_names_.size();

        est_vel_.assign(n_joints_, 0.0);

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

        RCLCPP_INFO(this->get_logger(),
                    "Traj Streamer v3 ready (%.0f Hz, dt=%.3fs, blend_dur=%.3fs)",
                    rate_hz_, dt_, blend_dur_);
    }

private:
    // =========================================================================
    // State machines
    // =========================================================================
    enum State      { STATE_WAIT_JOINT, STATE_ARMING, STATE_STREAMING, STATE_STOPPED };
    enum BlendState { BLEND_IDLE, BLEND_ACTIVE };

    // =========================================================================
    // Joint state callback — LP-filtered velocity estimation
    // =========================================================================
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (last_joint_state_)
        {
            const double dt_js = (rclcpp::Time(msg->header.stamp) -
                                  rclcpp::Time(last_joint_state_->header.stamp)).seconds();
            if (dt_js > 1e-4 && dt_js < 0.5)
            {
                for (size_t i = 0; i < n_joints_; ++i)
                {
                    auto it  = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
                    auto itp = std::find(last_joint_state_->name.begin(),
                                         last_joint_state_->name.end(), joint_names_[i]);
                    if (it != msg->name.end() && itp != last_joint_state_->name.end())
                    {
                        const size_t idx  = static_cast<size_t>(std::distance(msg->name.begin(), it));
                        const size_t idxp = static_cast<size_t>(
                            std::distance(last_joint_state_->name.begin(), itp));
                        const double raw = (msg->position[idx] - last_joint_state_->position[idxp]) / dt_js;
                        est_vel_[i] = 0.3 * raw + 0.7 * est_vel_[i];  // 70/30 LP filter
                    }
                }
            }
        }
        last_joint_state_ = msg;
    }

    // =========================================================================
    // Trajectory callback
    // =========================================================================
    /**
     * MPC design note:
     *   The planner re-plans from the current robot state every tick, so each
     *   incoming trajectory starts at ≈ current position (t=0 = now).
     *   We simply replace the active trajectory and reset traj_start_time_ to NOW.
     *   A tiny Hermite blend (blend_ticks) smooths the velocity transition only.
     */
    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (msg->points.size() <= 1) return;   // arm-trigger — ignore

        // Debounce: reject if too soon after last accepted trajectory
        const double since_last = (this->now() - last_traj_accept_time_).seconds();
        if (last_traj_accept_time_.nanoseconds() > 0 &&
            since_last < MIN_TRAJ_UPDATE_INTERVAL_S)
        {
            RCLCPP_DEBUG(this->get_logger(), "Traj debounced (%.1f ms)", since_last * 1e3);
            return;
        }
        last_traj_accept_time_ = this->now();

        // Only accept during streaming — ignore before arm completes
        if (state_ != STATE_STREAMING) return;

        // Build joint-order mapping: canonical → msg column
        std::vector<int> src_idx(n_joints_, -1);
        for (size_t i = 0; i < n_joints_; ++i)
        {
            auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), joint_names_[i]);
            if (it != msg->joint_names.end())
                src_idx[i] = static_cast<int>(std::distance(msg->joint_names.begin(), it));
        }

        // -----------------------------------------------------------------------
        // Start Hermite blend from current commanded state → first waypoint of new traj
        // This guarantees velocity continuity at the join point.
        // Duration is very short (blend_ticks control ticks) — just enough to
        // smooth the velocity, not so long that it delays following.
        // -----------------------------------------------------------------------
        if (!tracked_pos_.empty())
        {
            const auto& wp0  = msg->points.front();
            hermite_.resize(n_joints_);
            for (size_t i = 0; i < n_joints_; ++i)
            {
                int s = src_idx[i];
                hermite_[i].p0 = tracked_pos_[i];
                hermite_[i].v0 = est_vel_[i];
                hermite_[i].p1 = (s >= 0 && static_cast<size_t>(s) < wp0.positions.size())
                                     ? wp0.positions[static_cast<size_t>(s)] : tracked_pos_[i];
                hermite_[i].v1 = (s >= 0 && static_cast<size_t>(s) < wp0.velocities.size())
                                     ? wp0.velocities[static_cast<size_t>(s)] : 0.0;
                hermite_[i].T  = blend_dur_;
            }
            blend_state_      = BLEND_ACTIVE;
            blend_start_time_ = this->now();
        }

        // Accept the new trajectory — starts from t=0 NOW
        // (The MPC planner seeds each trajectory from the current state, so
        //  this is always correct regardless of when the trajectory was planned.)
        cached_traj_      = msg;
        cached_src_idx_   = src_idx;
        traj_start_time_  = this->now();

        const double t_end = rclcpp::Duration(msg->points.back().time_from_start).seconds();
        RCLCPP_DEBUG(this->get_logger(),
                     "New traj accepted: %zu pts, t_end=%.3fs", msg->points.size(), t_end);
    }

    // =========================================================================
    // Service callbacks
    // =========================================================================
    void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        state_            = STATE_WAIT_JOINT;
        blend_state_      = BLEND_IDLE;
        tracked_pos_.clear();
        est_vel_.assign(n_joints_, 0.0);
        cached_traj_      = nullptr;
        arm_trigger_sent_ = false;
        last_traj_accept_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        res->success      = true;
        res->message      = "Re-arming: waiting for joint state.";
        RCLCPP_INFO(this->get_logger(), "Start requested — re-arming.");
    }

    void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        state_       = STATE_STOPPED;
        blend_state_ = BLEND_IDLE;
        cached_traj_ = nullptr;
        res->success = true;
        res->message = "Streaming stopped.";
        RCLCPP_INFO(this->get_logger(), "Stop requested.");
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
                                last_joint_state_->name.end(), joint_names_[i]);
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

    void publishStreamPoint(const std::vector<double>& pos, const std::vector<double>& vel)
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp     = this->now();
        traj.joint_names      = joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions          = pos;
        pt.velocities         = vel;
        pt.time_from_start    = rclcpp::Duration::from_seconds(dt_);
        traj.points.push_back(pt);
        pub_stream_->publish(traj);
    }

    // O(log n) binary search: find segment k s.t. pts[k].t <= elapsed < pts[k+1].t
    size_t findSegment(const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& pts,
                       double elapsed) const
    {
        size_t lo = 0, hi = pts.size() - 1;
        while (lo + 1 < hi)
        {
            const size_t mid = (lo + hi) / 2;
            if (rclcpp::Duration(pts[mid].time_from_start).seconds() <= elapsed)
                lo = mid;
            else
                hi = mid;
        }
        return lo;
    }

    // =========================================================================
    // doStream — called at 50 Hz while STATE_STREAMING
    //
    // Three phases (in priority order):
    //   A) Hermite blend active  → interpolate blend curve
    //   B) Trajectory available  → interpolate across waypoints
    //   C) No trajectory         → hold tracked_pos_ (zero vel)
    // =========================================================================
    void doStream()
    {
        std::vector<double> target_pos = tracked_pos_;
        std::vector<double> target_vel(n_joints_, 0.0);

        // ── Phase A: Hermite velocity blend ───────────────────────────────────
        if (blend_state_ == BLEND_ACTIVE)
        {
            const double be = (this->now() - blend_start_time_).seconds();
            if (be >= blend_dur_)
            {
                blend_state_ = BLEND_IDLE;
                RCLCPP_DEBUG(this->get_logger(), "Hermite blend complete.");
            }
            else
            {
                for (size_t i = 0; i < n_joints_; ++i)
                {
                    target_pos[i] = hermite_[i].pos(be);
                    target_vel[i] = hermite_[i].vel(be);
                }
                // Skip trajectory interpolation during blend — apply safety then publish
                applyLeadClamp(target_pos, target_vel);
                if (checkLimits(target_pos))
                    tracked_pos_ = target_pos;
                else
                    target_vel.assign(n_joints_, 0.0);
                publishStreamPoint(tracked_pos_, target_vel);
                return;
            }
        }

        // ── Phase B: Trajectory interpolation ─────────────────────────────────
        if (cached_traj_ && !cached_traj_->points.empty())
        {
            const double elapsed = (this->now() - traj_start_time_).seconds();
            const auto&  pts     = cached_traj_->points;
            const size_t n       = pts.size();
            const double t_end   = rclcpp::Duration(pts.back().time_from_start).seconds();

            if (elapsed >= t_end)
            {
                // ── Trajectory finished: hold last waypoint, wait for new plan ──
                const auto& last = pts.back();
                for (size_t i = 0; i < n_joints_; ++i)
                {
                    int s = cached_src_idx_[i];
                    if (s >= 0 && static_cast<size_t>(s) < last.positions.size())
                        target_pos[i] = last.positions[static_cast<size_t>(s)];
                }
                // target_vel stays zero — hold
            }
            else if (n >= 2)
            {
                // ── Active tracking: binary search + linear interpolation ──────
                const size_t k  = findSegment(pts, elapsed);
                const size_t k1 = std::min(k + 1, n - 1);

                const double t0    = rclcpp::Duration(pts[k ].time_from_start).seconds();
                const double t1    = rclcpp::Duration(pts[k1].time_from_start).seconds();
                const double alpha = (elapsed - t0) / std::max(1e-9, t1 - t0);

                for (size_t i = 0; i < n_joints_; ++i)
                {
                    const int s = cached_src_idx_[i];
                    if (s < 0) continue;
                    const size_t si = static_cast<size_t>(s);

                    auto getPos = [&](const trajectory_msgs::msg::JointTrajectoryPoint& p) {
                        return si < p.positions.size()  ? p.positions[si]  : 0.0;
                    };
                    auto getVel = [&](const trajectory_msgs::msg::JointTrajectoryPoint& p) {
                        return si < p.velocities.size() ? p.velocities[si] : 0.0;
                    };

                    target_pos[i] = getPos(pts[k]) + alpha * (getPos(pts[k1]) - getPos(pts[k]));
                    target_vel[i] = getVel(pts[k]) + alpha * (getVel(pts[k1]) - getVel(pts[k]));
                }
            }
            else
            {
                // Single-point trajectory — hold it
                const int s0 = (n >= 1 && !cached_src_idx_.empty()) ? 0 : -1;
                (void)s0;
                const auto& pt0 = pts.front();
                for (size_t i = 0; i < n_joints_; ++i)
                {
                    const int s = cached_src_idx_[i];
                    if (s >= 0 && static_cast<size_t>(s) < pt0.positions.size())
                        target_pos[i] = pt0.positions[static_cast<size_t>(s)];
                }
            }
        }
        // Phase C: no trajectory → target = tracked_pos_ (hold), vel = 0 (already set)

        // ── Safety: lead clamp + joint limits ────────────────────────────────
        applyLeadClamp(target_pos, target_vel);

        if (!checkLimits(target_pos))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Joint limit breach — holding safe posture.");
            target_vel.assign(n_joints_, 0.0);
        }
        else
        {
            tracked_pos_ = target_pos;
        }

        publishStreamPoint(tracked_pos_, target_vel);
    }

    /// Clamp commanded position within MAX_LEAD_RAD of actual; scale velocity proportionally.
    void applyLeadClamp(std::vector<double>& pos, std::vector<double>& vel)
    {
        if (!last_joint_state_) return;
        for (size_t i = 0; i < n_joints_; ++i)
        {
            auto it = std::find(last_joint_state_->name.begin(),
                                last_joint_state_->name.end(), joint_names_[i]);
            if (it == last_joint_state_->name.end()) continue;
            const double actual = last_joint_state_->position[
                static_cast<size_t>(std::distance(last_joint_state_->name.begin(), it))];
            const double lead = pos[i] - actual;
            if (std::abs(lead) > MAX_LEAD_RAD)
            {
                const double scale = MAX_LEAD_RAD / std::abs(lead);
                pos[i] = actual + std::copysign(MAX_LEAD_RAD, lead);
                vel[i] *= scale;
            }
        }
    }

    // =========================================================================
    // State-machine tick (hardware interface — structure unchanged)
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
                const double elapsed = (this->now() - arm_entry_time_).seconds();
                if (!arm_trigger_sent_)
                {
                    if (elapsed < 0.5) return;
                    // 1-point arming message to prime the hardware controller
                    trajectory_msgs::msg::JointTrajectory traj;
                    traj.header.stamp = this->now();
                    traj.joint_names  = joint_names_;
                    trajectory_msgs::msg::JointTrajectoryPoint pt;
                    pt.positions      = tracked_pos_;
                    pt.velocities.assign(n_joints_, 0.0);
                    pt.time_from_start = rclcpp::Duration::from_seconds(0.5);
                    traj.points.push_back(pt);
                    pub_arm_->publish(traj);
                    arm_trigger_sent_ = true;
                    return;
                }
                if (elapsed < 1.5) return;
                if (!initTrackedPositions()) return;
                est_vel_.assign(n_joints_, 0.0);
                traj_start_time_ = this->now();
                publishStreamPoint(tracked_pos_, std::vector<double>(n_joints_, 0.0));
                state_ = STATE_STREAMING;
                RCLCPP_INFO(this->get_logger(), "Streaming started.");
                return;
            }

            case STATE_STREAMING:
                doStream();
                return;
        }
    }

    // =========================================================================
    // Members
    // =========================================================================
    std::vector<std::string> joint_names_;
    size_t  n_joints_{6};
    double  rate_hz_{50.0};
    double  dt_{0.02};
    double  blend_dur_{0.04};   // 2 ticks × dt at 50 Hz

    State      state_{STATE_WAIT_JOINT};
    BlendState blend_state_{BLEND_IDLE};

    rclcpp::Time arm_entry_time_{0, 0, RCL_ROS_TIME};
    bool         arm_trigger_sent_{false};

    rclcpp::Time last_traj_accept_time_{0, 0, RCL_ROS_TIME};

    // Streamer's running commanded position (integrates from actual at arm time)
    std::vector<double> tracked_pos_;

    // LP-filtered velocity estimate from consecutive joint states
    std::vector<double> est_vel_;

    // Active trajectory reference time — reset to NOW on every new trajectory
    rclcpp::Time traj_start_time_{0, 0, RCL_ROS_TIME};

    // Cached trajectory + pre-computed joint-order mapping
    trajectory_msgs::msg::JointTrajectory::SharedPtr cached_traj_;
    std::vector<int>                                  cached_src_idx_;

    // Hermite blend (velocity continuity on trajectory switch)
    std::vector<HermiteBlend> hermite_;
    rclcpp::Time              blend_start_time_{0, 0, RCL_ROS_TIME};

    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_stream_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr       sub_joint_state_;
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
