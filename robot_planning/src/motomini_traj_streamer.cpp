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
#include <future>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_srvs/srv/trigger.hpp>

// ============================================================================
// Joint limits (MotoMini hardware)
// ============================================================================
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

#define SAFETY_JOINT_PADDING_RAD (5.0 * M_PI / 180.0)
#define MAX_LEAD_RAD 0.10 // max commanded-vs-actual gap tolerated

// Minimum interval between accepted trajectories (prevents same-ms spam)
static constexpr double MIN_TRAJ_UPDATE_INTERVAL_S = 0.02; // 20 ms

// ============================================================================
// Data Structures
// ============================================================================

struct ProcessedCache {
    std::vector<std::array<double, 6>> pos;
    std::vector<std::array<double, 6>> vel;
    std::vector<double>                time;

    double   t_splice     {0.0};
    double   dt           {0.02};
    bool     dense_filled {false};
    rclcpp::Time created_at;
};

/**
 * Per-joint cubic Hermite segment.
 */
struct HermiteBlend
{
    double p0{0.0}, v0{0.0};
    double p1{0.0}, v1{0.0};
    double T{0.04}; 

    double pos(double t) const
    {
        const double s = std::min(std::max(t / std::max(T, 1e-9), 0.0), 1.0);
        const double s2 = s * s, s3 = s2 * s;
        return (2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * T * v0 + (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * T * v1;
    }

    double vel(double t) const
    {
        const double s = std::min(std::max(t / std::max(T, 1e-9), 0.0), 1.0);
        const double s2 = s * s;
        return ((6 * s2 - 6 * s) * p0 + (3 * s2 - 4 * s + 1) * T * v0 + (-6 * s2 + 6 * s) * p1 + (3 * s2 - 2 * s) * T * v1) / std::max(T, 1e-9);
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
        this->declare_parameter<double>("rate_hz", 50.0);
        this->declare_parameter<int>("blend_ticks", 1);
        this->declare_parameter<double>("splice_w_pos", 1.0);
        this->declare_parameter<double>("splice_w_vel", 0.5);
        this->declare_parameter<int>("prep_ticks", 4);

        rate_hz_ = this->get_parameter("rate_hz").as_double();
        dt_ = 1.0 / std::max(1.0, rate_hz_);
        blend_dur_ = dt_ * std::max(1, static_cast<int>(this->get_parameter("blend_ticks").as_int()));
        splice_w_pos_ = this->get_parameter("splice_w_pos").as_double();
        splice_w_vel_ = this->get_parameter("splice_w_vel").as_double();
        prep_ticks_   = std::max(0, static_cast<int>(this->get_parameter("prep_ticks").as_int()));

        joint_names_ = {"joint_1_s", "joint_2_l", "joint_3_u",
                        "joint_4_r", "joint_5_b", "joint_6_t"};
        n_joints_ = joint_names_.size();

        est_vel_.assign(n_joints_, 0.0);
        committed_pos_.assign(n_joints_, 0.0);
        committed_vel_.assign(n_joints_, 0.0);

        pub_arm_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_path_command", 10);
        pub_stream_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_command", 10);

        sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 20,
            std::bind(&MotoMiniTrajStreamer::jointStateCallback, this, std::placeholders::_1));

        sub_traj_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/path_command", 10,
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
                    "Traj Streamer thread-safe ready (%.0f Hz, dt=%.3fs, blend_dur=%.3fs)",
                    rate_hz_, dt_, blend_dur_);
    }

private:
    enum State
    {
        STATE_WAIT_JOINT,
        STATE_ARMING,
        STATE_STREAMING,
        STATE_STOPPED
    };
    enum BlendState
    {
        BLEND_IDLE,
        BLEND_ACTIVE
    };


    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(sensor_state_mutex_);
        if (last_joint_state_)
        {
            const double dt_js = (rclcpp::Time(msg->header.stamp) -
                                  rclcpp::Time(last_joint_state_->header.stamp))
                                     .seconds();
            if (dt_js > 1e-4 && dt_js < 0.5)
            {
                for (size_t i = 0; i < n_joints_; ++i)
                {
                    auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
                    auto itp = std::find(last_joint_state_->name.begin(),
                                         last_joint_state_->name.end(), joint_names_[i]);
                    if (it != msg->name.end() && itp != last_joint_state_->name.end())
                    {
                        const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
                        const size_t idxp = static_cast<size_t>(
                            std::distance(last_joint_state_->name.begin(), itp));
                        const double raw = (msg->position[idx] - last_joint_state_->position[idxp]) / dt_js;
                        est_vel_[i] = 0.3 * raw + 0.7 * est_vel_[i]; 
                    }
                }
            }
        }
        last_joint_state_ = msg;
    }

    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (msg->points.size() <= 1) return;
        if (state_.load() != STATE_STREAMING) return;

        const double since_last = (this->now() - last_traj_accept_time_).seconds();
        if (last_traj_accept_time_.nanoseconds() > 0 && since_last < MIN_TRAJ_UPDATE_INTERVAL_S) {
            return;
        }
        last_traj_accept_time_ = this->now();

        bool expected = false;
        if (!worker_busy_.compare_exchange_strong(expected, true)) {
            RCLCPP_DEBUG(this->get_logger(), "Worker thread busy, dropping incoming trajectory.");
            return;
        }

        worker_future_ = std::async(std::launch::async, [this, msg]() {
            this->processTrajectory(msg);
            this->worker_busy_.store(false, std::memory_order_release);
        });
    }

    void processTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        std::vector<double> snap_pos, snap_vel;
        {
            std::lock_guard<std::mutex> lock(splice_state_mutex_);
            snap_pos = committed_pos_;
            snap_vel = committed_vel_;
        }
        
        std::vector<int> src_idx(n_joints_, -1);
        for (size_t i = 0; i < n_joints_; ++i) {
            auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), joint_names_[i]);
            if (it != msg->joint_names.end())
                src_idx[i] = static_cast<int>(std::distance(msg->joint_names.begin(), it));
        }

        double t_splice = 0.0;
        double min_cost = std::numeric_limits<double>::max();
        double min_splice_time = 2.0 * dt_; 

        if (!snap_pos.empty() && !snap_vel.empty()) {
            for (const auto &pt : msg->points) {
                double t_k = rclcpp::Duration(pt.time_from_start).seconds();
                if (t_k < min_splice_time) continue;

                double cost_pos = 0.0;
                double cost_vel = 0.0;
                for (size_t i = 0; i < n_joints_; ++i) {
                    int s = src_idx[i];
                    if (s < 0) continue;
                    if (static_cast<size_t>(s) < pt.positions.size()) {
                        double d_pos = pt.positions[static_cast<size_t>(s)] - snap_pos[i];
                        cost_pos += d_pos * d_pos;
                    }
                    if (static_cast<size_t>(s) < pt.velocities.size()) {
                        double d_vel = pt.velocities[static_cast<size_t>(s)] - snap_vel[i];
                        cost_vel += d_vel * d_vel;
                    }
                }
                double J = splice_w_pos_ * cost_pos + splice_w_vel_ * cost_vel;
                if (J < min_cost) {
                    min_cost = J;
                    t_splice = t_k;
                }
            }
        }

        const double t_end_msg = rclcpp::Duration(msg->points.back().time_from_start).seconds();
        t_splice = std::clamp(t_splice, 0.0, t_end_msg * 0.5);

        // Map into dense cache
        auto cache = std::make_shared<ProcessedCache>();
        cache->t_splice = t_splice;
        cache->dt = dt_;
        cache->created_at = this->now();

        size_t n_pts = msg->points.size();
        std::vector<std::array<double, 6>> raw_pos(n_pts);
        std::vector<std::array<double, 6>> raw_vel(n_pts);
        std::vector<double> raw_time(n_pts);

        for (size_t k = 0; k < n_pts; ++k) {
            raw_time[k] = rclcpp::Duration(msg->points[k].time_from_start).seconds();
            for (size_t i = 0; i < n_joints_; ++i) {
                int s = src_idx[i];
                if (s >= 0 && static_cast<size_t>(s) < msg->points[k].positions.size())
                    raw_pos[k][i] = msg->points[k].positions[s];
                else
                    raw_pos[k][i] = (k == 0) ? snap_pos[i] : raw_pos[k-1][i];

                // If explicit velocity is missing, default to 0.0 BUT note it so we can recompute via central difference
                if (s >= 0 && static_cast<size_t>(s) < msg->points[k].velocities.size() && !msg->points[k].velocities.empty())
                    raw_vel[k][i] = msg->points[k].velocities[s];
                else
                    raw_vel[k][i] = 0.0;
            }
        }

        // Handle missing velocities numerically (boundary safe)
        for (size_t i = 0; i < n_joints_; ++i) {
            int s = src_idx[i];
            for (size_t k = 0; k < n_pts; ++k) {
                if (msg->points[k].velocities.empty() || s < 0 || static_cast<size_t>(s) >= msg->points[k].velocities.size()) { // If missing
                    if (k == 0 && n_pts > 1)
                        raw_vel[k][i] = (raw_pos[1][i] - raw_pos[0][i]) / std::max(1e-9, raw_time[1] - raw_time[0]);
                    else if (k == n_pts - 1 && n_pts > 1)
                        raw_vel[k][i] = (raw_pos[k][i] - raw_pos[k-1][i]) / std::max(1e-9, raw_time[k] - raw_time[k-1]);
                    else if (k > 0 && k < n_pts - 1)
                        raw_vel[k][i] = (raw_pos[k+1][i] - raw_pos[k-1][i]) / std::max(1e-9, raw_time[k+1] - raw_time[k-1]);
                }
            }
        }

        // Rate Adaptation: Resample to dt_
        double total_time = raw_time.back();
        size_t dense_n = static_cast<size_t>(std::ceil(total_time / dt_)) + 1;
        cache->pos.resize(dense_n);
        cache->vel.resize(dense_n);
        cache->time.resize(dense_n);

        size_t raw_idx = 0;
        for (size_t i = 0; i < dense_n; ++i) {
            double t = i * dt_;
            cache->time[i] = t;

            while (raw_idx + 1 < n_pts - 1 && raw_time[raw_idx + 1] < t) {
                raw_idx++;
            }

            double t0 = raw_time[raw_idx];
            double t1 = raw_time[std::min(raw_idx + 1, n_pts - 1)];
            double T = std::max(1e-9, t1 - t0);
            
            for (size_t j = 0; j < n_joints_; ++j) {
                HermiteBlend h;
                h.p0 = raw_pos[raw_idx][j];
                h.v0 = raw_vel[raw_idx][j];
                h.p1 = raw_pos[std::min(raw_idx + 1, n_pts - 1)][j];
                h.v1 = raw_vel[std::min(raw_idx + 1, n_pts - 1)][j];
                h.T = T;
                
                cache->pos[i][j] = h.pos(t - t0);
                cache->vel[i][j] = h.vel(t - t0);
            }
        }
        cache->dense_filled = true;

        // Gaussian pre-splice shape
        size_t i_splice = static_cast<size_t>(t_splice / dt_);
        if (i_splice >= static_cast<size_t>(prep_ticks_) && cache->pos.size() > 3) {
            int start = std::max(2, static_cast<int>(i_splice) - prep_ticks_);
            int end = std::min(static_cast<int>(cache->pos.size() - 3), static_cast<int>(i_splice));
            
            double w[5] = {0.0625, 0.25, 0.375, 0.25, 0.0625};
            auto pos_copy = cache->pos;

            for (int k = start; k <= end; ++k) {
                for (size_t j = 0; j < n_joints_; ++j) {
                    double smoothed = 0.0;
                    for (int o = -2; o <= 2; ++o) {
                        smoothed += w[o+2] * pos_copy[k + o][j];
                    }
                    cache->pos[k][j] = smoothed;
                }
            }
            // Recompute velocity after smoothing
            for (int k = start; k <= end; ++k) {
                for (size_t j = 0; j < n_joints_; ++j) {
                    cache->vel[k][j] = (cache->pos[k+1][j] - cache->pos[k-1][j]) / (2.0 * dt_);
                }
            }
        }

        std::atomic_store_explicit(&active_cache_, cache, std::memory_order_release);
    }

    void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        // Wait for any in-flight worker to finish before resetting state
        if (worker_future_.valid()) {
            worker_future_.wait();
        }
        
        state_.store(STATE_WAIT_JOINT);
        blend_state_.store(BLEND_IDLE);
        tracked_pos_.clear();
        
        {
            std::lock_guard<std::mutex> lock(sensor_state_mutex_);
            est_vel_.assign(n_joints_, 0.0);
        }

        std::atomic_store_explicit(&active_cache_, std::shared_ptr<ProcessedCache>(nullptr), std::memory_order_release);
        arm_trigger_sent_ = false;
        stream_epoch_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_traj_accept_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        res->success = true;
        res->message = "Re-arming: waiting for joint state.";
        RCLCPP_INFO(this->get_logger(), "Start requested — re-arming.");
    }

    void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        // Wait for any in-flight worker to finish before resetting state
        if (worker_future_.valid()) {
            worker_future_.wait();
        }

        state_.store(STATE_STOPPED);
        blend_state_.store(BLEND_IDLE);
        std::atomic_store_explicit(&active_cache_, std::shared_ptr<ProcessedCache>(nullptr), std::memory_order_release);

        if (!tracked_pos_.empty())
        {
            const double t_stop = (this->now() - stream_epoch_).seconds() + dt_;
            publishStreamPoint(tracked_pos_, std::vector<double>(n_joints_, 0.0), t_stop);
        }
        res->success = true;
        res->message = "Streaming stopped.";
        RCLCPP_INFO(this->get_logger(), "Stop requested.");
    }

    bool initTrackedPositions()
    {
        std::lock_guard<std::mutex> lock(sensor_state_mutex_);
        if (!last_joint_state_)
            return false;
        tracked_pos_.assign(n_joints_, 0.0);
        for (size_t i = 0; i < n_joints_; ++i)
        {
            auto it = std::find(last_joint_state_->name.begin(),
                                last_joint_state_->name.end(), joint_names_[i]);
            if (it == last_joint_state_->name.end())
                return false;
            tracked_pos_[i] = last_joint_state_->position[static_cast<size_t>(std::distance(last_joint_state_->name.begin(), it))];
        }
        return true;
    }

    bool checkLimits(const std::vector<double> &pos) const
    {
        static const double lower[] = {
            JOINT_1_S_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_2_L_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_3_U_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_4_R_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_5_B_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD,
            JOINT_6_T_LOWER_LIMIT_RAD + SAFETY_JOINT_PADDING_RAD};
        static const double upper[] = {
            JOINT_1_S_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_2_L_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_3_U_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_4_R_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_5_B_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD,
            JOINT_6_T_UPPER_LIMIT_RAD - SAFETY_JOINT_PADDING_RAD};
        for (size_t i = 0; i < pos.size() && i < 6; ++i)
            if (pos[i] <= lower[i] || pos[i] >= upper[i])
                return false;
        return true;
    }

    void publishStreamPoint(const std::vector<double> &pos,
                            const std::vector<double> &vel,
                            double time_from_start)
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now();
        traj.joint_names = joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions = pos;
        pt.velocities = vel;
        pt.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
        traj.points.push_back(pt);
        pub_stream_->publish(traj);
    }

    void doStream()
    {
        std::vector<double> target_pos = tracked_pos_;
        std::vector<double> target_vel(n_joints_, 0.0);

        auto cache = std::atomic_load_explicit(&active_cache_, std::memory_order_acquire);

        // Detect new cache
        if (cache && cache->created_at != current_cache_time_) {
            current_cache_time_ = cache->created_at;
            
            double d_sq = 0.0;
            if (cache->pos.size() > 0) {
                // To check jump distance against splice point
                size_t i_splice = static_cast<size_t>(cache->t_splice / cache->dt);
                i_splice = std::min(i_splice, cache->pos.size() - 1);
                for (size_t i = 0; i < n_joints_; ++i) {
                    double d = cache->pos[i_splice][i] - tracked_pos_[i];
                    d_sq += d * d;
                }
            }

            if (d_sq > 0.0001) { // 10 mrad sq
                hermite_.resize(n_joints_);
                size_t i_splice = static_cast<size_t>(cache->t_splice / cache->dt);
                i_splice = std::min(i_splice, cache->pos.size() - 1);

                std::vector<double> local_est_vel;
                {
                    std::lock_guard<std::mutex> lock(sensor_state_mutex_);
                    local_est_vel = est_vel_;
                }

                for (size_t i = 0; i < n_joints_; ++i) {
                    hermite_[i].p0 = tracked_pos_[i];
                    hermite_[i].v0 = local_est_vel[i];
                    hermite_[i].p1 = cache->pos[i_splice][i];
                    hermite_[i].v1 = cache->vel[i_splice][i];
                    hermite_[i].T = blend_dur_;
                }
                blend_state_.store(BLEND_ACTIVE);
                blend_start_time_ = this->now();
            } else {
                blend_state_.store(BLEND_IDLE);
            }
            traj_start_time_ = this->now() - rclcpp::Duration::from_seconds(cache->t_splice);
        }

        if (blend_state_.load() == BLEND_ACTIVE)
        {
            const double be = (this->now() - blend_start_time_).seconds();
            if (be >= blend_dur_)
            {
                blend_state_.store(BLEND_IDLE);
                if (cache && cache->pos.size() > 0) {
                    double min_d = std::numeric_limits<double>::max();
                    double t_resplice = 0.0;
                    for (size_t k = 0; k < cache->pos.size(); ++k) {
                        double d = 0.0;
                        for(size_t i = 0; i < n_joints_; ++i) {
                            double dd = cache->pos[k][i] - tracked_pos_[i];
                            d += dd * dd;
                        }
                        if (d < min_d) {
                            min_d = d;
                            t_resplice = cache->time[k];
                        }
                    }
                    traj_start_time_ = this->now() - rclcpp::Duration::from_seconds(t_resplice);
                } else {
                    traj_start_time_ = this->now();
                }
            }
            else
            {
                for (size_t i = 0; i < n_joints_; ++i)
                {
                    target_pos[i] = hermite_[i].pos(be);
                    target_vel[i] = hermite_[i].vel(be);
                }
                
                applyLeadClamp(target_pos, target_vel);
                if (checkLimits(target_pos))
                    tracked_pos_ = target_pos;
                else
                    target_vel.assign(n_joints_, 0.0);
                
                {
                    std::lock_guard<std::mutex> lock(splice_state_mutex_);
                    committed_pos_ = tracked_pos_;
                    committed_vel_ = target_vel;
                }
                publishStreamPoint(tracked_pos_, target_vel, (this->now() - stream_epoch_).seconds());
                return;
            }
        }

        if (cache && cache->pos.size() > 0)
        {
            const double elapsed = (this->now() - traj_start_time_).seconds();
            if (elapsed >= 0.0) { // Safety bound!
                if (cache->dense_filled) {
                    double inv_dt = 1.0 / cache->dt;
                    size_t idx = static_cast<size_t>(elapsed * inv_dt);
                    if (idx >= cache->pos.size()) {
                        auto &last_p = cache->pos.back();
                        for(size_t i=0; i<n_joints_; ++i) {
                            target_pos[i] = last_p[i];
                            target_vel[i] = 0.0;
                        }
                    } else {
                        for(size_t i=0; i<n_joints_; ++i) {
                            target_pos[i] = cache->pos[idx][i];
                            target_vel[i] = cache->vel[idx][i];
                        }
                    }
                }
            }
        }

        applyLeadClamp(target_pos, target_vel);

        if (!checkLimits(target_pos))
        {
            target_vel.assign(n_joints_, 0.0);
        }
        else
        {
            tracked_pos_ = target_pos;
        }

        {
            std::lock_guard<std::mutex> lock(splice_state_mutex_);
            committed_pos_ = tracked_pos_;
            committed_vel_ = target_vel;
        }

        publishStreamPoint(tracked_pos_, target_vel, (this->now() - stream_epoch_).seconds());
    }

    void applyLeadClamp(std::vector<double> &pos, std::vector<double> &vel)
    {
        sensor_msgs::msg::JointState::SharedPtr local_js;
        {
            std::lock_guard<std::mutex> lock(sensor_state_mutex_);
            local_js = last_joint_state_;
        }
        
        if (!local_js)
            return;
        for (size_t i = 0; i < n_joints_; ++i)
        {
            auto it = std::find(local_js->name.begin(),
                                local_js->name.end(), joint_names_[i]);
            if (it == local_js->name.end())
                continue;
            const double actual = local_js->position[static_cast<size_t>(std::distance(local_js->name.begin(), it))];
            const double lead = pos[i] - actual;
            if (std::abs(lead) > MAX_LEAD_RAD)
            {
                const double scale = MAX_LEAD_RAD / std::abs(lead);
                pos[i] = actual + std::copysign(MAX_LEAD_RAD, lead);
                vel[i] *= scale;
            }
        }
    }

    void tick()
    {
        switch (state_.load())
        {
        case STATE_STOPPED:
            return;

        case STATE_WAIT_JOINT:
            if (!initTrackedPositions())
                return;
            arm_entry_time_ = this->now();
            arm_trigger_sent_ = false;
            state_.store(STATE_ARMING);
            return;

        case STATE_ARMING:
        {
            const double elapsed = (this->now() - arm_entry_time_).seconds();
            if (!arm_trigger_sent_)
            {
                if (elapsed < 0.5)
                    return;
                trajectory_msgs::msg::JointTrajectory traj;
                traj.header.stamp = this->now();
                traj.joint_names = joint_names_;
                trajectory_msgs::msg::JointTrajectoryPoint pt;
                pt.positions = tracked_pos_;
                pt.velocities.assign(n_joints_, 0.0);
                pt.time_from_start = rclcpp::Duration::from_seconds(0.5);
                traj.points.push_back(pt);
                pub_arm_->publish(traj);
                arm_trigger_sent_ = true;
                return;
            }
            if (elapsed < 1.5)
                return;
            if (!initTrackedPositions())
                return;
                
            {
                std::lock_guard<std::mutex> lock(sensor_state_mutex_);
                est_vel_.assign(n_joints_, 0.0);
            }
            stream_epoch_ = this->now(); 
            traj_start_time_ = stream_epoch_;
            {
                std::lock_guard<std::mutex> lock(splice_state_mutex_);
                committed_pos_ = tracked_pos_;
                committed_vel_.assign(n_joints_, 0.0);
            }
            publishStreamPoint(tracked_pos_, std::vector<double>(n_joints_, 0.0), 0.0);
            state_.store(STATE_STREAMING);
            RCLCPP_INFO(this->get_logger(), "Streaming started.");
            return;
        }

        case STATE_STREAMING:
            doStream();
            return;
        }
    }

    std::vector<std::string> joint_names_;
    size_t n_joints_{6};
    double rate_hz_{50.0};
    double dt_{0.02};
    double blend_dur_{0.04};                        
    rclcpp::Time stream_epoch_{0, 0, RCL_ROS_TIME}; 

    std::atomic<State> state_{STATE_WAIT_JOINT};
    std::atomic<BlendState> blend_state_{BLEND_IDLE};

    rclcpp::Time arm_entry_time_{0, 0, RCL_ROS_TIME};
    bool arm_trigger_sent_{false};

    rclcpp::Time last_traj_accept_time_{0, 0, RCL_ROS_TIME};

    std::vector<double> tracked_pos_;
    std::vector<double> est_vel_;

    rclcpp::Time traj_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time current_cache_time_{0, 0, RCL_ROS_TIME};

    std::vector<HermiteBlend> hermite_;
    rclcpp::Time blend_start_time_{0, 0, RCL_ROS_TIME};

    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_stream_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_traj_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_, srv_stop_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Concurrency
    std::future<void> worker_future_;
    std::atomic<bool> worker_busy_{false};
    

    std::shared_ptr<ProcessedCache> active_cache_{nullptr};
    
    std::mutex sensor_state_mutex_;
    std::mutex splice_state_mutex_;

    std::vector<double> committed_pos_;
    std::vector<double> committed_vel_;

    double splice_w_pos_{1.0};
    double splice_w_vel_{0.5};
    int prep_ticks_{4};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotoMiniTrajStreamer>());
    rclcpp::shutdown();
    return 0;
}
