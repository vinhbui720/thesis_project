/**
 * @file motomini_traj_streamer.cpp
 * @brief Motomini trajectory streamer — MPC-aware trajectory executor.
 *
 * Design rationale (v5):
 * ─────────────────────────────────────────────────────────────────────────────
 *  Dual-mode architecture. The worker thread decides mode on each new trajectory:
 *
 *  Mode A — FAITHFUL (header.frame_id == "new_path" OR first trajectory):
 *    Dense Hermite resampling only. No filtering. Geometric integrity preserved.
 *    RT thread starts from t=0 of the new trajectory.
 *
 *  Mode B — SPLICE (continuous MPC re-plans):
 *    1. Dense Hermite resampling.
 *    2. Full FIR Gaussian filter (removes MPC numeric chatter).
 *    3. Find merge index i_match (closest cache point to committed state).
 *    4. Hermite bridge: (snap_pos, snap_vel) → cache[i_match+hw].
 *       Guarantees vel[i_match] == snap_vel — zero velocity discontinuity.
 *    5. RT thread starts reading from i_match (the bridge entry).
 *
 *  RT thread (doStream) is a pure O(1) cache lookup — no clamping, no
 *  integrator, no blending. All smoothing is offline in the worker.
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
#include <std_msgs/msg/bool.hpp>
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

// Minimum interval between accepted trajectories (prevents same-ms spam)
static constexpr double MIN_TRAJ_UPDATE_INTERVAL_S = 0.02; // 20 ms

// Sigmoid handover window for Mode B (±ticks around splice point)
static constexpr int SPLICE_HALF_WINDOW = 4; // ±4 ticks = 80 ms at 50 Hz

// ============================================================================
// Data Structures
// ============================================================================

// Processing mode selected by the worker thread per incoming trajectory.
//   FAITHFUL — single/fresh path: Hermite resample only, no filtering, no blend.
//   SPLICE   — MPC continuous mode: sigmoid handover + full FIR filter.
enum class ProcessingMode
{
    FAITHFUL,
    SPLICE
};

struct ProcessedCache
{
    std::vector<std::array<double, 6>> pos;
    std::vector<std::array<double, 6>> vel;
    std::vector<double> time;

    double t_splice{0.0};
    double dt{0.02};
    bool dense_filled{false};
    bool skip_blend{false}; // Mode A: RT thread skips Hermite blend
    ProcessingMode mode{ProcessingMode::SPLICE};
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
        this->declare_parameter<double>("rate_hz", 125.0);
        this->declare_parameter<double>("splice_w_pos", 1.0);
        this->declare_parameter<double>("splice_w_vel", 0.5);
        this->declare_parameter<bool>("interpolate_stream_output", true);
        this->declare_parameter<double>("min_traj_update_interval_sec", 0.02);

        rate_hz_ = this->get_parameter("rate_hz").as_double();
        dt_ = 1.0 / std::max(1.0, rate_hz_);
        splice_w_pos_ = this->get_parameter("splice_w_pos").as_double();
        splice_w_vel_ = this->get_parameter("splice_w_vel").as_double();
        interpolate_stream_output_ =
            this->get_parameter("interpolate_stream_output").as_bool();
        min_traj_update_interval_sec_ =
            this->get_parameter("min_traj_update_interval_sec").as_double();

        joint_names_ = {"joint_1_s", "joint_2_l", "joint_3_u",
                        "joint_4_r", "joint_5_b", "joint_6_t"};
        n_joints_ = joint_names_.size();

        est_vel_.assign(n_joints_, 0.0);
        committed_pos_.assign(n_joints_, 0.0);
        committed_vel_.assign(n_joints_, 0.0);

        pub_arm_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_path_command", 10);
        pub_stream_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_command", 10);
        pub_execution_state_ = this->create_publisher<std_msgs::msg::Bool>(
            "/trajectory_executing", rclcpp::QoS(1).transient_local());
        publishExecutionState(false);

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
                    "Traj Streamer ready (%.0f Hz, dt=%.3fs, splice_window=%d ticks)",
                    rate_hz_, dt_, SPLICE_HALF_WINDOW);
    }

private:
    void publishExecutionState(bool executing)
    {
        if (execution_state_initialized_ && execution_state_ == executing)
            return;

        std_msgs::msg::Bool msg;
        msg.data = executing;
        pub_execution_state_->publish(msg);
        execution_state_ = executing;
        execution_state_initialized_ = true;
    }

    enum State
    {
        STATE_WAIT_JOINT,
        STATE_ARMING,
        STATE_STREAMING,
        STATE_STOPPED
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
        if (msg->points.size() <= 1)
            return;
        if (state_.load() != STATE_STREAMING)
            return;

        const double since_last = (this->now() - last_traj_accept_time_).seconds();
        const double min_update_interval =
            std::max(0.0, min_traj_update_interval_sec_);
        if (last_traj_accept_time_.nanoseconds() > 0 &&
            since_last < min_update_interval)
        {
            return;
        }
        last_traj_accept_time_ = this->now();

        bool expected = false;
        if (!worker_busy_.compare_exchange_strong(expected, true))
        {
            RCLCPP_DEBUG(this->get_logger(), "Worker thread busy, dropping incoming trajectory.");
            return;
        }

        worker_future_ = std::async(std::launch::async, [this, msg]()
                                    {
            this->processTrajectory(msg);
            this->worker_busy_.store(false, std::memory_order_release); });
    }

    void processTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        // ── Mode Detection ────────────────────────────────────────────────────
        // Mode A (FAITHFUL): first trajectory ever, OR planner signals "new_path"
        //   via header.frame_id. Hermite resample only — no FIR, no sigmoid.
        // Mode B (SPLICE): rapid MPC re-plans. Sigmoid handover at splice ±4
        //   ticks + full FIR to remove numeric chatter.
        auto existing = std::atomic_load_explicit(&active_cache_, std::memory_order_acquire);
        const bool is_faithful = (msg->header.frame_id == "new_path") || (!existing);

        // ── Snap committed state ──────────────────────────────────────────────
        std::vector<double> snap_pos, snap_vel;
        {
            std::lock_guard<std::mutex> lock(splice_state_mutex_);
            snap_pos = committed_pos_;
            snap_vel = committed_vel_;
        }

        // ── Joint index mapping ───────────────────────────────────────────────
        std::vector<int> src_idx(n_joints_, -1);
        for (size_t i = 0; i < n_joints_; ++i)
        {
            auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), joint_names_[i]);
            if (it != msg->joint_names.end())
                src_idx[i] = static_cast<int>(std::distance(msg->joint_names.begin(), it));
        }

        // ── Splice Point (Mode B only; Mode A always t_splice = 0) ───────────
        double t_splice = 0.0;
        if (!is_faithful && !snap_pos.empty() && !snap_vel.empty())
        {
            double min_cost = std::numeric_limits<double>::max();
            const double min_splice_time = 2.0 * dt_;
            for (const auto &pt : msg->points)
            {
                const double t_k = rclcpp::Duration(pt.time_from_start).seconds();
                if (t_k < min_splice_time)
                    continue;
                double cost_pos = 0.0, cost_vel = 0.0;
                for (size_t i = 0; i < n_joints_; ++i)
                {
                    int s = src_idx[i];
                    if (s < 0)
                        continue;
                    if (static_cast<size_t>(s) < pt.positions.size())
                    {
                        double d = pt.positions[static_cast<size_t>(s)] - snap_pos[i];
                        cost_pos += d * d;
                    }
                    if (static_cast<size_t>(s) < pt.velocities.size())
                    {
                        double d = pt.velocities[static_cast<size_t>(s)] - snap_vel[i];
                        cost_vel += d * d;
                    }
                }
                const double J = splice_w_pos_ * cost_pos + splice_w_vel_ * cost_vel;
                if (J < min_cost)
                {
                    min_cost = J;
                    t_splice = t_k;
                }
            }
            const double t_end = rclcpp::Duration(msg->points.back().time_from_start).seconds();
            t_splice = std::clamp(t_splice, 0.0, t_end * 0.5);
        }

        // ── Build cache header ────────────────────────────────────────────────
        auto cache = std::make_shared<ProcessedCache>();
        cache->t_splice = t_splice;
        cache->dt = dt_;
        cache->created_at = this->now();
        cache->mode = is_faithful ? ProcessingMode::FAITHFUL : ProcessingMode::SPLICE;
        cache->skip_blend = is_faithful;

        // ── Raw data extraction ───────────────────────────────────────────────
        const size_t n_pts = msg->points.size();
        std::vector<std::array<double, 6>> raw_pos(n_pts);
        std::vector<std::array<double, 6>> raw_vel(n_pts);
        std::vector<double> raw_time(n_pts);

        for (size_t k = 0; k < n_pts; ++k)
        {
            raw_time[k] = rclcpp::Duration(msg->points[k].time_from_start).seconds();
            for (size_t i = 0; i < n_joints_; ++i)
            {
                const int s = src_idx[i];
                raw_pos[k][i] = (s >= 0 && static_cast<size_t>(s) < msg->points[k].positions.size())
                                    ? msg->points[k].positions[static_cast<size_t>(s)]
                                    : ((k == 0) ? snap_pos[i] : raw_pos[k - 1][i]);
                raw_vel[k][i] = (s >= 0 && !msg->points[k].velocities.empty() && static_cast<size_t>(s) < msg->points[k].velocities.size())
                                    ? msg->points[k].velocities[static_cast<size_t>(s)]
                                    : 0.0;
            }
        }
        // Fill missing velocities via finite differences
        for (size_t i = 0; i < n_joints_; ++i)
        {
            const int s = src_idx[i];
            for (size_t k = 0; k < n_pts; ++k)
            {
                if (msg->points[k].velocities.empty() || s < 0 ||
                    static_cast<size_t>(s) >= msg->points[k].velocities.size())
                {
                    if (k == 0 && n_pts > 1)
                        raw_vel[k][i] = (raw_pos[1][i] - raw_pos[0][i]) /
                                        std::max(1e-9, raw_time[1] - raw_time[0]);
                    else if (k == n_pts - 1 && n_pts > 1)
                        raw_vel[k][i] = (raw_pos[k][i] - raw_pos[k - 1][i]) /
                                        std::max(1e-9, raw_time[k] - raw_time[k - 1]);
                    else if (k > 0 && k < n_pts - 1)
                        raw_vel[k][i] = (raw_pos[k + 1][i] - raw_pos[k - 1][i]) /
                                        std::max(1e-9, raw_time[k + 1] - raw_time[k - 1]);
                }
            }
        }

        // ── Dense Hermite Resampling (both modes) ─────────────────────────────
        const double total_time = raw_time.back();
        const size_t dense_n = static_cast<size_t>(std::ceil(total_time / dt_)) + 1;
        cache->pos.resize(dense_n);
        cache->vel.resize(dense_n);
        cache->time.resize(dense_n);

        size_t raw_idx = 0;
        for (size_t i = 0; i < dense_n; ++i)
        {
            const double t = i * dt_;
            cache->time[i] = t;
            while (raw_idx + 1 < n_pts - 1 && raw_time[raw_idx + 1] < t)
                raw_idx++;

            const double t0 = raw_time[raw_idx];
            const double t1 = raw_time[std::min(raw_idx + 1, n_pts - 1)];
            const double T = std::max(1e-9, t1 - t0);
            for (size_t j = 0; j < n_joints_; ++j)
            {
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

        // ── Mode B Post-Processing ────────────────────────────────────────────
        if (!is_faithful)
        {
            const int n = static_cast<int>(cache->pos.size());

            // Step 1 — Full FIR Gaussian filter: removes MPC numeric noise from the
            // resampled trajectory before we compute the merge waypoints.
            if (n > 4)
            {
                static const double w[5] = {0.0625, 0.25, 0.375, 0.25, 0.0625};
                auto pos_copy = cache->pos;
                for (int k = 2; k < n - 2; ++k)
                {
                    for (size_t j = 0; j < n_joints_; ++j)
                    {
                        double s = 0.0;
                        for (int o = -2; o <= 2; ++o)
                            s += w[o + 2] * pos_copy[k + o][j];
                        cache->pos[k][j] = s;
                    }
                }
                for (int k = 1; k < n - 1; ++k)
                {
                    for (size_t j = 0; j < n_joints_; ++j)
                        cache->vel[k][j] = (cache->pos[k + 1][j] - cache->pos[k - 1][j]) / (2.0 * dt_);
                }
            }

            // Step 2 — Find merge index: point in the FIR-smoothed cache closest to the
            // committed state. This is where the robot currently "is" in the new plan.
            size_t i_match = 0;
            {
                double min_d = std::numeric_limits<double>::max();
                for (size_t k = 0; k < cache->pos.size(); ++k)
                {
                    double d = 0.0;
                    for (size_t j = 0; j < n_joints_; ++j)
                    {
                        const double dd = cache->pos[k][j] - snap_pos[j];
                        d += dd * dd;
                    }
                    if (d < min_d)
                    {
                        min_d = d;
                        i_match = k;
                    }
                }
            }

            // Step 3 — Hermite bridge: overwrite cache[i_match .. i_match+hw] with a
            // cubic curve that starts at (snap_pos, snap_vel) and lands on the
            // FIR-smoothed trajectory hw ticks later.
            // This guarantees:  vel[i_match] == snap_vel  (zero velocity discontinuity).
            const int hw = SPLICE_HALF_WINDOW;
            const size_t i_end = std::min(i_match + static_cast<size_t>(hw),
                                          cache->pos.size() - 1);
            if (i_end > i_match && !snap_pos.empty())
            {
                const double T_bridge = static_cast<double>(i_end - i_match) * dt_;
                for (size_t j = 0; j < n_joints_; ++j)
                {
                    HermiteBlend h;
                    h.p0 = snap_pos[j];
                    h.v0 = snap_vel[j];
                    h.p1 = cache->pos[i_end][j];
                    h.v1 = cache->vel[i_end][j];
                    h.T = T_bridge;
                    for (size_t k = i_match; k <= i_end; ++k)
                    {
                        const double t_local = static_cast<double>(k - i_match) * dt_;
                        cache->pos[k][j] = h.pos(t_local);
                        cache->vel[k][j] = h.vel(t_local);
                    }
                }
            }

            // Step 4 — Advance splice time so the RT thread starts reading at i_match
            // (the bridge entry point = robot's current position).
            cache->t_splice = static_cast<double>(i_match) * dt_;

            RCLCPP_DEBUG(this->get_logger(),
                         "Mode B: i_match=%zu t_splice=%.3fs bridge→i_end=%zu", i_match, cache->t_splice, i_end);
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Mode A: faithful, %zu dense pts, no filtering", dense_n);
        }

        std::atomic_store_explicit(&active_cache_, cache, std::memory_order_release);
        // A valid cache means there are waypoints available for execution.
        publishExecutionState(true);
    }

    void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        // Wait for any in-flight worker to finish before resetting state
        if (worker_future_.valid())
        {
            worker_future_.wait();
        }

        state_.store(STATE_WAIT_JOINT);
        tracked_pos_.clear();

        {
            std::lock_guard<std::mutex> lock(sensor_state_mutex_);
            est_vel_.assign(n_joints_, 0.0);
        }

        std::atomic_store_explicit(&active_cache_, std::shared_ptr<ProcessedCache>(nullptr), std::memory_order_release);
        arm_trigger_sent_ = false;
        publishExecutionState(false);
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
        if (worker_future_.valid())
        {
            worker_future_.wait();
        }

        state_.store(STATE_STOPPED);
        std::atomic_store_explicit(&active_cache_, std::shared_ptr<ProcessedCache>(nullptr), std::memory_order_release);
        publishExecutionState(false);

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

    void sampleCache(const std::shared_ptr<ProcessedCache> &cache,
                     double elapsed,
                     std::vector<double> &out_pos,
                     std::vector<double> &out_vel) const
    {
        if (!cache || !cache->dense_filled || cache->pos.empty())
            return;

        if (elapsed <= 0.0)
        {
            for (size_t i = 0; i < n_joints_; ++i)
            {
                out_pos[i] = cache->pos.front()[i];
                out_vel[i] = cache->vel.front()[i];
            }
            return;
        }

        if (elapsed >= cache->time.back())
        {
            for (size_t i = 0; i < n_joints_; ++i)
            {
                out_pos[i] = cache->pos.back()[i];
                out_vel[i] = 0.0;
            }
            return;
        }

        const auto upper = std::upper_bound(cache->time.begin(),
                                            cache->time.end(),
                                            elapsed);
        size_t idx = static_cast<size_t>(
            std::distance(cache->time.begin(), upper));
        idx = std::clamp(idx, static_cast<size_t>(1), cache->time.size() - 1);
        const size_t i0 = idx - 1;
        const size_t i1 = idx;

        const double t0 = cache->time[i0];
        const double t1 = cache->time[i1];
        const double T = std::max(1e-9, t1 - t0);
        const double local_t = elapsed - t0;

        for (size_t i = 0; i < n_joints_; ++i)
        {
            if (interpolate_stream_output_)
            {
                HermiteBlend h;
                h.p0 = cache->pos[i0][i];
                h.v0 = cache->vel[i0][i];
                h.p1 = cache->pos[i1][i];
                h.v1 = cache->vel[i1][i];
                h.T = T;
                out_pos[i] = h.pos(local_t);
                out_vel[i] = h.vel(local_t);
            }
            else
            {
                out_pos[i] = cache->pos[i0][i];
                out_vel[i] = cache->vel[i0][i];
            }
        }
    }

    // RT thread: pure O(1) cache lookup — no clamping, no integrator.
    // All smoothing is pre-computed offline by the worker thread.
    void doStream()
    {
        auto cache = std::atomic_load_explicit(&active_cache_, std::memory_order_acquire);

        // New cache arrived — align stream clock to the bridge entry point
        if (cache && cache->created_at != current_cache_time_)
        {
            current_cache_time_ = cache->created_at;
            traj_start_time_ = this->now() - rclcpp::Duration::from_seconds(cache->t_splice);
            RCLCPP_DEBUG(this->get_logger(), "[RT] Cache swap: %s | pts=%zu t_splice=%.3fs",
                         cache->mode == ProcessingMode::FAITHFUL ? "Mode-A" : "Mode-B",
                         cache->pos.size(), cache->t_splice);
        }

        std::vector<double> out_pos = tracked_pos_;
        std::vector<double> out_vel(n_joints_, 0.0);

        if (cache && cache->dense_filled && !cache->pos.empty())
        {
            const double elapsed = (this->now() - traj_start_time_).seconds();
            if (elapsed >= 0.0)
            {
                sampleCache(cache, elapsed, out_pos, out_vel);
                if (elapsed >= cache->time.back())
                {
                    const bool was_executing = execution_state_;
                    publishExecutionState(false);
                    if (was_executing)
                    {
                        RCLCPP_INFO(this->get_logger(),
                                    "Final waypoint sent from active cache; /trajectory_executing=false.");
                    }
                }
            }
        }

        if (!checkLimits(out_pos))
        {
            out_vel.assign(n_joints_, 0.0);
        }
        else
        {
            tracked_pos_ = out_pos;
        }

        {
            std::lock_guard<std::mutex> lock(splice_state_mutex_);
            committed_pos_ = tracked_pos_;
            committed_vel_ = out_vel;
        }

        publishStreamPoint(tracked_pos_, out_vel, (this->now() - stream_epoch_).seconds());
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
    bool interpolate_stream_output_{true};
    double min_traj_update_interval_sec_{MIN_TRAJ_UPDATE_INTERVAL_S};
    rclcpp::Time stream_epoch_{0, 0, RCL_ROS_TIME};

    std::atomic<State> state_{STATE_WAIT_JOINT};

    rclcpp::Time arm_entry_time_{0, 0, RCL_ROS_TIME};
    bool arm_trigger_sent_{false};

    rclcpp::Time last_traj_accept_time_{0, 0, RCL_ROS_TIME};

    std::vector<double> tracked_pos_;
    std::vector<double> est_vel_;

    rclcpp::Time traj_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time current_cache_time_{0, 0, RCL_ROS_TIME};

    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_stream_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_execution_state_;
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
    bool execution_state_{false};
    bool execution_state_initialized_{false};

    double splice_w_pos_{1.0};
    double splice_w_vel_{0.5};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotoMiniTrajStreamer>());
    rclcpp::shutdown();
    return 0;
}
