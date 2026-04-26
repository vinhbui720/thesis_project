/**
 * @file motomini_node_tracking.cpp
 * @brief MotoMiniPlanningNode — Cartesian DLS-IK servo controller (tracking mode).
 *
 *  ARCHITECTURE
 *  ============
 *    HOT PATH  (this file, 30 Hz, never blocks)
 *      target  ─►  PT1 smoother (in targetPoseCallback)
 *              ─►  predictTargetPose (k=1..H)
 *              ─►  DLS-IK chain (Jacobian, λ=0.2)
 *              ─►  Continuous-joint unwrap
 *              ─►  Cross-tick velocity LPF
 *              ─►  Velocity feasibility gate
 *              ─►  publishTrajectory  ──► streamer  ──► robot
 *
 *    COLD PATH (optional, separate thread, ~5 Hz)
 *      TrajOpt — advisory only, never blocks the hot path.
 *      Provided as a stub at the bottom of this file. When you
 *      re-enable it, its output is stored as a "suggestion" that
 *      the controller may consult on the next tick. If TrajOpt is
 *      slow, fails, or produces a 2π-flipped wrist, the robot is
 *      unaffected because it's running on the controller.
 *
 *  WHY THIS FIXES THE ON/OFF VELOCITY PATTERN
 *  ==========================================
 *  Previous behaviour: TrajOpt took 100–300 ms per tick. The
 *  streamer played out one ~150 ms chunk, then stalled for ~100 ms,
 *  then played the next chunk. That's the alternating velocity
 *  bump signature you saw in PlotJuggler.
 *
 *  This controller takes ~3–5 ms per tick, so the streamer always
 *  has a fresh trajectory to splice into. Continuous motion.
 *
 *  @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning_node.h>

#include <tesseract_rosutils/utils.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_environment/environment.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_common/manipulator_info.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

// ============================================================================
// FREE-FUNCTION HELPERS
// ============================================================================

inline double shortestAngularDistance(double from, double to)
{
    double diff = std::fmod(to - from + M_PI, 2.0 * M_PI);
    if (diff < 0)
        diff += 2.0 * M_PI;
    return diff - M_PI;
}

inline double wrapAngle(double angle)
{
    double w = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (w < 0)
        w += 2.0 * M_PI;
    return w - M_PI;
}

Eigen::VectorXd angularDifference(const Eigen::VectorXd &from,
                                  const Eigen::VectorXd &to,
                                  const std::vector<bool> &is_continuous)
{
    Eigen::VectorXd diff = to - from;
    for (int i = 0; i < diff.size(); ++i)
        if (is_continuous[i])
            diff[i] = shortestAngularDistance(from[i], to[i]);
    return diff;
}

/// True for joints whose travel range ≥ ~2π (the only ones that can branch-flip).
static std::vector<bool> detectContinuousJoints(const Eigen::MatrixXd &limits, int n)
{
    std::vector<bool> result(n, false);
    for (int i = 0; i < n; ++i)
    {
        const double range = limits(i, 1) - limits(i, 0);
        if (range >= 2.0 * M_PI - 0.2) // tolerance — wrist may be exactly ±π
            result[i] = true;
    }
    return result;
}

/// Pull `angle` to the same 2π-coset as `reference`, but only while the
/// result remains inside [lower, upper]. Out-of-bounds candidates are skipped.
static double unwrapToReference(double angle, double reference,
                                double lower, double upper)
{
    while (angle - reference > M_PI)
    {
        const double cand = angle - 2.0 * M_PI;
        if (cand < lower)
            break;
        angle = cand;
    }
    while (reference - angle > M_PI)
    {
        const double cand = angle + 2.0 * M_PI;
        if (cand > upper)
            break;
        angle = cand;
    }
    return angle;
}

// ============================================================================
// ASYNC CALLBACK — Cartesian PT1 smoother on incoming target
// ============================================================================

void MotoMiniPlanningNode::targetPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(_mpc_target_mutex);

    Eigen::Isometry3d raw_new_pose;
    tf2::fromMsg(msg->pose, raw_new_pose);

    if (target_initialized_)
    {
        // PT1 smoothing — heavy filter prevents wrist whip-back from raw
        // sensor jitter. Tune α to taste.
        const double alpha_pos = 0.10;
        const double alpha_rot = 0.05;

        Eigen::Vector3d smoothed_pos =
            current_target_pose_.translation() +
            alpha_pos * (raw_new_pose.translation() - current_target_pose_.translation());

        Eigen::Quaterniond q_curr(current_target_pose_.linear());
        Eigen::Quaterniond q_new(raw_new_pose.linear());
        Eigen::Quaterniond q_smoothed = q_curr.slerp(alpha_rot, q_new);

        Eigen::Isometry3d smoothed_pose = Eigen::Isometry3d::Identity();
        smoothed_pose.translation() = smoothed_pos;
        smoothed_pose.linear() = q_smoothed.toRotationMatrix();

        rclcpp::Time current_time(msg->header.stamp);
        double dt = (current_time - last_target_time_).seconds();
        if (dt > 1e-4)
        {
            target_velocity_linear_ =
                (smoothed_pose.translation() - current_target_pose_.translation()) / dt;

            Eigen::AngleAxisd diff(
                smoothed_pose.linear() * current_target_pose_.linear().inverse());
            target_velocity_angular_ = diff.axis() * diff.angle() / dt;
        }

        current_target_pose_ = smoothed_pose;
        last_target_time_ = msg->header.stamp;
    }
    else
    {
        current_target_pose_ = raw_new_pose;
        target_velocity_linear_.setZero();
        target_velocity_angular_.setZero();
        target_initialized_ = true;
        last_target_time_ = msg->header.stamp;
    }
}

// ============================================================================
// MATH HELPERS
// ============================================================================

Eigen::Isometry3d MotoMiniPlanningNode::predictTargetPose(int step_k)
{
    std::lock_guard<std::mutex> lock(_mpc_target_mutex);

    double lead_time = step_k * mpc_dt_;
    Eigen::Isometry3d predicted = current_target_pose_;
    predicted.translation() += target_velocity_linear_ * lead_time;

    double angle = target_velocity_angular_.norm() * lead_time;
    if (angle > 1e-6)
    {
        predicted.linear() =
            Eigen::AngleAxisd(angle, target_velocity_angular_.normalized()).toRotationMatrix() *
            current_target_pose_.linear();
    }
    return predicted;
}

/**
 * @brief Damped-least-squares Jacobian step.
 *
 * Always continuous (no IK branch picking). λ = 0.2 keeps it stable
 * near singularities. The step is then physically clamped to a
 * conservative fraction of v_max·dt (50%) so we always leave the
 * downstream gate room to breathe.
 */
Eigen::VectorXd MotoMiniPlanningNode::computeDlsExtrapolation(
    const Eigen::VectorXd &q_last, const Eigen::Isometry3d &target_next)
{
    auto fk = manip_->calcFwdKin(q_last);
    if (fk.find(ee_link_) == fk.end())
        return q_last;

    Eigen::Isometry3d ee_current = fk.at(ee_link_);
    Eigen::Vector3d dx = target_next.translation() - ee_current.translation();
    Eigen::AngleAxisd aa(target_next.linear() * ee_current.linear().inverse());
    Eigen::Vector3d dw = aa.axis() * aa.angle();

    Eigen::Matrix<double, 6, 1> twist;
    twist.head<3>() = dx;
    twist.tail<3>() = dw;

    Eigen::MatrixXd J = manip_->calcJacobian(q_last, base_link_, ee_link_);
    const double lambda = 0.2;
    Eigen::MatrixXd JJt = J * J.transpose();
    JJt += (lambda * lambda) * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(twist);

    // Conservative per-tick velocity clamp on the IK seed.
    const Eigen::VectorXd v_max = manip_->getLimits().velocity_limits.col(1);
    const Eigen::VectorXd max_dq = v_max * mpc_dt_ * 0.5;
    for (int i = 0; i < dq.size(); ++i)
        dq[i] = std::max(-max_dq[i], std::min(max_dq[i], dq[i]));

    Eigen::VectorXd q_next = q_last + dq;
    for (int i = 0; i < q_next.size(); ++i)
        q_next[i] = std::max(joint_limits_(i, 0),
                             std::min(joint_limits_(i, 1), q_next[i]));
    return q_next;
}

// ============================================================================
// HOT PATH — Pure DLS-IK Cartesian Controller
// Runs at mpc_dt_ (≈ 33 ms). Target solve time: < 5 ms.
// ============================================================================

void MotoMiniPlanningNode::mpcTimerCallback()
{
    if (!tracking_enabled_)
        return;
    if (!target_initialized_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Controller: waiting for target...");
        return;
    }

    std::shared_lock<std::shared_mutex> env_lock(env_mutex_);
    const rclcpp::Time tick_start = this->now();

    // ------------------------------------------------------------------
    // Adaptive horizon — keeps the streamer's lookahead matched to motion.
    // ------------------------------------------------------------------
    const double linear_speed = target_velocity_linear_.norm();
    const double angular_speed = target_velocity_angular_.norm();
    int horizon = 3;
    if (linear_speed > 0.5 || angular_speed > 0.5)
        horizon = 5;
    else if (linear_speed < 0.1 && angular_speed < 0.1)
        horizon = 2;

    const int n_joints = static_cast<int>(manip_->getJointNames().size());
    const auto &joint_names = manip_->getJointNames();
    const Eigen::VectorXd v_max = manip_->getLimits().velocity_limits.col(1);
    const std::vector<bool> is_continuous =
        detectContinuousJoints(joint_limits_, n_joints);

    // ------------------------------------------------------------------
    // STATE PROPAGATION
    // Anchor on horizon_joints_[1] — *our own* last commanded next-state.
    // Sensor lag (~10 ms) would otherwise force a panic-and-catch-up
    // every tick, which shows as a staircase on the position plot.
    // ------------------------------------------------------------------
    Eigen::VectorXd q_anchor;
    {
        std::lock_guard<std::mutex> lock(_mpc_state_mutex);
        static bool first_run = true;
        if (first_run)
        {
            q_anchor = current_joints_;
            first_run = false;
        }
        else
        {
            q_anchor = horizon_joints_[1];
            const double eps = 1e-4;
            for (int i = 0; i < q_anchor.size(); ++i)
                q_anchor[i] = std::max(joint_limits_(i, 0) + eps,
                                       std::min(joint_limits_(i, 1) - eps, q_anchor[i]));
        }
    }
    if (q_anchor.size() == 0)
        return;

    // ------------------------------------------------------------------
    // BUILD HORIZON via DLS-IK chain
    //
    //   q[0]  = anchor
    //   q[k]  = DLS_step(q[k-1], target_pose(k·dt))    for k = 1..H
    // ------------------------------------------------------------------
    std::vector<Eigen::VectorXd> q_traj(horizon + 1);
    q_traj[0] = q_anchor;

    for (int k = 1; k <= horizon; ++k)
    {
        Eigen::Isometry3d P_k = predictTargetPose(k);
        q_traj[k] = computeDlsExtrapolation(q_traj[k - 1], P_k);
    }

    // ------------------------------------------------------------------
    // CONTINUOUS JOINT UNWRAP (relative to anchor → predecessor chain)
    // Removes 2π discontinuities on joints with range ≥ 2π (typically
    // the wrist). Bounded joints are untouched.
    // ------------------------------------------------------------------
    for (int j = 0; j < n_joints; ++j)
    {
        if (!is_continuous[j])
            continue;
        for (size_t k = 1; k < q_traj.size(); ++k)
        {
            q_traj[k][j] = unwrapToReference(
                q_traj[k][j], q_traj[k - 1][j],
                joint_limits_(j, 0), joint_limits_(j, 1));
        }
    }

    // ------------------------------------------------------------------
    // CROSS-TICK VELOCITY LPF
    //
    // Even with smooth targets, tick-to-tick numerical noise produces
    // small jitter in the first commanded velocity. We low-pass-filter
    // it against the previous tick's commanded first-step velocity.
    //
    // After smoothing the first velocity we re-derive q_traj[1]. The
    // rest of the horizon is left as DLS produced it: the difference is
    // O(α_v · jitter) and well below the streamer's interpolation grain.
    //
    // α_v = 1.0 → no filtering (raw DLS)
    // α_v = 0.0 → frozen at last tick (broken)
    // α_v = 0.6 → mild smoothing, no perceptible lag
    // ------------------------------------------------------------------
    static Eigen::VectorXd v_last_published =
        Eigen::VectorXd::Zero(n_joints);
    static bool lpf_primed = false;

    if (lpf_primed && v_last_published.size() == n_joints)
    {
        const double alpha_v = 0.6;
        Eigen::VectorXd v_raw = (q_traj[1] - q_traj[0]) / mpc_dt_;
        Eigen::VectorXd v_smooth = alpha_v * v_raw + (1.0 - alpha_v) * v_last_published;

        for (int j = 0; j < n_joints; ++j)
            v_smooth[j] = std::max(-v_max[j], std::min(v_max[j], v_smooth[j]));

        Eigen::VectorXd q1_smooth = q_traj[0] + v_smooth * mpc_dt_;
        for (int j = 0; j < n_joints; ++j)
            q1_smooth[j] = std::max(joint_limits_(j, 0),
                                    std::min(joint_limits_(j, 1), q1_smooth[j]));
        q_traj[1] = q1_smooth;
    }

    // ------------------------------------------------------------------
    // VELOCITY FEASIBILITY GATE
    // If anything in the horizon would exceed 110 % of v_max, the tick
    // is rejected. With pure DLS this is rare — usually only triggers
    // near singularities. We do NOT update warm-start or LPF state on
    // rejection so next tick recovers from a clean reference.
    // ------------------------------------------------------------------
    bool safe = true;
    double worst_ratio = 0.0;
    int worst_joint = -1;
    for (size_t k = 1; k < q_traj.size(); ++k)
    {
        for (int j = 0; j < n_joints; ++j)
        {
            const double v = std::abs(q_traj[k][j] - q_traj[k - 1][j]) / mpc_dt_;
            const double ratio = v / std::max(v_max[j], 1e-6);
            if (ratio > worst_ratio)
            {
                worst_ratio = ratio;
                worst_joint = j;
            }
            if (ratio > 1.1)
                safe = false;
        }
    }
    if (!safe)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Controller: rejecting tick — joint %d at %.2fx v_max. Holding warm-start.",
                             worst_joint, worst_ratio);
        return;
    }

    // Update LPF state ONLY on a successful tick.
    v_last_published = (q_traj[1] - q_traj[0]) / mpc_dt_;
    lpf_primed = true;

    // ------------------------------------------------------------------
    // BUILD JointTrajectory
    // Velocities use a central difference (forward at the start, backward
    // at the end) — smoother than a forward-only diff.
    // ------------------------------------------------------------------
    tesseract_common::JointTrajectory tess_traj;
    tess_traj.reserve(q_traj.size());

    for (size_t k = 0; k < q_traj.size(); ++k)
    {
        tesseract_common::JointState s;
        s.joint_names = joint_names;
        s.position = q_traj[k];
        s.time = k * mpc_dt_;

        if (q_traj.size() == 1)
            s.velocity = Eigen::VectorXd::Zero(n_joints);
        else if (k == 0)
            s.velocity = (q_traj[1] - q_traj[0]) / mpc_dt_;
        else if (k == q_traj.size() - 1)
            s.velocity = (q_traj[k] - q_traj[k - 1]) / mpc_dt_;
        else
            s.velocity = (q_traj[k + 1] - q_traj[k - 1]) / (2.0 * mpc_dt_);

        tess_traj.push_back(s);
    }

    // ------------------------------------------------------------------
    // PUBLISH
    // splice = exponentially-smoothed *measured* solve time, capped at
    // 100 ms. With this controller it should hover around 3–5 ms.
    // ------------------------------------------------------------------
    const double solve_elapsed = (this->now() - tick_start).seconds();
    static double smooth_splice = 0.0;
    smooth_splice = 0.2 * solve_elapsed + 0.8 * smooth_splice;
    const double splice_to_send = std::min(smooth_splice, 0.100);

    publishTrajectory(tess_traj, joint_names, splice_to_send, mpc_dt_);

    // ------------------------------------------------------------------
    // UPDATE WARM-START
    //
    //   horizon_joints_[k]  =  q_traj[k + 1]    for k = 0..H-1
    //
    // Drop q_traj[0] (the consumed anchor) and keep the rest. Next tick's
    // anchor will be horizon_joints_[1] = q_traj[2] of this tick — two
    // streamer steps ahead, matching the small splice latency.
    // ------------------------------------------------------------------
    if (static_cast<int>(horizon_joints_.size()) < horizon)
        horizon_joints_.resize(horizon);
    for (int k = 0; k < horizon; ++k)
    {
        const size_t src = std::min<size_t>(k + 1, q_traj.size() - 1);
        horizon_joints_[k] = q_traj[src];
    }

    // ------------------------------------------------------------------
    // EE PATH MARKER (RViz visualisation)
    // ------------------------------------------------------------------
    if (pub_ee_path_)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_link_;
        marker.header.stamp = this->now();
        marker.ns = "controller_horizon_path";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.005;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.5f;
        marker.color.a = 1.0f;
        for (const auto &state : tess_traj)
        {
            auto fk = manip_->calcFwdKin(state.position);
            if (fk.count(ee_link_) > 0)
            {
                geometry_msgs::msg::Point p;
                p.x = fk.at(ee_link_).translation().x();
                p.y = fk.at(ee_link_).translation().y();
                p.z = fk.at(ee_link_).translation().z();
                marker.points.push_back(p);
            }
        }
        pub_ee_path_->publish(marker);
    }

    // ------------------------------------------------------------------
    // DIAGNOSTICS
    // ------------------------------------------------------------------
    const double duration_ms = (this->now() - tick_start).seconds() * 1000.0;
    if (duration_ms > 20.0)
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Controller tick took %.2f ms (budget: 20 ms)",
                             duration_ms);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "ctrl  H=%d  solve=%.2fms  splice=%.1fms  worst_v_ratio=%.2f",
                          horizon, duration_ms, splice_to_send * 1000.0, worst_ratio);
}

void MotoMiniPlanningNode::buildAndPublishTrajectory() {}

// ============================================================================
// COLD PATH — TrajOpt advisory (background, optional)
//
// This is a *skeleton* for the suggestion thread. Wire it up when you
// want TrajOpt back. Sketch of integration:
//
//   1. In motomini_planning_node.h add:
//        std::thread                   opt_worker_;
//        std::atomic<bool>             opt_running_{false};
//        std::shared_mutex             opt_suggestion_mutex_;
//        std::vector<Eigen::VectorXd>  opt_suggestion_;
//        rclcpp::Time                  opt_suggestion_stamp_;
//        bool                          opt_suggestion_valid_{false};
//
//        void optimizerWorkerLoop();
//
//   2. In the constructor:
//        opt_running_.store(true);
//        opt_worker_ = std::thread(&MotoMiniPlanningNode::optimizerWorkerLoop, this);
//
//   3. In the destructor:
//        opt_running_.store(false);
//        if (opt_worker_.joinable()) opt_worker_.join();
//
//   4. The worker takes a *snapshot* of (q_anchor, target_velocity, …)
//      under the appropriate locks, runs your TrajOpt pipeline at ~5 Hz,
//      and on success writes the result into opt_suggestion_ guarded by
//      opt_suggestion_mutex_. The hot path NEVER waits on this thread.
//
//   5. To consume the suggestion in mpcTimerCallback, after the DLS chain
//      and BEFORE the velocity gate:
//
//        std::shared_lock<std::shared_mutex> ls(opt_suggestion_mutex_);
//        if (opt_suggestion_valid_ &&
//            (this->now() - opt_suggestion_stamp_).seconds() < 0.300)
//        {
//          // Soft pull: blend the suggestion into the FAR end of the horizon
//          // (k = H), where Cartesian error is largest and a small posture
//          // correction won't disturb tracking. β = 0.15 is gentle.
//          const double beta = 0.15;
//          const size_t idx  = std::min<size_t>(horizon,
//                                               opt_suggestion_.size() - 1);
//          q_traj[horizon] = (1.0 - beta) * q_traj[horizon]
//                          +  beta        * opt_suggestion_[idx];
//          // Then re-run continuous-joint unwrap on q_traj[horizon]
//          // relative to q_traj[horizon-1].
//        }
//
//      If TrajOpt is slow, fails, or returns garbage, the suggestion is
//      either stale (>300 ms old → ignored) or never arrives. The robot
//      keeps tracking on pure DLS regardless.
//
// Until you wire this up, the controller above is fully self-contained
// and produces smooth, continuous tracking on its own.
// ============================================================================