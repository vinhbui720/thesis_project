/**
 * @file motomini_node_tracking.cpp
 * @brief MotoMiniPlanningNode — DLS-IK Cartesian servo controller with
 *        Cartesian-space repulsion and adaptive task-axis relaxation.
 *
 *  WHAT'S DIFFERENT IN THIS REVISION
 *  =================================
 *  Two real bugs were preventing the robot from dodging:
 *
 *  1) Repulsion was added in JOINT space and treated by DLS as if it were
 *     part of the tracking task. The regulariser λ²I then "blended it down"
 *     and the velocity-clamp at the bottom truncated whatever survived.
 *     RESULT: robot pushes back maybe 1–2 mm, gets stuck in the velocity
 *     gate, freezes.
 *
 *  2) When the obstacle is straight ahead and the operator wants to "hop
 *     over it", the robot needs permission to drop the Z-tracking task
 *     temporarily. Without that permission, the DLS objective insists on
 *     hitting the target's exact Z, no matter what — repulsion would have
 *     to fight it. RESULT: even with strong repulsion, the EE just tries
 *     harder to plough through.
 *
 *  THE FIX
 *  =======
 *
 *  Repulsion is now added directly to the desired CARTESIAN twist v_des,
 *  not to the joint solution. That makes it a peer of the tracking task
 *  and lets DLS resolve the two together correctly:
 *
 *      v_des = v_track + v_rep              (both in Cartesian space)
 *      W_pos = base weights × per-axis gain  (relaxed under threat)
 *      W_rot = base weights × per-axis gain
 *      dq    = (J^T W J + λ²I)^{-1} J^T W v_des
 *
 *  The W diagonals are scheduled by proximity to obstacles: when the EE
 *  is within `relax_distance` of a contact, the Z weight drops to as
 *  little as 5 % of nominal, freeing the controller to "hop". The X/Y
 *  weights stay high so the robot still tracks side-to-side accurately.
 *
 *  This is, in spirit, a one-row "task-priority IK" — the same idea used
 *  in classical hierarchical-control formulations but without the SVD
 *  gymnastics that complicate real-time use.
 *
 *  TUNABLES
 *  ========
 *    repulsion_lookahead      m       distance at which repulsion turns on
 *    repulsion_safety         m       distance below which it saturates
 *    repulsion_max_speed      m/s     cap on Cartesian repulsion magnitude
 *    relax_distance           m       distance at which Z weight starts
 *                                     dropping to allow "hop-over"
 *    relax_z_min_factor       —       lowest fraction of nominal Z weight
 *                                     (e.g. 0.05 = 5 %)
 *    relax_rot_min_factor     —       same idea for orientation tracking
 *
 *  @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning_node.h>

#include <tesseract_rosutils/utils.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_environment/environment.h>
#include <tesseract_state_solver/state_solver.h>

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>

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

static std::vector<bool> detectContinuousJoints(const Eigen::MatrixXd &limits, int n)
{
    std::vector<bool> r(n, false);
    for (int i = 0; i < n; ++i)
        if ((limits(i, 1) - limits(i, 0)) >= 2.0 * M_PI - 0.2)
            r[i] = true;
    return r;
}

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

static void unwrapHorizonContinuousJoints(std::vector<Eigen::VectorXd> &q_traj,
                                          const std::vector<bool> &is_cont,
                                          const Eigen::MatrixXd &limits)
{
    if (q_traj.size() < 2)
        return;
    const int n = static_cast<int>(q_traj[0].size());
    for (int j = 0; j < n; ++j)
    {
        if (!is_cont[j])
            continue;
        for (size_t k = 1; k < q_traj.size(); ++k)
            q_traj[k][j] = unwrapToReference(q_traj[k][j], q_traj[k - 1][j],
                                             limits(j, 0), limits(j, 1));
    }
}

// ============================================================================
// ASYNC CALLBACK — Cartesian PT1 smoother
// ============================================================================

void MotoMiniPlanningNode::targetPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(_mpc_target_mutex);

    Eigen::Isometry3d raw_new_pose;
    tf2::fromMsg(msg->pose, raw_new_pose);

    if (target_initialized_)
    {
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
        predicted.linear() =
            Eigen::AngleAxisd(angle, target_velocity_angular_.normalized()).toRotationMatrix() *
            current_target_pose_.linear();
    return predicted;
}

// ============================================================================
//                  Cartesian repulsion + adaptive task weights
//
//   Side effects: also computes adaptive task weights (W diagonal) based
//   on the closest-contact distance, so the caller can relax Z (and
//   optionally orientation) tracking when an obstacle is in the way.
//
//   Returns: aggregated Cartesian repulsion twist `v_rep` (length 6).
//   Outputs `min_dist` and `weight_diag` via reference parameters.
// ============================================================================

Eigen::Matrix<double, 6, 1> MotoMiniPlanningNode::computeCartesianRepulsion(
    const Eigen::VectorXd &q,
    Eigen::Matrix<double, 6, 1> &track_scale_diag, // [out] tracking gain diagonal
    double &min_dist)                              // [out] closest contact
{
    Eigen::Matrix<double, 6, 1> v_rep = Eigen::Matrix<double, 6, 1>::Zero();
    min_dist = std::numeric_limits<double>::infinity();

    // Nominal tracking scale — follow the target 100%.
    track_scale_diag.setConstant(1.0);

    auto base_manager = env_->getDiscreteContactManager();
    if (!base_manager)
        return v_rep;

    auto manager = base_manager->clone();
    manager->setActiveCollisionObjects(env_->getActiveLinkNames());
    manager->setDefaultCollisionMargin(repulsion_lookahead_);

    auto state_solver = env_->getStateSolver();
    auto scene_state = state_solver->getState(manip_->getJointNames(), q);
    manager->setCollisionObjectsTransform(scene_state.link_transforms);

    tesseract_collision::ContactRequest req;
    req.type = tesseract_collision::ContactTestType::ALL;
    req.calculate_distance = true;

    tesseract_collision::ContactResultMap contacts;
    manager->contactTest(contacts, req);

    auto acm = env_->getAllowedCollisionMatrix();
    const auto active_links = manip_->getActiveLinkNames();
    const std::set<std::string> robot_links(active_links.begin(),
                                            active_links.end());

    auto fk = manip_->calcFwdKin(q);
    if (fk.find(ee_link_) == fk.end())
        return v_rep;
    const Eigen::Vector3d ee_pos = fk.at(ee_link_).translation();

    int rep_count = 0;
    double closest_to_ee_axis_z = std::numeric_limits<double>::infinity();
    Eigen::Vector3d escape_dir_sum = Eigen::Vector3d::Zero();

    for (const auto &kv : contacts)
    {
        for (const auto &c : kv.second)
        {
            if (acm && acm->isCollisionAllowed(c.link_names[0],
                                               c.link_names[1]))
                continue;

            // --- FILTER: Only care about the Obstacle or Workcell Links ---
            bool involves_target = (c.link_names[0].find("obstacle") != std::string::npos || 
                                    c.link_names[0].find("workcell") != std::string::npos ||
                                    c.link_names[1].find("obstacle") != std::string::npos ||
                                    c.link_names[1].find("workcell") != std::string::npos);
            if (!involves_target)
                continue;

            int robot_side = -1;
            if (robot_links.count(c.link_names[0]))
                robot_side = 0;
            else if (robot_links.count(c.link_names[1]))
                robot_side = 1;
            else
                continue;

            if (c.distance < min_dist)
                min_dist = c.distance;
            if (c.distance >= repulsion_lookahead_)
                continue;

            Eigen::Vector3d sep = c.nearest_points[robot_side] - c.nearest_points[1 - robot_side];
            Eigen::Vector3d nrm;
            if (sep.norm() > 1e-6)
            {
                nrm = sep.normalized();
            }
            else
            {
                nrm = (robot_side == 0) ? -c.normal : c.normal;
                nrm.normalize();
            }

            // --- Linear Magnitude Ramp (Smoother) ---
            double t = (repulsion_lookahead_ - c.distance) / 
                       std::max(1e-6, repulsion_lookahead_ - repulsion_safety_);
            t = std::max(0.0, t); 
            
            // Softer growth: max 1.2x at deep penetration to prevent hitting joint-speed bottlenecks
            double mag = repulsion_max_speed_ * std::min(1.2, t); 

            v_rep.head<3>() += mag * nrm;
            escape_dir_sum += nrm;
            ++rep_count;

            const Eigen::Vector3d &cp = c.nearest_points[1 - robot_side];
            const double horiz_dist =
                (cp.head<2>() - ee_pos.head<2>()).norm();
            if (horiz_dist < 0.10 && c.distance < closest_to_ee_axis_z)
                closest_to_ee_axis_z = c.distance;
        }
    }

    // --- EMERGENCY UPWARD KICK ---
    if (min_dist < repulsion_safety_ && escape_dir_sum.z() > 0.3)
    {
        v_rep.z() += 0.03; // Even gentler 3 cm/s upward bias
    }

    // Tighten limit to stay within joint physical execution window
    const double absolute_max_v = 1.02 * repulsion_max_speed_;
    if (v_rep.head<3>().norm() > absolute_max_v)
        v_rep.head<3>() *= absolute_max_v / v_rep.head<3>().norm();

    // -------- Adaptive tracking relaxation ----------------------------
    if (min_dist < relax_distance_)
    {
        if (min_dist <= 0.0) 
        {
            // HARD KILL: Tracking must be ZERO during penetration or the target will pin us.
            track_scale_diag.setZero();
        }
        else
        {
            const double s = std::max(0.0,
                                      std::min(1.0, (min_dist - 0.0) / (relax_distance_ - 0.0)));
            const double factor = relax_z_min_factor_ + s * (1.0 - relax_z_min_factor_);
            track_scale_diag.head<3>().setConstant(factor);
            track_scale_diag.tail<3>().setConstant(relax_rot_min_factor_ + s * (1.0 - relax_rot_min_factor_));
        }
    }

    static int log_div = 0;
    if (rep_count > 0 && (++log_div % 30 == 0))
    {
        RCLCPP_INFO(this->get_logger(),
                    "[Repulse] %d contacts  min_d=%.3fm  W_track_z=%.2f  ‖v_rep‖=%.2fm/s",
                    rep_count, min_dist, track_scale_diag[2], v_rep.head<3>().norm());
    }

    return v_rep;
}

// ============================================================================
// DLS-IK with Cartesian repulsion + adaptive tracking scale
// ============================================================================

Eigen::VectorXd MotoMiniPlanningNode::solveQpVelocity(
    const Eigen::VectorXd &q,
    const Eigen::Isometry3d &target_pose)
{
    const int n = static_cast<int>(q.size());

    auto fk = manip_->calcFwdKin(q);
    if (fk.find(ee_link_) == fk.end())
        return Eigen::VectorXd::Zero(n);

    // ---------------- 1. Tracking twist v_track ----------------------
    const Eigen::Isometry3d ee = fk.at(ee_link_);
    Eigen::Vector3d dx = target_pose.translation() - ee.translation();
    Eigen::AngleAxisd aa(target_pose.linear() * ee.linear().inverse());
    Eigen::Vector3d dw = aa.axis() * aa.angle();

    const double dx_max = 0.30;
    const double dw_max = 1.50;
    if (dx.norm() > dx_max)
        dx *= dx_max / dx.norm();
    if (dw.norm() > dw_max)
        dw *= dw_max / dw.norm();

    const double Kp = 1.0 / std::max(mpc_dt_, 1e-3);
    Eigen::Matrix<double, 6, 1> v_track;
    v_track.head<3>() = Kp * dx;
    v_track.tail<3>() = Kp * dw;

    // ---------------- 2. Cartesian repulsion + tracking relaxation ---
    Eigen::Matrix<double, 6, 1> track_scale;
    double min_dist = std::numeric_limits<double>::infinity();
    Eigen::Matrix<double, 6, 1> v_rep =
        computeCartesianRepulsion(q, track_scale, min_dist);

    // Final desired twist: tracking is scaled down in axes where we are dodging.
    Eigen::Matrix<double, 6, 1> v_des;
    for (int i = 0; i < 6; ++i)
        v_des[i] = v_rep[i] + track_scale[i] * v_track[i];

    // ---------------- 3. Weighted DLS solve --------------------------
    //  q̇ = (Jᵀ W J + λ² I)⁻¹ Jᵀ W v_des
    //
    //  We use constant high weights (W) for the solve so the robot
    //  actually follows the combined v_des.
    Eigen::MatrixXd J = manip_->calcJacobian(q, base_link_, ee_link_);
    if (J.cols() != n || J.rows() < 6)
        return Eigen::VectorXd::Zero(n);

    Eigen::Matrix<double, 6, 1> W_diag;
    W_diag.head<3>().setConstant(mpc_w_cart_);
    W_diag.tail<3>().setConstant(mpc_w_vel_);
    const Eigen::Matrix<double, 6, 6> W = W_diag.asDiagonal();

    const double lambda = std::sqrt(qp_velocity_reg_);

    Eigen::MatrixXd A = J.transpose() * W * J +
                        (lambda * lambda) * Eigen::MatrixXd::Identity(n, n);
    Eigen::VectorXd b = J.transpose() * W * v_des;
    Eigen::VectorXd dq = A.colPivHouseholderQr().solve(b);

    // ---------------- 4. Per-joint clamping --------------------------
    const Eigen::VectorXd v_max = manip_->getLimits().velocity_limits.col(1);
    for (int i = 0; i < n; ++i)
    {
        dq[i] = std::max(-v_max[i], std::min(v_max[i], dq[i]));
        const double pos_lb = (joint_limits_(i, 0) - q[i]) / mpc_dt_;
        const double pos_ub = (joint_limits_(i, 1) - q[i]) / mpc_dt_;
        dq[i] = std::max(pos_lb, std::min(pos_ub, dq[i]));
    }

    return dq * mpc_dt_; // joint-velocity → joint-step
}

Eigen::VectorXd MotoMiniPlanningNode::computeDlsExtrapolation(
    const Eigen::VectorXd &q_last, const Eigen::Isometry3d &target_next)
{
    Eigen::VectorXd dq = solveQpVelocity(q_last, target_next);
    Eigen::VectorXd q_next = q_last + dq;
    for (int i = 0; i < q_next.size(); ++i)
        q_next[i] = std::max(joint_limits_(i, 0),
                             std::min(joint_limits_(i, 1), q_next[i]));
    return q_next;
}

// ============================================================================
// HOT PATH — runs at mpc_dt_ (≈33 ms)
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

    const int n_joints = static_cast<int>(manip_->getJointNames().size());
    const auto &joint_names = manip_->getJointNames();
    const Eigen::VectorXd v_max = manip_->getLimits().velocity_limits.col(1);
    const auto is_continuous = detectContinuousJoints(joint_limits_, n_joints);

    const double linear_speed = target_velocity_linear_.norm();
    const double angular_speed = target_velocity_angular_.norm();
    int horizon = 3;
    if (linear_speed > 0.5 || angular_speed > 0.5)
        horizon = 5;
    else if (linear_speed < 0.1 && angular_speed < 0.1)
        horizon = 2;

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
            // If penetrating (reality is 5cm behind previous prediction), 
            // use 100% feed-forward to "pull" the robot out.
            bool penetrating = false;
            if (current_joints_.size() == static_cast<long>(horizon_joints_[1].size()))
            {
                double err = (current_joints_ - horizon_joints_[0]).norm();
                if (err > 0.05) penetrating = true; 
            }

            if (penetrating)
            {
                q_anchor = horizon_joints_[1];
            }
            else if (current_joints_.size() == static_cast<long>(horizon_joints_[1].size()))
            {
                q_anchor = 0.8 * horizon_joints_[1] + 0.2 * current_joints_;
            }
            else
            {
                q_anchor = horizon_joints_[1];
            }

            const double eps = 1e-4;
            for (int i = 0; i < q_anchor.size(); ++i)
                q_anchor[i] = std::max(joint_limits_(i, 0) + eps,
                                       std::min(joint_limits_(i, 1) - eps, q_anchor[i]));
        }
    }
    if (q_anchor.size() == 0)
        return;

    std::vector<Eigen::VectorXd> q_traj(horizon + 1);
    q_traj[0] = q_anchor;
    for (int k = 1; k <= horizon; ++k)
    {
        Eigen::VectorXd dq = solveQpVelocity(q_traj[k - 1], predictTargetPose(k));
        Eigen::VectorXd q_next = q_traj[k - 1] + dq;
        for (int i = 0; i < q_next.size(); ++i)
            q_next[i] = std::max(joint_limits_(i, 0),
                                 std::min(joint_limits_(i, 1), q_next[i]));
        q_traj[k] = q_next;
    }

    unwrapHorizonContinuousJoints(q_traj, is_continuous, joint_limits_);

    static Eigen::VectorXd v_last_published =
        Eigen::VectorXd::Zero(n_joints);
    static bool lpf_primed = false;

    auto compute_worst_velocity_ratio = [&]() {
        double ratio = 0.0;
        for (size_t k = 1; k < q_traj.size(); ++k)
            for (int j = 0; j < n_joints; ++j)
            {
                const double v = std::abs(q_traj[k][j] - q_traj[k - 1][j]) / mpc_dt_;
                const double r = v / std::max(v_max[j], 1e-6);
                ratio = std::max(ratio, r);
            }
        return ratio;
    };

    double worst_ratio = compute_worst_velocity_ratio();

    // DISABLE LPF during collision/scaling to ensure immediate escape response
    bool disable_lpf = (worst_ratio > 0.8);

    if (!disable_lpf && lpf_primed && q_traj.size() >= 2 && v_last_published.size() == n_joints)
    {
        const double alpha_v = 0.6;
        Eigen::VectorXd v_raw = (q_traj[1] - q_traj[0]) / mpc_dt_;
        Eigen::VectorXd v_smooth = alpha_v * v_raw + (1.0 - alpha_v) * v_last_published;
        for (int j = 0; j < n_joints; ++j)
            v_smooth[j] = std::max(-v_max[j], std::min(v_max[j], v_smooth[j]));
        Eigen::VectorXd q1 = q_traj[0] + v_smooth * mpc_dt_;
        for (int j = 0; j < n_joints; ++j)
            q1[j] = std::max(joint_limits_(j, 0),
                             std::min(joint_limits_(j, 1), q1[j]));
        q_traj[1] = q1;
    }

    worst_ratio = compute_worst_velocity_ratio();

    // ADAPTIVE SCALING: If the move is too fast, scale the WHOLE trajectory down
    // so the worst joint is exactly at 100% v_max. This prevents the "stuck" freeze.
    if (worst_ratio > 1.0)
    {
        const double scale = 0.95 / worst_ratio; // 5% safety margin
        for (size_t k = 1; k < q_traj.size(); ++k)
        {
            Eigen::VectorXd dq = q_traj[k] - q_traj[k - 1];
            q_traj[k] = q_traj[k - 1] + scale * dq;
        }
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Controller: scaling move by %.2f to stay within limits.", scale);
    }
    
    v_last_published = (q_traj[1] - q_traj[0]) / mpc_dt_;
    lpf_primed = true;

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

    const double solve_elapsed = (this->now() - tick_start).seconds();
    static double smooth_splice = 0.0;
    smooth_splice = 0.2 * solve_elapsed + 0.8 * smooth_splice;
    const double splice_to_send = std::min(smooth_splice, 0.100);
    publishTrajectory(tess_traj, joint_names, splice_to_send, mpc_dt_);

    if (static_cast<int>(horizon_joints_.size()) < horizon)
        horizon_joints_.resize(horizon);
    for (int k = 0; k < horizon; ++k)
    {
        const size_t src = std::min<size_t>(k + 1, q_traj.size() - 1);
        horizon_joints_[k] = q_traj[src];
    }

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
        marker.color.r = 0.1f;
        marker.color.g = 0.9f;
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

    const double duration_ms = (this->now() - tick_start).seconds() * 1000.0;
    if (duration_ms > 20.0)
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Tick %.2f ms (budget 20 ms)", duration_ms);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                          "ctrl  H=%d  solve=%.2fms  splice=%.1fms  worst_v=%.2fx",
                          horizon, duration_ms, splice_to_send * 1000.0, worst_ratio);
}

void MotoMiniPlanningNode::buildAndPublishTrajectory() {}
