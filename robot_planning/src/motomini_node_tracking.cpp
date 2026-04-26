/**
 * @file motomini_node_tracking.cpp
 * @brief MotoMiniPlanningNode — Cartesian servo controller with reactive
 *        collision avoidance.
 *
 *  ARCHITECTURE
 *  ============
 *
 *      ┌───────────────────────── HOT PATH (30 Hz, never blocks) ──────────┐
 *      │                                                                   │
 *      │   target ─► PT1 smoother ─► predict horizon ─► DLS-IK chain       │
 *      │                                                  │                │
 *      │                                                  ▼                │
 *      │                                       ┌─── collision check ─┐    │
 *      │                                       │                     │    │
 *      │                            no collision                  collision│
 *      │                                       │                     │    │
 *      │                                       ▼                     ▼    │
 *      │                                 publish DLS         signal worker │
 *      │                                  (TRACKING)         publish HOLD  │
 *      │                                                  (AVOIDANCE_REQ)  │
 *      │                                                       │           │
 *      │                                                       ▼           │
 *      │                                                worker returns?    │
 *      │                                                       │           │
 *      │                                                  follow it        │
 *      │                                                  (AVOIDANCE_EXEC) │
 *      └───────────────────────────────────────────────────────────────────┘
 *
 *      ┌───────────────────────── COLD PATH (background thread) ───────────┐
 *      │                                                                   │
 *      │   wait for trigger → snapshot anchor + targets → run TrajOpt with │
 *      │   collision_cost ENABLED, longer horizon, stricter convergence    │
 *      │   → on success store result; on failure log and wait again.       │
 *      │                                                                   │
 *      └───────────────────────────────────────────────────────────────────┘
 *
 *  KEY PROPERTIES
 *  ==============
 *  • The controller never blocks on TrajOpt. Worst case (TrajOpt slow), the
 *    robot just *holds* its current pose until the avoidance plan arrives.
 *  • While following an avoidance trajectory, every tick re-checks for
 *    collisions against the *current* environment. If the world changes
 *    again, we re-plan.
 *  • The cross-tick velocity LPF (carried over from the tracker) is what
 *    smooths the seam when modes switch — no jumps.
 *  • Hand-off back to TRACKING happens automatically: when the avoidance
 *    trajectory is exhausted AND the DLS horizon is currently clear.
 *
 *  REQUIRED .h ADDITIONS
 *  =====================
 *  See  motomini_planning_node_additions.h  for the block to paste into
 *  your class. You also need to call  startAvoidanceWorker()  in the
 *  constructor and  stopAvoidanceWorker()  in the destructor.
 *
 *  @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning_node.h>

// ROS / Tesseract
#include <tesseract_rosutils/utils.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_environment/environment.h>
#include <tesseract_state_solver/state_solver.h>

// TrajOpt (used only by the background avoidance worker)
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_common/collision_types.h>

#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/constraints/joint_acceleration_constraint.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <trajopt_ifopt/costs/squared_cost.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/profile_dictionary.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>

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
// AVOIDANCE WORKER LIFECYCLE
// ============================================================================

void MotoMiniPlanningNode::startAvoidanceWorker()
{
    if (avoidance_running_.exchange(true))
        return; // already running
    avoidance_worker_ = std::thread(&MotoMiniPlanningNode::avoidanceWorkerLoop, this);
    RCLCPP_INFO(this->get_logger(), "Avoidance worker thread started.");
}

void MotoMiniPlanningNode::stopAvoidanceWorker()
{
    if (!avoidance_running_.exchange(false))
        return;
    avoidance_request_cv_.notify_all();
    if (avoidance_worker_.joinable())
        avoidance_worker_.join();
    RCLCPP_INFO(this->get_logger(), "Avoidance worker thread stopped.");
}

void MotoMiniPlanningNode::requestAvoidance(const Eigen::VectorXd &anchor)
{
    {
        std::lock_guard<std::mutex> lk(avoidance_request_mutex_);
        avoidance_request_anchor_ = anchor;
        avoidance_request_pending_.store(true);
    }
    avoidance_request_cv_.notify_one();
}

// ============================================================================
// COLLISION CHECK
// ============================================================================

bool MotoMiniPlanningNode::checkCollisionAtState(const Eigen::VectorXd &q)
{
    // env_mutex_ is assumed held (shared_lock) by the caller.
    auto manager = env_->getDiscreteContactManager();
    if (!manager)
        return false;

    auto state_solver = env_->getStateSolver();
    auto scene_state = state_solver->getState(manip_->getJointNames(), q);

    manager->setCollisionObjectsTransform(scene_state.link_transforms);

    tesseract_collision::ContactResultMap contacts;
    tesseract_collision::ContactRequest req;
    req.type = tesseract_collision::ContactTestType::FIRST;
    manager->contactTest(contacts, req);

    return !contacts.empty();
}

int MotoMiniPlanningNode::checkCollisionInHorizon(
    const std::vector<Eigen::VectorXd> &q_traj)
{
    // Skip k=0 (anchor) — that's "now", any collision there is a sensor
    // glitch we can't react to anyway. Check k=1..end.
    for (size_t k = 1; k < q_traj.size(); ++k)
        if (checkCollisionAtState(q_traj[k]))
            return static_cast<int>(k);
    return -1;
}

// ============================================================================
// AVOIDANCE WORKER — runs TrajOpt with collision cost ENABLED
// ============================================================================

void MotoMiniPlanningNode::avoidanceWorkerLoop()
{
    while (avoidance_running_.load())
    {
        Eigen::VectorXd anchor;
        {
            std::unique_lock<std::mutex> lk(avoidance_request_mutex_);
            avoidance_request_cv_.wait(lk, [this]()
                                       { return !avoidance_running_.load() ||
                                                avoidance_request_pending_.load(); });
            if (!avoidance_running_.load())
                return;
            anchor = avoidance_request_anchor_;
            avoidance_request_pending_.store(false);
        }
        if (anchor.size() == 0)
            continue;

        const auto t0 = this->now();

        std::vector<Eigen::VectorXd> result;
        bool ok = false;
        try
        {
            ok = runAvoidancePlan(anchor, result);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Avoidance worker exception: %s", e.what());
            ok = false;
        }

        const double elapsed_ms = (this->now() - t0).seconds() * 1000.0;

        if (ok && !result.empty())
        {
            std::unique_lock<std::shared_mutex> wl(avoidance_result_mutex_);
            avoidance_traj_ = std::move(result);
            avoidance_idx_ = 0;
            avoidance_traj_stamp_ = this->now();
            avoidance_traj_valid_.store(true);
            RCLCPP_INFO(this->get_logger(),
                        "Avoidance plan ready (%zu pts, %.0f ms).",
                        avoidance_traj_.size(), elapsed_ms);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "Avoidance planning failed after %.0f ms.", elapsed_ms);
        }
    }
}

bool MotoMiniPlanningNode::runAvoidancePlan(
    const Eigen::VectorXd &q_start,
    std::vector<Eigen::VectorXd> &out_traj)
{
    using namespace tesseract_planning;

    std::shared_lock<std::shared_mutex> env_lock(env_mutex_);

    const int H = avoidance_horizon_;
    const auto &joint_names = manip_->getJointNames();
    const int n_joints = static_cast<int>(joint_names.size());

    // Build the planning problem: start from q_start, then H Cartesian
    // waypoints predicted along the current target's trajectory.
    CompositeInstruction ci(
        "DEFAULT",
        tesseract_common::ManipulatorInfo(manip_->getName(), base_link_, ee_link_));

    ci.push_back(MoveInstruction(StateWaypoint(joint_names, q_start),
                                 MoveInstructionType::FREESPACE, "FREESPACE"));
    for (int k = 1; k <= H; ++k)
        ci.push_back(MoveInstruction(CartesianWaypoint(predictTargetPose(k)),
                                     MoveInstructionType::FREESPACE, "FREESPACE"));

    auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();

    // (1) MOVE — looser Cartesian cost so detours are allowed
    auto move = std::make_shared<TrajOptIfoptDefaultMoveProfile>();
    move->cartesian_cost_config.enabled = true;
    Eigen::VectorXd cw = Eigen::VectorXd::Zero(6);
    cw.head<3>().setConstant(5.0); // position — softer than tracking (10)
    cw.tail<3>().setConstant(0.5); // orientation — softer
    move->cartesian_cost_config.coeff = cw;
    move->cartesian_constraint_config.enabled = false;

    move->joint_cost_config.enabled = true;
    Eigen::VectorXd jc = Eigen::VectorXd::Ones(n_joints) * 0.5;
    jc[n_joints - 1] = 2.0;
    move->joint_cost_config.coeff = jc;

    // (2) COMPOSITE — collision cost ENABLED, larger margin
    auto comp = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
    comp->collision_cost_config.enabled = true;
    // Fields below are version-dependent. If your trajopt_ifopt version uses
    // different names, adjust here. The intent: discrete checks at every
    // waypoint, with a buffer slightly larger than collision_safety_margin_.
    // Common knobs:
    //   comp->collision_cost_config.type            = trajopt_common::CollisionEvaluatorType::DISCRETE;
    //   comp->collision_cost_config.safety_margin   = collision_safety_margin_;
    //   comp->collision_cost_config.safety_margin_buffer = 0.005;
    //   comp->collision_cost_config.coeff           = 20.0;

    comp->smooth_velocities = true;
    Eigen::VectorXd vw = Eigen::VectorXd::Ones(n_joints) * 5.0;
    vw[n_joints - 1] = 50.0;
    comp->velocity_coeff = vw;

    comp->smooth_accelerations = true;
    Eigen::VectorXd aw = Eigen::VectorXd::Ones(n_joints) * 1.0;
    aw[n_joints - 1] = 20.0;
    comp->acceleration_coeff = aw;

    comp->smooth_jerks = false;
    comp->jerk_coeff = Eigen::VectorXd::Ones(1) * 0.0;

    // (3) SOLVER — give it room to find a detour
    auto solver = std::make_shared<TrajOptIfoptOSQPSolverProfile>();
    solver->opt_params.max_iterations = 20;
    solver->opt_params.initial_trust_box_size = 0.1;
    solver->opt_params.min_approx_improve = 1e-3;
    solver->opt_params.cnt_tolerance = 1e-3;

    const std::string NS = "TrajOptIfoptMotionPlannerTask";
    profiles->addProfile(NS, "FREESPACE", move);
    profiles->addProfile(NS, "DEFAULT", comp);
    profiles->addProfile(NS, "DEFAULT", solver);

    std::shared_ptr<const tesseract_environment::Environment> const_env = env_;
    auto ds = std::make_unique<TaskComposerDataStorage>();
    ds->setData("planning_input", ci);
    ds->setData("environment", const_env);
    ds->setData("profiles", profiles);

    auto tc_ctx = std::make_shared<TaskComposerContext>(
        mpc_task_->getName(), std::move(ds));

    auto fut = task_executor_->run(*mpc_task_, std::move(tc_ctx));
    if (!fut)
        return false;
    fut->wait();

    const std::string out_key = mpc_task_->getOutputKeys().get("program");
    const auto stored = fut->context->data_storage->getData();
    if (stored.count(out_key) == 0)
        return false;

    auto ci_out = stored.at(out_key).template as<CompositeInstruction>();
    tesseract_planning::formatProgram(ci_out, *env_);
    auto tess_traj = toJointTrajectory(ci_out);
    if (tess_traj.empty())
        return false;

    // Convert to vector<VectorXd> and unwrap continuous joints.
    out_traj.clear();
    out_traj.reserve(tess_traj.size());
    for (const auto &s : tess_traj)
        out_traj.push_back(s.position);

    const auto is_cont = detectContinuousJoints(joint_limits_, n_joints);
    unwrapHorizonContinuousJoints(out_traj, is_cont, joint_limits_);

    // Final safety: re-validate the avoidance plan against the live env.
    // (The env may have updated mid-solve.)
    for (size_t i = 1; i < out_traj.size(); ++i)
        if (checkCollisionAtState(out_traj[i]))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Avoidance plan REJECTED: still in collision at idx %zu", i);
            return false;
        }

    return true;
}

// ============================================================================
// HOT PATH — Controller with TRACKING / AVOIDANCE state machine
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

    // ------------------------------------------------------------------
    // Adaptive horizon (tracking only — avoidance has its own H)
    // ------------------------------------------------------------------
    const double linear_speed = target_velocity_linear_.norm();
    const double angular_speed = target_velocity_angular_.norm();
    int horizon = 3;
    if (linear_speed > 0.5 || angular_speed > 0.5)
        horizon = 5;
    else if (linear_speed < 0.1 && angular_speed < 0.1)
        horizon = 2;

    // ------------------------------------------------------------------
    // Anchor
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
    // Always compute the DLS horizon. We need it as the tracking output,
    // and we need it to detect "is the path still clear so we can hand
    // back from AVOIDANCE_EXEC to TRACKING".
    // ------------------------------------------------------------------
    std::vector<Eigen::VectorXd> q_dls(horizon + 1);
    q_dls[0] = q_anchor;
    for (int k = 1; k <= horizon; ++k)
        q_dls[k] = computeDlsExtrapolation(q_dls[k - 1], predictTargetPose(k));
    unwrapHorizonContinuousJoints(q_dls, is_continuous, joint_limits_);

    const int dls_first_collision = checkCollisionInHorizon(q_dls);
    const bool dls_clear = (dls_first_collision < 0);

    // ------------------------------------------------------------------
    // STATE MACHINE — pick what to publish this tick.
    // ------------------------------------------------------------------
    std::vector<Eigen::VectorXd> q_publish;
    q_publish.reserve(horizon + 1);

    const char *mode_str = "TRACKING";
    auto current_mode = mode_.load();

    if (current_mode == ControllerMode::TRACKING)
    {
        if (!dls_clear)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "TRACKING: collision predicted at horizon idx %d → requesting avoidance.",
                                 dls_first_collision);
            avoidance_traj_valid_.store(false); // discard any stale plan
            requestAvoidance(q_anchor);
            mode_.store(ControllerMode::AVOIDANCE_REQUESTED);
            current_mode = ControllerMode::AVOIDANCE_REQUESTED;
        }
        else
        {
            q_publish = q_dls;
            mode_str = "TRACKING";
        }
    }

    if (current_mode == ControllerMode::AVOIDANCE_REQUESTED)
    {
        if (avoidance_traj_valid_.load())
        {
            std::unique_lock<std::shared_mutex> wl(avoidance_result_mutex_);
            avoidance_idx_ = 0;
            mode_.store(ControllerMode::AVOIDANCE_EXEC);
            current_mode = ControllerMode::AVOIDANCE_EXEC;
        }
        else
        {
            // Hold pose until plan arrives.
            q_publish.assign(horizon + 1, q_anchor);
            mode_str = "AVOIDANCE_REQUESTED (hold)";
        }
    }

    if (current_mode == ControllerMode::AVOIDANCE_EXEC)
    {
        std::shared_lock<std::shared_mutex> rl(avoidance_result_mutex_);

        const double age = (this->now() - avoidance_traj_stamp_).seconds();
        const bool exhausted = (avoidance_idx_ + 1 >= avoidance_traj_.size());
        const bool stale = (age > avoidance_traj_max_age_);

        if (!avoidance_traj_valid_.load() || exhausted)
        {
            // Plan finished. Hand back to tracking ONLY if the path is clear.
            if (dls_clear)
            {
                rl.unlock();
                mode_.store(ControllerMode::TRACKING);
                avoidance_traj_valid_.store(false);
                q_publish = q_dls;
                mode_str = "TRACKING (resumed)";
            }
            else
            {
                // Still blocked — request a fresh plan.
                rl.unlock();
                avoidance_traj_valid_.store(false);
                requestAvoidance(q_anchor);
                mode_.store(ControllerMode::AVOIDANCE_REQUESTED);
                q_publish.assign(horizon + 1, q_anchor);
                mode_str = "AVOIDANCE_EXEC → re-plan (path still blocked)";
            }
        }
        else if (stale)
        {
            // Old plan — ask for a fresh one.
            rl.unlock();
            avoidance_traj_valid_.store(false);
            requestAvoidance(q_anchor);
            mode_.store(ControllerMode::AVOIDANCE_REQUESTED);
            q_publish.assign(horizon + 1, q_anchor);
            mode_str = "AVOIDANCE_EXEC → stale, re-planning";
        }
        else
        {
            // Slice the avoidance trajectory into a horizon-sized window.
            // q_publish[0] = anchor (matches what we last commanded);
            // q_publish[1..H] = next H steps from the avoidance buffer.
            q_publish.resize(horizon + 1);
            q_publish[0] = q_anchor;
            for (int k = 1; k <= horizon; ++k)
            {
                const size_t src = std::min(avoidance_idx_ + k,
                                            avoidance_traj_.size() - 1);
                q_publish[k] = avoidance_traj_[src];
            }
            avoidance_idx_++;
            rl.unlock();

            // Re-validate against the LIVE environment (the obstacle
            // could have moved or grown since the plan was made).
            const int recheck = checkCollisionInHorizon(q_publish);
            if (recheck >= 0)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "AVOIDANCE_EXEC: live recheck failed at idx %d → re-planning.",
                                     recheck);
                avoidance_traj_valid_.store(false);
                requestAvoidance(q_anchor);
                mode_.store(ControllerMode::AVOIDANCE_REQUESTED);
                q_publish.assign(horizon + 1, q_anchor);
                mode_str = "AVOIDANCE_EXEC → live collision, re-planning";
            }
            else
            {
                mode_str = "AVOIDANCE_EXEC";
            }
        }
    }

    // Re-unwrap (avoidance slices may have been seeded fresh).
    unwrapHorizonContinuousJoints(q_publish, is_continuous, joint_limits_);

    // ------------------------------------------------------------------
    // CROSS-TICK VELOCITY LPF — produces the smooth merge between modes.
    // ------------------------------------------------------------------
    static Eigen::VectorXd v_last_published =
        Eigen::VectorXd::Zero(n_joints);
    static bool lpf_primed = false;

    if (lpf_primed && q_publish.size() >= 2 && v_last_published.size() == n_joints)
    {
        const double alpha_v = 0.6;
        Eigen::VectorXd v_raw = (q_publish[1] - q_publish[0]) / mpc_dt_;
        Eigen::VectorXd v_smooth = alpha_v * v_raw + (1.0 - alpha_v) * v_last_published;

        for (int j = 0; j < n_joints; ++j)
            v_smooth[j] = std::max(-v_max[j], std::min(v_max[j], v_smooth[j]));

        Eigen::VectorXd q1 = q_publish[0] + v_smooth * mpc_dt_;
        for (int j = 0; j < n_joints; ++j)
            q1[j] = std::max(joint_limits_(j, 0),
                             std::min(joint_limits_(j, 1), q1[j]));
        q_publish[1] = q1;
    }

    // ------------------------------------------------------------------
    // VELOCITY GATE (always on)
    // ------------------------------------------------------------------
    bool safe = true;
    double worst_ratio = 0.0;
    int worst_joint = -1;
    for (size_t k = 1; k < q_publish.size(); ++k)
        for (int j = 0; j < n_joints; ++j)
        {
            const double v = std::abs(q_publish[k][j] - q_publish[k - 1][j]) / mpc_dt_;
            const double r = v / std::max(v_max[j], 1e-6);
            if (r > worst_ratio)
            {
                worst_ratio = r;
                worst_joint = j;
            }
            if (r > 1.1)
                safe = false;
        }
    if (!safe)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Controller: rejecting tick — joint %d at %.2fx v_max (mode=%s).",
                             worst_joint, worst_ratio, mode_str);
        return;
    }
    v_last_published = (q_publish[1] - q_publish[0]) / mpc_dt_;
    lpf_primed = true;

    // ------------------------------------------------------------------
    // BUILD JointTrajectory
    // ------------------------------------------------------------------
    tesseract_common::JointTrajectory tess_traj;
    tess_traj.reserve(q_publish.size());
    for (size_t k = 0; k < q_publish.size(); ++k)
    {
        tesseract_common::JointState s;
        s.joint_names = joint_names;
        s.position = q_publish[k];
        s.time = k * mpc_dt_;
        if (q_publish.size() == 1)
            s.velocity = Eigen::VectorXd::Zero(n_joints);
        else if (k == 0)
            s.velocity = (q_publish[1] - q_publish[0]) / mpc_dt_;
        else if (k == q_publish.size() - 1)
            s.velocity = (q_publish[k] - q_publish[k - 1]) / mpc_dt_;
        else
            s.velocity = (q_publish[k + 1] - q_publish[k - 1]) / (2.0 * mpc_dt_);
        tess_traj.push_back(s);
    }

    // ------------------------------------------------------------------
    // PUBLISH
    // ------------------------------------------------------------------
    const double solve_elapsed = (this->now() - tick_start).seconds();
    static double smooth_splice = 0.0;
    smooth_splice = 0.2 * solve_elapsed + 0.8 * smooth_splice;
    const double splice_to_send = std::min(smooth_splice, 0.100);
    publishTrajectory(tess_traj, joint_names, splice_to_send, mpc_dt_);

    // ------------------------------------------------------------------
    // UPDATE WARM-START
    // ------------------------------------------------------------------
    if (static_cast<int>(horizon_joints_.size()) < horizon)
        horizon_joints_.resize(horizon);
    for (int k = 0; k < horizon; ++k)
    {
        const size_t src = std::min<size_t>(k + 1, q_publish.size() - 1);
        horizon_joints_[k] = q_publish[src];
    }

    // ------------------------------------------------------------------
    // VISUALISATION
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
        // Colour-code: green=tracking, yellow=requesting, red=avoiding
        const auto m = mode_.load();
        marker.color.a = 1.0f;
        if (m == ControllerMode::TRACKING)
        {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.5f;
        }
        else if (m == ControllerMode::AVOIDANCE_REQUESTED)
        {
            marker.color.r = 1.0f;
            marker.color.g = 0.8f;
            marker.color.b = 0.0f;
        }
        else
        {
            marker.color.r = 1.0f;
            marker.color.g = 0.2f;
            marker.color.b = 0.0f;
        }

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
                             "Tick %.2f ms (budget 20 ms) mode=%s",
                             duration_ms, mode_str);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                          "ctrl  mode=%s  H=%d  solve=%.2fms  splice=%.1fms  worst_v=%.2fx",
                          mode_str, horizon, duration_ms, splice_to_send * 1000.0, worst_ratio);
}

void MotoMiniPlanningNode::buildAndPublishTrajectory() {}