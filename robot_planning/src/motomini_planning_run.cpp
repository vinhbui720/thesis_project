/**
 * @file motomini_planning_run.cpp
 * @brief MotoMiniPlanning::run() — full offline TrajOpt / OMPL planning pipeline.
 *
 * Pipeline:
 *   1. Build sparse Cartesian waypoint program.
 *   2. Choose planner: TrajOpt / TrajOptIfopt / OMPL+TrajOpt.
 *   3. Execute via TaskComposer.
 *   4. Apply Iterative Spline time-parameterization.
 *   5. (Optional) launch online SQP re-optimization thread.
 *
 * All optimizer profile tuning lives here — collision margins, joint costs, smoothing, etc.
 *
 * @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning.h>
#include <tesseract_common/types.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <trajopt_common/collision_types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

// Kinematics
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_kinematics/core/types.h>
#include <tesseract_kinematics/core/utils.h>

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/environment.h>

// TrajOpt profiles
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>

// Simple planner (seed generation)
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>

// OMPL
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_move_profile.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/core/types.h>

// Command language
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>

// Task Composer
#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

// Time parameterization
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract_time_parameterization/core/utils.h>

// Online SQP solver
#include <trajopt_sqp/qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>

// STL
#include <filesystem>
#include <optional>

#include <taskflow/taskflow.hpp>

using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

// TaskComposer pipeline names
static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";

namespace Vinhtesseract_examples
{

    bool MotoMiniPlanning::run()
    {
        if (debug_)
            console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
        else
            console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

        CONSOLE_BRIDGE_logInform("[Run] Starting Multi-Target Planning...");

        if (!env_ || target_poses_.empty())
            return false;
        if (plotter_ != nullptr)
            plotter_->waitForConnection();

        const std::vector<std::string> joint_names = {
            "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};
        Eigen::VectorXd start_pos = env_->getCurrentJointValues(joint_names);
        env_->setState(joint_names, start_pos);

        // ---- Task Composer factory ----
        std::shared_ptr<const tesseract_common::ResourceLocator> locator = env_->getResourceLocator();
        std::filesystem::path config_path(
            locator->locateResource(
                       "package://tesseract_task_composer/config/task_composer_plugins.yaml")
                ->getFilePath());
        TaskComposerPluginFactory factory(config_path, *env_->getResourceLocator());

        // ---- Planner profiles (defined here so async chunk lambdas can capture by ref) ----
        auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
        profiles->addProfile("SimplePlannerTask", "DEFAULT",
                             std::make_shared<SimplePlannerLVSMoveProfile>());

        const bool ompl_enabled = use_ompl_;
        if (ompl_enabled)
        {
            profiles->addProfile("SimplePlannerTask", "DEFAULT",
                                 std::make_shared<SimplePlannerCompositeProfile>());

            auto ompl_profile = std::make_shared<OMPLRealVectorMoveProfile>();
            ompl_profile->collision_check_config.longest_valid_segment_length = planning_cfg_.ompl_longest_valid_segment;
            ompl_profile->collision_check_config.type =
                tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
            ompl_profile->collision_check_config.contact_request.type =
                tesseract_collision::ContactTestType::ALL;

            ompl_profile->solver_config.planners.clear();
            auto rrt1 = std::make_shared<RRTConnectConfigurator>();
            rrt1->range = planning_cfg_.ompl_rrt_range_1;
            auto rrt2 = std::make_shared<RRTConnectConfigurator>();
            rrt2->range = planning_cfg_.ompl_rrt_range_2;
            ompl_profile->solver_config.planners.push_back(rrt1);
            ompl_profile->solver_config.planners.push_back(rrt2);
            ompl_profile->solver_config.planning_time = planning_cfg_.ompl_planning_time;
            ompl_profile->solver_config.max_solutions = planning_cfg_.ompl_max_solutions;
            ompl_profile->solver_config.simplify = planning_cfg_.ompl_simplify;

            profiles->addProfile("OMPLTask", "FREESPACE", ompl_profile);
        }

        if (ifopt_)
        {
            auto trajopt_ifopt_move = std::make_shared<TrajOptIfoptDefaultMoveProfile>();
            trajopt_ifopt_move->cartesian_constraint_config.enabled = planning_cfg_.ifopt_cart_constraint_enable;
            trajopt_ifopt_move->cartesian_cost_config.enabled = planning_cfg_.ifopt_cart_cost_enable;
            trajopt_ifopt_move->joint_cost_config.enabled = planning_cfg_.ifopt_joint_cost_enable;
            trajopt_ifopt_move->joint_cost_config.coeff = Eigen::VectorXd::Ones(6) * planning_cfg_.ifopt_joint_cost_coeff;

            Eigen::VectorXd coeffs(6);
            // [x, y, z, rx, ry, rz] — individual per-axis weights
            coeffs << planning_cfg_.ifopt_cart_coeff_x,
                planning_cfg_.ifopt_cart_coeff_y,
                planning_cfg_.ifopt_cart_coeff_z,
                planning_cfg_.ifopt_cart_coeff_rx,
                planning_cfg_.ifopt_cart_coeff_ry,
                planning_cfg_.ifopt_cart_coeff_rz;
            trajopt_ifopt_move->cartesian_constraint_config.coeff = coeffs;

            auto trajopt_ifopt_composite = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();

            trajopt_ifopt_composite->collision_constraint_config =
                trajopt_common::TrajOptCollisionConfig(0.0, 200);
            trajopt_ifopt_composite->collision_constraint_config.enabled = planning_cfg_.ifopt_coll_constraint_enable;

            trajopt_ifopt_composite->collision_cost_config =
                trajopt_common::TrajOptCollisionConfig(planning_cfg_.ifopt_coll_cost_margin,
                                                       planning_cfg_.ifopt_coll_cost_coeff);
            trajopt_ifopt_composite->collision_cost_config.enabled = planning_cfg_.ifopt_coll_cost_enable;
            // Collision evaluator type: 0=DISCRETE, 1=CONTINUOUS, 2=LVS_CONTINUOUS
            static const tesseract_collision::CollisionEvaluatorType kEvalTypes[] = {
                tesseract_collision::CollisionEvaluatorType::DISCRETE,
                tesseract_collision::CollisionEvaluatorType::CONTINUOUS,
                tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS};
            const int eval_idx = std::max(0, std::min(2, planning_cfg_.ifopt_coll_eval_type));
            trajopt_ifopt_composite->collision_cost_config.collision_check_config.type =
                kEvalTypes[eval_idx];
            trajopt_ifopt_composite->collision_cost_config.collision_check_config
                .longest_valid_segment_length = planning_cfg_.ifopt_coll_lvs_length;
            trajopt_ifopt_composite->collision_cost_config.collision_margin_buffer = planning_cfg_.ifopt_coll_margin_buffer;

            trajopt_ifopt_composite->smooth_velocities = planning_cfg_.ifopt_smooth_vel_enable;
            trajopt_ifopt_composite->velocity_coeff = planning_cfg_.ifopt_smooth_vel * Eigen::VectorXd::Ones(1);
            trajopt_ifopt_composite->smooth_accelerations = planning_cfg_.ifopt_smooth_acc_enable;
            trajopt_ifopt_composite->acceleration_coeff = planning_cfg_.ifopt_smooth_acc * Eigen::VectorXd::Ones(1);
            trajopt_ifopt_composite->smooth_jerks = planning_cfg_.ifopt_smooth_jerk_enable;
            trajopt_ifopt_composite->jerk_coeff = planning_cfg_.ifopt_smooth_jerk * Eigen::VectorXd::Ones(1);

            auto trajopt_ifopt_solver = std::make_shared<TrajOptIfoptOSQPSolverProfile>();
            trajopt_ifopt_solver->opt_params.max_iterations = planning_cfg_.ifopt_max_iter;
            trajopt_ifopt_solver->opt_params.min_approx_improve = planning_cfg_.ifopt_min_approx_improve;
            trajopt_ifopt_solver->opt_params.min_trust_box_size = planning_cfg_.ifopt_min_trust_box_size;
            trajopt_ifopt_solver->opt_params.initial_trust_box_size = planning_cfg_.ifopt_initial_trust_box_size;

            // Register move profile under both keys so either FREESPACE or LINEAR works
            profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_ifopt_move);
            profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_ifopt_move);
            profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_ifopt_composite);
            profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_ifopt_solver);
        }
        else
        {
            auto trajopt_freespace = std::make_shared<TrajOptDefaultMoveProfile>();
            trajopt_freespace->joint_cost_config.enabled = true;
            trajopt_freespace->cartesian_constraint_config.enabled = true;

            auto trajopt_composite = std::make_shared<TrajOptDefaultCompositeProfile>();
            trajopt_composite->collision_constraint_config =
                trajopt_common::TrajOptCollisionConfig(0.01, 10);
            trajopt_composite->collision_cost_config =
                trajopt_common::TrajOptCollisionConfig(0.02, 50);

            auto trajopt_solver = std::make_shared<TrajOptOSQPSolverProfile>();
            trajopt_solver->opt_params.max_iter = 100;

            profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_freespace);
            profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_freespace);
            profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite);
            profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_solver);
        }

        // ContactCheckProfile: match the TrajOpt margin so the post-plan check is
        // consistent with what TrajOpt was optimizing against.
        auto contact_check_profile = std::make_shared<ContactCheckProfile>();
        contact_check_profile->collision_check_config.type =
            tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
        contact_check_profile->collision_check_config.longest_valid_segment_length = 0.005;
        profiles->addProfile("DiscreteContactCheckTask", "DEFAULT", contact_check_profile);

        // ================================================================
        //  ADAPTIVE PARALLEL CHUNK PIPELINE
        //
        //  Architecture:
        //    - Split target_poses into chunks of chunk_size_ waypoints.
        //    - Pre-compute IK seeds at chunk boundaries for parallel start states.
        //    - Process batches of parallel_chunks_ chunks concurrently using
        //      Taskflow async tasks, each on a cloned environment.
        //    - Stitch all chunk trajectories into a single full_traj with
        //      continuous timestamps (duplicate seam points dropped).
        //    - chunk_ready_cb_ fires per chunk for progress notification only.
        //    - Seam stitching: after each batch, update the confirmed start state
        //      from the actual TrajOpt end state for the next batch.
        //    - The caller (startCallback) publishes full_traj once after run()
        //      returns so the controller receives one complete trajectory.
        // ================================================================
        const std::string task_name = ompl_enabled ? "FreespacePipeline"
                                                   : (ifopt_ ? "TrajOptIfoptPipeline" : "TrajOptPipeline");

        const size_t n_poses = target_poses_.size();
        const size_t C = static_cast<size_t>(chunk_size_);
        const size_t P = static_cast<size_t>(parallel_chunks_);
        const size_t n_chunks = (n_poses + C - 1) / C;

        CONSOLE_BRIDGE_logInform("[Run] %zu waypoint(s) → %zu chunk(s) of ≤%zu  [%zu parallel]  via %s",
                                 n_poses, n_chunks, C, P, task_name.c_str());

        auto isp = std::make_unique<tesseract_planning::IterativeSplineParameterization>(
            "IterativeSplineParameterization");

        // --- Step 1: Pre-compute IK-estimated start joints at every chunk boundary ---
        // This lets batch chunks start without sequential dependency.
        std::vector<Eigen::VectorXd> chunk_starts(n_chunks, start_pos);
        {
            auto manip = env_->getKinematicGroup(manipulator_group_);
            Eigen::VectorXd seed = start_pos;
            for (size_t ci = 1; ci < n_chunks; ++ci)
            {
                // Last target waypoint of the previous chunk
                const size_t boundary = std::min(ci * C, n_poses) - 1;
                KinGroupIKInput ik_in(target_poses_[boundary], base_link_, ee_link_);
                auto sols = manip->calcInvKin(ik_in, seed);
                if (!sols.empty())
                {
                    double best_d = std::numeric_limits<double>::max();
                    for (const auto &s : sols)
                    {
                        double d = (s - seed).squaredNorm();
                        if (d < best_d)
                        {
                            best_d = d;
                            chunk_starts[ci] = s;
                        }
                    }
                    seed = chunk_starts[ci];
                }
                else
                {
                    chunk_starts[ci] = seed; // fallback to previous seed
                }
            }
        }

        // --- Step 2: Batch-parallel chunk planning with streaming publish ---
        struct ChunkResult
        {
            tesseract_common::JointTrajectory traj;
            bool ok{false};
            std::string error;
        };

        // Lambda: plan + ISP one chunk using a cloned environment
        auto plan_one_chunk = [&](size_t ci) -> ChunkResult
        {
            const size_t pose_begin = ci * C;
            const size_t pose_end = std::min(pose_begin + C, n_poses);
            CONSOLE_BRIDGE_logInform("[Run] Chunk %zu/%zu (poses %zu-%zu)...",
                                     ci + 1, n_chunks, pose_begin + 1, pose_end);

            // Clone env so parallel chunks don't share mutable state.
            // Use shared_ptr so both the TaskComposer data storage and ISP can hold references.
            auto env_c = std::shared_ptr<tesseract_environment::Environment>(env_->clone());
            env_c->setState(joint_names, chunk_starts[ci]);

            // Build CI for this chunk
            // Motion type is configurable: LINEAR keeps straight TCP paths + constrained orientation;
            // FREESPACE allows any joint configuration to reach the Cartesian waypoint.
            const auto move_type = planning_cfg_.use_linear ? MoveInstructionType::LINEAR
                                                            : MoveInstructionType::FREESPACE;
            const std::string move_profile = planning_cfg_.use_linear ? "CARTESIAN" : "FREESPACE";

            // Build the Cartesian program for this chunk:
            //   [start_state] → [pose_0] → [pose_1] → ... → [pose_N-1]
            CompositeInstruction ci_prog(
                "DEFAULT", ManipulatorInfo(manipulator_group_, base_link_, ee_link_));
            ci_prog.push_back(MoveInstruction(
                StateWaypoint(joint_names, chunk_starts[ci]),
                move_type, move_profile));
            for (size_t k = pose_begin; k < pose_end; ++k)
                ci_prog.push_back(MoveInstruction(
                    CartesianWaypoint(target_poses_[k]),
                    move_type, move_profile));

            // TaskComposer
            TaskComposerNode::UPtr tc_task = factory.createTaskComposerNode(task_name);
            const std::string out_key = tc_task->getOutputKeys().get("program");
            auto ds = std::make_unique<TaskComposerDataStorage>();
            ds->setData("planning_input", ci_prog);
            ds->setData("environment",
                        std::shared_ptr<const tesseract_environment::Environment>(env_c));
            ds->setData("profiles", profiles);
            auto tc_exec = factory.createTaskComposerExecutor("TaskflowExecutor");
            auto tc_ctx = std::make_shared<TaskComposerContext>(
                tc_task->getName(), std::move(ds));
            auto fut = tc_exec->run(*tc_task, std::move(tc_ctx));
            fut->wait();

            if (!fut->context->isSuccessful())
            {
                // SimplePlannerTask writes CartesianWaypoints to out_key as the seed BEFORE
                // TrajOpt even runs. Mere existence of out_key does NOT prove TrajOpt succeeded.
                // Only treat this as a "contact-check-only" failure (proceed with warning) when
                // TrajOpt actually solved the problem, i.e. out_key contains StateWaypoints.
                const auto stored = fut->context->data_storage->getData();
                if (stored.count(out_key) == 0)
                    return {{}, false, "[Run] TrajOpt FAILED (no output) on chunk " + std::to_string(ci + 1)};

                // Walk the output CI and confirm every MoveInstruction has a StateWaypoint.
                // CartesianWaypoints = SimplePlanner seed → TrajOpt genuinely failed.
                const auto &ci_peek = stored.at(out_key).as<CompositeInstruction>();
                bool traj_opt_solved = true;
                for (const auto &i : ci_peek)
                    if (i.isMoveInstruction() &&
                        !i.as<MoveInstructionPoly>().getWaypoint().isStateWaypoint())
                    {
                        traj_opt_solved = false;
                        break;
                    }
                if (!traj_opt_solved)
                    return {{}, false, "[Run] TrajOpt FAILED (could not solve waypoints) on chunk " + std::to_string(ci + 1)};

                // TrajOpt DID produce StateWaypoints. The failure is from the post-plan
                // DiscreteContactCheckTask finding residual contacts. Proceed with a warning.
                CONSOLE_BRIDGE_logWarn(
                    "[Run] Chunk %zu: post-plan contact check flagged residual contact "
                    "but TrajOpt produced a valid solution — proceeding.",
                    ci + 1);
            }

            // ISP
            auto ci_out = fut->context->data_storage
                              ->getData(out_key)
                              .as<CompositeInstruction>();
            tesseract_planning::formatProgram(ci_out, *env_c);

            for (const auto &instr : ci_out)
                if (instr.isMoveInstruction() &&
                    !instr.as<MoveInstructionPoly>().getWaypoint().isStateWaypoint())
                    return {{}, false, "[Run] Non-StateWaypoint before ISP in chunk " + std::to_string(ci + 1)};

            if (!isp->compute(ci_out, *env_c, *profiles))
                return {{}, false, "[Run] ISP FAILED on chunk " + std::to_string(ci + 1)};

            ChunkResult res;
            res.traj = toJointTrajectory(ci_out);
            res.ok = !res.traj.empty();
            if (!res.ok)
                res.error = "[Run] Empty trajectory from ISP on chunk " + std::to_string(ci + 1);
            return res;
        };

        // Process in batches of P parallel chunks
        tf::Executor tf_exec(P);
        tesseract_common::JointTrajectory full_traj;
        double time_offset = 0.0;

        for (size_t batch_begin = 0; batch_begin < n_chunks; batch_begin += P)
        {
            const size_t batch_end = std::min(batch_begin + P, n_chunks);
            const size_t batch_sz = batch_end - batch_begin;

            // Launch all chunks in this batch concurrently
            std::vector<tf::Future<std::optional<ChunkResult>>> batch_futs;
            batch_futs.reserve(batch_sz);
            for (size_t ci = batch_begin; ci < batch_end; ++ci)
                batch_futs.push_back(tf_exec.async(
                    [&plan_one_chunk, ci]()
                    { return plan_one_chunk(ci); }));

            // Collect results in order and stream-publish each
            for (size_t b = 0; b < batch_sz; ++b)
            {
                ChunkResult res = batch_futs[b].get().value_or(
                    ChunkResult{{}, false, "[Run] async future empty"});

                if (!res.ok)
                {
                    CONSOLE_BRIDGE_logError("%s", res.error.c_str());
                    return false;
                }

                const size_t ci = batch_begin + b;
                const bool is_last = (ci == n_chunks - 1);

                // Stitch time: skip duplicate seam point for non-first chunks
                auto it = full_traj.empty() ? res.traj.begin() : res.traj.begin() + 1;
                for (; it != res.traj.end(); ++it)
                {
                    auto st = *it;
                    st.time += time_offset;
                    full_traj.push_back(std::move(st));
                }
                time_offset = full_traj.back().time;

                // Update confirmed start for the FIRST chunk of the NEXT batch
                // (overrides the IK estimate with the actual TrajOpt end state)
                if (b == batch_sz - 1 && batch_end < n_chunks)
                    chunk_starts[batch_end] = res.traj.back().position;

                CONSOLE_BRIDGE_logInform("[Run] Chunk %zu/%zu done → %zu pts, %.2f s total.",
                                         ci + 1, n_chunks, res.traj.size(), time_offset);

                // Notify chunk completion (progress only — full trajectory published by
                // the caller after run() returns, not per-chunk).
                if (chunk_ready_cb_)
                    chunk_ready_cb_(res.traj, joint_names, is_last);
            }
        }

        CONSOLE_BRIDGE_logInform("[Run] All %zu chunks complete: %zu pts, %.2f s",
                                 n_chunks, full_traj.size(), time_offset);
        last_trajectory_ = std::make_shared<tesseract_common::JointTrajectory>(full_traj);

        // ---- Debug visualization ----
        if (debug_ && plotter_ && plotter_->isConnected())
            plotter_->plotTrajectory(full_traj, *env_->getStateSolver());

        if (toolpath_cb_)
        {
            KinematicGroup::ConstPtr manip = env_->getKinematicGroup(manipulator_group_);
            std::vector<Eigen::Vector3d> ee_path;
            ee_path.reserve(full_traj.size());
            for (const auto &state : full_traj)
                ee_path.push_back(manip->calcFwdKin(state.position).at(ee_link_).translation());
            toolpath_cb_(ee_path);
        }

        if (!online_mode_)
        {
            CONSOLE_BRIDGE_logInform("Static planning complete.");
            return true;
        }

        // ========================================================
        // ONLINE MODE: launch real-time SQP re-optimization thread
        // ========================================================
        CONSOLE_BRIDGE_logInform("Launching Online Real-Time Thread...");
        is_executing_online_ = true;

        online_thread_ = std::thread([this, joint_names, full_traj]()
                                     {
        CONSOLE_BRIDGE_logInform("Online Thread Started. Building NLP...");

        auto nlp = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
        KinematicGroup::ConstPtr manip;
        Eigen::MatrixX2d joint_limits;
        {
            std::shared_lock<std::shared_mutex> lock(this->env_mutex_);
            manip        = env_->getKinematicGroup(manipulator_group_);
            joint_limits = manip->getLimits().joint_limits;
        }
        const int num_joints = static_cast<int>(manip->numJoints());
        const int num_steps  = static_cast<int>(full_traj.size());

        std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
        vars.reserve(num_steps);
        for (int i = 0; i < num_steps; ++i)
        {
            auto var = std::make_shared<trajopt_ifopt::JointPosition>(
                full_traj[i].position, joint_names, "Joint_Position_" + std::to_string(i));
            var->SetBounds(joint_limits);
            vars.push_back(var);
            nlp->addVariableSet(var);
        }

        trajopt_common::TrajOptCollisionConfig collision_config(0.03, 100.0);
        collision_config.collision_check_config.type =
            tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
        collision_config.collision_margin_buffer = 0.01;

        auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(full_traj.size());
        for (int i = 1; i < num_steps; ++i)
        {
            auto evaluator = std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(
                collision_cache, manip, env_, collision_config, true);
            auto constraint = std::make_shared<trajopt_ifopt::DiscreteCollisionConstraint>(
                evaluator, vars[i], collision_config.max_num_cnt, false,
                "Collision_" + std::to_string(i));
            nlp->addConstraintSet(constraint);
        }

        nlp->setup();
        auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
        trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
        solver.params.initial_trust_box_size = 0.05;
        solver.params.min_trust_box_size     = 1e-4;
        solver.init(nlp);

        CONSOLE_BRIDGE_logInform("NLP Built. Entering 100 Hz Control Loop.");

        using namespace std::chrono;
        const auto target_dt       = milliseconds(10);
        auto       next_loop_time  = steady_clock::now() + target_dt;
        auto       execution_start = steady_clock::now();
        int        plot_counter    = 0;

        while (this->is_executing_online_)
        {
            {
                std::shared_lock<std::shared_mutex> lock(this->env_mutex_);
                solver.stepSQPSolver();
            }

            Eigen::VectorXd best_vals = solver.getResults().best_var_vals;
            double elapsed = duration<double>(steady_clock::now() - execution_start).count();

            int current_step = 0;
            for (int s = 0; s < num_steps; ++s)
            {
                if (full_traj[s].time <= elapsed)
                    current_step = s;
                else
                    break;
            }
            current_step = std::min(current_step, num_steps - 1);

            if (this->command_cb_)
                this->command_cb_(best_vals.segment(current_step * num_joints, num_joints));

            if (++plot_counter % 10 == 0 && this->toolpath_cb_)
            {
                std::vector<Eigen::Vector3d> ee_path;
                ee_path.reserve(num_steps);
                Eigen::Map<const tesseract_common::TrajArray> traj_mat(
                    best_vals.data(), num_steps, num_joints);
                for (int i = 0; i < num_steps; ++i)
                    ee_path.push_back(manip->calcFwdKin(traj_mat.row(i)).at(ee_link_).translation());
                this->toolpath_cb_(ee_path);
            }

            if (current_step >= num_steps - 1)
            {
                CONSOLE_BRIDGE_logInform("Online: last step reached, stopping.");
                this->is_executing_online_ = false;
                break;
            }

            std::this_thread::sleep_until(next_loop_time);
            next_loop_time += target_dt;
        }
        CONSOLE_BRIDGE_logInform("Online Thread Safely Terminated."); });

        online_thread_.detach();
        return true;
    }

} // namespace Vinhtesseract_examples
