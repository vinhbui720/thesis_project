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

        // ---- Build sparse Cartesian program ----
        CONSOLE_BRIDGE_logInform("Generating Native Sparse Seed...");

        CompositeInstruction sparse_program(
            "DEFAULT", ManipulatorInfo(manipulator_group_, base_link_, ee_link_));
        StateWaypoint start_wp(joint_names, start_pos);
        sparse_program.push_back(MoveInstruction(start_wp, MoveInstructionType::FREESPACE, "FREESPACE"));
        for (const auto &target_pose : target_poses_)
        {
            CartesianWaypoint target_wp(target_pose);
            sparse_program.push_back(
                MoveInstruction(target_wp, MoveInstructionType::FREESPACE, "FREESPACE"));
        }

        // ---- Planner profiles ----
        auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
        profiles->addProfile("SimplePlannerTask", "DEFAULT",
                             std::make_shared<SimplePlannerLVSMoveProfile>());

        if (use_ompl_)
        {
            profiles->addProfile("SimplePlannerTask", "DEFAULT",
                                 std::make_shared<SimplePlannerCompositeProfile>());

            auto ompl_profile = std::make_shared<OMPLRealVectorMoveProfile>();
            ompl_profile->collision_check_config.longest_valid_segment_length = 0.005;
            ompl_profile->collision_check_config.type =
                tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
            ompl_profile->collision_check_config.contact_request.type =
                tesseract_collision::ContactTestType::ALL;

            ompl_profile->solver_config.planners.clear();
            auto rrt1 = std::make_shared<RRTConnectConfigurator>();
            rrt1->range = 0.05;
            auto rrt2 = std::make_shared<RRTConnectConfigurator>();
            rrt2->range = 0.1;
            ompl_profile->solver_config.planners.push_back(rrt1);
            ompl_profile->solver_config.planners.push_back(rrt2);
            ompl_profile->solver_config.planning_time = 10.0;
            ompl_profile->solver_config.max_solutions = 5;
            ompl_profile->solver_config.simplify = true;

            profiles->addProfile("OMPLTask", "FREESPACE", ompl_profile);
        }

        if (ifopt_)
        {
            auto trajopt_ifopt_move = std::make_shared<TrajOptIfoptDefaultMoveProfile>();
            trajopt_ifopt_move->cartesian_constraint_config.enabled = true;
            trajopt_ifopt_move->cartesian_cost_config.enabled = false;
            trajopt_ifopt_move->joint_cost_config.enabled = true;
            trajopt_ifopt_move->joint_cost_config.coeff = Eigen::VectorXd::Ones(6) * 5.0;

            Eigen::VectorXd coeffs(6);
            coeffs << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
            trajopt_ifopt_move->cartesian_constraint_config.coeff = coeffs;

            auto trajopt_ifopt_composite = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();

            // Collision constraint (hard boundary)
            trajopt_ifopt_composite->collision_constraint_config =
                trajopt_common::TrajOptCollisionConfig(0.0, 200);
            trajopt_ifopt_composite->collision_constraint_config.enabled = true;
            trajopt_ifopt_composite->collision_constraint_config.collision_check_config.type =
                tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
            trajopt_ifopt_composite->collision_constraint_config.collision_check_config
                .longest_valid_segment_length = 0.05;
            trajopt_ifopt_composite->collision_constraint_config.collision_margin_buffer = 0.005;

            // Collision cost (soft penalty)
            trajopt_ifopt_composite->collision_cost_config =
                trajopt_common::TrajOptCollisionConfig(0.005, 500);
            trajopt_ifopt_composite->collision_cost_config.enabled = true;
            trajopt_ifopt_composite->collision_cost_config.collision_check_config.type =
                tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
            trajopt_ifopt_composite->collision_cost_config.collision_check_config
                .longest_valid_segment_length = 0.05;
            trajopt_ifopt_composite->collision_cost_config.collision_margin_buffer = 0.01;

            // Smoothing
            trajopt_ifopt_composite->smooth_velocities = true;
            trajopt_ifopt_composite->velocity_coeff = 0.1 * Eigen::VectorXd::Ones(1);
            trajopt_ifopt_composite->smooth_accelerations = true;
            trajopt_ifopt_composite->acceleration_coeff = Eigen::VectorXd::Ones(1);
            trajopt_ifopt_composite->smooth_jerks = true;
            trajopt_ifopt_composite->jerk_coeff = Eigen::VectorXd::Ones(1);

            auto trajopt_ifopt_solver = std::make_shared<TrajOptIfoptOSQPSolverProfile>();
            trajopt_ifopt_solver->opt_params.max_iterations = 200;
            trajopt_ifopt_solver->opt_params.min_approx_improve = 1e-3;
            trajopt_ifopt_solver->opt_params.min_trust_box_size = 1e-3;

            profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_ifopt_move);
            profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_ifopt_composite);
            profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_ifopt_solver);
        }
        else
        {
            auto trajopt_freespace = std::make_shared<TrajOptDefaultMoveProfile>();
            trajopt_freespace->joint_cost_config.enabled = true;
            trajopt_freespace->cartesian_constraint_config.enabled = true;

            auto trajopt_composite = std::make_shared<TrajOptDefaultCompositeProfile>();
            trajopt_composite->collision_constraint_config = trajopt_common::TrajOptCollisionConfig(0.01, 10);
            trajopt_composite->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.02, 50);

            auto trajopt_solver = std::make_shared<TrajOptOSQPSolverProfile>();
            trajopt_solver->opt_params.max_iter = 100;

            profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_freespace);
            profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite);
            profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_solver);
        }

        profiles->addProfile("DiscreteContactCheckTask", "DEFAULT",
                             std::make_shared<ContactCheckProfile>());

        // ---- Task selection ----
        std::string task_name;
        if (use_ompl_)
            task_name = "FreespacePipeline";
        else
            task_name = ifopt_ ? "TrajOptIfoptPipeline" : "TrajOptPipeline";

        CONSOLE_BRIDGE_logInform("Executing %s...", task_name.c_str());

        TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
        const std::string output_key = task->getOutputKeys().get("program");

        auto data_storage = std::make_unique<TaskComposerDataStorage>();
        data_storage->setData("planning_input", sparse_program);
        data_storage->setData("environment",
                              std::shared_ptr<const tesseract_environment::Environment>(env_));
        data_storage->setData("profiles", profiles);

        auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");
        auto context = std::make_shared<TaskComposerContext>(task->getName(), std::move(data_storage));

        TaskComposerFuture::UPtr future = executor->run(*task, std::move(context));
        future->wait();

        if (!future->context->isSuccessful())
            CONSOLE_BRIDGE_logError("Optimization FAILED!");

        // ---- Time parameterization ----
        CONSOLE_BRIDGE_logInform("Optimization SUCCESS!");
        auto ci = future->context->data_storage->getData(output_key).as<CompositeInstruction>();
        tesseract_planning::formatProgram(ci, *env_);

        CONSOLE_BRIDGE_logInform("Applying Time Parameterization (ISP)...");
        for (const auto &instr : ci)
        {
            if (instr.isMoveInstruction())
            {
                const auto &mi = instr.as<MoveInstructionPoly>();
                if (!mi.getWaypoint().isStateWaypoint())
                {
                    CONSOLE_BRIDGE_logError("Found non-StateWaypoint before ISP!");
                    return false;
                }
            }
        }

        auto isp = std::make_unique<tesseract_planning::IterativeSplineParameterization>(
            "IterativeSplineParameterization");
        try
        {
            if (!isp->compute(ci, *env_, *profiles))
            {
                CONSOLE_BRIDGE_logError("Time Parameterization FAILED!");
                return false;
            }
        }
        catch (const std::exception &e)
        {
            CONSOLE_BRIDGE_logError("ISP crashed: %s", e.what());
            return false;
        }
        CONSOLE_BRIDGE_logInform("Time Parameterization SUCCESS!");

        CompositeInstruction nested_program("DEFAULT");
        nested_program.push_back(ci);
        tesseract_planning::rescaleTimings(nested_program, {1.0});
        CONSOLE_BRIDGE_logInform("Successfully scaled timings.");

        tesseract_common::JointTrajectory trajectory = toJointTrajectory(nested_program);
        last_trajectory_ = std::make_shared<tesseract_common::JointTrajectory>(trajectory);

        // ---- Debug visualization ----
        if (debug_ && plotter_ && plotter_->isConnected())
            plotter_->plotTrajectory(trajectory, *env_->getStateSolver());

        if (toolpath_cb_)
        {
            KinematicGroup::ConstPtr manip = env_->getKinematicGroup(manipulator_group_);
            std::vector<Eigen::Vector3d> ee_path;
            ee_path.reserve(trajectory.size());
            for (const auto &state : trajectory)
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

        online_thread_ = std::thread([this, joint_names, trajectory]()
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
        const int num_steps  = static_cast<int>(trajectory.size());

        std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
        vars.reserve(num_steps);
        for (int i = 0; i < num_steps; ++i)
        {
            auto var = std::make_shared<trajopt_ifopt::JointPosition>(
                trajectory[i].position, joint_names, "Joint_Position_" + std::to_string(i));
            var->SetBounds(joint_limits);
            vars.push_back(var);
            nlp->addVariableSet(var);
        }

        trajopt_common::TrajOptCollisionConfig collision_config(0.03, 100.0);
        collision_config.collision_check_config.type =
            tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
        collision_config.collision_margin_buffer = 0.01;

        auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(trajectory.size());
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
                if (trajectory[s].time <= elapsed)
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
