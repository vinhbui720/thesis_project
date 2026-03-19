/**
 * @file motomini_planning.cpp
 * @brief Implementation of MotoMini planning logic using "Pro" Ifopt Subdivision
 * @author Bùi Quang Vinh
 */

#include <robot_planning/motomini_planning.h>
#include <tesseract_common/types.h>

// --- INCLUDES ---
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <trajopt_common/collision_types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

// REMOVED: #include <tesseract_time_parameterization/...> (Not needed, we do it manually)

// --- KINEMATICS HEADERS ---
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_kinematics/core/types.h>
#include <tesseract_kinematics/core/utils.h>

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/environment.h>

// --- TRAJOPT PROFILES ---
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_move_profile.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/core/utils.h>

// Command Language
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

// Time Parameterization
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
// #include <tesseract_time_parameterization/isp/iterative_spline_parameterization_profile.h>
#include <tesseract_time_parameterization/core/utils.h>

// --- ONLINE SQP SOLVER HEADERS ---
#include <tesseract_common/profile_dictionary.h>
#include <trajopt_sqp/qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>

using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

// --- CONFIGURATION ---
static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";

namespace Vinhtesseract_examples
{
    MotoMiniPlanning::MotoMiniPlanning(
        std::shared_ptr<tesseract_environment::Environment> env,
        std::shared_ptr<tesseract_visualization::Visualization> plotter,
        std::string manipulator_group,
        std::string base_link,
        std::string ee_link,
        bool debug,
        bool ifopt,
        bool use_ompl,
        bool online_mode)
        : Example(std::move(env), std::move(plotter)), manipulator_group_(std::move(manipulator_group)),
          base_link_(std::move(base_link)), ee_link_(std::move(ee_link)), debug_(debug), ifopt_(ifopt), use_ompl_(use_ompl), online_mode_(online_mode)
    {
        last_trajectory_ = nullptr;
    }

    void MotoMiniPlanning::setTargetPoses(const std::vector<Eigen::Isometry3d> &poses)
    {
        target_poses_ = poses;
    }

    std::shared_ptr<tesseract_common::JointTrajectory> MotoMiniPlanning::getTrajectory() const
    {
        return last_trajectory_;
    }
    void MotoMiniPlanning::setCommandCallback(CommandCallback cb)
    {
        command_cb_ = std::move(cb);
    }
    void MotoMiniPlanning::updateEnvironmentState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_pos)
    {
        std::unique_lock<std::shared_mutex> lock(env_mutex_);
        env_->setState(joint_names, joint_pos);
    }
    void MotoMiniPlanning::setToolpathCallback(ToolpathCallback cb)
    {
        toolpath_cb_ = std::move(cb);
    }
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

        std::vector<std::string> joint_names = {"joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};
        Eigen::VectorXd start_pos = env_->getCurrentJointValues(joint_names);
        env_->setState(joint_names, start_pos);

        std::shared_ptr<const tesseract_common::ResourceLocator> locator = env_->getResourceLocator();
        std::filesystem::path config_path(
            locator->locateResource("package://tesseract_task_composer/config/task_composer_plugins.yaml")->getFilePath());
        TaskComposerPluginFactory factory(config_path, *env_->getResourceLocator());

        CONSOLE_BRIDGE_logInform("Generating Native Sparse Seed...");

        CompositeInstruction sparse_program("DEFAULT", tesseract_common::ManipulatorInfo(manipulator_group_, base_link_, ee_link_));
        StateWaypoint start_wp(joint_names, start_pos);
        MoveInstruction start_instr(start_wp, MoveInstructionType::FREESPACE, "FREESPACE");
        sparse_program.push_back(start_instr);
        for (const auto &target_pose : target_poses_)
        {
            CartesianWaypoint target_wp(target_pose);
            MoveInstruction target_instr(target_wp, MoveInstructionType::FREESPACE, "FREESPACE");
            sparse_program.push_back(target_instr);
        }

        auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
        auto simple_move_profile = std::make_shared<tesseract_planning::SimplePlannerLVSMoveProfile>();

        profiles->addProfile("SimplePlannerTask", "DEFAULT", simple_move_profile);
        if (use_ompl_)
        {
            // 2. Add the Composite Profile (NO template brackets!)
            auto simple_composite_profile = std::make_shared<tesseract_planning::SimplePlannerCompositeProfile>();
            profiles->addProfile("SimplePlannerTask", "DEFAULT", simple_composite_profile);

            auto ompl_profile = std::make_shared<tesseract_planning::OMPLRealVectorMoveProfile>();
            ompl_profile->solver_config.planners.clear();
            auto rrt_planner = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
            ompl_profile->solver_config.planners.push_back(rrt_planner);
            profiles->addProfile("OMPLTask", "FREESPACE", ompl_profile);
        }

        if (ifopt_)
        {
            auto trajopt_ifopt_move = std::make_shared<TrajOptIfoptDefaultMoveProfile>();
            trajopt_ifopt_move->cartesian_constraint_config.enabled = true;
            trajopt_ifopt_move->cartesian_constraint_config.coeff = Eigen::VectorXd::Constant(6, 1, 1.0);

            auto trajopt_ifopt_composite = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
            trajopt_ifopt_composite->collision_cost_config = trajopt_common::TrajOptCollisionConfig(1.0e-10, 20);
            trajopt_ifopt_composite->collision_cost_config.enabled = true;
            trajopt_ifopt_composite->collision_constraint_config.enabled = true;

            trajopt_ifopt_composite->smooth_velocities = false;
            trajopt_ifopt_composite->velocity_coeff = Eigen::VectorXd::Ones(1);
            trajopt_ifopt_composite->smooth_accelerations = true;
            trajopt_ifopt_composite->acceleration_coeff = Eigen::VectorXd::Ones(1);
            trajopt_ifopt_composite->smooth_jerks = true;
            trajopt_ifopt_composite->jerk_coeff = Eigen::VectorXd::Ones(1);

            auto trajopt_ifopt_solver = std::make_shared<TrajOptIfoptOSQPSolverProfile>();
            trajopt_ifopt_solver->opt_params.max_iterations = 100;

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
            trajopt_composite->collision_constraint_config = trajopt_common::TrajOptCollisionConfig(0.0, 10);
            trajopt_composite->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.005, 50);

            auto trajopt_solver = std::make_shared<TrajOptOSQPSolverProfile>();
            trajopt_solver->opt_params.max_iter = 100;

            profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_freespace);
            profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite);
            profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_solver);
        }

        auto post_check = std::make_shared<ContactCheckProfile>();
        profiles->addProfile("DiscreteContactCheckTask", "DEFAULT", post_check);

        std::string task_name;
        if (use_ompl_)
        {
            // Global -> Local pipeline (Uses OMPL to find a seed, then optimizes)
            task_name = "FreespacePipeline";
        }
        else
        {
            // Local optimization only (Skips OMPL, uses linear interpolation as seed)
            task_name = ifopt_ ? "TrajOptIfoptPipeline" : "TrajOptPipeline";
        }
        CONSOLE_BRIDGE_logInform("Executing %s (Global -> Local Pipeline)...", task_name.c_str());

        TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
        const std::string output_key = task->getOutputKeys().get("program");

        auto data_storage = std::make_unique<tesseract_planning::TaskComposerDataStorage>();
        // data_storage->setData("planning_input", dense_program);
        data_storage->setData("planning_input", sparse_program);
        data_storage->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
        data_storage->setData("profiles", profiles);

        auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");
        auto context = std::make_shared<tesseract_planning::TaskComposerContext>(task->getName(), std::move(data_storage));

        TaskComposerFuture::UPtr future = executor->run(*task, std::move(context));
        future->wait();

        if (!future->context->isSuccessful())
        {
            CONSOLE_BRIDGE_logError("Optimization FAILED!");
            return false;
        }

        CONSOLE_BRIDGE_logInform("Optimization SUCCESS!");
        auto ci = future->context->data_storage->getData(output_key).as<CompositeInstruction>();

        std::unique_ptr<tesseract_planning::TimeParameterization> time_parameterization_alg =
            std::make_unique<tesseract_planning::IterativeSplineParameterization>("IterativeSplineParameterization");

        // Compute the timestams at 100% maximum speed
        CONSOLE_BRIDGE_logInform("Applying Time Parameterization (ISP)...");

        if (time_parameterization_alg)
        {
            bool success = time_parameterization_alg->compute(ci, *env_, *profiles);
            if (!success)
            {
                CONSOLE_BRIDGE_logError("Time Parameterization FAILED!");
                return false;
            }

            CONSOLE_BRIDGE_logInform("Time Parameterization SUCCESS!");
        }
        tesseract_planning::CompositeInstruction nested_program("DEFAULT");
        nested_program.push_back(ci);
        std::vector<double> speed_scalings = {1.0};
        tesseract_planning::rescaleTimings(nested_program, speed_scalings);
        CONSOLE_BRIDGE_logInform("Successfully scaled timings using Tesseract utils.");
        tesseract_common::JointTrajectory trajectory = toJointTrajectory(nested_program);

        last_trajectory_ = std::make_shared<tesseract_common::JointTrajectory>(trajectory);

        if (debug_ && plotter_ != nullptr && plotter_->isConnected())
        {
            plotter_->plotTrajectory(trajectory, *env_->getStateSolver());
        }

        if (toolpath_cb_)
        {
            tesseract_kinematics::KinematicGroup::ConstPtr manip = env_->getKinematicGroup(manipulator_group_);
            std::vector<Eigen::Vector3d> ee_path;
            ee_path.reserve(trajectory.size());

            for (const auto &state : trajectory)
            {
                Eigen::Isometry3d tf = manip->calcFwdKin(state.position).at(this->ee_link_);
                ee_path.push_back(tf.translation());
            }
            toolpath_cb_(ee_path);
        }
        // ==========================================
        // ONLINE PLANNING MODE TRANSITION
        // ==========================================
        if (!online_mode_)
        {
            CONSOLE_BRIDGE_logInform("Static planning complete. Exiting run().");
            return true;
        }

        CONSOLE_BRIDGE_logInform("Launching Online Real-Time Thread...");

        is_executing_online_ = true;

        // Launch the isolated control loop thread
        online_thread_ = std::thread([this, joint_names, trajectory]()
                                     {
            
            CONSOLE_BRIDGE_logInform("Online Thread Started. Building NLP...");

            auto nlp = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
            // tesseract_kinematics::KinematicGroup::ConstPtr manip = env_->getKinematicGroup(manipulator_group_);
            // Eigen::MatrixX2d joint_limits = manip->getLimits().joint_limits;
            tesseract_kinematics::KinematicGroup::ConstPtr manip;
            Eigen::MatrixX2d joint_limits;
            { // Thread-safe read of the environment
                std::shared_lock<std::shared_mutex> lock(this->env_mutex_);
                manip = env_->getKinematicGroup(manipulator_group_);
                joint_limits = manip->getLimits().joint_limits;
            }
            std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;

            for (size_t i = 0; i < trajectory.size(); ++i)
            {
                auto var = std::make_shared<trajopt_ifopt::JointPosition>(
                    trajectory[i].position, joint_names, "Joint_Position_" + std::to_string(i));
                var->SetBounds(joint_limits);
                vars.push_back(var);
                nlp->addVariableSet(var);
            }

            trajopt_common::TrajOptCollisionConfig collision_config(0.05, 10.0);
            collision_config.collision_check_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
            auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(trajectory.size());

            for (size_t i = 1; i < trajectory.size(); i++)
            {
                auto collision_evaluator = std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(
                    collision_cache, manip, env_, collision_config, true);
                auto collision_constraint = std::make_shared<trajopt_ifopt::DiscreteCollisionConstraint>(
                    collision_evaluator, vars[i], collision_config.max_num_cnt, false, "Collision_" + std::to_string(i));
                nlp->addConstraintSet(collision_constraint);
            }

            nlp->setup();
            auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
            trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
            double box_size = 0.05;
            solver.params.initial_trust_box_size = box_size;
            solver.init(nlp);

            CONSOLE_BRIDGE_logInform("NLP Built. Entering 100Hz Control Loop.");
            using namespace std::chrono;
            const auto target_dt = milliseconds(10); 
            auto next_loop_time = steady_clock::now() + target_dt;

            int num_joints = manip->numJoints(); 
            int num_steps = trajectory.size();
            int plot_throttle_counter = 0;
            while (this->is_executing_online_)
            {
                { // Lock the environment just long enough for the solver to read it
                    std::shared_lock<std::shared_mutex> lock(this->env_mutex_);
                    solver.stepSQPSolver();
                }

                Eigen::VectorXd safe_trajectory_vector = solver.getResults().best_var_vals;
                Eigen::VectorXd safe_next_step = safe_trajectory_vector.segment(0, 6);

                if (this->command_cb_) {
                    this->command_cb_(safe_next_step);
                }
                if (plot_throttle_counter++ % 10 == 0 && this->toolpath_cb_) 
                {
                    std::vector<Eigen::Vector3d> ee_path;
                    ee_path.reserve(num_steps); 

                    // Map the 1D vector back into an N x J matrix
                    Eigen::Map<const tesseract_common::TrajArray> traj_matrix(
                        safe_trajectory_vector.data(), num_steps, num_joints);

                    // Calculate FK for every waypoint in the live trajectory
                    for (int i = 0; i < num_steps; ++i) {
                        Eigen::Isometry3d tf = manip->calcFwdKin(traj_matrix.row(i)).at(this->ee_link_);
                        ee_path.push_back(tf.translation());
                    }

                    this->toolpath_cb_(ee_path);
                }
                // 4. Reset the trust box
                solver.setBoxSize(box_size);

                // 5. Enforce loop timing (100Hz)
                std::this_thread::sleep_until(next_loop_time);
                next_loop_time += target_dt;
            }
            CONSOLE_BRIDGE_logInform("Online Thread Safely Terminated."); });

        // Detach so run() returns immediately to the ROS 2 node
        online_thread_.detach();

        return true;
    }
}