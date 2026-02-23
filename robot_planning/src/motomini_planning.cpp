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

using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

// --- CONFIGURATION ---
const std::string MANIPULATOR_GROUP = "manipulator";
const std::string LINK_BASE = "world";
const std::string LINK_TIP = "tool0";
static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";

namespace Vinhtesseract_examples
{
    // --- HELPER 1: TRAJECTORY SUBDIVISION ---
    CompositeInstruction subdivideProgram(
        const std::vector<Eigen::VectorXd> &seed_points,
        const std::vector<std::string> &joint_names,
        int steps_per_segment)
    {
        CompositeInstruction dense_program("DEFAULT", ManipulatorInfo(MANIPULATOR_GROUP, LINK_BASE, LINK_TIP));

        if (seed_points.size() < 2)
            return dense_program;

        StateWaypoint start_wp(joint_names, seed_points[0]);
        MoveInstruction start_instr(start_wp, MoveInstructionType::FREESPACE, "FREESPACE");
        start_instr.setDescription("Start");
        dense_program.push_back(start_instr);

        for (size_t i = 0; i < seed_points.size() - 1; ++i)
        {
            Eigen::VectorXd p_start = seed_points[i];
            Eigen::VectorXd p_end = seed_points[i + 1];

            for (int step = 1; step <= steps_per_segment; ++step)
            {
                double t = (double)step / (double)steps_per_segment;
                Eigen::VectorXd p_interp = p_start + t * (p_end - p_start);

                StateWaypoint wp(joint_names, p_interp);
                MoveInstruction instr(wp, MoveInstructionType::FREESPACE, "FREESPACE");

                if (step == steps_per_segment)
                {
                    instr.setDescription("Target_" + std::to_string(i));
                }
                else
                {
                    instr.setDescription("Substep_" + std::to_string(i) + "_" + std::to_string(step));
                }
                dense_program.push_back(instr);
            }
        }
        return dense_program;
    }

    // --- HELPER 2: ROBUST IK SEED GENERATOR ---
    std::vector<Eigen::VectorXd> generateIKSeed(
        const std::shared_ptr<const tesseract_environment::Environment> &env,
        const std::vector<std::string> &joint_names,
        const Eigen::VectorXd &start_pos,
        const std::vector<Eigen::Isometry3d> &target_poses)
    {
        std::vector<Eigen::VectorXd> seed_traj;
        auto kin_group = env->getKinematicGroup(MANIPULATOR_GROUP);

        if (!kin_group)
        {
            CONSOLE_BRIDGE_logError("Manipulator group '%s' not found for seeding!", MANIPULATOR_GROUP.c_str());
            return seed_traj;
        }

        seed_traj.push_back(start_pos);
        Eigen::VectorXd last_solution = start_pos;

        for (size_t i = 0; i < target_poses.size(); ++i)
        {
            tesseract_kinematics::KinGroupIKInput ik_input;
            ik_input.pose = target_poses[i];
            ik_input.working_frame = LINK_BASE;
            ik_input.tip_link_name = LINK_TIP;

            auto solutions = kin_group->calcInvKin(ik_input, last_solution);

            if (!solutions.empty())
            {
                last_solution = solutions[0];
                seed_traj.push_back(last_solution);
                CONSOLE_BRIDGE_logInform("  -> IK Solved using neighbor seed for Target %zu", i);
            }
            else
            {
                bool found = false;
                for (int r = 0; r < 20; ++r)
                {
                    Eigen::VectorXd random_seed = Eigen::VectorXd::Random(joint_names.size());
                    solutions = kin_group->calcInvKin(ik_input, random_seed);
                    if (!solutions.empty())
                    {
                        last_solution = solutions[0];
                        seed_traj.push_back(last_solution);
                        found = true;
                        CONSOLE_BRIDGE_logInform("  -> IK Solved using RANDOM seed for Target %zu (Attempt %d)", i, r + 1);
                        break;
                    }
                }
                if (!found)
                {
                    CONSOLE_BRIDGE_logError("  -> IK FAILED for Target %zu. Optimization will likely fail.", i);
                    seed_traj.push_back(last_solution);
                }
            }
        }
        return seed_traj;
    }

    MotoMiniPlanning::MotoMiniPlanning(std::shared_ptr<tesseract_environment::Environment> env,
                                       std::shared_ptr<tesseract_visualization::Visualization> plotter,
                                       bool debug,
                                       bool ifopt)
        : Example(std::move(env), std::move(plotter)), debug_(debug), ifopt_(ifopt)
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

        CONSOLE_BRIDGE_logInform("Generating & Subdividing Seed...");
        std::vector<Eigen::VectorXd> sparse_seed = generateIKSeed(env_, joint_names, start_pos, target_poses_);

        // SUBDIVISION: 20 steps per segment.
        CompositeInstruction dense_program = subdivideProgram(sparse_seed, joint_names, 20);

        auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();

        if (ifopt_)
        {
            auto trajopt_ifopt_move = std::make_shared<TrajOptIfoptDefaultMoveProfile>();
            trajopt_ifopt_move->cartesian_constraint_config.enabled = true;
            trajopt_ifopt_move->cartesian_constraint_config.coeff = Eigen::VectorXd::Constant(6, 1, 1.0);

            auto trajopt_ifopt_composite = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
            trajopt_ifopt_composite->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.010, 20);
            trajopt_ifopt_composite->collision_cost_config.enabled = true;
            trajopt_ifopt_composite->collision_constraint_config.enabled = false;

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

        std::string task_name = (ifopt_) ? "TrajOptIfoptPipeline" : "TrajOptPipeline";
        CONSOLE_BRIDGE_logInform("Executing %s with Dense Seed...", task_name.c_str());

        TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
        const std::string output_key = task->getOutputKeys().get("program");

        auto data_storage = std::make_unique<tesseract_planning::TaskComposerDataStorage>();
        data_storage->setData("planning_input", dense_program);
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
        tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);

        // --- FIX: MANUAL TIME PARAMETERIZATION ---
        // Since Tesseract Time Parameterization library is missing, we calculate
        // timestamps manually based on constant velocity. This ensures dt > 0.

        double max_velocity = 0.5; // rad/s (Adjust this to control speed!)
        double min_dt = 0.01;      // Minimum time step (10ms) to satisfy ros2_control

        if (!trajectory.empty())
        {
            trajectory[0].time = 0.0;

            for (size_t i = 1; i < trajectory.size(); ++i)
            {
                // 1. Calculate the longest distance any joint has to travel in this step
                double max_joint_dist = 0.0;
                for (long j = 0; j < trajectory[i].position.rows(); ++j)
                {
                    double dist = std::abs(trajectory[i].position[j] - trajectory[i - 1].position[j]);
                    if (dist > max_joint_dist)
                        max_joint_dist = dist;
                }

                // 2. Calculate time required: t = distance / velocity
                double dt = max_joint_dist / max_velocity;

                // 3. ENFORCE MINIMUM TIME STEP
                // This prevents "Time between points is 0.000" errors from ros2_control
                if (dt < min_dt)
                    dt = min_dt;

                trajectory[i].time = trajectory[i - 1].time + dt;

                // (Optional) Calculate rudimentary velocity for controller (v = d/t)
                trajectory[i].velocity = (trajectory[i].position - trajectory[i - 1].position) / dt;
            }
        }

        last_trajectory_ = std::make_shared<tesseract_common::JointTrajectory>(trajectory);

        if (plotter_ != nullptr && plotter_->isConnected())
        {
            plotter_->plotTrajectory(trajectory, *env_->getStateSolver());
        }

        return true;
    }
}