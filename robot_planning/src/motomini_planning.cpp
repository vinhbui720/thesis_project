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
#include <tesseract_common/profile_dictionary.h>

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

        CompositeInstruction sparse_program("DEFAULT", tesseract_common::ManipulatorInfo(MANIPULATOR_GROUP, LINK_BASE, LINK_TIP));
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

        profiles->addProfile(
            "SimplePlannerTask",
            "FREESPACE",
            simple_move_profile);

        // 2. Add the Composite Profile (NO template brackets!)
        auto simple_composite_profile = std::make_shared<tesseract_planning::SimplePlannerCompositeProfile>();
        profiles->addProfile(
            "SimplePlannerTask",
            "DEFAULT",
            simple_composite_profile);

        if (ifopt_)
        {
            auto trajopt_ifopt_move = std::make_shared<TrajOptIfoptDefaultMoveProfile>();
            trajopt_ifopt_move->cartesian_constraint_config.enabled = true;
            trajopt_ifopt_move->cartesian_constraint_config.coeff = Eigen::VectorXd::Constant(6, 1, 1.0);

            auto trajopt_ifopt_composite = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
            trajopt_ifopt_composite->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.001, 20);
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

        if (plotter_ != nullptr && plotter_->isConnected())
        {
            plotter_->plotTrajectory(trajectory, *env_->getStateSolver());
        }

        return true;
    }
}