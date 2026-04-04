/**
 * @file INTEGRATION_EXAMPLE.cpp
 * @brief Complete example of integrating enhanced debug features into motomini_planning_run.cpp
 *
 * This file demonstrates how to add all visualization features to your planning pipeline:
 *   1. Collision monitoring
 *   2. Cartesian error tracking
 *   3. Trajectory visualization
 *   4. SQP optimization debugging
 *
 * Copy-paste relevant sections into your actual planning code.
 */

// ============================================================================
// EXAMPLE 1: Basic Integration in MotoMiniPlanning::run()
// ============================================================================

/*
In motomini_planning_run.cpp, add after line where you initialize profiles:

#include <rclcpp/rclcpp.hpp>
#include "robot_planning/enhanced_debug_node.cpp"
#include "robot_planning/sqp_debug_callback.hpp"

// ... existing code ...

// Create debug node (if not already created)
static std::shared_ptr<EnhancedDebugNode> debug_node = nullptr;
if (!debug_node) {
    debug_node = std::make_shared<EnhancedDebugNode>();
}

// Set target cartesian poses for visualization
for (size_t i = 0; i < target_poses_.size(); ++i) {
    debug_node->setTargetCartesianPose(ee_link_, target_poses_[i]);
    CONSOLE_BRIDGE_logInform("[Debug] Target %zu: [%.3f, %.3f, %.3f]",
        i, target_poses_[i].translation().x(),
        target_poses_[i].translation().y(),
        target_poses_[i].translation().z());
}

// After planning each chunk, visualize trajectory
auto plan_one_chunk = [&](size_t ci) -> ChunkResult {
    // ... existing chunk planning code ...

    // AFTER successful trajectory:
    ChunkResult res;
    res.traj = toJointTrajectory(ci_out);
    res.ok = !res.traj.empty();

    // VIZ: Add trajectory waypoints for animation
    double chunk_time_start = (ci * C) * 0.1;  // Approximate timing
    for (const auto& state : res.traj) {
        debug_node->addTrajectoryWaypoint(
            state.position,
            chunk_time_start + state.time);
    }

    // VIZ: Get collision debug info for this chunk
    const auto& collision_info = debug_node->getCollisionDebugInfo();
    if (!collision_info.empty()) {
        CONSOLE_BRIDGE_logWarn("[Debug] Chunk %zu has %zu collision violations",
            ci, collision_info.size());
        for (const auto& info : collision_info) {
            CONSOLE_BRIDGE_logWarn("  %s ↔ %s: %.3f m",
                info.link1.c_str(), info.link2.c_str(), info.distance);
        }
    }

    return res;
};
*/

// ============================================================================
// EXAMPLE 2: Online Mode Integration with SQP Debug Callback
// ============================================================================

/*
In the online_thread_ of motomini_planning_run.cpp:

// Create debug callback for SQP monitoring
auto sqp_debug = std::make_shared<Vinhtesseract_examples::SQPDebugCallback>();

// Register callbacks
sqp_debug->setOnIterationCallback([this](const auto& metrics) {
    CONSOLE_BRIDGE_logInform(
        "[SQP Iter %d] Cost=%.4f | Collision_Cost=%.4f | "
        "Collisions=%d | Cartesian_Errors=%d | "
        "MaxVel=%.3f | MaxAcc=%.3f",
        metrics.iteration,
        metrics.total_cost,
        metrics.collision_cost,
        metrics.collision_violations,
        metrics.cartesian_violations,
        metrics.max_joint_velocity,
        metrics.max_joint_acceleration);
});

sqp_debug->setOnCollisionCallback([this](const auto& data) {
    if (data.total_violations > 0) {
        CONSOLE_BRIDGE_logWarn(
            "[SQP Collisions] Total violations: %d",
            data.total_violations);
        for (const auto& pair : data.collision_pairs) {
            CONSOLE_BRIDGE_logWarn(
                "  %s ↔ %s: min_dist=%.4f, margin=%.4f, viol=%d",
                pair.link1.c_str(), pair.link2.c_str(),
                pair.min_distance, pair.margin, pair.violations);
        }
    }
});

sqp_debug->setOnCartesianCallback([this](const auto& data) {
    for (const auto& constraint : data.constraints) {
        if (!constraint.is_satisfied) {
            CONSOLE_BRIDGE_logWarn(
                "[Cartesian] %s error: pos=%.4f",
                constraint.link_name.c_str(),
                constraint.position_error_norm);
        }
    }
});

// In the solver loop (100Hz):
for (int iteration = 0; iteration < max_iterations && is_executing_online_; ++iteration) {
    {
        std::shared_lock<std::shared_mutex> lock(env_mutex_);
        solver.stepSQPSolver();
    }

    // Collect metrics
    Vinhtesseract_examples::SQPIterationMetrics metrics;
    metrics.iteration = iteration;
    metrics.total_cost = solver.getResults().best_cost;
    metrics.collision_cost = 0.0;  // Extract from solver
    metrics.cartesian_cost = 0.0;  // Extract from solver

    // Analyze trajectory
    Eigen::VectorXd best_vals = solver.getResults().best_var_vals;
    Eigen::Map<const tesseract_common::TrajArray> traj_mat(
        best_vals.data(), num_steps, num_joints);

    for (int i = 0; i < num_steps; ++i) {
        tesseract_common::JointState state;
        state.position = traj_mat.row(i);
        state.time = full_traj[i].time;
        // Add to temporary trajectory for analysis
    }

    tesseract_common::JointTrajectory temp_traj;
    // ... populate temp_traj ...
    Vinhtesseract_examples::SQPTrajectoryAnalyzer::analyzeTrajectory(
        temp_traj, joint_names, metrics);

    // Fire callbacks
    sqp_debug->updateMetrics(metrics);

    // Publish command at current step...
}
*/

// ============================================================================
// EXAMPLE 3: Standalone Debug Monitor
// ============================================================================

/*
Create debug_monitor.cpp for standalone monitoring:

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "robot_planning/enhanced_debug_node.cpp"

class DebugMonitor : public rclcpp::Node {
public:
    DebugMonitor() : Node("debug_monitor") {
        debug_node_ = std::make_shared<EnhancedDebugNode>();

        // Set up your targets
        Eigen::Isometry3d target;
        target.setIdentity();
        target.translation() = Eigen::Vector3d(0.5, 0.3, 0.2);
        debug_node_->setTargetCartesianPose("ee_link", target);
    }

private:
    std::shared_ptr<EnhancedDebugNode> debug_node_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugMonitor>());
    rclcpp::shutdown();
    return 0;
}
*/

// ============================================================================
// EXAMPLE 4: Metrics Collection Helper
// ============================================================================

class PlanningMetricsCollector
{
public:
    struct Metrics
    {
        double planning_time_ms = 0.0;
        double trajectory_duration_sec = 0.0;
        double toolpath_length_m = 0.0;
        int trajectory_points = 0;
        int collision_violations = 0;
        double max_cartesian_error_m = 0.0;

        void print() const
        {
            std::cout << "Planning Metrics:\n"
                      << "  Time: " << planning_time_ms << " ms\n"
                      << "  Duration: " << trajectory_duration_sec << " s\n"
                      << "  Path Length: " << toolpath_length_m << " m\n"
                      << "  Points: " << trajectory_points << "\n"
                      << "  Collisions: " << collision_violations << "\n"
                      << "  Max Error: " << max_cartesian_error_m << " m\n";
        }
    };

    static Metrics collect(
        const std::shared_ptr<EnhancedDebugNode> &debug_node,
        const tesseract_common::JointTrajectory &traj,
        const std::shared_ptr<tesseract_kinematics::KinematicGroup> &manip,
        const std::string &ee_link,
        double planning_time_ms)
    {
        Metrics m;
        m.planning_time_ms = planning_time_ms;

        if (!traj.empty())
        {
            m.trajectory_points = traj.size();
            m.trajectory_duration_sec = traj.back().time;

            // Calculate toolpath length
            Eigen::Vector3d prev = manip->calcFwdKin(traj[0].position)
                                       .at(ee_link)
                                       .translation();
            for (size_t i = 1; i < traj.size(); ++i)
            {
                Eigen::Vector3d curr = manip->calcFwdKin(traj[i].position)
                                           .at(ee_link)
                                           .translation();
                m.toolpath_length_m += (curr - prev).norm();
                prev = curr;
            }
        }

        const auto &coll_info = debug_node->getCollisionDebugInfo();
        m.collision_violations = coll_info.size();

        // Calculate max cartesian error (if targets set)
        // ... implementation specific to your tracking system ...

        return m;
    }
};

// ============================================================================
// EXAMPLE 5: RViz Configuration for Debug Visualization
// ============================================================================

/*
Save as debug_rviz_config.rviz in your config directory:

Visualization Manager:
  Class: ""
  Displays:
    - Class: RobotModel
      Name: Robot
      Alpha: 1.0

    - Class: Grid
      Name: Grid
      Cell Size: 0.1
      Color: {r: 128, g: 128, b: 128}

    - Class: MarkerArray
      Name: DebugMarkers
      Topic: /debug_markers

    - Class: Axes
      Name: World
      Frame: world

  Tools:
    - Class: Interact
      Tool Name: Interact
    - Class: MoveCamera
      Tool Name: Move Camera

  Property Tree Widget:
    Expanded: ~
    Splitter Geometry: 320, 669, 720
    Tree Height: 669

View Manager:
  Class: rviz/View
  Config Name: Top-Down

  Top-Down:
    Name: Top-Down
    Class: rviz/TopDownOrtho
    Distance: 5.0
*/

// ============================================================================
// EXAMPLE 6: Launch File Integration
// ============================================================================

/*
In your launch file (e.g., planning_with_debug.launch.py):

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Planning node
    planning = Node(
        package='robot_planning',
        executable='motomini_planning_node',
        output='screen',
        parameters=[
            {'planning_config': '/path/to/config.yaml'},
        ]
    )

    # Enhanced debug node
    debug = Node(
        package='robot_planning',
        executable='enhanced_debug_node',
        output='screen',
        parameters=[
            {'collision_threshold': 0.10},
            {'enable_collision_gradient': True},
        ]
    )

    # RViz visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', '/path/to/debug_rviz_config.rviz'],
    )

    return LaunchDescription([
        planning,
        debug,
        rviz,
    ])
*/

// ============================================================================
// EXAMPLE 7: Adding Custom Metrics to Debug Callback
// ============================================================================

/*
Extend SQPDebugCallback for project-specific metrics:

class MotoMiniSQPDebugCallback : public Vinhtesseract_examples::SQPDebugCallback {
public:
    void updateWithMotominoData(
        const Eigen::VectorXd& joint_positions,
        const std::vector<std::string>& joint_names,
        const std::shared_ptr<tesseract_kinematics::KinematicGroup>& manip,
        const std::string& ee_link)
    {
        // Project-specific metrics
        try {
            Eigen::Isometry3d fk = manip->calcFwdKin(joint_positions).at(ee_link);
            current_ee_position_ = fk.translation();
            current_ee_orientation_ = fk.rotation();

            // Update metrics with motomini-specific info
            if (on_iteration_cb_) {
                auto metrics = current_metrics_;
                metrics.iteration++;
                on_iteration_cb_(metrics);
            }
        } catch (const std::exception& e) {
            std::cerr << "FK calculation failed: " << e.what() << std::endl;
        }
    }

private:
    Eigen::Vector3d current_ee_position_;
    Eigen::Matrix3d current_ee_orientation_;
};
*/

#endif // INTEGRATION_EXAMPLE_CPP
