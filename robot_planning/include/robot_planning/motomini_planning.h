/**
 * @file motomini_planning.h
 * @brief Planning logic for MotoMini robot using Tesseract
 *
 * @author Bùi Quang Vinh
 * @date January 2026
 */

#ifndef ROBOT_PLANNING_MOTOMINI_PLANNING_H
#define ROBOT_PLANNING_MOTOMINI_PLANNING_H

#include <tesseract_common/macros.h>
#include <tesseract_kinematics/core/kinematic_group.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory> // Required for std::shared_ptr
#include <Eigen/Geometry>
#include <thread>
#include <atomic>
#include <functional>
#include <shared_mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

// Include the base class
#include <robot_planning/example.h>
// Forward declare the class so we don't need the header file here
namespace tesseract_common
{
    struct JointTrajectory;
}

namespace Vinhtesseract_examples
{

    class MotoMiniPlanning : public Example
    {
    public:
        using CommandCallback = std::function<void(const Eigen::VectorXd &)>;

        // All runtime-tunable planning hyperparameters (loaded from YAML, no rebuild needed)
        struct PlanningConfig
        {
            // --- OMPL ---
            double ompl_planning_time{10.0};
            int ompl_max_solutions{5};
            bool ompl_simplify{true};
            double ompl_longest_valid_segment{0.005};
            double ompl_rrt_range_1{0.05};
            double ompl_rrt_range_2{0.10};

            // --- Runtime OMPL toggle (overrides launch-time use_ompl flag) ---
            bool use_ompl_runtime{false};

            // --- Motion instruction type ---
            bool use_linear{false}; // false = FREESPACE, true = LINEAR

            // --- TrajOptIfopt: Cartesian constraint coefficients [x, y, z, rx, ry, rz] ---
            double ifopt_cart_coeff_x{100.0};  // weight for X translation
            double ifopt_cart_coeff_y{100.0};  // weight for Y translation
            double ifopt_cart_coeff_z{100.0};  // weight for Z translation
            double ifopt_cart_coeff_rx{0.0};   // weight for roll  (0 = free)
            double ifopt_cart_coeff_ry{0.0};   // weight for pitch (0 = free)
            double ifopt_cart_coeff_rz{0.0};   // weight for yaw   (0 = free)

            // --- TrajOptIfopt: Collision evaluator ---
            // 0 = DISCRETE, 1 = CONTINUOUS, 2 = LVS_CONTINUOUS
            int    ifopt_coll_eval_type{2};
            double ifopt_coll_lvs_length{0.005}; // longest_valid_segment_length (m)

            // --- TrajOptIfopt: Joint cost (regularizer) ---
            double ifopt_joint_cost_coeff{5.0};

            // --- TrajOptIfopt: Collision avoidance (soft cost) ---
            double ifopt_coll_cost_margin{0.02};   // minimum clearance (m)
            double ifopt_coll_cost_coeff{500.0};   // penalty weight
            double ifopt_coll_margin_buffer{0.02}; // LVS swept-volume buffer (m)

            // --- TrajOptIfopt: Trajectory smoothing ---
            double ifopt_smooth_vel{0.1};
            double ifopt_smooth_acc{1.0};
            double ifopt_smooth_jerk{1.0};

            // --- TrajOptIfopt: SQP solver ---
            int ifopt_max_iter{300};
            double ifopt_min_approx_improve{1e-6};
            double ifopt_min_trust_box_size{1e-5};
            double ifopt_initial_trust_box_size{0.5};
        };

        void configurePlanningParams(const PlanningConfig &cfg)
        {
            planning_cfg_ = cfg;
            use_ompl_ = cfg.use_ompl_runtime; // propagate runtime toggle
        }
        PlanningConfig getPlanningConfig() const { return planning_cfg_; }

        MotoMiniPlanning(std::shared_ptr<tesseract_environment::Environment> env,
                         std::shared_ptr<tesseract_visualization::Visualization> plotter = nullptr,
                         std::string manipulator_group = "manipulator",
                         std::string base_link = "world",
                         std::string ee_link = "tool0",
                         bool debug = false,
                         bool ifopt = false,
                         bool use_ompl = false,
                         bool online_mode = false);

        ~MotoMiniPlanning() override = default;
        MotoMiniPlanning(const MotoMiniPlanning &) = default;
        MotoMiniPlanning &operator=(const MotoMiniPlanning &) = default;
        MotoMiniPlanning(MotoMiniPlanning &&) = default;
        MotoMiniPlanning &operator=(MotoMiniPlanning &&) = default;

        bool run() override final;

        void setTargetPoses(const std::vector<Eigen::Isometry3d> &poses);

        // Return a pointer instead of the full object
        std::shared_ptr<tesseract_common::JointTrajectory> getTrajectory() const;
        void setCommandCallback(CommandCallback cb);
        void stopOnlinePlanner()
        {
            is_executing_online_ = false;
            if (online_thread_.joinable())
            {
                online_thread_.join();
            }
        }
        void updateEnvironmentState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_pos);
        using ToolpathCallback = std::function<void(const std::vector<Eigen::Vector3d> &)>;
        // Called for every completed chunk: (chunk_trajectory, joint_names, is_last_chunk)
        using ChunkReadyCallback = std::function<void(
            const tesseract_common::JointTrajectory &,
            const std::vector<std::string> &,
            bool)>;

        void setToolpathCallback(ToolpathCallback cb);
        void setChunkReadyCallback(ChunkReadyCallback cb) { chunk_ready_cb_ = std::move(cb); }

        // Lightweight tracking planner (collision check only, no optimization yet)
        bool runTrackingPlanner(const Eigen::Isometry3d &target_pose);

        // Configure tracking parameters from ROS node
        void configureTracking(bool use_trajopt, bool enable_collision,
                               int num_steps, int trajopt_max_iter,
                               double max_joint_step);

        // Configure offline chunked planning
        void configureChunking(int chunk_size, int parallel_chunks)
        {
            chunk_size_ = std::max(1, chunk_size);
            parallel_chunks_ = std::max(1, parallel_chunks);
        }

        // Get velocity limits for validation
        Eigen::MatrixX2d getTrackingVelocityLimits() const
        {
            return tracking_velocity_limits_;
        }

    private:
        std::string manipulator_group_;
        std::string base_link_;
        std::string ee_link_;
        bool online_mode_;
        bool debug_;
        bool ifopt_;
        bool use_ompl_;
        std::vector<Eigen::Isometry3d> target_poses_;

        std::shared_ptr<tesseract_common::JointTrajectory> last_trajectory_;
        std::thread online_thread_;
        std::atomic<bool> is_executing_online_{false};
        CommandCallback command_cb_;
        mutable std::shared_mutex env_mutex_;
        ToolpathCallback toolpath_cb_;
        ChunkReadyCallback chunk_ready_cb_;
        Eigen::VectorXd last_tracking_command_;
        bool has_last_tracking_command_{false};
        // --- Runtime-tunable planning config (loaded from YAML) ---
        PlanningConfig planning_cfg_;
        // --- Offline chunked planning ---
        int chunk_size_{20};     // waypoints per TrajOpt solve
        int parallel_chunks_{2}; // max chunks in flight simultaneously
        // --- Tracking TrajOpt configuration ---
        bool tracking_use_trajopt_{true};       // TrajOpt smoothing + optional collision
        bool tracking_enable_collision_{false}; // collision avoidance (slower)
        int tracking_num_steps_{5};             // interpolation steps
        int tracking_trajopt_max_iter_{5};      // max SQP iterations
        double tracking_max_joint_step_{0.15};  // rad per tick

        // --- Cached objects for fast repeated tracking calls ---
        tesseract_kinematics::KinematicGroup::ConstPtr tracking_manip_;
        Eigen::MatrixX2d tracking_joint_limits_;
        Eigen::MatrixX2d tracking_velocity_limits_;
        bool tracking_caches_valid_{false};
        void ensureTrackingCaches();
    };

} // namespace Vinhtesseract_examples

#endif // ROBOT_PLANNING_MOTOMINI_PLANNING_H