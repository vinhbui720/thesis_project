/**
 * @file sqp_debug_callback.hpp
 * @brief SQP Optimization Debug Callback Framework
 *
 * Enables real-time monitoring of TrajOpt/TrajOptIfopt optimization:
 *   - Collision constraint monitoring
 *   - Cartesian error convergence
 *   - Joint trajectory evolution
 *   - Cost function progression
 *
 * Integration with enhanced_debug_node allows visualization of:
 *   - Per-iteration collision status
 *   - Cartesian constraint satisfaction
 *   - Trajectory smoothness metrics
 *
 * @author Bùi Quang Vinh
 */

#pragma once

#include <memory>
#include <vector>
#include <functional>
#include <array>

#include <Eigen/Dense>
#include <tesseract_common/types.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_environment/environment.h>

namespace Vinhtesseract_examples
{

    /**
     * @brief Per-iteration optimization metrics for debugging
     */
    struct SQPIterationMetrics
    {
        int iteration{0};
        int phase{0}; // Trust region phase

        // Costs
        double total_cost{0.0};
        double collision_cost{0.0};
        double smoothness_cost{0.0};
        double cartesian_cost{0.0};
        double joint_cost{0.0};

        // Constraint satisfaction
        int collision_violations{0};
        double max_collision_violation{0.0};
        int cartesian_violations{0};
        double max_cartesian_violation{0.0};

        // Trajectory statistics
        double max_joint_velocity{0.0};
        double max_joint_acceleration{0.0};
        std::vector<double> joint_ranges; // Per-joint range in current iteration

        // Trust region info
        double trust_box_size{0.0};
        double accept_ratio{0.0};
    };

    /**
     * @brief Collision debug data from current iteration
     */
    struct SQPCollisionDebugData
    {
        struct CollisionPair
        {
            std::string link1, link2;
            double min_distance{0.0};
            double margin{0.0};
            int violations{0}; // How many timesteps violate this constraint
        };

        std::vector<CollisionPair> collision_pairs;
        int total_violations{0};
    };

    /**
     * @brief Cartesian constraint debug data
     */
    struct SQPCartesianDebugData
    {
        struct ConstraintStatus
        {
            std::string link_name;
            Eigen::Isometry3d target_pose{Eigen::Isometry3d::Identity()};
            Eigen::Isometry3d current_pose{Eigen::Isometry3d::Identity()};
            Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
            Eigen::Vector3d orientation_error{Eigen::Vector3d::Zero()};
            double position_error_norm{0.0};
            bool is_satisfied{false};
        };

        std::vector<ConstraintStatus> constraints;
    };

    /**
     * @brief Callback interface for SQP optimization debugging
     *
     * This is called at each optimization iteration and can be used to:
     *   - Visualize trajectory evolution
     *   - Monitor constraint satisfaction
     *   - Detect convergence issues
     *   - Stream data to enhanced_debug_node
     */
    class SQPDebugCallback
    {
    public:
        using Ptr = std::shared_ptr<SQPDebugCallback>;
        using OnIterationCallback = std::function<void(const SQPIterationMetrics &)>;
        using OnCollisionCallback = std::function<void(const SQPCollisionDebugData &)>;
        using OnCartesianCallback = std::function<void(const SQPCartesianDebugData &)>;

        SQPDebugCallback() = default;
        virtual ~SQPDebugCallback() = default;

        /**
         * @brief Register callback fired at each iteration completion
         */
        void setOnIterationCallback(OnIterationCallback cb)
        {
            on_iteration_cb_ = std::move(cb);
        }

        /**
         * @brief Register callback fired when collision violations detected
         */
        void setOnCollisionCallback(OnCollisionCallback cb)
        {
            on_collision_cb_ = std::move(cb);
        }

        /**
         * @brief Register callback fired when cartesian constraints updated
         */
        void setOnCartesianCallback(OnCartesianCallback cb)
        {
            on_cartesian_cb_ = std::move(cb);
        }

        /**
         * @brief Update metrics and fire appropriate callbacks
         */
        virtual void updateMetrics(const SQPIterationMetrics &metrics)
        {
            current_metrics_ = metrics;
            if (on_iteration_cb_)
                on_iteration_cb_(metrics);
        }

        /**
         * @brief Update collision debug data
         */
        virtual void updateCollisionData(const SQPCollisionDebugData &data)
        {
            current_collision_data_ = data;
            if (on_collision_cb_)
                on_collision_cb_(data);
        }

        /**
         * @brief Update cartesian constraint data
         */
        virtual void updateCartesianData(const SQPCartesianDebugData &data)
        {
            current_cartesian_data_ = data;
            if (on_cartesian_cb_)
                on_cartesian_cb_(data);
        }

        // Getters for external monitoring
        const SQPIterationMetrics &getCurrentMetrics() const { return current_metrics_; }
        const SQPCollisionDebugData &getCurrentCollisionData() const { return current_collision_data_; }
        const SQPCartesianDebugData &getCurrentCartesianData() const { return current_cartesian_data_; }

    protected:
        SQPIterationMetrics current_metrics_;
        SQPCollisionDebugData current_collision_data_;
        SQPCartesianDebugData current_cartesian_data_;

        OnIterationCallback on_iteration_cb_;
        OnCollisionCallback on_collision_cb_;
        OnCartesianCallback on_cartesian_cb_;
    };

    /**
     * @brief Helper to analyze trajectory metrics for SQP debugging
     */
    class SQPTrajectoryAnalyzer
    {
    public:
        /**
         * @brief Compute trajectory statistics
         */
        static void analyzeTrajectory(
            const tesseract_common::JointTrajectory &trajectory,
            const std::vector<std::string> &joint_names,
            SQPIterationMetrics &metrics)
        {
            if (trajectory.empty())
                return;

            metrics.joint_ranges.resize(joint_names.size(), 0.0);

            Eigen::VectorXd prev_pos = trajectory[0].position;
            Eigen::VectorXd prev_vel = Eigen::VectorXd::Zero(joint_names.size());

            for (size_t i = 1; i < trajectory.size(); ++i)
            {
                const auto &state = trajectory[i];
                double dt = state.time - trajectory[i - 1].time;
                if (dt < 1e-6)
                    dt = 0.001; // Default small dt

                Eigen::VectorXd velocity = (state.position - prev_pos) / dt;
                Eigen::VectorXd acceleration = (velocity - prev_vel) / dt;

                double max_vel = velocity.cwiseAbs().maxCoeff();
                double max_acc = acceleration.cwiseAbs().maxCoeff();

                metrics.max_joint_velocity = std::max(metrics.max_joint_velocity, max_vel);
                metrics.max_joint_acceleration = std::max(metrics.max_joint_acceleration, max_acc);

                // Track per-joint ranges
                for (size_t j = 0; j < joint_names.size(); ++j)
                {
                    double range = std::abs(state.position(j) - prev_pos(j));
                    metrics.joint_ranges[j] = std::max(metrics.joint_ranges[j], range);
                }

                prev_pos = state.position;
                prev_vel = velocity;
            }
        }

        /**
         * @brief Compute cartesian path metrics
         */
        static double computeToolpathLength(
            const tesseract_common::JointTrajectory &trajectory,
            const std::shared_ptr<tesseract_kinematics::KinematicGroup> &manip,
            const std::string &ee_link)
        {
            double length = 0.0;
            if (!manip || trajectory.size() < 2)
                return length;

            try
            {
                Eigen::Vector3d prev_pos = manip->calcFwdKin(trajectory[0].position)
                                               .at(ee_link)
                                               .translation();

                for (size_t i = 1; i < trajectory.size(); ++i)
                {
                    Eigen::Vector3d curr_pos = manip->calcFwdKin(trajectory[i].position)
                                                   .at(ee_link)
                                                   .translation();
                    length += (curr_pos - prev_pos).norm();
                    prev_pos = curr_pos;
                }
            }
            catch (const std::exception &)
            {
                // Ignore FK failures
            }

            return length;
        }

        /**
         * @brief Check for trajectory discontinuities (signs of solver issues)
         */
        static int detectDiscontinuities(
            const tesseract_common::JointTrajectory &trajectory,
            double velocity_threshold = 1.0)
        {
            int discontinuities = 0;

            for (size_t i = 1; i < trajectory.size(); ++i)
            {
                double dt = trajectory[i].time - trajectory[i - 1].time;
                if (dt < 1e-6)
                    continue;

                Eigen::VectorXd velocity = (trajectory[i].position - trajectory[i - 1].position) / dt;
                if (velocity.cwiseAbs().maxCoeff() > velocity_threshold)
                {
                    discontinuities++;
                }
            }

            return discontinuities;
        }
    };

} // namespace Vinhtesseract_examples
