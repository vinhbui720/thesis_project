// ============================================================================
// ADD THIS BLOCK TO motomini_planning_node.h INSIDE THE CLASS
// ============================================================================
//
// In the existing #include section at the top of the .h file, add:
//
//   #include <atomic>
//   #include <thread>
//   #include <condition_variable>
//   #include <shared_mutex>
//
// Then inside the class body, in the `private:` section, add:

private:
    // ---------- State machine -------------------------------------------
    enum class ControllerMode : uint8_t
    {
        TRACKING            = 0,  // pure DLS-IK Cartesian servoing
        AVOIDANCE_REQUESTED = 1,  // collision detected, TrajOpt computing
        AVOIDANCE_EXEC      = 2,  // following pre-computed collision-free path
    };
    std::atomic<ControllerMode> mode_{ControllerMode::TRACKING};

    // ---------- Avoidance worker (background thread) --------------------
    std::thread                  avoidance_worker_;
    std::atomic<bool>            avoidance_running_{false};

    // request side
    std::mutex                   avoidance_request_mutex_;
    std::condition_variable      avoidance_request_cv_;
    std::atomic<bool>            avoidance_request_pending_{false};
    Eigen::VectorXd              avoidance_request_anchor_;

    // result side
    mutable std::shared_mutex    avoidance_result_mutex_;
    std::vector<Eigen::VectorXd> avoidance_traj_;
    std::size_t                  avoidance_idx_{0};
    rclcpp::Time                 avoidance_traj_stamp_;
    std::atomic<bool>            avoidance_traj_valid_{false};

    // ---------- Tunables -----------------------------------------------
    double collision_safety_margin_{0.02};   // [m] inflated link margin
    int    avoidance_horizon_{10};            // longer than tracking
    double avoidance_traj_max_age_{1.0};      // [s] before forcing re-plan

public:
    // Lifecycle — call these from constructor / destructor.
    void startAvoidanceWorker();
    void stopAvoidanceWorker();

private:
    // Internal helpers (declarations only)
    void   avoidanceWorkerLoop();
    bool   runAvoidancePlan(const Eigen::VectorXd &q_start,
                            std::vector<Eigen::VectorXd> &out_traj);
    int    checkCollisionInHorizon(const std::vector<Eigen::VectorXd> &q_traj);
    bool   checkCollisionAtState(const Eigen::VectorXd &q);
    void   requestAvoidance(const Eigen::VectorXd &anchor);

// ============================================================================
// END OF HEADER ADDITIONS
//
// In the constructor of MotoMiniPlanningNode (after task_factory_/task_executor_
// are initialised), add:
//
//   startAvoidanceWorker();
//
// In the destructor (or in a shutdown method called before destruction), add:
//
//   stopAvoidanceWorker();
//
// ============================================================================
