// ============================================================================
// HEADER CLEANUP — motomini_planning_node.h
// ============================================================================

// ---------------------------------------------------------------------------
// 1) REMOVE these previously-added items (state machine + worker thread).
//    Search for these names in your .h and delete them entirely:
// ---------------------------------------------------------------------------
//
//   enum class ControllerMode { ... };
//   std::atomic<ControllerMode> mode_;
//
//   std::thread                  avoidance_worker_;
//   std::atomic<bool>            avoidance_running_;
//   std::mutex                   avoidance_request_mutex_;
//   std::condition_variable      avoidance_request_cv_;
//   std::atomic<bool>            avoidance_request_pending_;
//   Eigen::VectorXd              avoidance_request_anchor_;
//
//   mutable std::shared_mutex    avoidance_result_mutex_;
//   std::vector<Eigen::VectorXd> avoidance_traj_;
//   std::size_t                  avoidance_idx_;
//   rclcpp::Time                 avoidance_traj_stamp_;
//   std::atomic<bool>            avoidance_traj_valid_;
//
//   double collision_safety_margin_;
//   int    avoidance_horizon_;
//   double avoidance_traj_max_age_;
//
//   void  startAvoidanceWorker();
//   void  stopAvoidanceWorker();
//   void  avoidanceWorkerLoop();
//   bool  runAvoidancePlan(...);
//   int   checkCollisionInHorizon(...);
//   bool  checkCollisionAtState(...);
//   void  requestAvoidance(...);
//
// ---------------------------------------------------------------------------
// 2) REMOVE these calls from your constructor and destructor:
// ---------------------------------------------------------------------------
//
//   startAvoidanceWorker();   // <-- DELETE from constructor
//   stopAvoidanceWorker();    // <-- DELETE from destructor
//
// ---------------------------------------------------------------------------
// 3) ADD this block in the `private:` section of MotoMiniPlanningNode:
// ---------------------------------------------------------------------------

private:
    // -------- Reactive obstacle avoidance (potential field, FIRAS-style) ---
    //
    //   When any robot link comes within `rep_distance_threshold_` of an
    //   obstacle, a Cartesian repulsive velocity is generated at the contact
    //   point and projected back into joint space via J^T. Summed with the
    //   tracking DLS step, this produces "string-like" deflection: the
    //   robot continues to chase the target but is shoved sideways around
    //   the obstacle, returning to perfect tracking once the obstacle is
    //   no longer in the way.
    //
    //   Tune these:
    //     rep_distance_threshold_  larger = robot dodges further out
    //     rep_gain_                larger = harder push (may oscillate)
    //     rep_velocity_max_        cap when penetrating (numerical stability)
    //     rep_joint_velocity_cap_  per-tick safety lid on repulsion's dq
    //
    double rep_distance_threshold_{0.10};   // [m]    activation distance
    double rep_gain_{0.5};                  // [m²/s] FIRAS coefficient
    double rep_velocity_max_{0.6};          // [m/s]  saturation when penetrating
    double rep_joint_velocity_cap_{1.0};    // [rad/s] per-joint repulsion cap

    /// Returns dq from repulsive forces only. Returns zeros if nothing is
    /// inside `rep_distance_threshold_`. Cheap to call every tick.
    Eigen::VectorXd computeRepulsiveJointVelocity(const Eigen::VectorXd& q);

// ---------------------------------------------------------------------------
// END of header changes. The constructor/destructor stay clean of any
// avoidance setup — repulsion needs no initialization, no thread, no state.
// ---------------------------------------------------------------------------
