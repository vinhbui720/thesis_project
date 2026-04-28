You are given my ROS2 C++ file `motomini_feedback_stream.cpp`.

Please modify the controller from the current Jacobian-based Cartesian PD controller into an **adaptive Cartesian impedance/admittance-style velocity controller**, while keeping the existing ROS node structure unchanged.

Important constraints:

1. **Do not change the existing publishers, subscribers, services, topic names, state machine logic, or publish logic.**
2. Keep using the existing output format:
   - publish joint positions
   - publish joint velocities
   - keep `publishTrajectory()`, `publishToTopic()`, `handleInit()`, and `handlePoseFollow()` logic mostly unchanged.

3. The main change should be inside `computeControlStep()`.
4. Keep the existing SR-inverse Jacobian mapping:
   [
   \dot q_{cmd} = J^#*{SR}(q)\dot x*{cmd}
   ]
5. Replace the old PD Cartesian velocity command:
   [
   \dot x = K_p e + K_d \Delta e
   ]
   with an adaptive impedance/admittance-style virtual dynamics controller:
   [
   \ddot{x}\_{ref}
   ==============

   M_d(e,w)^{-1}
   \left(
   K_d(e,w)e
   - D_e(e,w)\dot e
   - ## F\_{collision}

   D*d(e,w)\dot{x}*{ref}
   \right)
   ]

6. For now, implement `F_collision` only as a placeholder:

   ```cpp
   Eigen::Matrix<double, 6, 1> F_collision;
   F_collision.setZero();
   ```

   Do not implement obstacle detection yet.

7. Implement adaptive parameters:
   [
   s_t = 1 - e^{-\lambda t}
   ]
   [
   s_e = \tanh(\alpha |e|)
   ]
   [
   s_w = clamp(w/w_0, 0.2, 1.0)
   ]
   [
   K*d = s_w \left[K*{min} + s*t s_e (K*{max}-K*{min})\right]
   ]
   [
   M_d = M*{max} - s*t s_e (M*{max}-M\_{min})
   ]
   [
   D_d = 2\zeta\sqrt{M_dK_d}
   ]
8. Use separate translational and rotational adaptive values:
   - position stiffness/inertia/damping
   - orientation stiffness/inertia/damping

9. Compute velocity error using target velocity if available:
   - `latest_target_vel_` already exists
   - if target velocity is stale, use zero desired velocity
   - compute current Cartesian velocity as:
     [
     \dot x = J(q)\dot q
     ]
   - estimate `qdot` from `last_joint_state_->velocity` if available and valid; otherwise estimate from previous `q`.

10. Add persistent member variable:

```cpp
Eigen::Matrix<double, 6, 1> xdot_ref_;
```

This stores the virtual Cartesian velocity state. 11. Reset `xdot_ref_` to zero whenever entering IDLE, STOP, INIT start, or POSE_FOLLOW start. 12. Add safety limits:

- clamp `xdot_ref_` linear norm
- clamp `xdot_ref_` angular norm
- keep existing joint velocity and joint position safety checks unchanged.

13. Add ROS parameters for tuning:

```cpp
m_pos_min, m_pos_max
k_pos_min, k_pos_max
zeta_pos

m_ori_min, m_ori_max
k_ori_min, k_ori_max
zeta_ori

adaptive_lambda
adaptive_alpha_pos
adaptive_alpha_ori

max_cart_linear_vel
max_cart_angular_vel
```

14. Keep backward compatibility with existing params like `kp_max`, `ko_max`, `w0`, `k0`.
15. Add clear comments explaining that this is an admittance-inspired Cartesian velocity controller, not a full torque-level impedance controller.
16. Return the modified full C++ file, not just a diff.

Current controller behavior to preserve:

- state machine: IDLE, INIT, POSE_FOLLOW, STOP
- target pose integration from `/motomini/target_vel`
- feedback publishers
- SR inverse
- joint safety checks
- trajectory publishing

Goal:
Upgrade only the main Cartesian control law so the robot moves smoother at startup, adapts faster when error is larger, softens near singularities, and is ready for future collision-force insertion.
