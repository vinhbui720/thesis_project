# MotoMini Cartesian Pose + Velocity Tracking Controller Fix

This note gives a corrected task-space control equation for the current `motomini_feedback_stream.cpp` controller.

The main fix is:

> Use **one damping term** on the Cartesian reference velocity error:
>
> \[
> D(\dot{x}_d - \dot{x}_{ref})
> \]
>
> Do **not** also subtract another `D * xdot_ref_`.

The current code effectively applies:

\[
K e + D(\dot{x}_d - \dot{x}_{ref}) - D\dot{x}_{ref}
=
K e + D\dot{x}_d - 2D\dot{x}_{ref}
\]

That can make the robot match the target velocity while leaving a steady pose offset.

---

## 1. Definitions

Let the desired end-effector pose be:

\[
T_d =
\begin{bmatrix}
R_d & p_d \\
0 & 1
\end{bmatrix}
\]

Let the measured end-effector pose be:

\[
T =
\begin{bmatrix}
R & p \\
0 & 1
\end{bmatrix}
\]

Position error:

\[
e_p = p_d - p
\]

Orientation error, expressed as an axis-angle vector:

\[
e_R = \mathrm{Log}(R_d R^T)
\]

Task-space pose error:

\[
e =
\begin{bmatrix}
e_p \\
e_R
\end{bmatrix}
\]

Desired Cartesian velocity:

\[
\dot{x}_d =
\begin{bmatrix}
v_d \\
\omega_d
\end{bmatrix}
\]

where:

- \(v_d\) is desired linear velocity in the base/world frame.
- \(\omega_d\) is desired angular velocity in the base/world frame.
- If `/motomini/target_vel.angular` is body-frame, convert it with:

\[
\omega_d^{base} = R_d \omega_d^{body}
\]

The controller internal reference velocity is:

\[
\dot{x}_{ref}
\]

The commanded joint velocity is:

\[
\dot{q}_{cmd}
\]

---

## 2. Recommended optimal/LQR-shaped task-space controller

Use the following second-order Cartesian reference dynamics:

\[
\ddot{x}_{ref}
=
M^{-1}
\left[
K e
+
D(\dot{x}_d - \dot{x}_{ref})
-
F_{coll}
\right]
\]

Then integrate:

\[
\dot{x}_{ref,k+1}
=
\dot{x}_{ref,k}
+
\ddot{x}_{ref,k}\Delta t
\]

Apply Cartesian velocity and acceleration limits:

\[
\dot{x}_{ref,k+1}
\leftarrow
\mathrm{sat}_{v,a}(\dot{x}_{ref,k+1})
\]

Then map Cartesian reference velocity to joint velocity:

\[
\dot{q}_{cmd}
=
J^{\#}_{SR}(q)\dot{x}_{ref}
\]

where the singularity-robust inverse is:

\[
J^{\#}_{SR}
=
J^T
\left(
JJ^T + \lambda_{SR} I
\right)^{-1}
\]

with:

\[
\lambda_{SR}
=
\begin{cases}
k_0 \left(1 - \frac{w}{w_0}\right)^2, & w < w_0 \\
0, & w \geq w_0
\end{cases}
\]

and manipulability:

\[
w = \sqrt{\det(JJ^T)}
\]

Finally command the next joint position as:

\[
q_{cmd,k+1}
=
q_{meas,k}
+
\dot{q}_{cmd,k}\Delta t
\]

This matches your existing architecture: Cartesian reference velocity integrator + SR inverse Jacobian + joint trajectory streaming.

---

## 3. Why this fixes the residual pose error

The desired behavior is:

\[
\dot{x}_{ref} \rightarrow \dot{x}_d
\quad \text{and} \quad
e \rightarrow 0
\]

The corrected equation is:

\[
\ddot{x}_{ref}
=
M^{-1}
\left[
K e
+
D(\dot{x}_d - \dot{x}_{ref})
\right]
\]

At steady state, assuming no collision term and no saturation:

\[
\ddot{x}_{ref} = 0
\]

so:

\[
K e
+
D(\dot{x}_d - \dot{x}_{ref})
=
0
\]

If the robot has caught the desired velocity:

\[
\dot{x}_{ref} = \dot{x}_d
\]

then:

\[
K e = 0
\]

therefore:

\[
e = 0
\]

So with the corrected equation, matching target velocity is not enough; the controller still drives pose error to zero.

The old equation was closer to:

\[
\ddot{x}_{ref}
=
M^{-1}
\left[
K e
+
D(\dot{x}_d - \dot{x}_{ref})
-
D\dot{x}_{ref}
\right]
\]

which becomes:

\[
\ddot{x}_{ref}
=
M^{-1}
\left[
K e
+
D\dot{x}_d
-
2D\dot{x}_{ref}
\right]
\]

That extra damping can make the controller overly velocity-dominant and leave pose error behind.

---

## 4. Translational and rotational split

Use separate gains for position and orientation.

Translational controller:

\[
\ddot{p}_{ref}
=
\frac{
K_p e_p
+
D_p(v_d - v_{ref})
-
F_{coll,p}
}{
M_p
}
\]

Rotational controller:

\[
\dot{\omega}_{ref}
=
\frac{
K_R e_R
+
D_R(\omega_d - \omega_{ref})
-
\tau_{coll}
}{
M_R
}
\]

Then:

\[
\dot{x}_{ref}
=
\begin{bmatrix}
v_{ref} \\
\omega_{ref}
\end{bmatrix}
\]

---

## 5. Gain choice

A clean way to choose gains is to pick a desired natural frequency \(\omega_n\) and damping ratio \(\zeta\).

For translation:

\[
K_p = M_p \omega_{n,p}^2
\]

\[
D_p = 2\zeta_p M_p \omega_{n,p}
\]

For orientation:

\[
K_R = M_R \omega_{n,R}^2
\]

\[
D_R = 2\zeta_R M_R \omega_{n,R}
\]

Recommended starting values at 50 Hz:

```text
zeta_pos = 1.0
zeta_ori = 1.0

M_pos = 1.0 to 2.0
omega_n_pos = 4.0 to 8.0 rad/s

K_pos = M_pos * omega_n_pos^2
D_pos = 2 * zeta_pos * M_pos * omega_n_pos

M_ori = 0.5 to 1.0
omega_n_ori = 3.0 to 6.0 rad/s

K_ori = M_ori * omega_n_ori^2
D_ori = 2 * zeta_ori * M_ori * omega_n_ori
```

Example conservative gains:

```text
M_pos = 1.5
omega_n_pos = 5.0
K_pos = 37.5
D_pos = 15.0

M_ori = 0.8
omega_n_ori = 4.0
K_ori = 12.8
D_ori = 6.4
```

If you keep adaptive gains, make sure the **minimum stiffness is not too weak**. Your current small-error behavior can become soft because `k_pos_min` and `m_pos_max` dominate near zero error.

A better minimum range is often:

```text
k_pos_min = 20.0 to 40.0
k_pos_max = 60.0 to 120.0

m_pos_min = 0.8 to 1.5
m_pos_max = 1.5 to 3.0
```

---

## 6. Code change inside `computeControlStep()`

### Replace this block

```cpp
const Eigen::Matrix<double, 6, 1> edot_ref = xdot_des - xdot_ref_;

Eigen::Vector3d F_goal_pos = k_pos_var * e_p_ + d_pos_var * edot_ref.head<3>();

// ...

xddot_ref.head<3>() =
    (F_goal_pos
     - F_collision.head<3>()
     - d_pos_var * xdot_ref_.head<3>())
    / std::max(1e-9, m_pos_var);

xddot_ref.tail<3>() =
    (k_ori_var * e_o_
     + d_ori_var * edot_ref.tail<3>()
     - F_collision.tail<3>()
     - d_ori_var * xdot_ref_.tail<3>())
    / std::max(1e-9, m_ori_var);
```

### With this

```cpp
const Eigen::Matrix<double, 6, 1> v_err = xdot_des - xdot_ref_;

Eigen::Vector3d F_goal_pos =
    k_pos_var * e_p_
    + d_pos_var * v_err.head<3>();

Eigen::Vector3d F_goal_ori =
    k_ori_var * e_o_
    + d_ori_var * v_err.tail<3>();

if (collision_goal_suppression_ && gamma > 0.0)
{
    const double goal_into = F_goal_pos.dot(n_away);

    // If n_away points away from the obstacle, negative dot means
    // the goal force is pushing into the obstacle.
    if (goal_into < 0.0)
        F_goal_pos -= gamma * goal_into * n_away;
}

Eigen::Matrix<double, 6, 1> xddot_ref;
xddot_ref.head<3>() =
    (F_goal_pos - F_collision.head<3>())
    / std::max(1e-9, m_pos_var);

xddot_ref.tail<3>() =
    (F_goal_ori - F_collision.tail<3>())
    / std::max(1e-9, m_ori_var);
```

Important: keep the existing collision sign convention consistent. If `F_collision` is stored as the controller-side force that should be subtracted, the above is correct. If you later change the callback to store a repulsive push-away force directly, then the equation should use `+ F_repulsive` instead of `- F_collision`.

---

## 7. Avoid double use of `target_vel`

You currently use `/motomini/target_vel` in two places:

1. You integrate it into `desired_pose_`.
2. You also use it as `xdot_des` feedforward in `computeControlStep()`.

That is only correct if this node is the **only** source of target pose generation.

Use one of these two modes.

---

### Mode A: External pose generator

Use this when another node publishes `/motomini/target_pose`.

In this mode:

\[
x_d = \text{received target pose}
\]

\[
\dot{x}_d = \text{received target velocity}
\]

Do **not** integrate `target_vel` into `desired_pose_` inside `handlePoseFollow()`.

Disable or guard this block:

```cpp
if (vel_active)
{
    des_pos.x() += latest_target_vel_(0) * dt_safe;
    des_pos.y() += latest_target_vel_(1) * dt_safe;
    des_pos.z() += latest_target_vel_(2) * dt_safe;

    const Eigen::Quaterniond delta =
        Eigen::AngleAxisd(latest_target_vel_(3) * dt_safe, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(latest_target_vel_(4) * dt_safe, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(latest_target_vel_(5) * dt_safe, Eigen::Vector3d::UnitZ());

    q_des = (q_des * delta).normalized();

    desired_pose_.pose.position.x = des_pos.x();
    desired_pose_.pose.position.y = des_pos.y();
    desired_pose_.pose.position.z = des_pos.z();
    desired_pose_.pose.orientation.w = q_des.w();
    desired_pose_.pose.orientation.x = q_des.x();
    desired_pose_.pose.orientation.y = q_des.y();
    desired_pose_.pose.orientation.z = q_des.z();

    t_last_pose_cb_ = this->now();
}
```

For this mode, `target_vel` should be feedforward only.

---

### Mode B: Velocity-only joystick mode

Use this when there is no external pose trajectory and `/motomini/target_vel` is the command.

In this mode:

\[
x_d(t)
=
x_d(0)
+
\int \dot{x}_d dt
\]

Then it is okay to integrate `target_vel` into `desired_pose_`.

But when the user releases the joystick and sends zero velocity, freeze the desired pose at the current end-effector pose or at the last generated target pose. Do not let an old target velocity keep integrating.

---

## 8. Recommended implementation parameter

Add a parameter:

```cpp
this->declare_parameter<bool>("integrate_target_vel_to_pose", false);
```

Read it:

```cpp
integrate_target_vel_to_pose_ =
    this->get_parameter("integrate_target_vel_to_pose").as_bool();
```

Then change the integration condition in `handlePoseFollow()`:

```cpp
if (integrate_target_vel_to_pose_ && vel_active)
{
    // existing target_vel integration block
}
```

Add the member variable:

```cpp
bool integrate_target_vel_to_pose_{false};
```

Use:

```text
integrate_target_vel_to_pose = false
```

when `/motomini/target_pose` is externally streamed.

Use:

```text
integrate_target_vel_to_pose = true
```

only for velocity-only control.

---

## 9. Complete controller summary

The fixed controller should be:

\[
e_p = p_d - p
\]

\[
e_R = \mathrm{Log}(R_d R^T)
\]

\[
e =
\begin{bmatrix}
e_p \\
e_R
\end{bmatrix}
\]

\[
\dot{x}_d =
\begin{bmatrix}
v_d \\
R_d \omega_d^{body}
\end{bmatrix}
\]

\[
F_{goal}
=
K e
+
D(\dot{x}_d - \dot{x}_{ref})
\]

\[
\ddot{x}_{ref}
=
M^{-1}
\left(
F_{goal}
-
F_{coll}
\right)
\]

\[
\dot{x}_{ref}
\leftarrow
\mathrm{sat}_{v,a}
\left(
\dot{x}_{ref}
+
\ddot{x}_{ref}\Delta t
\right)
\]

\[
\dot{q}_{cmd}
=
J^{\#}_{SR}(q)\dot{x}_{ref}
\]

\[
q_{cmd}
=
q_{meas}
+
\dot{q}_{cmd}\Delta t
\]

This gives:

- velocity feedforward tracking through \(\dot{x}_d\),
- pose convergence through \(K e\),
- damping through \(D(\dot{x}_d - \dot{x}_{ref})\),
- singularity robustness through \(J^{\#}_{SR}\),
- safety through velocity, acceleration, joint-limit, and collision projection clamps.

---

## 10. Minimum patch checklist

Apply these first:

```text
[ ] Remove the extra -d_pos_var * xdot_ref_ term.
[ ] Remove the extra -d_ori_var * xdot_ref_ term.
[ ] Keep only D * (xdot_des - xdot_ref_) as the damping term.
[ ] Do not integrate target_vel into desired_pose_ if target_pose is already streamed externally.
[ ] Raise k_pos_min if small residual position error remains.
[ ] Check that max_cart_linear_acc is not too low for the expected correction speed.
```

The most important code fix is the damping replacement in Section 6.
