# Solving Real Robot Instability: Encoder Noise, Transmission Lag, and Safe Admittance Control

This document explains why the real MotoMini may oscillate or shake when the adaptive Cartesian admittance controller uses real encoder velocity feedback, and gives a practical implementation plan to stabilize the controller.

## 1. Problem Summary

On the real robot, the controller may become unstable even if it behaves well in simulation.

Typical symptoms:

- high-frequency buzzing or humming
- violent shaking near the target
- low-frequency swaying or bouncing
- sudden jerk when switching into `POSE_FOLLOW`
- overshoot caused by network or driver lag
- controller fighting delayed encoder velocity

The main causes are:

```text
1. encoder velocity noise
2. delayed joint-state feedback
3. using delayed velocity in damping
4. command integration wind-up
```

## 2. Why Encoder Velocity Can Destabilize the Controller

The real robot publishes `/joint_states`.

The controller can read:

```cpp
last_joint_state_->velocity
```

Then compute Cartesian velocity:

```cpp
xdot_actual = J(q) * qdot_actual;
```

This seems correct, but it can be dangerous in an admittance controller.

### 2.1 Encoder Velocity Noise

Velocity is often derived by differentiating encoder position:

```text
qdot = dq / dt
```

When `dt` is small, tiny encoder noise becomes large velocity spikes.

If this noisy velocity enters the damping term:

```text
F_damping = -D * xdot_actual
```

then the controller repeatedly brakes and releases, causing vibration.

### 2.2 Transmission Lag

On a real robot, `/joint_states` is delayed.

The controller sees the robot state from the past. If the delay is large enough, damping becomes harmful:

```text
delayed damping can behave like negative damping
```

Instead of removing energy, it injects energy and makes oscillation grow.

## 3. Key Rule

Do not use raw real encoder velocity directly inside the high-gain virtual damping loop.

Use real encoder velocity for:

```text
safety monitoring
state transition initialization
telemetry
debug feedback
```

But use the controller's internal virtual velocity for damping:

```text
xdot_ref_
```

## 4. Stable Controller Architecture

Use this structure:

```text
target pose / target velocity
        ->
pose error and velocity target
        ->
adaptive virtual dynamics
        ->
damping based on xdot_ref_, not raw xdot_actual
        ->
collision wrench / safety projection
        ->
Cartesian velocity command
        ->
SR inverse Jacobian
        ->
joint velocity command
        ->
stateless joint position overwrite
        ->
publish joint trajectory
```

## 5. Fix 1: Reference-State Damping

### Old unstable form

```text
F_damping = -D * (xdot_actual - xdot_des)
```

Problem:

```text
xdot_actual contains encoder noise + communication lag
```

### New stable form

Use the internal virtual velocity:

```text
F_damping = -D * (xdot_ref_ - xdot_des)
```

or equivalently:

```text
F_total =
    K * e
  + D_error * (xdot_des - xdot_ref_)
  + F_collision
  - D_virtual * xdot_ref_
```

This keeps the virtual mass-spring-damper loop stable in software memory.

## 6. Recommended Virtual Dynamics Equation

Pose error:

```text
e = [p_des - p_ee ; axisAngle(R_des * R_ee^T)]
```

Desired Cartesian velocity:

```text
xdot_des = target velocity input
```

Internal reference velocity:

```text
xdot_ref_ = controller internal Cartesian velocity state
```

Velocity error for control:

```text
edot_ref = xdot_des - xdot_ref_
```

Virtual force:

```text
F_virtual =
    K_adapt * e
  + D_error * edot_ref
  + F_collision
  - D_adapt * xdot_ref_
```

Virtual acceleration:

```text
xddot_ref = M_adapt^-1 * F_virtual
```

Integrate:

```text
xdot_ref_ = xdot_ref_ + xddot_ref * dt
```

Clamp:

```text
|xdot_ref_.linear|  <= max_cart_linear_vel
|xdot_ref_.angular| <= max_cart_angular_vel
```

Map to joints:

```text
qdot_cmd = J_sr_inverse * xdot_ref_
```

## 7. Fix 2: Adaptive Low-Pass Filter for Encoder Velocity

Even though raw encoder velocity should not drive damping, you still need clean measured velocity for:

- safety cut-offs
- initializing `xdot_ref_`
- publishing `/motomini/feedback_vel`
- debugging

Use an adaptive exponential moving average filter.

### Add Parameters

```cpp
this->declare_parameter<double>("velocity_filter_cutoff_hz", 15.0);
this->declare_parameter<bool>("real_robot", false);
```

Recommended values:

```yaml
real_robot: true
velocity_filter_cutoff_hz: 10.0
```

For simulation:

```yaml
real_robot: false
velocity_filter_cutoff_hz: 20.0
```

### Add Member Variables

```cpp
Eigen::VectorXd qdot_filtered_;
bool first_velocity_read_{true};
double velocity_filter_cutoff_hz_{15.0};
bool real_robot_{false};
```

Initialize after joint names are known:

```cpp
qdot_filtered_ =
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));
```

### Adaptive Filter Equation

The filter cutoff is independent of loop rate:

```text
RC = 1 / (2*pi*f_cutoff)
alpha = dt / (RC + dt)
```

Filtered velocity:

```text
qdot_filtered = alpha*qdot_raw + (1-alpha)*qdot_filtered
```

### C++ Helper

```cpp
Eigen::VectorXd filterJointVelocity(
    const Eigen::VectorXd &qdot_raw,
    double dt)
{
    if (qdot_filtered_.size() != qdot_raw.size())
    {
        qdot_filtered_ = qdot_raw;
        first_velocity_read_ = false;
        return qdot_filtered_;
    }

    const double current_dt =
        (dt > 1e-6) ? dt : (1.0 / std::max(1.0, rate_hz_));

    const double cutoff =
        std::clamp(velocity_filter_cutoff_hz_, 1.0, 0.45 * rate_hz_);

    const double rc = 1.0 / (2.0 * M_PI * cutoff);
    const double alpha = current_dt / (rc + current_dt);

    if (first_velocity_read_)
    {
        qdot_filtered_ = qdot_raw;
        first_velocity_read_ = false;
    }
    else
    {
        qdot_filtered_ =
            alpha * qdot_raw + (1.0 - alpha) * qdot_filtered_;
    }

    return qdot_filtered_;
}
```

## 8. Fix 3: Safe qdot Source

Use driver velocity if available and valid. Otherwise, estimate from position.

```cpp
bool getMeasuredJointVelocity(
    const Eigen::VectorXd &q,
    double dt,
    Eigen::VectorXd &qdot_out)
{
    qdot_out =
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));

    bool velocity_valid = false;

    if (last_joint_state_ &&
        last_joint_state_->velocity.size() >= joint_names_.size())
    {
        velocity_valid = true;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            auto it = std::find(
                last_joint_state_->name.begin(),
                last_joint_state_->name.end(),
                joint_names_[i]);

            if (it == last_joint_state_->name.end())
            {
                velocity_valid = false;
                break;
            }

            const size_t idx =
                std::distance(last_joint_state_->name.begin(), it);

            const double v = last_joint_state_->velocity[idx];

            if (!std::isfinite(v))
            {
                velocity_valid = false;
                break;
            }

            qdot_out[static_cast<Eigen::Index>(i)] = v;
        }
    }

    if (!velocity_valid)
    {
        if (have_q_prev_ && q_prev_.size() == q.size() && dt > 1e-6)
            qdot_out = (q - q_prev_) / dt;
        else
            qdot_out.setZero();
    }

    if (real_robot_)
        qdot_out = filterJointVelocity(qdot_out, dt);

    return true;
}
```

## 9. Fix 4: Stateless Overwrite Integration

Avoid this:

```cpp
tracked_positions_[i] += theta_d[i] * dt;
```

Why it is dangerous:

```text
It assumes the robot perfectly executed the previous command.
If the real robot lags, the software command runs away.
```

Use this instead:

```cpp
tracked_positions_[i] =
    q[static_cast<Eigen::Index>(i)]
    + theta_d[static_cast<Eigen::Index>(i)] * dt;
```

This anchors every new command to the real measured robot position.

### Correct Integration Block

In `handleInit()` and `handlePoseFollow()`, replace:

```cpp
tracked_positions_[i] += theta_d[i] * dt;
```

with:

```cpp
tracked_positions_[i] =
    q[static_cast<Eigen::Index>(i)]
    + theta_d[static_cast<Eigen::Index>(i)] * dt;

tracked_velocities_[i] =
    theta_d[static_cast<Eigen::Index>(i)];
```

This is one of the most important real-robot safety changes.

## 10. Fix 5: Initialize xdot_ref_ Smoothly

When entering active control, initialize:

```cpp
xdot_ref_ = measured_cartesian_velocity_filtered;
```

not zero.

This avoids a jerk if the robot is already moving.

### On IDLE to POSE_FOLLOW

```cpp
Eigen::VectorXd q;
if (currentJointVector(q))
{
    Eigen::MatrixXd J = manip_->calcJacobian(q, base_link_, ee_link_);

    Eigen::VectorXd qdot_meas;
    getMeasuredJointVelocity(q, 1.0 / rate_hz_, qdot_meas);

    Eigen::VectorXd xdot_meas = J * qdot_meas;

    xdot_ref_ = xdot_meas;
}
else
{
    xdot_ref_.setZero();
}
```

### On STOP or IDLE Reset

```cpp
xdot_ref_.setZero();
first_velocity_read_ = true;
```

## 11. Fix 6: Measured Velocity Safety Monitor

Do not use measured velocity for damping, but do use it for safety.

```cpp
bool checkCartesianVelocitySafety(
    const Eigen::Matrix<double, 6, 1> &xdot_actual) const
{
    const double lin = xdot_actual.head<3>().norm();
    const double ang = xdot_actual.tail<3>().norm();

    if (lin > measured_cart_linear_vel_limit_)
        return false;

    if (ang > measured_cart_angular_vel_limit_)
        return false;

    return true;
}
```

Recommended YAML:

```yaml
measured_cart_linear_vel_limit: 1.0
measured_cart_angular_vel_limit: 3.0
```

If exceeded:

```cpp
state_ = STATE_STOP;
```

## 12. Fix 7: Rate Limit xdot_ref_

Even if velocity is clamped, acceleration can be too sharp.

Add Cartesian acceleration limits:

```yaml
max_cart_linear_acc: 0.8
max_cart_angular_acc: 2.5
```

Equation:

```text
delta_v = xdot_ref_next - xdot_ref_prev
|delta_v_linear| <= max_cart_linear_acc * dt
|delta_v_angular| <= max_cart_angular_acc * dt
```

C++ helper:

```cpp
void limitCartesianAcceleration(
    Eigen::Matrix<double, 6, 1> &xdot_next,
    const Eigen::Matrix<double, 6, 1> &xdot_prev,
    double dt)
{
    Eigen::Vector3d dv_lin =
        xdot_next.head<3>() - xdot_prev.head<3>();

    const double max_dv_lin = max_cart_linear_acc_ * dt;
    if (dv_lin.norm() > max_dv_lin && dv_lin.norm() > 1e-9)
    {
        xdot_next.head<3>() =
            xdot_prev.head<3>() + dv_lin.normalized() * max_dv_lin;
    }

    Eigen::Vector3d dv_ang =
        xdot_next.tail<3>() - xdot_prev.tail<3>();

    const double max_dv_ang = max_cart_angular_acc_ * dt;
    if (dv_ang.norm() > max_dv_ang && dv_ang.norm() > 1e-9)
    {
        xdot_next.tail<3>() =
            xdot_prev.tail<3>() + dv_ang.normalized() * max_dv_ang;
    }
}
```

## 13. Recommended Real-Robot YAML

```yaml
motomini_feedback_stream:
  ros__parameters:
    real_robot: true
    velocity_filter_cutoff_hz: 10.0

    # Conservative virtual dynamics
    m_pos_min: 0.8
    m_pos_max: 6.0
    k_pos_min: 3.0
    k_pos_max: 25.0
    zeta_pos: 1.5

    m_ori_min: 0.4
    m_ori_max: 3.0
    k_ori_min: 1.0
    k_ori_max: 10.0
    zeta_ori: 1.5

    adaptive_lambda: 0.6
    adaptive_alpha_pos: 20.0
    adaptive_alpha_ori: 4.0

    max_cart_linear_vel: 0.20
    max_cart_angular_vel: 0.80

    max_cart_linear_acc: 0.50
    max_cart_angular_acc: 1.50

    measured_cart_linear_vel_limit: 0.80
    measured_cart_angular_vel_limit: 2.50

    w0: 0.0005
    k0: 0.02
```

## 14. Tuning Guide

### Symptom: High-frequency buzzing or humming

Likely cause:

```text
encoder velocity noise or too much stiffness
```

Actions:

```yaml
velocity_filter_cutoff_hz: 8.0
k_pos_max: lower
k_ori_max: lower
zeta_pos: 1.5
zeta_ori: 1.5
```

### Symptom: Large slow oscillation / swaying

Likely cause:

```text
transmission lag and insufficient damping
```

Actions:

```yaml
zeta_pos: 1.8
zeta_ori: 1.8
m_pos_max: increase
m_ori_max: increase
adaptive_lambda: lower
```

### Symptom: Robot jerks on start

Likely cause:

```text
xdot_ref_ initialized to zero or command not anchored to measured q
```

Actions:

```text
initialize xdot_ref_ from filtered measured velocity
use tracked_positions_[i] = q[i] + theta_d[i] * dt
```

### Symptom: Robot lags behind target

Likely cause:

```text
controller too soft or velocity limit too low
```

Actions:

```yaml
k_pos_min: increase slowly
k_pos_max: increase slowly
max_cart_linear_vel: increase carefully
adaptive_lambda: increase carefully
```

### Symptom: Joint velocity safety trips

Likely cause:

```text
Cartesian command too aggressive near singularity or large error
```

Actions:

```yaml
max_cart_linear_vel: lower
max_cart_angular_vel: lower
k_pos_max: lower
k_ori_max: lower
w0: increase
k0: increase
```

## 15. Implementation Checklist

Use this checklist before real robot testing:

```text
[ ] real_robot parameter exists
[ ] velocity_filter_cutoff_hz parameter exists
[ ] qdot_filtered_ member exists
[ ] measured qdot is filtered on real robot
[ ] xdot_actual is used only for safety/telemetry/start initialization
[ ] virtual damping uses xdot_ref_, not raw xdot_actual
[ ] xdot_ref_ is initialized smoothly on mode entry
[ ] tracked_positions_ is overwritten from actual q every tick
[ ] Cartesian velocity clamp exists
[ ] Cartesian acceleration clamp exists
[ ] measured Cartesian velocity safety check exists
[ ] conservative real robot YAML is used first
```

## 16. Final Recommendation

For real MotoMini testing:

```text
Use measured encoder velocity for safety and initialization.
Do not use raw measured velocity for virtual damping.
Use xdot_ref_ for damping.
Filter measured qdot.
Anchor every joint command to actual q.
Start with low stiffness and high damping.
```

This gives stable, predictable real-robot behavior while keeping the controller responsive enough for later collision avoidance and close-work tasks.
