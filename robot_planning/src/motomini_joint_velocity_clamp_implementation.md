# MotoMini Joint Velocity Clamp Implementation

## Goal

Keep the current controller and current tuning structure.

Do **not** add more velocity tuning parameters.

Only change this behavior:

```text
old behavior:
    if any joint velocity is over the software limit:
        enter STATE_STOP

new behavior:
    if any joint velocity is over the software limit:
        scale / clamp the commanded joint velocity to the limit
        keep following the target
```

This is needed because a small overshoot of the software velocity limit should not stop the whole controller. For example, if the controller requests `3.592 rad/s` and the limit is `3.573 rad/s`, the correct behavior is to clamp the command, not enter `STATE_STOP`.

---

## Why clamping is better than stopping

The controller already has warm-up and Cartesian acceleration limiting. So when the command becomes slightly too fast, that is not necessarily a failure.

The current failure mode is:

```text
theta_d slightly over limit
→ checkVelocityLimits() returns false
→ STATE_STOP
→ robot stops tracking completely
```

That makes target following fragile.

The desired behavior is:

```text
theta_d slightly over limit
→ clamp theta_d to the allowed joint velocity
→ publish the clamped command
→ continue tracking
```

This lets the robot move at the maximum allowed speed instead of stopping.

---

## Recommended clamp type

Use **uniform joint velocity scaling**, not independent per-joint clipping.

### Do not do this as the first choice

```cpp
theta_d[i] = std::clamp(theta_d[i], -lim[i], lim[i]);
```

This is simple, but it changes the direction of the joint velocity vector.

### Prefer this

```text
scale = min_i(limit_i / abs(theta_d_i))

theta_d = scale * theta_d
```

This keeps the same joint-space direction and guarantees every joint stays inside its limit.

---

## Existing limit source

Use the existing constants:

```cpp
JOINT_1_S_VEL_LIMIT_RADSEC
JOINT_2_L_VEL_LIMIT_RADSEC
JOINT_3_U_VEL_LIMIT_RADSEC
JOINT_4_R_VEL_LIMIT_RADSEC
JOINT_5_B_VEL_LIMIT_RADSEC
JOINT_6_T_VEL_LIMIT_RADSEC
SAFETY_VELOCITY_ALPHA
```

No new tuning parameter is required.

The active software limit remains:

```cpp
limit_i = joint_hardware_limit_i * SAFETY_VELOCITY_ALPHA;
```

So if `SAFETY_VELOCITY_ALPHA` is currently `0.65`, the clamp still respects that same `65%` safety margin.

---

## Step 1 — Replace `checkVelocityLimits()`

Replace the old function:

```cpp
bool checkVelocityLimits(const Eigen::VectorXd &theta_d) const
{
    static const double lim[NUMBER_OF_JOINT] = {
        JOINT_1_S_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
        JOINT_2_L_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
        JOINT_3_U_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
        JOINT_4_R_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
        JOINT_5_B_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
        JOINT_6_T_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
    };

    for (int i = 0; i < NUMBER_OF_JOINT; ++i)
    {
        if (std::abs(theta_d[i]) > lim[i])
        {
            RCLCPP_WARN(this->get_logger(),
                        "Joint %d velocity %.4f rad/s exceeds limit %.4f rad/s",
                        i, theta_d[i], lim[i]);
            return false;
        }
    }

    return true;
}
```

with this new function:

```cpp
double clampJointVelocityLimits(Eigen::VectorXd &theta_d) const
{
    static const double lim[NUMBER_OF_JOINT] = {
        JOINT_1_S_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
        JOINT_2_L_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
        JOINT_3_U_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
        JOINT_4_R_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
        JOINT_5_B_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
        JOINT_6_T_VEL_LIMIT_RADSEC * SAFETY_VELOCITY_ALPHA,
    };

    double scale = 1.0;

    const int n = std::min<int>(NUMBER_OF_JOINT, theta_d.size());

    for (int i = 0; i < n; ++i)
    {
        const double a = std::abs(theta_d[i]);

        if (a > lim[i] && a > 1e-12)
        {
            scale = std::min(scale, lim[i] / a);
        }
    }

    if (scale < 1.0)
    {
        theta_d *= scale;

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Joint velocity clamped with scale %.3f. Continuing tracking.",
            scale);
    }

    return scale;
}
```

This returns:

```text
scale = 1.0      no clamp needed
scale < 1.0      velocity was reduced to stay inside limits
```

---

## Step 2 — Clamp inside `computeControlStep()`

Find this part near the end of `computeControlStep()`:

```cpp
// --- SR-inverse Jacobian → joint velocity command ---
theta_d = calcSrInverse(J, w, w0_, k0_) * xdot_ref_;

return true;
```

Replace it with:

```cpp
// --- SR-inverse Jacobian → joint velocity command ---
theta_d = calcSrInverse(J, w, w0_, k0_) * xdot_ref_;

// Clamp joint velocity instead of stopping the controller.
const double joint_scale = clampJointVelocityLimits(theta_d);

// Anti-windup for the Cartesian velocity integrator.
// After clamping theta_d, update xdot_ref_ to the Cartesian velocity
// that the clamped joint command can actually produce.
if (joint_scale < 1.0)
{
    const Eigen::VectorXd xdot_limited = J * theta_d;

    if (xdot_limited.size() == 6 && xdot_limited.allFinite())
    {
        xdot_ref_ = xdot_limited;
        clampCartesianVelocity(xdot_ref_);
    }
}

return true;
```

The anti-windup part is important.

Without it:

```text
xdot_ref_ stays too high
→ theta_d gets clamped every tick
→ controller keeps asking for impossible speed
```

With it:

```text
theta_d is clamped
→ xdot_ref_ is reset to what the robot can actually do
→ controller stays stable at the limit
```

This does not add another tuning parameter.

---

## Step 3 — Remove the `STATE_STOP` velocity checks

After adding the clamp inside `computeControlStep()`, the old hard-stop checks are no longer needed.

### In `handlePoseFollow()`

Remove or comment out this block:

```cpp
// Velocity safety
if (!checkVelocityLimits(theta_d))
{
    RCLCPP_WARN(this->get_logger(),
                "[FOLLOW] Joint velocity exceeded limit → STATE_STOP");
    state_ = STATE_STOP;
    return;
}
```

Replace it with a comment:

```cpp
// Joint velocity is clamped inside computeControlStep().
// Do not enter STATE_STOP for normal command saturation.
```

---

### In `handleInit()`

Remove or comment out this block:

```cpp
// Velocity safety
if (!checkVelocityLimits(theta_d))
{
    RCLCPP_WARN(this->get_logger(),
                "[INIT] Joint velocity exceeded limit → STATE_STOP");
    state_ = STATE_STOP;
    return;
}
```

Replace it with:

```cpp
// Joint velocity is clamped inside computeControlStep().
// Do not enter STATE_STOP for normal command saturation.
```

---

## Step 4 — Keep position-limit safety unchanged

Do **not** remove the joint position limit check.

Keep this logic:

```cpp
if (!checkPositionLimits(tracked_positions_))
{
    RCLCPP_WARN(this->get_logger(), "[FOLLOW] Joint position limit → holding.");
    tracked_positions_ = prev_pos;
    std::fill(tracked_velocities_.begin(), tracked_velocities_.end(), 0.0);
}
```

Velocity saturation is normal during fast tracking.

Joint position limit violation is different and should still be treated as a hard safety condition.

---

## Step 5 — Optional small cleanup for published velocity

This is not required for the clamp, but it makes the published trajectory command more consistent.

Current code:

```cpp
tracked_velocities_[i] = (tracked_positions_[i] - prev_pos[i]) / dt_safe;
```

Better:

```cpp
tracked_velocities_[i] = theta_d[static_cast<Eigen::Index>(i)];
```

Full block:

```cpp
for (size_t i = 0; i < tracked_positions_.size(); ++i)
{
    tracked_positions_[i] =
        q[static_cast<Eigen::Index>(i)] +
        theta_d[static_cast<Eigen::Index>(i)] * dt_safe;

    tracked_velocities_[i] =
        theta_d[static_cast<Eigen::Index>(i)];
}
```

Reason:

```text
theta_d is the actual clamped joint velocity command.
```

The old calculation can be inconsistent when the measured joint position `q` lags behind the previous commanded position.

---

## Final behavior after implementation

The controller behavior becomes:

```text
1. Compute Cartesian reference velocity xdot_ref_
2. Convert to joint velocity theta_d using SR inverse Jacobian
3. If theta_d is inside limits:
       publish normally
4. If theta_d is over limits:
       scale theta_d down to the nearest valid speed
       update xdot_ref_ using J * theta_d
       publish the clamped command
       keep tracking
```

So the robot can run at the allowed speed limit without dropping into `STATE_STOP`.

---

## Minimal patch checklist

```text
[ ] Replace checkVelocityLimits() with clampJointVelocityLimits()
[ ] Call clampJointVelocityLimits(theta_d) inside computeControlStep()
[ ] Update xdot_ref_ = J * theta_d when clamping happens
[ ] Remove velocity-limit STATE_STOP blocks from handlePoseFollow()
[ ] Remove velocity-limit STATE_STOP blocks from handleInit()
[ ] Keep joint position limit safety unchanged
[ ] Optionally publish tracked_velocities_[i] = theta_d[i]
```

This keeps the controller simple and avoids adding more tuning knobs.
