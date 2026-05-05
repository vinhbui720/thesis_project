# OnlineCollisionDebugger: Code Updates for Stable Collision Escape

This patch keeps the tangential escape force, but makes the whole collision response safer and less oscillatory near contact.

The main goals are:

1. Keep tangential force so the robot can slide away instead of getting stuck.
2. Smooth the contact normal so the wrench direction does not flip frame-to-frame.
3. Smooth the tangential direction so escape motion does not jump suddenly.
4. Prevent tangential force from pushing into the obstacle.
5. Reduce sudden damping spikes.
6. Clamp the final wrench after normal + tangent are combined.
7. Reset filters cleanly when contact disappears.

---

## Problem in the current code

The current code does this near the closest contact:

```cpp
total_force = push_force_for_control + tangential_force;
```

That is conceptually okay, but three things make it unstable:

1. `closest_n_away` can change abruptly when the closest contact pair changes.
2. `last_tangent_dir_` only provides sign continuity, not real direction smoothing.
3. Normal damping and tangential escape can both create fast force changes near collision.

So the robot may receive a wrench that changes magnitude and direction too quickly.

---

## Update 1: Add new filter state variables

Add these member variables near the bottom of the class, next to the existing filter variables:

```cpp
Eigen::Vector3d filtered_n_away_{Eigen::Vector3d::Zero()};
bool have_filtered_n_away_{false};

Eigen::Vector3d filtered_tangent_dir_{Eigen::Vector3d::Zero()};
bool have_filtered_tangent_dir_{false};

Eigen::Vector3d filtered_tangent_force_{Eigen::Vector3d::Zero()};
bool have_filtered_tangent_force_{false};
```

Keep the existing variable:

```cpp
Eigen::Vector3d last_tangent_dir_{Eigen::Vector3d::Zero()};
```

It can still be used, but the new filtered tangent direction is more stable.

---

## Update 2: Add a smooth vector direction helper

Add this helper function in the private section:

```cpp
Eigen::Vector3d smoothDirection(const Eigen::Vector3d& raw_dir,
                                Eigen::Vector3d& filtered_dir,
                                bool& have_filtered_dir,
                                double alpha)
{
    const double eps = 1e-9;

    if (!raw_dir.allFinite() || raw_dir.norm() < eps)
        return Eigen::Vector3d::Zero();

    Eigen::Vector3d d = raw_dir.normalized();

    if (!have_filtered_dir ||
        !filtered_dir.allFinite() ||
        filtered_dir.norm() < eps)
    {
        filtered_dir = d;
        have_filtered_dir = true;
        return filtered_dir;
    }

    // Prevent sudden 180 degree sign flip.
    if (d.dot(filtered_dir) < 0.0)
        d = -d;

    filtered_dir = (1.0 - alpha) * filtered_dir + alpha * d;

    if (!filtered_dir.allFinite() || filtered_dir.norm() < eps)
    {
        filtered_dir = d;
    }
    else
    {
        filtered_dir.normalize();
    }

    return filtered_dir;
}
```

---

## Update 3: Add dedicated normal smoothing

Add this helper function:

```cpp
Eigen::Vector3d smoothContactNormal(const Eigen::Vector3d& raw_n)
{
    // Small alpha = more stable normal, less jitter.
    return smoothDirection(raw_n,
                           filtered_n_away_,
                           have_filtered_n_away_,
                           0.15);
}
```

This prevents the collision wrench from flipping direction when Tesseract changes the closest contact.

---

## Update 4: Add tangential force smoothing

Add this helper function:

```cpp
Eigen::Vector3d smoothTangentialForce(const Eigen::Vector3d& raw_force)
{
    const rclcpp::Time now = this->now();

    double dt = 0.0;
    if (have_force_filter_state_)
        dt = (now - t_last_force_filter_update_).seconds();

    if (!raw_force.allFinite())
        return Eigen::Vector3d::Zero();

    if (!have_filtered_tangent_force_ || dt <= 0.0 || dt > 1.0)
    {
        filtered_tangent_force_ = raw_force;
        have_filtered_tangent_force_ = true;
        return filtered_tangent_force_;
    }

    // Tangent should be slower than normal force because it changes direction more easily.
    const double alpha = lowPassAlpha(4.0, dt);
    filtered_tangent_force_ += alpha * (raw_force - filtered_tangent_force_);

    if (filtered_tangent_force_.norm() < 1e-6 && raw_force.norm() < 1e-6)
        filtered_tangent_force_.setZero();

    return filtered_tangent_force_;
}
```

Note: this function uses the same time base as the existing force filter. For a cleaner version, you can add a separate timestamp for tangent filtering, but this version minimizes changes.

---

## Update 5: Clamp normal damping separately

Replace the current `normalDampingMag()` with this version:

```cpp
double normalDampingMag(const Eigen::Vector3d& n_away) const
{
    const double age = (this->now() - t_last_feedback_vel_cb_).seconds();
    if (age > 0.2)
        return 0.0;

    if (!n_away.allFinite() || n_away.norm() < 1e-9)
        return 0.0;

    const Eigen::Vector3d n = n_away.normalized();
    const double v_n = latest_feedback_vel_linear_.dot(n);

    // Only damp motion going into the obstacle.
    if (v_n >= 0.0)
        return 0.0;

    const double raw = collision_normal_damping_ * (-v_n);

    // Damping must not dominate the repulsive force.
    const double damping_cap = 0.5 * collision_normal_force_max_;

    return std::clamp(raw, 0.0, damping_cap);
}
```

This keeps damping useful, but stops it from becoming a huge impulse.

---

## Update 6: Make normal force curve softer

Replace this inside `computeCollisionForce()`:

```cpp
const double s = std::clamp((d0 - distance) / std::max(1e-6, d0 - d_safe), 0.0, 1.0);
const double rep_mag = collision_normal_force_max_ * smoothstep(s);
```

with this:

```cpp
const double raw_s =
    std::clamp((d0 - distance) / std::max(1e-6, d0 - d_safe), 0.0, 1.0);

const double s = smoothstep(raw_s);

// Softer activation than plain smoothstep.
const double rep_mag = collision_normal_force_max_ * s * s;
```

This makes the force ramp less impulsive.

---

## Update 7: Improve tangential escape computation

Inside `computeTangentialForceAdvanced()`, keep the same general idea, but modify the final part so tangent direction is smoothed and guaranteed to stay tangential.

Find this part near the end of `computeTangentialForceAdvanced()`:

```cpp
const double total_limit = std::max(0.0, collision_force_max_total_);
const double push_norm = f_push.norm();
const double remaining = std::sqrt(std::max(0.0, total_limit * total_limit - push_norm * push_norm));
const double tangent_cap = std::min(tangentialForceLimit(), remaining);
const double active_limit = w * tangent_cap;

return smoothSaturate(f_raw, active_limit);
```

Replace it with this:

```cpp
// Remove any numerical normal component from the tangent force.
Eigen::Vector3d f_tan_raw = P * f_raw;

if (!f_tan_raw.allFinite() || f_tan_raw.norm() < 1e-9)
    return Eigen::Vector3d::Zero();

// Smooth the tangent direction to avoid side-to-side oscillation.
Eigen::Vector3d tangent_dir_smooth = smoothDirection(
    f_tan_raw,
    filtered_tangent_dir_,
    have_filtered_tangent_dir_,
    0.12);

if (!tangent_dir_smooth.allFinite() || tangent_dir_smooth.norm() < 1e-9)
    return Eigen::Vector3d::Zero();

last_tangent_dir_ = tangent_dir_smooth;

const double total_limit = std::max(0.0, collision_force_max_total_);
const double push_norm = f_push.norm();
const double remaining =
    std::sqrt(std::max(0.0, total_limit * total_limit - push_norm * push_norm));

const double tangent_cap = std::min(tangentialForceLimit(), remaining);
const double active_limit = w * tangent_cap;

// Use raw magnitude but smoothed direction.
const double raw_mag = std::min(f_tan_raw.norm(), active_limit);
Eigen::Vector3d f_tan = raw_mag * tangent_dir_smooth;

// Smooth tangent force separately because tangent direction changes more than normal direction.
f_tan = smoothTangentialForce(f_tan);

// Ensure the filtered force is still tangent after smoothing.
f_tan = P * f_tan;

return smoothSaturate(f_tan, active_limit);
```

This keeps tangential escape active, but makes it much harder for the tangent force to jitter.

---

## Update 8: Validate closest contact before publishing force

In `jointStateCallback()`, before computing final force, add validation:

```cpp
if (has_closest && !std::isfinite(closest_distance))
{
    has_closest = false;
}

if (has_closest &&
    (!closest_n_away.allFinite() || closest_n_away.norm() < 1e-9))
{
    has_closest = false;
}
```

This avoids publishing a bad wrench if collision data becomes invalid.

---

## Update 9: Replace the final force block in `jointStateCallback()`

Replace the existing final force section:

```cpp
Eigen::Vector3d tangential_force = Eigen::Vector3d::Zero();
Eigen::Vector3d push_force_for_control = Eigen::Vector3d::Zero();
if (has_closest)
{
    push_force_for_control = computeCollisionForce(closest_distance, closest_n_away);
    tangential_force = computeTangentialForceAdvanced(
        closest_distance,
        closest_n_away,
        wrench_ref,
        closest_obstacle_point,
        push_force_for_control);
    total_force = push_force_for_control + tangential_force;
}
else
{
    total_force.setZero();
    total_torque.setZero();
}

total_force = smoothPublishedForce(total_force);
if (has_closest)
    total_torque = (closest_contact_point - wrench_ref).cross(total_force);
```

with this:

```cpp
Eigen::Vector3d tangential_force = Eigen::Vector3d::Zero();
Eigen::Vector3d push_force_for_control = Eigen::Vector3d::Zero();

if (has_closest && !std::isfinite(closest_distance))
{
    has_closest = false;
}

if (has_closest &&
    (!closest_n_away.allFinite() || closest_n_away.norm() < 1e-9))
{
    has_closest = false;
}

if (has_closest)
{
    // Smooth the normal before using it for force and controller constraint.
    closest_n_away = smoothContactNormal(closest_n_away);

    push_force_for_control =
        computeCollisionForce(closest_distance, closest_n_away);

    // Keep tangential escape enabled, but make it distance-aware and filtered.
    tangential_force = computeTangentialForceAdvanced(
        closest_distance,
        closest_n_away,
        wrench_ref,
        closest_obstacle_point,
        push_force_for_control);

    // Remove any numerical component of tangent that points along the normal.
    const Eigen::Matrix3d P =
        Eigen::Matrix3d::Identity() - closest_n_away * closest_n_away.transpose();
    tangential_force = P * tangential_force;

    total_force = push_force_for_control + tangential_force;

    // Final safety clamp after normal + tangent are combined.
    total_force = smoothSaturate(total_force, collision_force_max_total_);
}
else
{
    total_force.setZero();
    total_torque.setZero();

    have_filtered_n_away_ = false;
    filtered_n_away_.setZero();

    have_filtered_tangent_dir_ = false;
    filtered_tangent_dir_.setZero();

    have_filtered_tangent_force_ = false;
    filtered_tangent_force_.setZero();

    last_tangent_dir_.setZero();
}

total_force = smoothPublishedForce(total_force);

if (has_closest)
    total_torque = (closest_contact_point - wrench_ref).cross(total_force);
```

This keeps tangent escape active at all distances, but controls it better.

---

## Update 10: Publish the smoothed normal to the controller

Right now, `publishCollisionConstraint()` is called after the force calculation:

```cpp
publishCollisionConstraint(has_closest, closest_distance, closest_n_away);
```

After Update 9, `closest_n_away` has already been smoothed before this call, so the controller receives the stable normal direction.

That is good. Keep this call after the smoothing block.

---

## Important note about tangent behavior

Do not fully disable tangent force if your robot needs to slide out of collision. Instead, make tangent force:

1. Purely tangential using projection `P = I - nn^T`.
2. Direction-smoothed using `smoothDirection()`.
3. Force-smoothed using `smoothTangentialForce()`.
4. Limited by the remaining total force budget.
5. Final-clamped after adding normal + tangent.

That gives the robot an escape direction without creating a sudden sideways launch.

---

## Minimal checklist

Apply these changes in this order:

1. Add new filter state variables.
2. Add `smoothDirection()`.
3. Add `smoothContactNormal()`.
4. Add `smoothTangentialForce()`.
5. Replace `normalDampingMag()`.
6. Soften the force curve in `computeCollisionForce()`.
7. Replace the final tangent return block in `computeTangentialForceAdvanced()`.
8. Replace the final force block in `jointStateCallback()`.

After this, tune parameters.

---

## Expected result

After these code updates:

- The robot should still slide away from obstacles.
- The robot should not get stuck as easily.
- The collision normal should not flip violently.
- The tangential escape direction should not jitter left-right.
- The published wrench should be bounded after normal and tangent are combined.
- Near collision behavior should feel much less like a sudden impulse.

