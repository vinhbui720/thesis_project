# Close-Work Collision Safety Design

This document describes where to implement close-distance collision behavior for the MotoMini controller when the robot must work near a task object such as a bell.

## Short Answer

Implement it in **both** places:

```text
collision / wrench calculator:
    detects contact distance, normal, object class, and safety zone
    publishes collision wrench + constraint data

main controller:
    consumes wrench + constraint data
    applies safety projection / velocity filtering before Jacobian inverse
```

The collision node should **measure and describe danger**.  
The controller should **enforce safe motion**.

Do not rely only on repulsive force. Near a close-work object, repulsive force can fight the goal controller and cause oscillation.

---

## Problem

Current behavior:

```text
goal controller wants to move toward target
collision force pushes away
```

So near an obstacle:

```text
F_total = F_goal + F_collision
```

If the goal is behind/inside the obstacle direction, the two forces fight each other. This can create:

- oscillation
- bouncing near the object
- deeper collision due to velocity integration
- stronger force after penetration
- unsafe crash behavior

For close-work tasks, the robot may need to stay very near the bell. Therefore, “push away whenever close” is not enough.

---

## Required Behavior

For a task object like the bell:

```text
far away:
    normal tracking

near but safe:
    slow down approach

very close:
    block motion into the object
    allow tangent motion
    allow motion away

too close / penetration:
    only allow away or tangent motion
```

This behavior is better described as a **velocity constraint** than a force.

---

## Safety Zones

Use four distances:

```yaml
collision_influence_distance: 0.10   # d0
collision_guard_distance:     0.03   # d_guard
collision_task_distance:      0.005  # d_task
collision_stop_distance:      0.001  # d_stop
```

Meaning:

| Zone | Condition | Behavior |
|---|---|---|
| Free | d > d0 | normal tracking |
| Influence | d_guard < d <= d0 | soft repulsive force + damping |
| Guard | d_task < d <= d_guard | remove velocity into object, allow tangent |
| Task clearance | d_stop < d <= d_task | strong no-deeper-approach rule |
| Stop / penetration | d <= d_stop | only allow away/tangent, optionally stop goal attraction into object |

---

## What the Collision/Wrench Calculator Should Do

The collision node should compute and publish:

1. Closest distance d
2. Push-away normal n_away
3. Contact point p_contact
4. Object/link pair
5. Collision zone
6. Soft repulsive wrench
7. Velocity constraint data

### Direction Convention

Use:

```text
n_away = unit vector pointing away from obstacle into safe direction
```

So if the robot velocity satisfies:

```text
xdot . n_away < 0
```

the robot is moving **toward** the obstacle.

If:

```text
xdot . n_away > 0
```

the robot is moving **away** from the obstacle.

---

## Wrench Calculation

For soft avoidance only:

```text
F_rep = k_rep * (1/d - 1/d0) * (1/d^2) * n_away
```

only when:

```text
d < d0
```

Close-range hold force:

```text
F_hold = k_hold * (d_safe - d) * n_away
```

only when:

```text
d < d_safe
```

Total per-contact force:

```text
F_i = F_rep_i + F_hold_i
```

Clamp:

```text
norm(F_i) <= F_max_contact
```

Total wrench:

```text
W_collision = sum_i [ F_i ; (p_i - p_ref) x F_i ]
```

For the first version, the controller can use only:

```text
F_collision = [Fx, Fy, Fz]^T
```

and ignore torque.

---

## Constraint Data to Publish

Repulsive wrench alone is not enough. The collision node should also publish the closest constraint:

```text
(d_min, n_away_min, p_contact, zone)
```

Recommended topic:

```text
/motomini/collision_constraint
```

Recommended message options:

### Simple option

Use `geometry_msgs/msg/Vector3Stamped` for the closest normal:

```text
/motomini/collision_normal
```

and `std_msgs/msg/Float64` for closest distance:

```text
/motomini/collision_distance
```

### Better option

Create a custom message:

```text
CollisionConstraint.msg
```

```text
std_msgs/Header header

float64 distance
geometry_msgs/Vector3 normal_away
geometry_msgs/Point contact_point

string link0
string link1
string zone

bool active
bool guard_active
bool stop_active
```

This is the cleanest option.

---

## What the Main Controller Should Do

The controller should use the wrench as a soft input:

```text
xddot_ref = M^-1 * (K*e + D*edot + F_collision - D_m*xdot_ref)
```

Then it should apply a hard safety filter to the resulting Cartesian velocity.

Raw velocity:

```text
xdot_raw = xdot_ref
```

Safety projection:

```text
xdot_safe = xdot_raw - min(0, xdot_raw . n_away) * n_away
```

This removes only the component moving into the obstacle.

Then:

```text
qdot_cmd = J_sr_inverse * xdot_safe
```

---

## Distance-Based Projection Strength

Do not always fully project. Blend based on distance.

Define:

```text
gamma(d) = 1 - clamp((d - d_task) / (d_guard - d_task), 0, 1)
```

So:

```text
d >= d_guard:
    gamma = 0

d <= d_task:
    gamma = 1
```

Filtered velocity:

```text
xdot_safe = xdot_raw - gamma(d) * min(0, xdot_raw . n_away) * n_away
```

This gives smooth activation.

---

## Goal Force Suppression

Also prevent the goal controller from pulling into the obstacle.

Goal force:

```text
F_goal = K*e + D*edot
```

If:

```text
F_goal . n_away < 0
```

then the goal is pulling into the obstacle.

Remove that component near the object:

```text
F_goal_safe = F_goal - gamma(d) * min(0, F_goal . n_away) * n_away
```

Then use:

```text
xddot_ref = M^-1 * (F_goal_safe + F_collision - D_m*xdot_ref)
```

This is very important for close-work tasks.

---

## Recommended Final Controller Stack

```text
target pose / target velocity
        ↓
adaptive impedance/admittance law
        ↓
goal force suppression near obstacle
        ↓
small collision wrench input
        ↓
virtual Cartesian velocity integration
        ↓
velocity projection safety filter
        ↓
SR inverse Jacobian
        ↓
joint velocity command
        ↓
joint position integration
        ↓
publish joint trajectory
```

---

## Controller Equations

Pose error:

```text
e = [ p_d - p ; axisAngle(R_d * R^T) ]
```

Velocity error:

```text
edot = xdot_d - xdot
```

Adaptive goal force:

```text
F_goal = K(e,w)*e + D(e,w)*edot
```

Collision-aware goal force:

```text
F_goal_safe_pos =
    F_goal_pos - gamma(d) * min(0, F_goal_pos . n_away) * n_away
```

Virtual dynamics:

```text
xddot_ref = M^-1 * (F_goal_safe + F_collision - D_m*xdot_ref)
```

Integrate:

```text
xdot_raw = xdot_ref + xddot_ref * dt
```

Velocity projection:

```text
xdot_safe_pos =
    xdot_raw_pos - gamma(d) * min(0, xdot_raw_pos . n_away) * n_away
```

Full command:

```text
xdot_safe = [ xdot_safe_pos ; xdot_raw_ori ]
```

Joint velocity:

```text
qdot_cmd = J_sr_inverse * xdot_safe
```

Joint position:

```text
q_cmd_next = q_cmd + qdot_cmd * dt
```

---

## C++ Pseudocode in Controller

```cpp
Eigen::Vector3d n = latest_collision_normal_;
double d = latest_collision_distance_;

double gamma = 0.0;
if (collision_constraint_active_)
{
    gamma = 1.0 - std::clamp(
        (d - collision_task_distance_) /
        std::max(1e-6, collision_guard_distance_ - collision_task_distance_),
        0.0,
        1.0);
}

// 1. Build goal force
Eigen::Matrix<double, 6, 1> F_goal;
F_goal.head<3>() = K_pos * e_pos + D_pos * e_vel_pos;
F_goal.tail<3>() = K_ori * e_ori + D_ori * e_vel_ori;

// 2. Remove goal component into obstacle
double goal_into = F_goal.head<3>().dot(n);
if (gamma > 0.0 && goal_into < 0.0)
{
    F_goal.head<3>() -= gamma * goal_into * n;
}

// 3. Add soft collision force
Eigen::Matrix<double, 6, 1> F_total =
    F_goal + F_collision - D_adapt * xdot_ref_;

// 4. Virtual dynamics integration
Eigen::Matrix<double, 6, 1> xddot_ref = M_inv * F_total;
xdot_ref_ += xddot_ref * dt;

// 5. Velocity projection: remove motion into obstacle
double v_into = xdot_ref_.head<3>().dot(n);
if (gamma > 0.0 && v_into < 0.0)
{
    xdot_ref_.head<3>() -= gamma * v_into * n;
}

// 6. Cartesian velocity clamp
clampLinearAngular(xdot_ref_);

// 7. Map through SR inverse
theta_d = J_sr_inv * xdot_ref_;
```

---

## What to Implement in Each Node

### Collision / Wrench Node

Implement:

- contact distance calculation
- push-away normal calculation
- per-contact repulsive force
- total collision wrench publishing
- closest distance publishing
- closest normal publishing
- collision zone classification
- RViz debug markers:
  - distance line
  - contact points
  - force arrow at contact
  - total wrench arrow at EE/tool reference point

Do not implement final velocity blocking here because this node does not know the final Cartesian command after impedance, target tracking, saturation, and controller state.

### Main Controller

Implement:

- subscribe to collision wrench
- subscribe to closest distance + normal / constraint msg
- use collision wrench as soft force input
- suppress goal force into obstacle
- project Cartesian velocity away from obstacle
- keep existing publisher/subscriber/state-machine logic unchanged

This is where the safety rule belongs because only the controller knows the final intended motion.

---

## Debug Markers to Add

### In Collision Node

1. Contact nearest-point spheres
2. Distance line
3. Normal arrow
4. Per-contact force arrow
5. Total wrench force arrow at `wrench_reference_link`
6. Text marker:

```text
d_min: 0.0042 m
zone: TASK
|F_collision|: 2.8
gamma: 0.92
normal: [0.1, -0.8, 0.5]
```

### In Controller

Add optional controller debug marker:

1. Raw velocity arrow at EE
2. Safe velocity arrow at EE
3. Removed velocity component arrow
4. Goal force arrow
5. Suppressed goal force arrow

This makes it easy to see whether the robot is trying to move into the object and whether the safety projection is active.

---

## YAML Parameters

### Collision Node

```yaml
online_collision_debugger:
  ros__parameters:
    collision_threshold: 0.10
    contact_point_radius: 0.008

    debug_markers_enabled: true
    publish_collision_wrench: true
    publish_contact_debug_text: true

    marker_frame: world
    wrench_frame: world
    wrench_reference_link: tool0

    collision_influence_distance: 0.10
    collision_guard_distance: 0.03
    collision_task_distance: 0.005
    collision_stop_distance: 0.001

    collision_k_rep: 0.00005
    collision_k_hold: 5.0
    collision_force_max_per_contact: 2.0
    collision_force_max_total: 5.0

    force_arrow_min_length: 0.01
    force_arrow_max_length: 0.25
    force_arrow_length_gain: 0.04
```

### Main Controller

```yaml
motomini_feedback_stream:
  ros__parameters:
    enable_collision_wrench: true
    collision_wrench_timeout: 0.2

    enable_collision_projection: true
    collision_guard_distance: 0.03
    collision_task_distance: 0.005
    collision_stop_distance: 0.001

    collision_goal_suppression: true
    collision_projection_max_gamma: 1.0

    collision_force_scale: 1.0
    collision_force_max: 5.0
```

---

## Why Both Nodes Are Needed

| Feature | Collision node | Controller |
|---|---:|---:|
| Detect distance/contact | yes | no |
| Compute contact normal | yes | no |
| Publish wrench/debug markers | yes | optional |
| Know final desired velocity | no | yes |
| Block velocity into obstacle | no | yes |
| Suppress goal force into obstacle | no | yes |
| Preserve task tangent motion | no | yes |

Therefore:

```text
collision node = perception/geometry/force description
controller = safety enforcement and motion command
```

---

## Recommended Implementation Order

1. Collision node publishes:
   - collision wrench
   - closest distance
   - closest normal
   - zone

2. Controller subscribes and logs the values.

3. Add velocity projection only:

```text
xdot_safe = xdot_raw - gamma * min(0, xdot_raw . n_away) * n_away
```

4. Add goal force suppression.

5. Tune soft wrench force.

6. Add debug arrows for raw velocity vs safe velocity.

---

## Tuning Advice for Working Near the Bell

For close work, use weak force and strong projection.

Recommended starting point:

```yaml
collision_influence_distance: 0.08
collision_guard_distance: 0.02
collision_task_distance: 0.003
collision_stop_distance: 0.001

collision_k_rep: 0.00002
collision_k_hold: 2.0
collision_force_max_total: 2.0

enable_collision_projection: true
collision_goal_suppression: true
```

Reason:

- force should not violently push away from the bell
- projection should prevent going deeper
- tangent movement should remain possible
- away movement should always be allowed
