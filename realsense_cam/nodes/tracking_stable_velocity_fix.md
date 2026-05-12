# Fix Tracking Start: Wait for Stable Estimated Velocity After Line Crossing

This note describes how to change the current ROS2 tracker so it does **not** start publishing tracking output immediately from the true detected object pose after the object crosses the start line.

## Problem

The current logic starts the tracking session when:

```python
tracking_started == True
gate_active == True
```

Then it quickly publishes `/object/tracking_active = true` and uses the live object estimate from measurements. In the real world, the object can become covered or occluded, so this can make the estimator depend too much on the visible/true object instead of a stable predicted trajectory.

## Required Behavior

After the object crosses the start line:

1. Do **not** publish tracking as active yet.
2. Keep collecting velocity estimates.
3. Wait until the estimated velocity is stable.
4. When velocity is stable:
   - freeze that stable velocity value,
   - freeze the travel direction, called `chiều`,
   - set tracking publish state to active,
   - publish `/object/tracking_active = true`,
   - estimate the full trajectory over time using the frozen stable velocity.
5. During occlusion, continue predicting from the stable velocity instead of re-optimizing from the true visible object.

---

## High-Level State Machine

Replace the current direct start behavior:

```text
WAIT -> TRACKING
```

with:

```text
WAIT
  -> WAIT_VEL_STABLE
  -> PUBLISHING_STABLE_TRAJ
  -> DONE
```

Meaning:

| State | Meaning |
|---|---|
| `idle` | No line crossing yet. |
| `wait_vel_stable` | Object crossed line, but tracking is not published yet. Collect velocity samples. |
| `tracking` | Stable velocity is locked. Publish active tracking and predicted trajectory. |
| `done` | Stop line or prediction limit reached. |

---

## Add These Config Values

Add these under `ros2_publisher` in your config:

```yaml
ros2_publisher:
  velocity_stability_window: 8
  velocity_stability_std_thresh_mps: 0.015
  velocity_stability_min_speed_mps: 0.02
  velocity_stability_timeout_s: 1.0
  use_locked_velocity_after_stable: true
```

Recommended starting values:

| Parameter | Purpose |
|---|---|
| `velocity_stability_window` | Number of velocity samples to check. |
| `velocity_stability_std_thresh_mps` | Maximum std deviation allowed for stable velocity. |
| `velocity_stability_min_speed_mps` | Ignore almost-zero velocity. |
| `velocity_stability_timeout_s` | Safety timeout so the robot does not wait forever. |
| `use_locked_velocity_after_stable` | Use frozen stable velocity for future prediction. |

---

## Code Changes

### 1. Add new members in `__init__`

Add this block near the existing estimator configuration:

```python
# Wait for stable velocity after start-line crossing.
self._vel_stability_window = int(ros_cfg.get("velocity_stability_window", 8))
self._vel_stability_std_thresh = float(
    ros_cfg.get("velocity_stability_std_thresh_mps", 0.015)
)
self._vel_stability_min_speed = float(
    ros_cfg.get("velocity_stability_min_speed_mps", 0.02)
)
self._vel_stability_timeout = float(
    ros_cfg.get("velocity_stability_timeout_s", 1.0)
)
self._use_locked_velocity_after_stable = bool(
    ros_cfg.get("use_locked_velocity_after_stable", True)
)

self._vel_samples = []
self._wait_stable_started_t = None
self._stable_velocity = None
self._stable_direction = None
self._stable_start_pos = None
self._stable_start_t = None
self._tracking_publish_active = False
```

---

### 2. Reset the new state

Update `_reset_estimator()`:

```python
def _reset_estimator(self):
    self._pos_est = None
    self._vel_est = np.zeros(3, dtype=np.float64)
    self._t_est = None
    self._cam_t_last = None
    self._predict_started_t = None
    self._predict_start_pos = None
    self._occlusion_started_t = None
    self._occlusion_start_pos = None

    self._vel_samples = []
    self._wait_stable_started_t = None
    self._stable_velocity = None
    self._stable_direction = None
    self._stable_start_pos = None
    self._stable_start_t = None
    self._tracking_publish_active = False
```

Update `_reset_track_session()` too:

```python
def _reset_track_session(self):
    self._reset_estimator()
    self._est_state = "idle"
    self._publish_phase = "wait"
    self._publish_source = "none"
    self._status_error = ""
    self._pending_start = False
    self._bootstrap_pose = None
    self._bootstrap_quat = _IDENTITY_QUAT.copy()
    self._publish_pos = None
    self._publish_quat = _IDENTITY_QUAT.copy()
    self._track_started_t = None
    self._last_target_pos = None
    self._publish_t_last = None
    self._occlusion_started_t = None
    self._occlusion_start_pos = None

    self._tracking_publish_active = False
```

---

### 3. Change `_start_tracking_session()` into a velocity-stability waiting state

Replace the current end of `_start_tracking_session()` with this behavior:

```python
def _start_tracking_session(self):
    bootstrap_pos, bootstrap_quat = self._try_lookup_bootstrap_pose()
    if bootstrap_pos is None:
        return False

    self._reset_estimator()
    self._est_state = "wait_vel_stable"
    self._publish_phase = "wait_vel_stable"
    self._publish_source = "waiting_for_stable_velocity"
    self._status_error = ""

    self._bootstrap_pose = bootstrap_pos
    self._bootstrap_quat = bootstrap_quat
    self._publish_pos = bootstrap_pos.copy()
    self._publish_quat = bootstrap_quat.copy()
    self._track_started_t = time.monotonic()
    self._wait_stable_started_t = self._track_started_t
    self._last_target_pos = bootstrap_pos.copy()
    self._publish_t_last = self._track_started_t
    self._tracking_publish_active = False

    self.get_logger().info(
        "Start-line crossed — waiting for stable velocity before publishing tracking."
    )
    return True
```

Important change:

```python
self._est_state = "wait_vel_stable"
```

not:

```python
self._est_state = "tracking"
```

---

### 4. Add velocity stability helper functions

Add these methods inside `ROS2Publisher`:

```python
def _record_velocity_sample(self, vel):
    vel = np.asarray(vel, dtype=np.float64).copy()

    if not np.all(np.isfinite(vel)):
        return

    speed = np.linalg.norm(vel[:2])
    if speed < self._vel_stability_min_speed:
        return

    self._vel_samples.append(vel)

    if len(self._vel_samples) > self._vel_stability_window:
        self._vel_samples.pop(0)


def _velocity_is_stable(self):
    if len(self._vel_samples) < self._vel_stability_window:
        return False, None, None

    samples = np.asarray(self._vel_samples, dtype=np.float64)
    mean_vel = np.mean(samples, axis=0)
    std_vel = np.std(samples[:, :2], axis=0)
    std_norm = np.linalg.norm(std_vel)

    speed = np.linalg.norm(mean_vel[:2])
    if speed < self._vel_stability_min_speed:
        return False, None, None

    if std_norm > self._vel_stability_std_thresh:
        return False, None, None

    direction = mean_vel.copy()
    direction_norm = np.linalg.norm(direction[:2])
    if direction_norm < 1e-9:
        return False, None, None

    direction[:2] = direction[:2] / direction_norm
    direction[2] = 0.0

    return True, mean_vel, direction


def _lock_stable_velocity(self, now):
    stable, mean_vel, direction = self._velocity_is_stable()

    timed_out = False
    if self._wait_stable_started_t is not None:
        timed_out = (now - self._wait_stable_started_t) >= self._vel_stability_timeout

    if not stable and not timed_out:
        return False

    if stable:
        self._stable_velocity = mean_vel.copy()
        self._stable_direction = direction.copy()
    else:
        # Fallback: timeout. Use the best current estimate.
        self._stable_velocity = self._vel_est.copy()
        speed = np.linalg.norm(self._stable_velocity[:2])
        if speed < self._vel_stability_min_speed:
            self._status_error = "stable_velocity_timeout_but_speed_too_low"
            return False

        self._stable_direction = self._stable_velocity.copy()
        self._stable_direction[:2] /= max(np.linalg.norm(self._stable_direction[:2]), 1e-9)
        self._stable_direction[2] = 0.0

    if self._pos_est is not None:
        self._stable_start_pos = self._pos_est.copy()
    elif self._publish_pos is not None:
        self._stable_start_pos = self._publish_pos.copy()
    elif self._bootstrap_pose is not None:
        self._stable_start_pos = self._bootstrap_pose.copy()
    else:
        self._status_error = "stable_velocity_locked_but_no_start_position"
        return False

    self._stable_start_t = now
    self._vel_est = self._stable_velocity.copy()
    self._t_est = now

    self._est_state = "tracking"
    self._publish_phase = "follow"
    self._publish_source = "stable_velocity_locked"
    self._tracking_publish_active = True

    self.get_logger().info(
        "Stable velocity locked: "
        f"vel=[{self._stable_velocity[0]:.4f}, "
        f"{self._stable_velocity[1]:.4f}, "
        f"{self._stable_velocity[2]:.4f}], "
        f"direction=[{self._stable_direction[0]:.4f}, "
        f"{self._stable_direction[1]:.4f}, "
        f"{self._stable_direction[2]:.4f}]"
    )

    return True
```

---

### 5. Add stable-trajectory prediction

Add this method:

```python
def _stable_trajectory_position(self, now):
    if self._stable_start_pos is None:
        return None
    if self._stable_velocity is None:
        return None
    if self._stable_start_t is None:
        return None

    dt = max(0.0, now - self._stable_start_t)
    return self._stable_start_pos + self._stable_velocity * dt
```

This is the key behavior change. After stable velocity is locked, the object trajectory is:

```python
position(t) = stable_start_pos + stable_velocity * elapsed_time
```

So the trajectory is no longer dependent on the visible true object pose every frame.

---

### 6. Update `_desired_future_target()`

Replace the start of `_desired_future_target()` with:

```python
def _desired_future_target(self, now):
    """Predict future target.

    After velocity is stable, use the locked stable velocity and direction.
    This avoids depending on true visible object pose during occlusion.
    """
    if (
        self._use_locked_velocity_after_stable
        and self._tracking_publish_active
        and self._stable_velocity is not None
        and self._stable_start_pos is not None
    ):
        base_pos = self._stable_trajectory_position(now)
        if base_pos is None:
            return None

        lookahead = self._lookahead_steps * self._lookahead_dt
        lookahead = min(lookahead, self._lookahead_max_time)
        return base_pos + self._stable_velocity * lookahead

    if self._pos_est is None:
        return None

    dt_from_est = 0.0 if self._t_est is None else max(0.0, now - self._t_est)
    lookahead = self._lookahead_steps * self._lookahead_dt
    lookahead = min(lookahead, self._lookahead_max_time)
    horizon = dt_from_est + lookahead
    return self._pos_est + self._vel_est * horizon
```

---

### 7. Do not publish active while waiting for stable velocity

In `process()`, after measurement extraction:

```python
meas_pos, meas_vel = self._extract_measurement(data)
now = time.monotonic()
```

add this block **before** the current `if self._est_state != "tracking":` block:

```python
if self._est_state == "wait_vel_stable":
    has_measurement = meas_pos is not None

    if has_measurement:
        self._fuse_measurement(meas_pos, meas_vel, now)
        self._record_velocity_sample(self._vel_est)

    locked = self._lock_stable_velocity(now)

    if not locked:
        # Still waiting. Publish debug transform only, but do not publish active tracking.
        if self._publish_pos is not None:
            self._publish_transform(
                stamp,
                self._publish_pos,
                self._publish_quat,
                publish_debug=False,
            )

        self._publish_active_flag(False)
        self._publish_status(stamp, data)

        self._log_throttled(
            "wait_vel_stable",
            "info",
            0.5,
            f"Waiting for stable velocity: "
            f"samples={len(self._vel_samples)}/{self._vel_stability_window} "
            f"vel=[{self._vel_est[0]:.4f}, {self._vel_est[1]:.4f}, {self._vel_est[2]:.4f}]"
        )
        return data

    # Stable velocity locked. Continue into normal tracking flow below.
```

Then change the old non-tracking check from:

```python
if self._est_state != "tracking":
```

to:

```python
if self._est_state not in ("tracking",):
```

This ensures that `wait_vel_stable` is handled before the idle branch.

---

### 8. In tracking mode, prefer locked velocity

Inside the tracking branch in `process()`, update the measurement section.

Current behavior:

```python
if has_measurement:
    self._fuse_measurement(meas_pos, meas_vel, now)
    self._predict_started_t = None
    self._predict_start_pos = None
    if self._publish_phase != "bootstrap":
        self._publish_phase = "follow"
```

Replace with:

```python
if has_measurement:
    if self._use_locked_velocity_after_stable and self._stable_velocity is not None:
        # Do not re-optimize velocity from the visible true object after stable lock.
        traj_pos = self._stable_trajectory_position(now)
        if traj_pos is not None:
            self._pos_est = traj_pos.copy()
            self._vel_est = self._stable_velocity.copy()
            self._t_est = now
            self._cam_t_last = now
    else:
        self._fuse_measurement(meas_pos, meas_vel, now)

    self._predict_started_t = None
    self._predict_start_pos = None

    if self._publish_phase != "bootstrap":
        self._publish_phase = "follow"
```

This prevents the estimator from continuously snapping back to the true detected object.

---

### 9. Publish `/object/tracking_active` only after stable velocity is locked

Change `_publish_target_pose()` final line.

Current:

```python
self._publish_active_flag(True)
```

Replace with:

```python
self._publish_active_flag(self._tracking_publish_active)
```

This is important because `/object/tracking_active` should mean:

```text
stable velocity is locked and full trajectory publishing has started
```

not merely:

```text
object crossed the line
```

---

### 10. Add stable velocity info to tracking status

Update `_publish_status()` JSON:

```python
stable_speed = 0.0
if self._stable_velocity is not None:
    stable_speed = float(np.linalg.norm(self._stable_velocity[:2]))

status = json.dumps({
    "tracking": tracking,
    "est_state": self._est_state,
    "publish_phase": self._publish_phase,
    "publish_source": self._publish_source,
    "tracking_publish_active": self._tracking_publish_active,
    "stable_velocity_locked": self._stable_velocity is not None,
    "stable_speed_mps": round(stable_speed, 4),
    "stable_direction": (
        None if self._stable_direction is None else [
            round(float(self._stable_direction[0]), 4),
            round(float(self._stable_direction[1]), 4),
            round(float(self._stable_direction[2]), 4),
        ]
    ),
    "velocity_samples": len(self._vel_samples),
    "icp": icp_state,
    "has_pose": has_pose,
    "icp_fitness": round(float(fitness), 4),
    "icp_rmse": round(float(rmse), 5),
    "cam_fresh": cam_ok,
    "cam_age_ms": round(dt_cam * 1000.0),
    "prediction_age_ms": round(pred_age_ms),
    "occlusion_age_ms": round(occlusion_age_ms),
    "error": self._status_error,
})
```

---

## Expected Runtime Behavior

### Before start line

```json
{
  "est_state": "idle",
  "tracking_publish_active": false
}
```

### After start line crossing

```json
{
  "est_state": "wait_vel_stable",
  "publish_phase": "wait_vel_stable",
  "tracking_publish_active": false,
  "velocity_samples": 3
}
```

### When velocity becomes stable

```json
{
  "est_state": "tracking",
  "publish_phase": "follow",
  "publish_source": "stable_velocity_locked",
  "tracking_publish_active": true,
  "stable_velocity_locked": true,
  "stable_speed_mps": 0.124,
  "stable_direction": [0.9981, 0.0612, 0.0]
}
```

### During occlusion

```json
{
  "est_state": "tracking",
  "publish_source": "predicted_occlusion",
  "tracking_publish_active": true,
  "stable_velocity_locked": true
}
```

---

## Important Notes

### Why this fixes the real-world occlusion problem

The old code can keep correcting the estimate from the measured pose:

```python
self._pos_est = (1.0 - alpha) * pos_pred + alpha * meas_pos
self._vel_est = self._vel_alpha * self._vel_est + (1.0 - self._vel_alpha) * meas_vel
```

That is useful when the object is always visible, but it is wrong when the robot or another object covers the target.

The new logic waits until velocity is stable, then locks:

```python
self._stable_velocity
self._stable_direction
self._stable_start_pos
self._stable_start_t
```

From that point, the full trajectory is predicted over time:

```python
pos = stable_start_pos + stable_velocity * dt
```

So the object can be covered and the robot still has a consistent trajectory to follow.

### What `chiều` means here

In the code, `chiều` is stored as:

```python
self._stable_direction
```

It is the normalized XY travel direction:

```python
direction = stable_velocity / ||stable_velocity_xy||
```

Example:

```python
stable_velocity = [0.12, 0.03, 0.0]
stable_direction = [0.9701, 0.2425, 0.0]
```

---

## Minimal Checklist

- [ ] Add velocity stability config.
- [ ] Add stable velocity state variables.
- [ ] Start with `wait_vel_stable`, not `tracking`.
- [ ] Collect velocity samples after line crossing.
- [ ] Lock stable velocity and direction.
- [ ] Publish `/object/tracking_active = true` only after velocity lock.
- [ ] Predict full trajectory using locked velocity.
- [ ] During occlusion, continue using locked velocity.
- [ ] Add status fields for debugging.

---

## Quick Test Plan

### Test 1: Object crosses line but velocity is noisy

Expected:

```text
/object/tracking_active = false
est_state = wait_vel_stable
```

### Test 2: Velocity becomes stable

Expected:

```text
/object/tracking_active = true
est_state = tracking
publish_source = stable_velocity_locked
```

### Test 3: Object becomes covered after stable lock

Expected:

```text
/object/tracking_active remains true
trajectory continues from stable velocity
```

### Test 4: No stable velocity before timeout

Expected:

```text
If current speed is usable, lock best current velocity.
If speed is too low, stay inactive and report:
stable_velocity_timeout_but_speed_too_low
```
