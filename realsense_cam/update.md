Below is the full update guide in chat.

# Update guide: `ROS2Publisher` smooth target + future tracking + object disappearance handling

## Goal

Your current pipeline is:

```python
pipeline = Pipeline([
    src,
    Preprocess(CONFIG),
    Background(CONFIG, src.width, src.height, initial_bg=initial_bg),
    Depth(CONFIG, intr),
    Fusion(),
    Detect(CONFIG),
    TrackerKalman(CONFIG, (src.fx, src.fy, src.cx, src.cy)),
    icp_node,
    PoseFusion(CONFIG),
    ros2_pub,
    Visualize(CONFIG, (src.fx, src.fy, src.cx, src.cy)),
])
```

So the tracking estimate already comes from `TrackerKalman` before `ROS2Publisher`. `TrackerKalman` gives:

```python
data["center_3d"] = self.center_3d_ema
data["velocity"] = (vx_m, vy_m)
data["center"] = (int(px), int(py))
```

That means `ROS2Publisher` should not create a complicated second tracker. It should only do three things:

```text
1. Receive position + velocity from TrackerKalman.
2. Predict a small future target using that velocity.
3. Publish a smooth target to the robot, even when the object disappears from the camera.
```

Your current `ROS2Publisher` already has a prediction branch when `meas_pos is None`, but the published target can still jump because `_update_publish_target()` directly copies `target_pos` in `"follow"` and `"predict"` phases. It also stops after fixed prediction limits, which is a problem because object disappearance is expected when the robot hovers above the object.

---

# Important rule

Do **not** change your orientation design.

Keep this behavior:

```python
self._publish_quat = self._bootstrap_quat.copy()
```

and keep publishing:

```python
self._publish_target_pose(stamp, publish_pos, self._vel_est, self._publish_quat)
```

or if you later want controller velocity to match the smoothed target:

```python
self._publish_target_pose(stamp, publish_pos, self._publish_vel, self._publish_quat)
```

But do not change the quaternion logic if your current orientation is intentional.

---

# Problem in current code

Your current `_update_publish_target()` does this:

```python
elif self._publish_phase == "predict":
    self._publish_pos = target_pos.copy()
    self._publish_source = "predicted"
else:
    self._publish_phase = "follow"
    self._publish_pos = target_pos.copy()
    self._publish_source = "measured"
```

That means after bootstrap, the target sent to the robot can jump directly to the newest tracking estimate.

The smoother bootstrap is only used here:

```python
if self._publish_phase == "bootstrap":
    ...
    self._publish_pos = self._publish_pos + blend * (target_pos - self._publish_pos)
```

So the fix is simple:

```text
Never publish raw target_pos directly.

Always move _publish_pos smoothly toward the desired target.
```

---

# Desired behavior

The final logic should be:

```text
Object visible:
    update estimate from measurement
    compute future target
    smoothly publish toward future target

Object hidden / occluded:
    keep predicting from last known position + velocity
    smoothly publish toward predicted future target

Occlusion short:
    continue active tracking

Occlusion too long:
    either hold last target or finish tracking, depending on config
```

This is important because in your real system, object disappearance is not always a failure. When the robot hovers above the object, the camera may lose the object because the robot blocks the view. So `meas_pos is None` should often mean:

```text
expected occlusion
```

not:

```text
tracking failed
```

---

# 1. Add new config values

Add these under your `ros2_publisher` config:

```yaml
ros2_publisher:
  # Existing values
  vel_smooth_alpha: 0.85
  pos_correct_alpha: 0.30
  cam_timeout_s: 0.25
  target_z_offset_m: 0.01
  bootstrap_gain: 5.0
  bootstrap_pos_tolerance_m: 0.01
  bootstrap_max_duration_s: 0.35

  # Smooth published target
  target_smooth_tau_s: 0.12
  target_max_step_m: 0.015

  # Future tracking from Kalman velocity
  lookahead_steps: 3
  lookahead_dt_s: 0.033
  lookahead_max_time_s: 0.20

  # Expected camera occlusion when robot hovers above object
  occlusion_expected_s: 1.20
  occlusion_hold_after_s: 1.80
  occlusion_max_predict_distance_m: 0.35
  hold_target_on_occlusion_timeout: true
```

Recommended starting values:

```yaml
target_smooth_tau_s: 0.10
target_max_step_m: 0.010
lookahead_steps: 3
lookahead_dt_s: 0.033
lookahead_max_time_s: 0.20
occlusion_expected_s: 1.20
occlusion_hold_after_s: 1.80
occlusion_max_predict_distance_m: 0.35
hold_target_on_occlusion_timeout: true
```

Meaning:

```text
target_smooth_tau_s:
    how soft the target motion is

target_max_step_m:
    maximum target movement per publisher update

lookahead_steps:
    how many tracker steps into the future to aim

occlusion_expected_s:
    how long camera disappearance is considered normal

occlusion_hold_after_s:
    after this time, stop predicting forward and hold last target

occlusion_max_predict_distance_m:
    safety limit for prediction distance during occlusion

hold_target_on_occlusion_timeout:
    true  -> hold the last smooth target
    false -> finish tracking
```

---

# 2. Add variables in `__init__`

Inside `ROS2Publisher.__init__`, after your existing prediction parameters:

```python
self._prediction_max_time = ros_cfg.get("prediction_max_time_s", 0.75)
self._prediction_max_distance = ros_cfg.get("prediction_max_distance_m", 0.25)
```

add:

```python
# Smooth published target.
self._target_smooth_tau = ros_cfg.get("target_smooth_tau_s", 0.12)
self._target_max_step = ros_cfg.get("target_max_step_m", 0.015)

# Future tracking using Kalman velocity.
self._lookahead_steps = ros_cfg.get("lookahead_steps", 3)
self._lookahead_dt = ros_cfg.get("lookahead_dt_s", 0.033)
self._lookahead_max_time = ros_cfg.get("lookahead_max_time_s", 0.20)

# Object disappearance / robot occlusion handling.
self._occlusion_expected_s = ros_cfg.get("occlusion_expected_s", 1.20)
self._occlusion_hold_after_s = ros_cfg.get("occlusion_hold_after_s", 1.80)
self._occlusion_max_predict_distance = ros_cfg.get(
    "occlusion_max_predict_distance_m", 0.35
)
self._hold_target_on_occlusion_timeout = ros_cfg.get(
    "hold_target_on_occlusion_timeout", True
)

self._occlusion_started_t = None
self._occlusion_start_pos = None
```

Do not change orientation variables.

Keep:

```python
self._publish_quat = _IDENTITY_QUAT.copy()
```

and:

```python
self._bootstrap_quat = _IDENTITY_QUAT.copy()
```

as your current design requires.

---

# 3. Reset occlusion state

In `_reset_estimator()`, add:

```python
self._occlusion_started_t = None
self._occlusion_start_pos = None
```

Full updated method:

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
```

In `_reset_track_session()`, also add:

```python
self._occlusion_started_t = None
self._occlusion_start_pos = None
```

Full updated method:

```python
def _reset_track_session(self):
    self._reset_estimator()
    self._est_state = "idle"
    self._publish_phase = "wait"
    self._publish_source = "none"
    self._status_error = ""
    self._bootstrap_pose = None
    self._bootstrap_quat = _IDENTITY_QUAT.copy()
    self._publish_pos = None
    self._publish_quat = _IDENTITY_QUAT.copy()
    self._track_started_t = None
    self._last_target_pos = None
    self._publish_t_last = None
    self._occlusion_started_t = None
    self._occlusion_start_pos = None
```

---

# 4. Add future target function

Add this method inside `ROS2Publisher`:

```python
def _desired_future_target(self, now):
    """
    Predict a small future target using the current Kalman-based estimate.

    TrackerKalman already estimates velocity, so ROS2Publisher only projects
    forward by a short lookahead time.
    """
    if self._pos_est is None:
        return None

    dt_from_est = 0.0 if self._t_est is None else max(0.0, now - self._t_est)

    lookahead = self._lookahead_steps * self._lookahead_dt
    lookahead = min(lookahead, self._lookahead_max_time)

    horizon = dt_from_est + lookahead

    return self._pos_est + self._vel_est * horizon
```

This gives:

```text
future_target = current_estimated_position + velocity * lookahead_time
```

Because your object moves on a belt and velocity is stable, this is enough. You do not need a heavy planner here.

---

# 5. Add smooth published target function

Add this method:

```python
def _smooth_publish_target(self, desired_pos, now):
    """
    Smoothly move the published robot target toward desired_pos.

    This prevents jumps when the tracking estimate changes suddenly.
    This only changes position, not orientation.
    """
    desired_pos = np.asarray(desired_pos, dtype=np.float64)

    if self._publish_pos is None:
        self._publish_pos = desired_pos.copy()
        self._publish_t_last = now
        return self._publish_pos

    dt = 0.0 if self._publish_t_last is None else max(1e-3, now - self._publish_t_last)

    alpha = 1.0 - math.exp(-dt / max(self._target_smooth_tau, 1e-3))
    next_pos = self._publish_pos + alpha * (desired_pos - self._publish_pos)

    step = next_pos - self._publish_pos
    step_norm = np.linalg.norm(step)

    if step_norm > self._target_max_step:
        next_pos = self._publish_pos + step * (self._target_max_step / step_norm)

    self._publish_pos = next_pos
    self._publish_t_last = now

    return self._publish_pos
```

This is intentionally simple.

It does not change orientation.

It does not create another velocity controller.

It only makes sure that the target sent to the robot does not jump.

---

# 6. Add occlusion helper

Add this method:

```python
def _update_occlusion_state(self, now, has_measurement):
    """
    Track whether the object is currently hidden from the camera.

    This is expected when the robot hovers over the object and blocks the view.
    """
    if has_measurement:
        self._occlusion_started_t = None
        self._occlusion_start_pos = None
        return 0.0

    if self._occlusion_started_t is None:
        self._occlusion_started_t = now
        if self._pos_est is not None:
            self._occlusion_start_pos = self._pos_est.copy()
        else:
            self._occlusion_start_pos = None

    return now - self._occlusion_started_t
```

---

# 7. Replace `_prediction_limits_reached()`

Your current function is:

```python
def _prediction_limits_reached(self, now):
    if self._predict_started_t is None or self._predict_start_pos is None:
        return False
    age = now - self._predict_started_t
    dist = np.linalg.norm(self._pos_est - self._predict_start_pos)
    return age > self._prediction_max_time or dist > self._prediction_max_distance
```

Replace it with this:

```python
def _prediction_limits_reached(self, now):
    """
    Prediction limit during object disappearance.

    Different from the old version:
    - Short disappearance is expected.
    - Robot-hover occlusion is expected.
    - We only stop or hold after the occlusion becomes too long or too far.
    """
    if self._predict_started_t is None or self._predict_start_pos is None:
        return False

    if self._pos_est is None:
        return False

    age = now - self._predict_started_t
    dist = np.linalg.norm(self._pos_est - self._predict_start_pos)

    # Allow expected robot occlusion longer than normal camera timeout.
    if age <= self._occlusion_expected_s and dist <= self._occlusion_max_predict_distance:
        return False

    # After hold time, prediction is no longer trusted.
    if age > self._occlusion_hold_after_s:
        return True

    # Distance safety limit.
    if dist > self._occlusion_max_predict_distance:
        return True

    return False
```

This makes prediction tolerant of temporary camera loss.

---

# 8. Replace `_update_publish_target()`

Replace your whole `_update_publish_target()` with this:

```python
def _update_publish_target(self, now):
    """
    Compute the desired future target and publish a smooth target.

    Important:
    - Never directly copy _pos_est to _publish_pos.
    - Always smooth the published target.
    - Orientation is not changed here.
    """
    desired_pos = self._desired_future_target(now)

    if desired_pos is None:
        if self._publish_pos is not None:
            self._publish_source = "hold_no_estimate"
        return self._publish_pos

    if self._publish_phase == "bootstrap":
        publish_pos = self._smooth_publish_target(desired_pos, now)

        elapsed = 0.0 if self._track_started_t is None else (now - self._track_started_t)
        err = np.linalg.norm(publish_pos - desired_pos)

        if err <= self._bootstrap_tol or elapsed >= self._bootstrap_max_duration:
            self._publish_phase = "follow"

        self._publish_source = "magnetic_bootstrap"

    elif self._publish_phase == "predict":
        publish_pos = self._smooth_publish_target(desired_pos, now)
        self._publish_source = "predicted_occlusion"

    else:
        self._publish_phase = "follow"
        publish_pos = self._smooth_publish_target(desired_pos, now)
        self._publish_source = "measured"

    self._last_target_pos = self._publish_pos.copy()
    return self._publish_pos
```

Key change:

```python
self._publish_pos = target_pos.copy()
```

is removed.

Now all phases use:

```python
self._smooth_publish_target(desired_pos, now)
```

---

# 9. Update the `process()` measurement / occlusion section

Find this part in `process()`:

```python
if meas_pos is not None:
    self._fuse_measurement(meas_pos, meas_vel, now)
    self._predict_started_t = None
    self._predict_start_pos = None
    if self._publish_phase != "bootstrap":
        self._publish_phase = "follow"
else:
    if not self._propagate_prediction(now):
        target = self._publish_pos if self._publish_pos is not None else self._bootstrap_pose
        if target is not None:
            self._publish_target_pose(stamp, target, self._vel_est, self._publish_quat)
        self._publish_status(stamp, data)
        return data
    self._publish_phase = "predict"
    if self._prediction_limits_reached(now):
        self._finish_tracking("prediction_limit")
        self._publish_active_flag(False)
        self._publish_status(stamp, data)
        return data
```

Replace it with:

```python
has_measurement = meas_pos is not None
occlusion_age = self._update_occlusion_state(now, has_measurement)

if has_measurement:
    self._fuse_measurement(meas_pos, meas_vel, now)

    self._predict_started_t = None
    self._predict_start_pos = None

    if self._publish_phase != "bootstrap":
        self._publish_phase = "follow"

else:
    # Object not visible.
    # This is expected when the robot moves over the object and blocks the camera.
    if not self._propagate_prediction(now):
        target = self._publish_pos if self._publish_pos is not None else self._bootstrap_pose
        if target is not None:
            self._publish_target_pose(stamp, target, self._vel_est, self._publish_quat)
        self._publish_status(stamp, data)
        return data

    self._publish_phase = "predict"

    if self._prediction_limits_reached(now):
        if self._hold_target_on_occlusion_timeout:
            # Do not kill tracking immediately.
            # Hold the last safe smooth target.
            self._publish_phase = "hold"
            self._publish_source = "occlusion_hold"

            target = self._publish_pos if self._publish_pos is not None else self._pos_est
            if target is not None:
                self._publish_target_pose(stamp, target, self._vel_est, self._publish_quat)

            self._publish_status(stamp, data)
            return data

        else:
            self._finish_tracking("occlusion_prediction_limit")
            self._publish_active_flag(False)
            self._publish_status(stamp, data)
            return data
```

This is the main part for your robot-hover case.

Now, when the robot hides the object:

```text
meas_pos == None
```

the publisher keeps predicting instead of stopping immediately.

---

# 10. Keep publish call mostly same

You currently have:

```python
publish_pos = self._update_publish_target(now)
if publish_pos is not None:
    self._publish_target_pose(stamp, publish_pos, self._vel_est, self._publish_quat)
else:
    self._publish_active_flag(False)
```

You can keep it as-is.

This means:

```text
pose target = smoothed future position
velocity = estimated object velocity
orientation = your designed orientation
```

That is okay.

Optional alternative:

If your robot controller uses `/motomini/target_vel` strongly, you may later want to publish a smoothed target velocity instead of raw object velocity. But for now, because your belt velocity is stable and you already use Kalman velocity, keeping `self._vel_est` is fine.

---

# 11. Update `_publish_status()`

In `_publish_status()`, add occlusion information.

Find:

```python
pred_age_ms = 0.0
if self._predict_started_t is not None:
    pred_age_ms = (now - self._predict_started_t) * 1000.0
```

After that, add:

```python
occlusion_age_ms = 0.0
if self._occlusion_started_t is not None:
    occlusion_age_ms = (now - self._occlusion_started_t) * 1000.0
```

Then update the JSON:

```python
status = json.dumps({
    "tracking": tracking,
    "est_state": self._est_state,
    "publish_phase": self._publish_phase,
    "publish_source": self._publish_source,
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

This helps debugging. You will be able to see:

```text
publish_phase: predict
publish_source: predicted_occlusion
occlusion_age_ms: ...
```

or:

```text
publish_phase: hold
publish_source: occlusion_hold
```

---

# 12. Full patch summary

## Add to `__init__`

```python
self._target_smooth_tau = ros_cfg.get("target_smooth_tau_s", 0.12)
self._target_max_step = ros_cfg.get("target_max_step_m", 0.015)

self._lookahead_steps = ros_cfg.get("lookahead_steps", 3)
self._lookahead_dt = ros_cfg.get("lookahead_dt_s", 0.033)
self._lookahead_max_time = ros_cfg.get("lookahead_max_time_s", 0.20)

self._occlusion_expected_s = ros_cfg.get("occlusion_expected_s", 1.20)
self._occlusion_hold_after_s = ros_cfg.get("occlusion_hold_after_s", 1.80)
self._occlusion_max_predict_distance = ros_cfg.get(
    "occlusion_max_predict_distance_m", 0.35
)
self._hold_target_on_occlusion_timeout = ros_cfg.get(
    "hold_target_on_occlusion_timeout", True
)

self._occlusion_started_t = None
self._occlusion_start_pos = None
```

## Add to `_reset_estimator()`

```python
self._occlusion_started_t = None
self._occlusion_start_pos = None
```

## Add these methods

```python
def _desired_future_target(self, now):
    if self._pos_est is None:
        return None

    dt_from_est = 0.0 if self._t_est is None else max(0.0, now - self._t_est)

    lookahead = self._lookahead_steps * self._lookahead_dt
    lookahead = min(lookahead, self._lookahead_max_time)

    horizon = dt_from_est + lookahead

    return self._pos_est + self._vel_est * horizon
```

```python
def _smooth_publish_target(self, desired_pos, now):
    desired_pos = np.asarray(desired_pos, dtype=np.float64)

    if self._publish_pos is None:
        self._publish_pos = desired_pos.copy()
        self._publish_t_last = now
        return self._publish_pos

    dt = 0.0 if self._publish_t_last is None else max(1e-3, now - self._publish_t_last)

    alpha = 1.0 - math.exp(-dt / max(self._target_smooth_tau, 1e-3))
    next_pos = self._publish_pos + alpha * (desired_pos - self._publish_pos)

    step = next_pos - self._publish_pos
    step_norm = np.linalg.norm(step)

    if step_norm > self._target_max_step:
        next_pos = self._publish_pos + step * (self._target_max_step / step_norm)

    self._publish_pos = next_pos
    self._publish_t_last = now

    return self._publish_pos
```

```python
def _update_occlusion_state(self, now, has_measurement):
    if has_measurement:
        self._occlusion_started_t = None
        self._occlusion_start_pos = None
        return 0.0

    if self._occlusion_started_t is None:
        self._occlusion_started_t = now
        if self._pos_est is not None:
            self._occlusion_start_pos = self._pos_est.copy()
        else:
            self._occlusion_start_pos = None

    return now - self._occlusion_started_t
```

## Replace `_prediction_limits_reached()`

```python
def _prediction_limits_reached(self, now):
    if self._predict_started_t is None or self._predict_start_pos is None:
        return False

    if self._pos_est is None:
        return False

    age = now - self._predict_started_t
    dist = np.linalg.norm(self._pos_est - self._predict_start_pos)

    if age <= self._occlusion_expected_s and dist <= self._occlusion_max_predict_distance:
        return False

    if age > self._occlusion_hold_after_s:
        return True

    if dist > self._occlusion_max_predict_distance:
        return True

    return False
```

## Replace `_update_publish_target()`

```python
def _update_publish_target(self, now):
    desired_pos = self._desired_future_target(now)

    if desired_pos is None:
        if self._publish_pos is not None:
            self._publish_source = "hold_no_estimate"
        return self._publish_pos

    if self._publish_phase == "bootstrap":
        publish_pos = self._smooth_publish_target(desired_pos, now)

        elapsed = 0.0 if self._track_started_t is None else (now - self._track_started_t)
        err = np.linalg.norm(publish_pos - desired_pos)

        if err <= self._bootstrap_tol or elapsed >= self._bootstrap_max_duration:
            self._publish_phase = "follow"

        self._publish_source = "magnetic_bootstrap"

    elif self._publish_phase == "predict":
        publish_pos = self._smooth_publish_target(desired_pos, now)
        self._publish_source = "predicted_occlusion"

    else:
        self._publish_phase = "follow"
        publish_pos = self._smooth_publish_target(desired_pos, now)
        self._publish_source = "measured"

    self._last_target_pos = self._publish_pos.copy()
    return self._publish_pos
```

## Replace measurement section in `process()`

```python
has_measurement = meas_pos is not None
occlusion_age = self._update_occlusion_state(now, has_measurement)

if has_measurement:
    self._fuse_measurement(meas_pos, meas_vel, now)

    self._predict_started_t = None
    self._predict_start_pos = None

    if self._publish_phase != "bootstrap":
        self._publish_phase = "follow"

else:
    if not self._propagate_prediction(now):
        target = self._publish_pos if self._publish_pos is not None else self._bootstrap_pose
        if target is not None:
            self._publish_target_pose(stamp, target, self._vel_est, self._publish_quat)
        self._publish_status(stamp, data)
        return data

    self._publish_phase = "predict"

    if self._prediction_limits_reached(now):
        if self._hold_target_on_occlusion_timeout:
            self._publish_phase = "hold"
            self._publish_source = "occlusion_hold"

            target = self._publish_pos if self._publish_pos is not None else self._pos_est
            if target is not None:
                self._publish_target_pose(stamp, target, self._vel_est, self._publish_quat)

            self._publish_status(stamp, data)
            return data

        else:
            self._finish_tracking("occlusion_prediction_limit")
            self._publish_active_flag(False)
            self._publish_status(stamp, data)
            return data
```

---

# 13. Final behavior

After this update:

```text
Before object is seen:
    robot target starts at magnetic_link

When object is first tracked:
    target moves smoothly from magnetic_link toward tracking estimate

While object is visible:
    target follows future Kalman estimate smoothly

When robot blocks camera:
    target continues using last Kalman velocity prediction

If camera comes back:
    target smoothly corrects to measurement again

If camera stays blocked too long:
    target holds last safe position, or stops tracking depending on config
```

---

# 14. Best first test config

Use this first:

```yaml
ros2_publisher:
  target_smooth_tau_s: 0.10
  target_max_step_m: 0.012

  lookahead_steps: 3
  lookahead_dt_s: 0.033
  lookahead_max_time_s: 0.15

  occlusion_expected_s: 1.20
  occlusion_hold_after_s: 1.80
  occlusion_max_predict_distance_m: 0.30
  hold_target_on_occlusion_timeout: true
```

If the robot is late:

```yaml
lookahead_steps: 4
lookahead_max_time_s: 0.20
```

If the target still jumps too much:

```yaml
target_smooth_tau_s: 0.15
target_max_step_m: 0.008
```

If the robot is too slow:

```yaml
target_smooth_tau_s: 0.07
target_max_step_m: 0.018
```

---

# 15. Main concept

The final design should be:

```text
TrackerKalman:
    estimate object position and velocity

ROS2Publisher:
    use that velocity to predict n steps ahead
    smooth the target sent to robot
    keep predicting when camera is occluded by robot
    hold target if occlusion lasts too long
```

Do not let this happen:

```text
robot target = raw tracking estimate
```

Always do this:

```text
robot target = smooth target moving toward future tracking estimate
```
