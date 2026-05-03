# MotoMini Collision-Force Bayesian/Optuna Tuning Guide

## 1. Mục tiêu

Mục tiêu của bộ tuning này là tối ưu **bộ tạo lực va chạm** để robot có behavior:

```text
1. Khi chạm vật cản: không rung / không bật nảy mạnh.
2. Khi target đi vào vật cản vuông góc: robot không cố đâm sâu vào vật cản.
3. Có lực tiếp tuyến đủ lớn để robot trượt theo bề mặt / đi vòng qua cạnh vật cản.
4. Sau khi vượt qua vật cản: lực va chạm giảm mượt, robot không mất ổn định.
5. Robot tiếp tục tracking target sau obstacle.
```

Quan trọng: phần tuning này **không tune lại bộ điều khiển tracking chính** như `m_pos_min`, `k_pos_max`, `zeta_pos`, `adaptive_lambda`.  
Bộ này chỉ tune:

```text
- force generator trong online_collision_debugger
- collision projection / suppression params phía controller nếu cần
```

---

# 2. Hiện trạng code collision hiện tại

Trong node `online_collision_debugger`, code hiện đang:

```text
1. Dùng Tesseract contact manager để lấy contact distance, contact normal.
2. Tính normal repulsive force bằng computeCollisionForce().
3. Tính tangential force bằng computeTangentialForce().
4. Cộng total_force rồi publish /motomini/collision_wrench.
5. Publish thêm /motomini/collision_distance và /motomini/collision_normal cho controller.
```

Các topic output chính:

```text
/motomini/collision_wrench
/motomini/collision_distance
/motomini/collision_normal
collision_markers
collision_debug_contacts
```

## 2.1. Normal force hiện tại

Hiện tại normal force dùng dạng:

```cpp
rep_mag =
    spring_scale *
    collision_k_rep_ *
    (1.0 / d - 1.0 / d0) /
    (d * d);
```

với:

```cpp
wall_gamma = computeProjectionGamma(distance);
spring_scale = 1.0 - wall_gamma;
```

Ý nghĩa tốt của thiết kế này:

```text
- Khi còn xa wall/task zone: có repulsive force.
- Khi vào vùng projection wall: normal spring giảm để tránh spring/string oscillation.
- Hold force chỉ dùng emergency khi distance < collision_stop_distance.
```

Điểm nên cải thiện:

```text
- Công thức inverse-distance rất nhạy khi d nhỏ.
- collision_k_rep khó tune bằng Bayesian vì scale không trực quan.
- Chưa có measured normal damping theo vận tốc thật của EE.
```

---

## 2.2. Tangential force hiện tại

Tangential force hiện đang:

```cpp
if v_goal đi vào vật cản:
    lấy v_goal chiếu lên tangent plane
    tạo tangent direction
    F_tan = tangentialForceGain() * gamma^2 * approach_ratio
```

Trong code hiện tại:

```cpp
tangentialForceLimit() = 0.35 * collision_force_max_total_
tangentialForceGain()  = tangentialForceLimit()
```

Điểm tốt:

```text
- Chỉ tạo lực tiếp tuyến khi target velocity có thành phần đâm vào vật cản.
- Có dùng gamma nên gần vật cản mới tạo lực.
- Có giữ last_tangent_dir để tránh đổi hướng đột ngột.
```

Điểm nên cải thiện:

```text
- Tangential gain/limit đang hard-code theo 0.35 * total_force.
- tangentialVelocityDeadband() và tangentialSpeedScale() đang suy ra từ khoảng cách, hơi lẫn đơn vị m và m/s.
- targetVelocityTimeout() đang phụ thuộc vào collision_influence_distance, cũng hơi lẫn đơn vị m và s.
- Chưa có rate limit riêng cho force.
- Chưa có filter direction rõ ràng cho tangent.
```

---

# 3. Đúng, optimizer trước chưa thực sự tune collision force

Optimizer trước chủ yếu tune các tham số controller:

```python
m_pos_min
m_pos_max
k_pos_min
k_pos_max
zeta_pos
adaptive_lambda
adaptive_alpha_pos
```

Nó chưa tune các tham số như:

```text
collision_k_rep
collision_k_hold
collision_force_max_total
collision_force_max_per_contact
collision_guard_distance
collision_task_distance
collision_force_scale
collision_projection_max_gamma
tangential force gain
normal damping
force filtering
```

Vì vậy nên tạo **một optimizer riêng cho collision force**.

---

# 4. Kiến trúc tuning nên dùng

Nên tách thành 2 optimizer:

## 4.1. Optimizer A — Tracking controller optimizer

Tune:

```text
m_pos_min
m_pos_max
k_pos_min
k_pos_max
zeta_pos
adaptive_lambda
adaptive_alpha_pos
max_cart_linear_vel
max_cart_linear_acc
```

Mục tiêu:

```text
Bám target tốt khi không va chạm.
```

## 4.2. Optimizer B — Collision force optimizer

Tune:

```text
normal collision force
tangential sliding force
force filtering
projection/suppression params
```

Mục tiêu:

```text
Khi gặp vật cản: trượt mượt, không rung, không bật nảy, tiếp tục tracking.
```

Trong file này chỉ nói về **Optimizer B**.

---

# 5. Các thay đổi nên thêm vào force model

## 5.1. Thêm measured velocity damping theo normal

Hiện force generator đang có `/motomini/target_vel`, nhưng nên subscribe thêm:

```text
/motomini/feedback_vel
```

để biết vận tốc thật của end-effector.

Thêm member:

```cpp
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr feedback_vel_sub_;
Eigen::Vector3d latest_feedback_vel_linear_{Eigen::Vector3d::Zero()};
rclcpp::Time t_last_feedback_vel_cb_{0, 0, RCL_ROS_TIME};
```

Thêm callback:

```cpp
void feedbackVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    latest_feedback_vel_linear_ << msg->linear.x, msg->linear.y, msg->linear.z;
    t_last_feedback_vel_cb_ = this->now();
}
```

Thêm subscription:

```cpp
feedback_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/motomini/feedback_vel", 10,
    std::bind(&OnlineCollisionDebugger::feedbackVelCallback, this, std::placeholders::_1));
```

Sau đó trong normal force:

```cpp
double normalDampingMag(const Eigen::Vector3d& n_away) const
{
    const double age = (this->now() - t_last_feedback_vel_cb_).seconds();
    if (age > collision_feedback_vel_timeout_sec_)
        return 0.0;

    const double v_n = latest_feedback_vel_linear_.dot(n_away);

    // v_n < 0 nghĩa là robot đang đi vào vật cản.
    if (v_n >= 0.0)
        return 0.0;

    return collision_normal_damping_ * (-v_n);
}
```

Lực damping:

```cpp
F_damp = normal_damping_mag * n_away
```

Mục tiêu:

```text
- Giảm rung khi robot chạm vào vật.
- Giảm bật nảy vì damping chỉ chống chuyển động đi vào vật.
- Không cản robot khi nó đang đi ra khỏi vật.
```

---

## 5.2. Nên đổi normal force sang smooth bounded force

Công thức hiện tại inverse-distance có thể dùng được, nhưng khó tune.  
Để optimizer dễ học hơn, nên dùng dạng bounded smoothstep.

Thêm params:

```cpp
this->declare_parameter<double>("collision_normal_force_max", 6.0);
this->declare_parameter<double>("collision_normal_gain", 1.0);
this->declare_parameter<double>("collision_normal_fade_power", 1.0);
this->declare_parameter<double>("collision_normal_damping", 8.0);
this->declare_parameter<double>("collision_feedback_vel_timeout_sec", 0.2);
```

Công thức đề xuất:

```cpp
double smoothstep(double x)
{
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

Eigen::Vector3d computeCollisionForce(double distance,
                                      const Eigen::Vector3d &push_dir) const
{
    const double d0 = std::max(1e-6, collision_influence_distance_);
    const double d_safe = std::clamp(collision_safe_distance_, 0.0, d0 - 1e-6);

    if (distance >= d0)
        return Eigen::Vector3d::Zero();

    Eigen::Vector3d n = push_dir;
    if (!n.allFinite() || n.norm() < 1e-9)
        return Eigen::Vector3d::Zero();
    n.normalize();

    const double s = std::clamp((d0 - distance) / std::max(1e-6, d0 - d_safe), 0.0, 1.0);
    const double wall_gamma = computeProjectionGamma(distance);

    // Khi controller projection wall active, normal force nên fade để tránh bounce.
    const double fade = std::pow(std::clamp(1.0 - wall_gamma, 0.0, 1.0),
                                 collision_normal_fade_power_);

    const double rep_mag =
        collision_normal_force_max_ *
        collision_normal_gain_ *
        smoothstep(s) *
        fade;

    double hold_mag = 0.0;
    if (distance < collision_stop_distance_)
    {
        hold_mag =
            collision_k_hold_ *
            (collision_stop_distance_ - distance);
    }

    const double damping_mag = normalDampingMag(n);

    Eigen::Vector3d f = (rep_mag + hold_mag + damping_mag) * n;

    return clampNorm(f, collision_force_max_per_contact_);
}
```

Nếu không muốn đổi công thức ngay, vẫn có thể giữ inverse formula hiện tại, nhưng nên thêm:

```text
collision_normal_damping
collision_force_rate_limit
collision_force_attack_hz
collision_force_release_hz
```

---

## 5.3. Expose tangential force params

Hiện tangential gain/limit đang hard-code. Nên đổi thành parameter.

Thêm params:

```cpp
this->declare_parameter<bool>("collision_tangent_enabled", true);
this->declare_parameter<double>("collision_tangent_gain", 4.0);
this->declare_parameter<double>("collision_tangent_force_max", 4.0);
this->declare_parameter<double>("collision_tangent_force_ratio", 0.35);
this->declare_parameter<double>("collision_tangent_gamma_power", 2.0);
this->declare_parameter<double>("collision_tangent_speed_scale", 0.05);
this->declare_parameter<double>("collision_tangent_velocity_deadband", 0.01);
this->declare_parameter<double>("collision_target_vel_timeout_sec", 0.2);
```

Thay các hàm hard-code:

```cpp
double targetVelocityTimeout() const
{
    return collision_target_vel_timeout_sec_;
}

double tangentialVelocityDeadband() const
{
    return collision_tangent_velocity_deadband_;
}

double tangentialSpeedScale() const
{
    return collision_tangent_speed_scale_;
}

double tangentialForceLimit() const
{
    return std::min(collision_tangent_force_max_,
                    collision_tangent_force_ratio_ * collision_force_max_total_);
}

double tangentialForceGain() const
{
    return collision_tangent_gain_;
}
```

Trong `computeTangentialForce()` thay:

```cpp
const double tangential_mag =
    tangentialForceGain() * gamma * gamma * approach_ratio;
```

bằng:

```cpp
const double gamma_shape = std::pow(
    std::clamp(gamma, 0.0, 1.0),
    collision_tangent_gamma_power_);

const double tangential_mag =
    collision_tangent_gain_ * gamma_shape * approach_ratio;
```

sau đó clamp:

```cpp
return clampNorm(tangential_mag * tangent, tangentialForceLimit());
```

Ý nghĩa tuning:

| Param | Tác dụng |
|---|---|
| `collision_tangent_gain` | tăng lực trượt theo bề mặt |
| `collision_tangent_force_max` | giới hạn lực tiếp tuyến tuyệt đối |
| `collision_tangent_force_ratio` | giới hạn lực tiếp tuyến so với tổng lực |
| `collision_tangent_gamma_power` | lực tiếp tuyến xuất hiện sớm hay muộn |
| `collision_tangent_speed_scale` | target đâm vào vật nhanh bao nhiêu thì đạt full tangent force |
| `collision_tangent_velocity_deadband` | tránh tạo tangent force do noise nhỏ |
| `collision_target_vel_timeout_sec` | tránh dùng target velocity cũ |

---

## 5.4. Thêm force slew-rate limit

Hiện code có low-pass filter:

```cpp
smoothPublishedForce()
```

Nhưng low-pass filter đôi khi vẫn cho force đổi nhanh nếu input nhảy mạnh.  
Nên thêm slew-rate limit:

```cpp
this->declare_parameter<double>("collision_force_slew_rate", 80.0); // N/s
```

Sau khi low-pass hoặc trước khi low-pass:

```cpp
Eigen::Vector3d limitForceRate(const Eigen::Vector3d& raw_force, double dt)
{
    if (!have_force_filter_state_ || dt <= 0.0)
        return raw_force;

    Eigen::Vector3d df = raw_force - filtered_total_force_;
    const double max_df = collision_force_slew_rate_ * dt;

    if (df.norm() > max_df && df.norm() > 1e-9)
        return filtered_total_force_ + df.normalized() * max_df;

    return raw_force;
}
```

Mục tiêu:

```text
- Giảm giật lực khi contact normal đổi đột ngột.
- Giảm rung do contact pair thay đổi.
- Giảm bounce khi vượt qua cạnh vật cản.
```

---

## 5.5. Expose filter attack/release Hz

Hiện attack/release Hz được suy ra từ distance:

```cpp
collisionForceAttackHz()
collisionForceReleaseHz()
```

Nên expose ra params:

```cpp
this->declare_parameter<double>("collision_force_attack_hz", 20.0);
this->declare_parameter<double>("collision_force_release_hz", 8.0);
```

Gợi ý:

```text
attack_hz cao hơn release_hz:
    force xuất hiện nhanh khi sắp va chạm,
    force giảm mượt khi rời contact.

release_hz quá thấp:
    force còn kéo sau khi đã rời vật -> mất tracking.

release_hz quá cao:
    lực biến mất đột ngột -> bounce/rebound.
```

---

# 6. Bộ params collision đề xuất

## 6.1. Force generator params

```yaml
# Contact detection
collision_threshold: 0.10
collision_influence_distance: 0.10
collision_safe_distance: 0.04

# Normal force
collision_normal_force_max: 6.0
collision_normal_gain: 1.0
collision_normal_fade_power: 1.0
collision_normal_damping: 8.0
collision_feedback_vel_timeout_sec: 0.2

# Emergency hold
collision_k_hold: 50.0

# Tangential sliding force
collision_tangent_enabled: true
collision_tangent_gain: 4.0
collision_tangent_force_max: 4.0
collision_tangent_force_ratio: 0.35
collision_tangent_gamma_power: 2.0
collision_tangent_speed_scale: 0.05
collision_tangent_velocity_deadband: 0.01
collision_target_vel_timeout_sec: 0.2

# Force limits
collision_force_max_per_contact: 8.0
collision_force_max_total: 12.0

# Force filtering
collision_force_attack_hz: 20.0
collision_force_release_hz: 8.0
collision_force_slew_rate: 80.0
```

---

## 6.2. Controller collision params

Các params này nằm phía controller tracking, không phải force generator.  
Có thể tune sau khi force generator đã ổn.

```yaml
enable_collision_projection: true
collision_goal_suppression: true

collision_guard_distance: 0.03
collision_task_distance: 0.015
collision_stop_distance: 0.001

collision_projection_max_gamma: 1.0
collision_constraint_timeout_sec: 0.2

collision_force_scale: 1.0
collision_force_max: 5.0
collision_wrench_timeout_sec: 0.2
```

Ý nghĩa:

| Param | Tác dụng |
|---|---|
| `collision_guard_distance` | bắt đầu blend projection |
| `collision_task_distance` | vùng cấm đi sâu, projection gần max |
| `collision_projection_max_gamma` | mức chặn vận tốc đi vào vật |
| `collision_goal_suppression` | bỏ thành phần goal force đâm vào vật |
| `collision_force_scale` | scale wrench từ force generator |
| `collision_force_max` | clamp force phía controller |

---

# 7. Search space cho optimizer

Không nên optimize tất cả cùng lúc. Chia 3 phase.

---

## 7.1. Phase 1 — Normal contact stability

Mục tiêu:

```text
Robot chạm vật mà không rung, không bật nảy, không xuyên sâu.
```

Tune:

```python
collision_normal_force_max
collision_normal_damping
collision_force_max_per_contact
collision_force_attack_hz
collision_force_release_hz
collision_force_slew_rate
```

Search space:

```python
normal_space = {
    "collision_normal_force_max": (2.0, 10.0),
    "collision_normal_damping": (2.0, 25.0),
    "collision_force_max_per_contact": (2.0, 10.0),
    "collision_force_attack_hz": (8.0, 35.0),
    "collision_force_release_hz": (3.0, 20.0),
    "collision_force_slew_rate": (20.0, 150.0),
}
```

Giữ cố định:

```yaml
collision_tangent_enabled: false
collision_projection_max_gamma: 1.0
collision_goal_suppression: true
```

---

## 7.2. Phase 2 — Tangential sliding

Mục tiêu:

```text
Khi target đâm vào vật vuông góc, robot trượt theo tangent và vượt cạnh vật cản.
```

Tune:

```python
collision_tangent_gain
collision_tangent_force_max
collision_tangent_force_ratio
collision_tangent_gamma_power
collision_tangent_speed_scale
collision_tangent_velocity_deadband
```

Search space:

```python
tangent_space = {
    "collision_tangent_gain": (0.5, 10.0),
    "collision_tangent_force_max": (1.0, 8.0),
    "collision_tangent_force_ratio": (0.15, 0.60),
    "collision_tangent_gamma_power": (1.0, 3.5),
    "collision_tangent_speed_scale": (0.02, 0.15),
    "collision_tangent_velocity_deadband": (0.003, 0.03),
}
```

Giữ fixed bộ normal tốt nhất từ phase 1.

---

## 7.3. Phase 3 — Release and recovery

Mục tiêu:

```text
Sau khi vượt qua vật cản, force giảm mượt, robot không mất ổn định, tiếp tục tracking.
```

Tune:

```python
collision_force_release_hz
collision_force_slew_rate
collision_normal_fade_power
collision_force_scale
collision_force_max
collision_constraint_timeout_sec
```

Search space:

```python
release_space = {
    "collision_force_release_hz": (4.0, 25.0),
    "collision_force_slew_rate": (20.0, 150.0),
    "collision_normal_fade_power": (0.7, 3.0),
    "collision_force_scale": (0.3, 1.5),
    "collision_force_max": (2.0, 8.0),
    "collision_constraint_timeout_sec": (0.08, 0.35),
}
```

---

# 8. Test trajectory cho collision optimizer

Dùng trajectory chính của bạn:

```text
Start:
x = 0.25
y = -0.15
z = 0.15

Motion:
move along +Y

End:
y = 0.10
```

Ban đầu nên chạy velocity thấp:

```text
target_velocity = 0.1 -> 0.2 m/s
```

Sau khi ổn mới tăng:

```text
target_velocity = 0.4 -> 0.8 m/s
```

Không nên bắt đầu bằng `0.8 m/s` ngay khi tuning collision force.

---

# 9. Kịch bản obstacle test

Nên có 3 loại test.

## 9.1. Test A — Contact wall

Target đi vào bề mặt gần vuông góc.

Mục tiêu:

```text
- Không xuyên quá stop distance.
- Không rung theo normal.
- Không velocity spike.
```

Dùng để tune phase 1.

---

## 9.2. Test B — Square obstacle sliding

Đặt vật cản dạng box chắn đường +Y.

Mục tiêu:

```text
- Robot tiếp cận vật.
- Khi target vẫn đi +Y, robot có tangent force để trượt theo bề mặt.
- Robot đi qua cạnh vật cản.
```

Dùng để tune phase 2.

---

## 9.3. Test C — Release after obstacle

Sau khi robot vượt cạnh vật cản:

```text
- collision force phải decay.
- robot không bị văng khỏi path.
- robot tiếp tục track target.
```

Dùng để tune phase 3.

---

# 10. Metrics cần log

Trong collision trial, log các topic:

```text
/motomini/feedback
/motomini/feedback_vel
/motomini/target_pose
/motomini/target_vel
/motomini/collision_wrench
/motomini/collision_distance
/motomini/collision_normal
/joint_states
/rosout
```

Từ đó tính metrics:

---

## 10.1. Contact metrics

```text
min_distance
penetration_depth
contact_time
distance_jitter
normal_velocity_rms
normal_velocity_peak
```

Cách tính:

```python
contact_mask = collision_distance < collision_influence_distance
penetration_depth = max(0, collision_stop_distance - min_distance)
distance_jitter = std(collision_distance[contact_mask])
```

Normal velocity:

```python
v_n = dot(feedback_vel_linear, collision_normal)
```

Trong đó:

```text
v_n < 0: robot đang đi vào vật
v_n > 0: robot đang đi ra khỏi vật
```

Nên log:

```python
normal_into_rms = rms(min(v_n, 0))
normal_out_peak = max(v_n)
```

---

## 10.2. Tangential metrics

```text
tangent_velocity_mean
tangent_progress
progress_ratio
stuck_time
```

Tính:

```python
v_tangent = feedback_vel - v_n * normal
tangent_speed = norm(v_tangent)
```

Progress theo mục tiêu:

```python
progress_ratio = (final_y - start_y) / (end_y - start_y)
progress_ratio = clamp(progress_ratio, 0, 1)
```

Nếu progress thấp trong khi contact active, nghĩa là robot bị kẹt trước obstacle.

---

## 10.3. Force smoothness metrics

```text
force_peak
force_rms
force_rate_rms
force_saturation_ratio
force_tail_after_contact
```

Tính:

```python
F = collision_wrench.force
force_norm = norm(F)
dFdt = diff(F) / dt
force_rate_rms = rms(norm(dFdt))
```

Sau khi contact kết thúc:

```python
force_tail_after_contact = mean(force_norm[t > last_contact_time + 0.2])
```

Nếu force tail lớn, robot có thể mất tracking sau khi vượt vật.

---

## 10.4. Recovery metrics

```text
post_contact_rmse
recovery_time
bounce_distance
post_contact_velocity_spike
```

Bounce distance:

```text
độ lệch lớn nhất khỏi target/path sau khi contact kết thúc
```

Recovery time:

```text
thời gian sau contact để error quay về dưới threshold, ví dụ 2 mm
```

---

# 11. Cost functions

## 11.1. Hard constraints

Nếu vi phạm thì fail trial:

```python
if min_distance < collision_stop_distance - 0.001:
    fail

if max_joint_vel_ratio > 0.95:
    fail

if max_feedback_vel > measured_cart_limit:
    fail

if safety_stop:
    fail

if progress_ratio < 0.8:
    fail for full obstacle-crossing test
```

---

## 11.2. Phase 1 cost — Normal stability

```python
J_normal = (
    10.0 * penetration_depth / 0.001
    + 4.0 * normal_into_rms / 0.02
    + 3.0 * distance_jitter / 0.001
    + 2.0 * force_rate_rms / 50.0
    + 2.0 * force_saturation_ratio
    + safety_penalty
)
```

Mục tiêu:

```text
- penetration_depth nhỏ
- normal velocity vào vật nhỏ
- distance jitter nhỏ
- force không giật
```

---

## 11.3. Phase 2 cost — Tangential sliding

```python
J_tangent = (
    8.0 * (1.0 - progress_ratio)
    + 3.0 * stuck_time / 1.0
    + 2.0 * abs(tangent_speed_mean - desired_tangent_speed) / 0.05
    + 2.0 * force_rate_rms / 50.0
    + 3.0 * normal_into_rms / 0.02
    + safety_penalty
)
```

Mục tiêu:

```text
- progress_ratio gần 1
- stuck_time thấp
- tangent motion đủ lớn nhưng không giật
- normal motion vào vật thấp
```

---

## 11.4. Phase 3 cost — Release and recovery

```python
J_release = (
    5.0 * post_contact_rmse / 0.002
    + 4.0 * recovery_time / 1.0
    + 5.0 * bounce_distance / 0.003
    + 3.0 * force_tail_after_contact / 1.0
    + 2.0 * post_contact_velocity_spike / 0.05
    + safety_penalty
)
```

Mục tiêu:

```text
- sau contact, robot quay lại tracking nhanh
- không bật khỏi path
- force biến mất đủ nhanh nhưng không giật
```

---

## 11.5. Total cost

Sau khi từng phase đã ổn, chạy combined test:

```python
J_total = (
    0.35 * J_normal
    + 0.35 * J_tangent
    + 0.30 * J_release
)
```

Nếu mục tiêu ưu tiên "không rung" hơn "đi nhanh":

```python
J_total = (
    0.45 * J_normal
    + 0.30 * J_tangent
    + 0.25 * J_release
)
```

Nếu mục tiêu ưu tiên "đi qua vật cản":

```python
J_total = (
    0.25 * J_normal
    + 0.50 * J_tangent
    + 0.25 * J_release
)
```

---

# 12. Cấu trúc file optimizer đề xuất

```text
motomini/
├── config/
│   ├── feedback_controller_collision_base.yaml
│   ├── collision_force_base.yaml
│   ├── collision_force_trial.yaml
│   └── feedback_controller_collision_trial.yaml
│
├── launch/
│   └── motomini_collision_optimization.launch.py
│
├── scripts/
│   ├── run_collision_force_optimizer.py
│   ├── run_collision_trial.py
│   ├── analyze_collision_trial.py
│   └── plot_collision_optimizer_summary.py
│
└── tuning_results/
    ├── collision_trials_summary.csv
    ├── collision_params_history.csv
    ├── collision_optimizer_state.json
    ├── collision_best_params.yaml
    └── optuna_collision_trials.csv
```

---

# 13. Launch file riêng cho collision optimization

Nên tạo launch riêng:

```text
motomini_collision_optimization.launch.py
```

Ý tưởng:

```python
DeclareLaunchArgument('feedback_yaml_file')
DeclareLaunchArgument('collision_yaml_file')
DeclareLaunchArgument('real_robot')
DeclareLaunchArgument('debug')
```

Controller node dùng:

```python
parameters=[feedback_yaml_file]
```

Collision debugger dùng:

```python
parameters=[
    {'robot_description': robot_description},
    {'robot_description_semantic': robot_description_semantic},
    collision_yaml_file
]
```

Optimizer mỗi trial chỉ ghi:

```text
collision_force_trial.yaml
```

Nếu phase 3 cần tune controller collision params thì ghi thêm:

```text
feedback_controller_collision_trial.yaml
```

---

# 14. Pseudo-code `run_collision_force_optimizer.py`

```python
def objective(trial):
    params = dict(fixed_best_params)

    if phase == "normal":
        params.update({
            "collision_normal_force_max": trial.suggest_float("collision_normal_force_max", 2.0, 10.0),
            "collision_normal_damping": trial.suggest_float("collision_normal_damping", 2.0, 25.0),
            "collision_force_max_per_contact": trial.suggest_float("collision_force_max_per_contact", 2.0, 10.0),
            "collision_force_attack_hz": trial.suggest_float("collision_force_attack_hz", 8.0, 35.0),
            "collision_force_release_hz": trial.suggest_float("collision_force_release_hz", 3.0, 20.0),
            "collision_force_slew_rate": trial.suggest_float("collision_force_slew_rate", 20.0, 150.0),
        })

    elif phase == "tangent":
        params.update({
            "collision_tangent_gain": trial.suggest_float("collision_tangent_gain", 0.5, 10.0),
            "collision_tangent_force_max": trial.suggest_float("collision_tangent_force_max", 1.0, 8.0),
            "collision_tangent_force_ratio": trial.suggest_float("collision_tangent_force_ratio", 0.15, 0.60),
            "collision_tangent_gamma_power": trial.suggest_float("collision_tangent_gamma_power", 1.0, 3.5),
            "collision_tangent_speed_scale": trial.suggest_float("collision_tangent_speed_scale", 0.02, 0.15),
            "collision_tangent_velocity_deadband": trial.suggest_float("collision_tangent_velocity_deadband", 0.003, 0.03),
        })

    elif phase == "release":
        params.update({
            "collision_force_release_hz": trial.suggest_float("collision_force_release_hz", 4.0, 25.0),
            "collision_force_slew_rate": trial.suggest_float("collision_force_slew_rate", 20.0, 150.0),
            "collision_normal_fade_power": trial.suggest_float("collision_normal_fade_power", 0.7, 3.0),
            "collision_force_scale": trial.suggest_float("collision_force_scale", 0.3, 1.5),
            "collision_force_max": trial.suggest_float("collision_force_max", 2.0, 8.0),
            "collision_constraint_timeout_sec": trial.suggest_float("collision_constraint_timeout_sec", 0.08, 0.35),
        })

    write_collision_yaml(params)
    launch_collision_optimization()
    metrics = run_collision_trial()
    cost = compute_collision_cost(metrics, phase)
    save_collision_trial(trial.number, params, metrics, cost)
    return cost
```

---

# 15. Optuna sampler khuyến nghị

```python
sampler = optuna.samplers.TPESampler(
    n_startup_trials=10,
    multivariate=True,
    group=True,
    seed=0,
)

study = optuna.create_study(
    direction="minimize",
    sampler=sampler,
)
```

Số trial đề xuất:

```text
normal phase:   30-50 trials
tangent phase:  40-70 trials
release phase:  30-50 trials
combined:       20-30 trials
```

---

# 16. Logging optimizer

Không log toàn bộ raw data vào CSV chính.  
CSV chính chỉ cần:

```csv
trial,
phase,
cost,
best_cost_so_far,
safety_fail,
fail_reason,
progress_ratio,
min_distance,
penetration_depth,
distance_jitter,
normal_into_rms,
tangent_speed_mean,
stuck_time,
force_peak,
force_rate_rms,
force_tail_after_contact,
post_contact_rmse,
recovery_time,
bounce_distance,
max_joint_vel_ratio,
max_feedback_vel
```

Params để file riêng:

```csv
collision_params_history.csv
```

---

# 17. Dấu hiệu tune đúng

## 17.1. Force quá yếu

Biểu hiện:

```text
robot vẫn đâm sâu vào vật
min_distance thấp
normal_into_rms cao
progress_ratio thấp
```

Cách sửa:

```text
tăng collision_normal_force_max
tăng collision_normal_damping
tăng collision_projection_max_gamma
tăng collision_tangent_gain nếu bị kẹt
```

---

## 17.2. Force quá mạnh

Biểu hiện:

```text
robot bật khỏi vật
bounce_distance cao
post_contact_rmse cao
force_peak cao
```

Cách sửa:

```text
giảm collision_normal_force_max
giảm collision_force_max_total
tăng normal damping nếu bounce do velocity
giảm collision_force_slew_rate nếu force giật
giảm collision_tangent_force_max nếu bị kéo ngang quá mạnh
```

---

## 17.3. Rung khi contact

Biểu hiện:

```text
distance_jitter cao
force_rate_rms cao
normal velocity đổi dấu liên tục
```

Cách sửa:

```text
tăng collision_normal_damping
giảm collision_force_attack_hz
giảm collision_force_slew_rate
tăng collision_tangent_velocity_deadband
tăng collision_tangent_gamma_power
```

---

## 17.4. Không trượt theo bề mặt

Biểu hiện:

```text
stuck_time cao
progress_ratio thấp
tangent_speed_mean thấp
```

Cách sửa:

```text
tăng collision_tangent_gain
tăng collision_tangent_force_max
tăng collision_tangent_force_ratio
giảm collision_tangent_gamma_power để tangent force xuất hiện sớm hơn
giảm collision_tangent_speed_scale nếu target velocity nhỏ
```

---

## 17.5. Vượt vật cản xong bị mất tracking

Biểu hiện:

```text
force_tail_after_contact cao
post_contact_rmse cao
recovery_time cao
bounce_distance cao
```

Cách sửa:

```text
tăng collision_force_release_hz nếu force còn kéo quá lâu
giảm collision_force_release_hz nếu force biến mất quá đột ngột
giảm collision_force_slew_rate nếu force giật
giảm collision_force_scale phía controller
giảm collision_constraint_timeout_sec nếu projection giữ quá lâu
```

---

# 18. Quy trình tuning đầy đủ

## Step 0 — Baseline

Chạy trajectory không có obstacle.

Mục tiêu:

```text
tracking ổn trước khi collision active
```

Không tune collision ở bước này.

---

## Step 1 — Normal force

```text
obstacle: wall / box face
tangent disabled
```

Tune:

```text
collision_normal_force_max
collision_normal_damping
collision_force_attack_hz
collision_force_release_hz
collision_force_slew_rate
```

Chọn bộ có:

```text
min_distance an toàn
normal_into_rms thấp
distance_jitter thấp
force_rate_rms thấp
```

---

## Step 2 — Tangential force

```text
obstacle: square obstacle chắn đường +Y
tangent enabled
```

Tune:

```text
collision_tangent_gain
collision_tangent_force_max
collision_tangent_force_ratio
collision_tangent_gamma_power
collision_tangent_speed_scale
collision_tangent_velocity_deadband
```

Chọn bộ có:

```text
progress_ratio cao
stuck_time thấp
tangent_speed_mean đủ lớn
normal_into_rms vẫn thấp
```

---

## Step 3 — Release/recovery

```text
robot vượt qua cạnh vật cản
```

Tune:

```text
collision_force_release_hz
collision_force_slew_rate
collision_normal_fade_power
collision_force_scale
collision_force_max
collision_constraint_timeout_sec
```

Chọn bộ có:

```text
post_contact_rmse thấp
recovery_time thấp
force_tail_after_contact thấp
bounce_distance thấp
```

---

## Step 4 — Combined test

Chạy full trajectory:

```text
start: x=0.25, y=-0.15, z=0.15
end:   y=0.10
velocity tăng dần: 0.1 -> 0.2 -> 0.4 -> 0.8 m/s
```

Chỉ nhận bộ params nếu:

```text
không safety fail
progress_ratio > 0.95
overshoot/bounce thấp
tracking sau contact ổn
max_joint_vel_ratio < 0.95
```

---

# 19. Bộ starting params khuyến nghị

Dùng bộ này sau khi thêm params mới:

```yaml
collision_threshold: 0.10
collision_influence_distance: 0.10
collision_safe_distance: 0.04

collision_guard_distance: 0.03
collision_task_distance: 0.015
collision_stop_distance: 0.001

collision_normal_force_max: 5.0
collision_normal_gain: 1.0
collision_normal_fade_power: 1.2
collision_normal_damping: 10.0
collision_feedback_vel_timeout_sec: 0.2

collision_k_hold: 50.0

collision_tangent_enabled: true
collision_tangent_gain: 3.0
collision_tangent_force_max: 3.5
collision_tangent_force_ratio: 0.35
collision_tangent_gamma_power: 2.0
collision_tangent_speed_scale: 0.06
collision_tangent_velocity_deadband: 0.01
collision_target_vel_timeout_sec: 0.2

collision_force_max_per_contact: 6.0
collision_force_max_total: 10.0

collision_force_attack_hz: 18.0
collision_force_release_hz: 8.0
collision_force_slew_rate: 70.0
```

Controller collision params:

```yaml
enable_collision_projection: true
collision_goal_suppression: true
collision_force_scale: 1.0
collision_force_max: 5.0
collision_projection_max_gamma: 1.0
collision_constraint_timeout_sec: 0.2
collision_wrench_timeout_sec: 0.2
```

---

# 20. Kết luận

Bộ collision optimizer nên tối ưu theo thứ tự:

```text
1. Normal stability
   -> không rung, không xuyên, không bật nảy

2. Tangential sliding
   -> có lực tiếp tuyến để đi qua vật cản vuông góc

3. Release/recovery
   -> vượt vật xong vẫn track target ổn định
```

Không nên tune collision force chung với tracking controller ngay từ đầu.  
Hãy giữ bộ tracking controller tốt nhất cố định, rồi tune collision force riêng.

Thứ tự quan trọng nhất:

```text
normal damping trước
tangent gain sau
release/filter cuối cùng
```
