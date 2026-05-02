# Hướng dẫn tạo Bayesian Optimizer để tìm bộ tham số tối ưu cho MotoMini adaptive Cartesian controller

Tài liệu này mô tả cách thêm một bộ tối ưu hóa bằng Python để tìm tham số tốt cho `motomini_feedback_stream` với mục tiêu:

- bám nhanh target pose / target velocity,
- sai số tracking thấp,
- không overshoot hoặc overshoot rất nhỏ,
- không vượt giới hạn vận tốc / gia tốc / joint limit,
- không thay đổi nhiều cấu trúc controller hiện tại, chủ yếu thay đổi tham số YAML.

Controller hiện tại là adaptive Cartesian admittance-inspired velocity controller. Các tham số chính gồm `m_pos_min`, `m_pos_max`, `k_pos_min`, `k_pos_max`, `zeta_pos`, `adaptive_lambda`, `adaptive_alpha_pos`, `max_cart_linear_vel`, `max_cart_linear_acc`, sau đó velocity Cartesian được map qua SR inverse Jacobian thành joint velocity.

---

## 1. Ý tưởng tổng thể

Không dùng Bayesian filter để tune online trực tiếp. Nên dùng **Bayesian Optimization** dạng offline / semi-offline:

```text
1 trial = 1 bộ tham số
      -> ghi YAML trial
      -> launch controller với YAML đó
      -> chạy cùng một quỹ đạo test
      -> ghi rosbag / log topic
      -> tính cost
      -> optimizer chọn bộ tham số tiếp theo
```

Ưu điểm:

- không phải sửa sâu controller,
- dễ rollback nếu bộ tham số xấu,
- có thể đặt penalty cho overshoot và safety,
- phù hợp với bài toán black-box: input là tham số, output là tracking quality.

---

## 2. Các file nên thêm vào package

Đề xuất thêm các file sau vào package ROS2 hiện tại, ví dụ package `robot_planning` hoặc package riêng `motomini_tuning`.

```text
robot_planning/
├── config/
│   ├── feedback_controller.yaml              # file chính đang dùng
│   ├── feedback_controller_opt_base.yaml     # base config cho optimizer
│   └── feedback_controller_trial.yaml        # file optimizer tự ghi mỗi trial
│
├── launch/
│   ├── motomini_feedback_stream.launch.py    # launch đang dùng
│   └── optimize_feedback_controller.launch.py # launch riêng cho tối ưu
│
├── scripts/
│   ├── bayes_optimize_feedback.py            # vòng Bayesian Optimization chính
│   ├── run_tracking_trial.py                 # chạy 1 trial và log data
│   ├── analyze_tracking_log.py               # tính metric/cost
│   └── publish_test_trajectory.py            # publish target pose/vel chuẩn
│
└── tuning_results/
    ├── trials.csv
    ├── best_params.yaml
    ├── plots/
    └── bags/
```

Nếu không muốn tạo package mới, có thể để toàn bộ trong package hiện tại. Quan trọng là optimizer có thể:

1. ghi file YAML trial,
2. launch node bằng YAML trial,
3. publish quỹ đạo test,
4. đọc log,
5. tính cost,
6. lưu kết quả.

---

## 3. Thay đổi nhỏ nên thêm vào code controller hiện tại

### 3.1. Không bắt buộc: dynamic parameter callback

Cách ít thay đổi nhất là **khởi động lại node controller mỗi trial**. Vì trong code hiện tại, parameter được đọc trong constructor, nên optimizer chỉ cần tạo YAML mới rồi restart node.

Cách này an toàn và đơn giản:

```text
write feedback_controller_trial.yaml
restart motomini_feedback_stream
run trajectory
stop node
analyze result
```

Tuy nhiên, nếu muốn optimizer chạy nhanh hơn, có thể thêm dynamic parameter callback để cập nhật tham số runtime bằng `ros2 param set`. Nhưng điều này có rủi ro hơn vì controller đang chạy có state nội bộ như `xdot_ref_`, `tracked_positions_`, velocity filter, collision state.

Khuyến nghị ban đầu:

```text
Không thêm dynamic param callback.
Mỗi trial restart node.
```

### 3.2. Nên thêm parameter để chọn mode target velocity

Nếu bạn có external node publish `/motomini/target_pose`, thì không nên để controller tự integrate `/motomini/target_vel` vào `desired_pose_`. Nếu không, target velocity có thể bị dùng 2 lần.

Trong YAML nên có:

```yaml
integrate_target_vel_to_pose: false
```

Logic trong code:

```cpp
this->declare_parameter<bool>("integrate_target_vel_to_pose", false);
integrate_target_vel_to_pose_ = this->get_parameter("integrate_target_vel_to_pose").as_bool();
```

Trong `handlePoseFollow()`, bọc đoạn integrate target velocity:

```cpp
if (integrate_target_vel_to_pose_ && vel_active)
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

Thêm member:

```cpp
bool integrate_target_vel_to_pose_{false};
```

Mode đề xuất khi tuning:

```yaml
integrate_target_vel_to_pose: false
```

Vì optimizer nên test controller bám một `/motomini/target_pose` xác định, có thể thêm `/motomini/target_vel` làm feedforward nếu cần.

### 3.3. Nên thêm debug topic để optimizer đọc safety dễ hơn

Không bắt buộc, nhưng rất hữu ích. Thêm publisher debug dạng `diagnostic_msgs/msg/KeyValue` hoặc custom message. Nếu muốn đơn giản, publish các topic scalar:

```text
/motomini/debug/pos_error_norm
/motomini/debug/ori_error_norm
/motomini/debug/w_manipulability
/motomini/debug/k_pos_current
/motomini/debug/m_pos_current
/motomini/debug/d_pos_current
/motomini/debug/xdot_ref_norm
/motomini/debug/theta_d_max_abs
/motomini/debug/safety_stop
```

Nếu chưa muốn sửa code, optimizer vẫn có thể dùng:

```text
/motomini/feedback
/motomini/target_pose
/motomini/feedback_vel
/joint_states
```

Nhưng debug topic sẽ giúp phân tích nhanh hơn.

### 3.4. Nên có service reset trước mỗi trial

Code hiện tại đã có:

```text
/pose_following/start
/pose_following/stop
/pose_following/init_start
```

Optimizer nên gọi:

```text
/pose_following/stop
/pose_following/start
```

trước mỗi trajectory để reset state.

---

## 4. Launch file riêng cho tối ưu

Tạo file:

```text
launch/optimize_feedback_controller.launch.py
```

Ví dụ skeleton:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_planning')

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(pkg_share, 'config', 'feedback_controller_trial.yaml'),
        description='YAML parameter file for one optimization trial'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    feedback_node = Node(
        package='robot_planning',
        executable='motomini_feedback_stream',
        name='motomini_feedback_stream',
        output='screen',
        parameters=[
            LaunchConfiguration('param_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        param_file_arg,
        use_sim_time_arg,
        feedback_node,
    ])
```

Chạy thủ công một trial:

```bash
ros2 launch robot_planning optimize_feedback_controller.launch.py \
  param_file:=/path/to/feedback_controller_trial.yaml
```

Optimizer Python sẽ gọi launch này bằng `subprocess.Popen`.

---

## 5. Base YAML cho optimizer

Tạo:

```text
config/feedback_controller_opt_base.yaml
```

Nội dung đề xuất:

```yaml
/**:
  ros__parameters:
    rate_hz: 50.0

    # Position tuning baseline
    m_pos_min: 0.02
    m_pos_max: 0.12
    k_pos_min: 70.0
    k_pos_max: 220.0
    zeta_pos: 0.8

    # Keep orientation stable during position tuning
    m_ori_min: 0.02
    m_ori_max: 0.15
    k_ori_min: 2.0
    k_ori_max: 15.0
    zeta_ori: 1.0

    adaptive_lambda: 15.0
    adaptive_alpha_pos: 40.0
    adaptive_alpha_ori: 5.0

    max_cart_linear_vel: 0.50
    max_cart_angular_vel: 2.0
    max_cart_linear_acc: 1.2
    max_cart_angular_acc: 1.5

    w0: 0.001
    k0: 0.01

    enable_seed: false
    integrate_target_vel_to_pose: false

    # Collision settings can remain enabled if they are part of real task.
    # For pure tracking benchmark, disable collision wrench publisher or keep no active constraint.
    enable_collision_projection: true
    collision_goal_suppression: true
    collision_guard_distance: 0.03
    collision_task_distance: 0.015
    collision_stop_distance: 0.001
    collision_projection_max_gamma: 1.0
    collision_constraint_timeout_sec: 0.2
    collision_force_scale: 1.0
    collision_force_max: 5.0
```

Optimizer sẽ đọc base YAML, thay các field cần tối ưu, rồi ghi ra:

```text
config/feedback_controller_trial.yaml
```

---

## 6. Quỹ đạo test chuẩn

Ban đầu chỉ tune position X, không tune 3D phức tạp ngay.

### 6.1. Test trajectory 1: step-ramp X 30 mm

```text
0.0 s -> 0.5 s: hold current pose
0.5 s -> 1.2 s: ramp x + 0.03 m
1.2 s -> 3.5 s: hold final pose
```

Giữ:

```text
y, z constant
orientation constant
```

Lý do:

- đoạn ramp đo lag và khả năng bám velocity,
- đoạn hold đo overshoot, settling, jitter,
- an toàn vì chỉ 30 mm.

### 6.2. Test trajectory 2: X/Y/Z small set

Sau khi tốt trên X:

```text
x + 0.03 m
y + 0.02 m
z + 0.02 m
x - 0.03 m
```

Mỗi segment có ramp + hold.

### 6.3. Test trajectory 3: task-like trajectory

Cuối cùng test quỹ đạo thật của task. Chỉ dùng sau khi bộ tham số đã pass test 1 và 2.

---

## 7. Script publish_test_trajectory.py

File:

```text
scripts/publish_test_trajectory.py
```

Skeleton:

```python
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist


class TestTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('publish_test_trajectory')
        self.pose_pub = self.create_publisher(PoseStamped, '/motomini/target_pose', 10)
        self.vel_pub = self.create_publisher(Twist, '/motomini/target_vel', 10)

        self.declare_parameter('duration', 3.5)
        self.declare_parameter('ramp_start', 0.5)
        self.declare_parameter('ramp_end', 1.2)
        self.declare_parameter('dx', 0.03)
        self.declare_parameter('rate_hz', 50.0)

        self.duration = float(self.get_parameter('duration').value)
        self.ramp_start = float(self.get_parameter('ramp_start').value)
        self.ramp_end = float(self.get_parameter('ramp_end').value)
        self.dx = float(self.get_parameter('dx').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        # TODO: replace by reading current feedback once before start.
        self.x0 = 0.20
        self.y0 = 0.00
        self.z0 = 0.20
        self.qw = 1.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0

    def pose_at(self, t):
        if t < self.ramp_start:
            s = 0.0
            v = 0.0
        elif t < self.ramp_end:
            s = (t - self.ramp_start) / (self.ramp_end - self.ramp_start)
            v = self.dx / (self.ramp_end - self.ramp_start)
        else:
            s = 1.0
            v = 0.0
        return self.x0 + self.dx * s, self.y0, self.z0, v

    def run(self):
        dt = 1.0 / self.rate_hz
        t0 = self.get_clock().now()
        while rclpy.ok():
            now = self.get_clock().now()
            t = (now - t0).nanoseconds * 1e-9
            if t > self.duration:
                break

            x, y, z, vx = self.pose_at(t)

            pose = PoseStamped()
            pose.header.stamp = now.to_msg()
            pose.header.frame_id = 'base_link'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = self.qw
            pose.pose.orientation.x = self.qx
            pose.pose.orientation.y = self.qy
            pose.pose.orientation.z = self.qz
            self.pose_pub.publish(pose)

            vel = Twist()
            vel.linear.x = vx
            vel.linear.y = 0.0
            vel.linear.z = 0.0
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = 0.0
            self.vel_pub.publish(vel)

            rclpy.spin_once(self, timeout_sec=dt)

        # publish final zero velocity several times
        for _ in range(10):
            vel = Twist()
            self.vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=dt)


def main():
    rclpy.init()
    node = TestTrajectoryPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Trong bản thật, nên đọc `/motomini/feedback` trước để lấy `x0,y0,z0,orientation` hiện tại, thay vì hard-code.

---

## 8. Metrics và cost function

Optimizer cần biến log thành một số cost càng nhỏ càng tốt.

### 8.1. Topic cần log

```text
/motomini/feedback
/motomini/feedback_vel
/motomini/target_pose
/motomini/target_vel
/joint_states
```

Nếu có debug topic thì log thêm:

```text
/motomini/debug/pos_error_norm
/motomini/debug/theta_d_max_abs
/motomini/debug/w_manipulability
```

### 8.2. Metrics cần tính

```text
tracking_rmse:
  RMS norm(target_pos - feedback_pos)

ramp_lag:
  mean abs error trong đoạn target đang ramp

overshoot:
  max(feedback_x - final_target_x, 0) nếu target đi theo +X

settling_time:
  thời gian từ lúc ramp kết thúc tới khi abs(error) < threshold và giữ ổn định

arrival_jitter:
  std(feedback_x) trong đoạn cuối sau khi đã hold

max_feedback_vel:
  max norm(/motomini/feedback_vel.linear)

max_joint_vel:
  max abs(joint_states.velocity)

safety_fail:
  true nếu node stop, missing feedback, velocity quá lớn, hoặc joint vượt ngưỡng
```

### 8.3. Cost đề xuất

```python
cost = (
    1.0  * tracking_rmse  / 0.001
    + 2.0  * ramp_lag     / 0.001
    + 30.0 * overshoot    / 0.001
    + 0.5  * settling_time
    + 5.0  * arrival_jitter / 0.0002
    + velocity_penalty
)
```

Penalty cứng:

```python
if overshoot > 0.001:
    cost += 100000.0 + overshoot * 1e7

if max_feedback_vel > measured_cart_linear_vel_limit:
    cost += 100000.0

if max_joint_vel > 0.80 * joint_limit:
    cost += 100000.0

if safety_fail:
    cost = 1e9
```

Vì mục tiêu của bạn là không overshoot, hệ số `overshoot` phải lớn hơn hệ số RMSE nhiều lần.

---

## 9. Script analyze_tracking_log.py

Có 2 cách:

### Cách A: log bằng rosbag2

Dùng `rosbag2_py` để đọc bag sau mỗi trial.

Ưu điểm:

- tách logging và analysis rõ ràng,
- dễ debug bằng plot sau này.

### Cách B: run_tracking_trial.py subscribe trực tiếp

Script trial subscribe topic, lưu data vào memory, sau khi trajectory xong thì tính metrics.

Ưu điểm:

- đơn giản hơn cho optimizer,
- không cần đọc bag.

Khuyến nghị ban đầu: dùng cách B cho nhanh, sau đó thêm rosbag khi cần lưu lịch sử đầy đủ.

Pseudo-code analyzer:

```python
import numpy as np


def compute_metrics(t, target_x, feedback_x, feedback_vx, joint_vel, ramp_start, ramp_end, final_time):
    error = target_x - feedback_x

    rmse = float(np.sqrt(np.mean(error ** 2)))

    ramp_mask = (t >= ramp_start) & (t <= ramp_end)
    ramp_lag = float(np.mean(np.abs(error[ramp_mask]))) if np.any(ramp_mask) else 1e3

    final_target = target_x[-1]
    overshoot = float(max(0.0, np.max(feedback_x - final_target)))

    hold_mask = t > ramp_end
    abs_err = np.abs(error)
    threshold = 0.0005
    settling_time = final_time
    for i in np.where(hold_mask)[0]:
        if np.all(abs_err[i:] < threshold):
            settling_time = float(t[i] - ramp_end)
            break

    tail_mask = t > (final_time - 0.5)
    jitter = float(np.std(feedback_x[tail_mask])) if np.any(tail_mask) else 1e3

    max_feedback_vel = float(np.max(np.abs(feedback_vx))) if len(feedback_vx) else 1e3
    max_joint_vel = float(np.max(np.abs(joint_vel))) if joint_vel.size else 1e3

    return {
        'rmse': rmse,
        'ramp_lag': ramp_lag,
        'overshoot': overshoot,
        'settling_time': settling_time,
        'arrival_jitter': jitter,
        'max_feedback_vel': max_feedback_vel,
        'max_joint_vel': max_joint_vel,
    }
```

---

## 10. Bayesian optimizer Python

Dùng một trong hai thư viện:

```bash
pip install scikit-optimize pyyaml pandas numpy matplotlib
```

hoặc:

```bash
pip install optuna pyyaml pandas numpy matplotlib
```

Khuyến nghị: **Optuna** dễ dùng, dễ lưu kết quả SQLite, có TPE sampler hoạt động tốt cho search space nhỏ. Nếu muốn Gaussian Process Bayesian Optimization đúng nghĩa hơn, dùng `scikit-optimize`.

### 10.1. Version Optuna khuyến nghị

File:

```text
scripts/bayes_optimize_feedback.py
```

Skeleton:

```python
#!/usr/bin/env python3
import os
import time
import yaml
import json
import signal
import subprocess
import pandas as pd
import optuna


BASE_YAML = 'config/feedback_controller_opt_base.yaml'
TRIAL_YAML = 'config/feedback_controller_trial.yaml'
RESULT_CSV = 'tuning_results/trials.csv'


def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def write_trial_yaml(base_path, trial_path, params):
    data = load_yaml(base_path)
    ros_params = data['/**']['ros__parameters']
    for k, v in params.items():
        ros_params[k] = float(v) if isinstance(v, (int, float)) else v
    with open(trial_path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)


def launch_controller(param_file):
    cmd = [
        'ros2', 'launch', 'robot_planning', 'optimize_feedback_controller.launch.py',
        f'param_file:={os.path.abspath(param_file)}'
    ]
    return subprocess.Popen(cmd, preexec_fn=os.setsid)


def stop_process(proc):
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5.0)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


def run_one_trial(params, trial_index):
    write_trial_yaml(BASE_YAML, TRIAL_YAML, params)

    proc = launch_controller(TRIAL_YAML)
    try:
        time.sleep(2.0)  # wait controller startup and joint state sync

        # Optional reset services
        subprocess.run(['ros2', 'service', 'call', '/pose_following/stop', 'std_srvs/srv/Trigger', '{}'], timeout=3)
        time.sleep(0.5)
        subprocess.run(['ros2', 'service', 'call', '/pose_following/start', 'std_srvs/srv/Trigger', '{}'], timeout=3)
        time.sleep(0.5)

        # Run tracking trial. This script should print JSON metrics to stdout.
        cmd = [
            'ros2', 'run', 'robot_planning', 'run_tracking_trial.py',
            '--ros-args',
            '-p', f'trial_index:={trial_index}'
        ]
        out = subprocess.check_output(cmd, text=True, timeout=15.0)
        metrics = json.loads(out.strip().splitlines()[-1])

        cost = compute_cost(metrics)
        save_trial(trial_index, params, metrics, cost)
        return cost

    except Exception as e:
        metrics = {'error': str(e), 'safety_fail': True}
        cost = 1e9
        save_trial(trial_index, params, metrics, cost)
        return cost
    finally:
        stop_process(proc)
        time.sleep(1.0)


def compute_cost(m):
    if m.get('safety_fail', False):
        return 1e9

    rmse = m['rmse']
    ramp_lag = m['ramp_lag']
    overshoot = m['overshoot']
    settling_time = m['settling_time']
    jitter = m['arrival_jitter']
    max_feedback_vel = m['max_feedback_vel']
    max_joint_vel = m['max_joint_vel']

    cost = (
        1.0 * rmse / 0.001
        + 2.0 * ramp_lag / 0.001
        + 30.0 * overshoot / 0.001
        + 0.5 * settling_time
        + 5.0 * jitter / 0.0002
    )

    if overshoot > 0.001:
        cost += 100000.0 + overshoot * 1e7

    # Tune these limits to match your safety policy.
    if max_feedback_vel > 0.8:
        cost += 100000.0
    if max_joint_vel > 5.0:
        cost += 100000.0

    return float(cost)


def save_trial(trial_index, params, metrics, cost):
    os.makedirs(os.path.dirname(RESULT_CSV), exist_ok=True)
    row = {'trial': trial_index, 'cost': cost}
    row.update(params)
    row.update(metrics)
    df = pd.DataFrame([row])
    if os.path.exists(RESULT_CSV):
        df.to_csv(RESULT_CSV, mode='a', index=False, header=False)
    else:
        df.to_csv(RESULT_CSV, index=False)


def objective(trial):
    # Stage 1 search space: speed + no overshoot
    params = {
        'zeta_pos': trial.suggest_float('zeta_pos', 0.65, 1.10),
        'k_pos_max': trial.suggest_float('k_pos_max', 150.0, 320.0),
        'max_cart_linear_acc': trial.suggest_float('max_cart_linear_acc', 0.8, 2.0),

        # Fixed values for stage 1
        'm_pos_min': 0.02,
        'm_pos_max': 0.12,
        'k_pos_min': 70.0,
        'adaptive_lambda': 12.0,
        'adaptive_alpha_pos': 40.0,
        'max_cart_linear_vel': 0.50,
    }
    return run_one_trial(params, trial.number)


def main():
    study = optuna.create_study(
        study_name='motomini_feedback_stage1',
        direction='minimize',
        storage='sqlite:///tuning_results/optuna.db',
        load_if_exists=True,
    )
    study.optimize(objective, n_trials=35)

    print('Best value:', study.best_value)
    print('Best params:', study.best_params)

    os.makedirs('tuning_results', exist_ok=True)
    with open('tuning_results/best_params_stage1.yaml', 'w') as f:
        yaml.safe_dump(study.best_params, f, sort_keys=False)


if __name__ == '__main__':
    main()
```

---

## 11. run_tracking_trial.py

File này nên là ROS2 node Python. Nó làm 4 việc:

1. subscribe feedback / target / velocity / joint state,
2. publish test trajectory,
3. lưu data vào memory,
4. in JSON metrics ở cuối.

Skeleton rất ngắn:

```python
#!/usr/bin/env python3
import json
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState


class TrackingTrial(Node):
    def __init__(self):
        super().__init__('run_tracking_trial')
        self.pose_pub = self.create_publisher(PoseStamped, '/motomini/target_pose', 10)
        self.vel_pub = self.create_publisher(Twist, '/motomini/target_vel', 10)

        self.create_subscription(Twist, '/motomini/feedback', self.feedback_cb, 50)
        self.create_subscription(Twist, '/motomini/feedback_vel', self.feedback_vel_cb, 50)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 50)

        self.t = []
        self.target_x = []
        self.feedback_x = []
        self.feedback_vx = []
        self.joint_vel_abs_max = []

        self.latest_feedback_x = None
        self.latest_feedback_vx = 0.0
        self.latest_joint_vel_abs_max = 0.0

        self.duration = 3.5
        self.ramp_start = 0.5
        self.ramp_end = 1.2
        self.dx = 0.03
        self.rate_hz = 50.0

        self.x0 = None
        self.y0 = None
        self.z0 = None

    def feedback_cb(self, msg):
        self.latest_feedback_x = msg.linear.x
        self.y_latest = msg.linear.y
        self.z_latest = msg.linear.z

    def feedback_vel_cb(self, msg):
        self.latest_feedback_vx = msg.linear.x

    def joint_cb(self, msg):
        if msg.velocity:
            self.latest_joint_vel_abs_max = max(abs(v) for v in msg.velocity)

    def wait_for_feedback(self):
        start = self.get_clock().now()
        while rclpy.ok() and self.latest_feedback_x is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.get_clock().now() - start).nanoseconds * 1e-9 > 3.0:
                raise RuntimeError('No /motomini/feedback received')
        self.x0 = self.latest_feedback_x
        self.y0 = getattr(self, 'y_latest', 0.0)
        self.z0 = getattr(self, 'z_latest', 0.2)

    def target_at(self, t):
        if t < self.ramp_start:
            s = 0.0
            vx = 0.0
        elif t < self.ramp_end:
            s = (t - self.ramp_start) / (self.ramp_end - self.ramp_start)
            vx = self.dx / (self.ramp_end - self.ramp_start)
        else:
            s = 1.0
            vx = 0.0
        return self.x0 + self.dx * s, vx

    def publish_target(self, x, vx):
        now = self.get_clock().now()

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = x
        pose.pose.position.y = self.y0
        pose.pose.position.z = self.z0
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        vel = Twist()
        vel.linear.x = vx
        self.vel_pub.publish(vel)

    def run(self):
        self.wait_for_feedback()
        t0 = self.get_clock().now()
        dt = 1.0 / self.rate_hz

        while rclpy.ok():
            now = self.get_clock().now()
            t = (now - t0).nanoseconds * 1e-9
            if t > self.duration:
                break

            x_target, vx_target = self.target_at(t)
            self.publish_target(x_target, vx_target)
            rclpy.spin_once(self, timeout_sec=dt)

            if self.latest_feedback_x is not None:
                self.t.append(t)
                self.target_x.append(x_target)
                self.feedback_x.append(self.latest_feedback_x)
                self.feedback_vx.append(self.latest_feedback_vx)
                self.joint_vel_abs_max.append(self.latest_joint_vel_abs_max)

        metrics = self.compute_metrics()
        print(json.dumps(metrics))

    def compute_metrics(self):
        t = np.array(self.t)
        target_x = np.array(self.target_x)
        feedback_x = np.array(self.feedback_x)
        feedback_vx = np.array(self.feedback_vx)
        joint_v = np.array(self.joint_vel_abs_max)

        if len(t) < 20:
            return {'safety_fail': True, 'reason': 'not enough samples'}

        error = target_x - feedback_x
        rmse = float(np.sqrt(np.mean(error ** 2)))

        ramp_mask = (t >= self.ramp_start) & (t <= self.ramp_end)
        ramp_lag = float(np.mean(np.abs(error[ramp_mask]))) if np.any(ramp_mask) else 1e3

        final_target = target_x[-1]
        overshoot = float(max(0.0, np.max(feedback_x - final_target)))

        abs_err = np.abs(error)
        threshold = 0.0005
        settling_time = self.duration
        for idx in np.where(t > self.ramp_end)[0]:
            if np.all(abs_err[idx:] < threshold):
                settling_time = float(t[idx] - self.ramp_end)
                break

        tail_mask = t > (self.duration - 0.5)
        jitter = float(np.std(feedback_x[tail_mask])) if np.any(tail_mask) else 1e3

        return {
            'safety_fail': False,
            'rmse': rmse,
            'ramp_lag': ramp_lag,
            'overshoot': overshoot,
            'settling_time': settling_time,
            'arrival_jitter': jitter,
            'max_feedback_vel': float(np.max(np.abs(feedback_vx))),
            'max_joint_vel': float(np.max(joint_v)),
        }


def main():
    rclpy.init()
    node = TrackingTrial()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Lưu ý: orientation đang set identity. Trong bản thật, nên copy orientation hiện tại từ feedback hoặc từ TF/FK để tránh tạo orientation jump.

---

## 12. Tối ưu từng phần

Không optimize tất cả ngay. Chạy theo stage.

### Stage 0: dry run không optimizer

Dùng baseline YAML:

```yaml
m_pos_min: 0.02
m_pos_max: 0.12
k_pos_min: 70.0
k_pos_max: 220.0
zeta_pos: 0.8
adaptive_lambda: 15.0
adaptive_alpha_pos: 40.0
max_cart_linear_vel: 0.50
max_cart_linear_acc: 1.2
```

Chạy 3 lần cùng trajectory. Nếu mỗi lần kết quả khác nhau quá nhiều, chưa nên chạy Bayesian Optimization. Cần kiểm tra noise, delay, `/joint_states`, controller arming, hoặc target publisher.

### Stage 1: tốc độ bám + không overshoot

Optimize:

```yaml
zeta_pos: 0.65 -> 1.10
k_pos_max: 150.0 -> 320.0
max_cart_linear_acc: 0.8 -> 2.0
```

Fixed:

```yaml
m_pos_min: 0.02
m_pos_max: 0.12
k_pos_min: 70.0
adaptive_lambda: 12.0
adaptive_alpha_pos: 40.0
max_cart_linear_vel: 0.50
```

Số trial:

```text
8 random initial trials
25-35 total trials
```

Chọn top 3 bộ, test lại mỗi bộ 3 lần. Lấy bộ có trung bình tốt nhất và không fail.

### Stage 2: near-target error và jitter

Giữ `zeta_pos`, `k_pos_max`, `max_cart_linear_acc` từ Stage 1.

Optimize:

```yaml
m_pos_min: 0.010 -> 0.035
m_pos_max: 0.08  -> 0.20
k_pos_min: 50.0  -> 120.0
```

Số trial:

```text
25-35 trials
```

Mục tiêu:

```text
final error thấp hơn
arrival jitter thấp hơn
không tăng overshoot
```

### Stage 3: adaptation profile

Giữ dynamics từ Stage 1 + 2.

Optimize:

```yaml
adaptive_lambda: 8.0  -> 25.0
adaptive_alpha_pos: 30.0 -> 60.0
max_cart_linear_vel: 0.35 -> 0.70
```

Số trial:

```text
20-30 trials
```

Mục tiêu:

```text
mode entry không giật
small target changes không chậm
ramp lag thấp
không vượt safety
```

### Stage 4: multi-axis validation

Không optimize nữa, chỉ validate:

```text
X +30 mm
Y +20 mm
Z +20 mm
X -30 mm
task trajectory thật
```

Nếu pass hết, lưu thành final YAML.

Nếu chỉ fail ở một hướng, có thể do kinematics / singularity / joint limit, không nhất thiết do tham số controller.

---

## 13. Search range an toàn đề xuất

Không cho optimizer đi quá rộng. Range đề xuất:

```yaml
m_pos_min:            [0.010, 0.035]
m_pos_max:            [0.08,  0.20]
k_pos_min:            [50.0,  120.0]
k_pos_max:            [150.0, 320.0]
zeta_pos:             [0.65,  1.10]
adaptive_lambda:      [8.0,   25.0]
adaptive_alpha_pos:   [30.0,  60.0]
max_cart_linear_vel:  [0.35,  0.70]
max_cart_linear_acc:  [0.8,   2.0]
```

Không cho optimizer dùng ban đầu:

```yaml
m_pos_min: 0.005
k_pos_max: 500.0
adaptive_lambda: 50.0
max_cart_linear_vel: 1.0
```

Đây là vùng rất aggressive, chỉ test sau khi có đủ confidence.

---

## 14. Lệnh chạy đề xuất

### 14.1. Cài dependency

```bash
pip install optuna pyyaml pandas numpy matplotlib
```

### 14.2. Build package

```bash
colcon build --packages-select robot_planning
source install/setup.bash
```

### 14.3. Test launch trial manually

```bash
ros2 launch robot_planning optimize_feedback_controller.launch.py \
  param_file:=/absolute/path/to/feedback_controller_opt_base.yaml
```

### 14.4. Test trajectory manually

```bash
ros2 run robot_planning run_tracking_trial.py
```

Nó nên in ra JSON kiểu:

```json
{"safety_fail": false, "rmse": 0.0012, "ramp_lag": 0.0021, "overshoot": 0.0001}
```

### 14.5. Chạy Bayesian Optimization Stage 1

```bash
ros2 run robot_planning bayes_optimize_feedback.py
```

Hoặc chạy Python trực tiếp:

```bash
python3 src/robot_planning/scripts/bayes_optimize_feedback.py
```

### 14.6. Xem kết quả

```bash
cat tuning_results/best_params_stage1.yaml
column -s, -t < tuning_results/trials.csv | less -S
```

---

## 15. Cách chọn bộ số cuối cùng

Không chọn duy nhất trial có cost thấp nhất một lần. Làm như sau:

```text
1. Sort trials theo cost.
2. Lấy top 5 trial.
3. Loại trial có overshoot gần threshold.
4. Test lại top 3 mỗi bộ 3 lần.
5. Lấy bộ có mean cost thấp nhất và std nhỏ nhất.
6. Validate trên multi-axis trajectory.
7. Ghi vào feedback_controller_final.yaml.
```

Tiêu chí final gợi ý:

```text
overshoot <= 0.5 mm
rmse <= 1.0 mm cho test 30 mm
arrival_jitter <= 0.2 mm
không joint velocity warning
không measured Cartesian velocity warning
không STATE_STOP
```

---

## 16. File YAML final mẫu sau tối ưu

Sau khi optimizer tìm được bộ tốt, tạo:

```text
config/feedback_controller_optimal.yaml
```

Ví dụ format:

```yaml
/**:
  ros__parameters:
    rate_hz: 50.0

    m_pos_min: 0.018
    m_pos_max: 0.13
    k_pos_min: 82.0
    k_pos_max: 245.0
    zeta_pos: 0.86

    m_ori_min: 0.02
    m_ori_max: 0.15
    k_ori_min: 2.0
    k_ori_max: 15.0
    zeta_ori: 1.0

    adaptive_lambda: 16.0
    adaptive_alpha_pos: 43.0
    adaptive_alpha_ori: 5.0

    max_cart_linear_vel: 0.55
    max_cart_angular_vel: 2.0
    max_cart_linear_acc: 1.35
    max_cart_angular_acc: 1.5

    w0: 0.001
    k0: 0.01
    enable_seed: false
    integrate_target_vel_to_pose: false
```

Các số trên chỉ là ví dụ format, không phải kết quả thật.

---

## 17. Checklist an toàn trước khi chạy trên robot thật

Trước khi bật optimizer trên real robot:

```text
[ ] Emergency stop sẵn sàng.
[ ] Test trajectory nhỏ: 5-10 mm trước.
[ ] Workspace không có vật cản.
[ ] Joint limits và velocity limits hoạt động.
[ ] Controller có thể /stop ngay.
[ ] Optimizer có timeout mỗi trial.
[ ] Nếu mất feedback hoặc node stop thì cost = 1e9.
[ ] Không để optimizer search range quá aggressive.
[ ] Lưu log từng trial.
```

Bắt đầu với:

```text
10 mm step-ramp
sau đó 20 mm
sau đó 30 mm
sau đó multi-axis
```

---

## 18. Tóm tắt workflow ngắn

```text
1. Thêm launch optimize_feedback_controller.launch.py.
2. Thêm feedback_controller_opt_base.yaml.
3. Thêm run_tracking_trial.py để publish quỹ đạo và tính metric.
4. Thêm bayes_optimize_feedback.py dùng Optuna.
5. Chạy dry run baseline 3 lần.
6. Stage 1 optimize zeta_pos, k_pos_max, max_cart_linear_acc.
7. Stage 2 optimize m_pos_min, m_pos_max, k_pos_min.
8. Stage 3 optimize adaptive_lambda, adaptive_alpha_pos, max_cart_linear_vel.
9. Test lại top 3 bộ nhiều lần.
10. Validate multi-axis và ghi feedback_controller_optimal.yaml.
```

Mục tiêu của optimizer không phải là tìm bộ nhanh nhất tuyệt đối, mà là tìm bộ:

```text
nhanh nhất trong vùng không overshoot, không jitter, không vượt safety.
```
