import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import time
import math
import numpy as np
import json
import threading

class TrackingTrialNode(Node):
    def __init__(self):
        super().__init__('tracking_trial_node')

        self.declare_parameter('trial_index', 0)
        self.declare_parameter('duration', 10.0)

        self.trial_index = self.get_parameter('trial_index').value
        self.duration = self.get_parameter('duration').value

        self.declare_parameter('start_x', 0.25)
        self.declare_parameter('start_y', -0.15)
        self.declare_parameter('start_z', 0.15)
        self.declare_parameter('end_y', 0.1)
        self.declare_parameter('target_velocity', 0.1)
        self.declare_parameter('warmup_time', 1.0)

        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.start_z = float(self.get_parameter('start_z').value)
        self.end_y = float(self.get_parameter('end_y').value)
        self.target_velocity = float(self.get_parameter('target_velocity').value)
        self.warmup_time = float(self.get_parameter('warmup_time').value)

        self.target_data = []
        self.feedback_data = []
        self.joint_vel_data = []
        self.feedback_vel_data = []

        self.recording_active = False
        self.ready_timeout = 40.0 # Wait up to 40 seconds for robot to be ready
        self.start_time = time.time()
        self.recording_start_time = None
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.initial_pose = None
        self.safety_fail = False
        self.safety_reason = ""

        self.rosout_sub = self.create_subscription(Log, '/rosout', self.rosout_cb, 100)
        self.feedback_sub = self.create_subscription(Twist, '/motomini/feedback', self.feedback_cb, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.feedback_vel_sub = self.create_subscription(Twist, '/motomini/feedback_vel', self.feedback_vel_cb, 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_topic = '/motomini/target_pose'
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)

        # Timer to publish pose at 50Hz
        self.timer_pose = self.create_timer(0.02, self.publish_pose)

    def publish_pose(self):

        if self.initial_pose is None:
            try:
                t = self.tf_buffer.lookup_transform('world', 'magnetic_link', rclpy.time.Time())
                self.initial_pose = t.transform
            except Exception:
                return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        current_time = (time.time() - self.recording_start_time) if self.recording_active else 0.0

        start_x = self.start_x
        start_y = self.start_y
        start_z = self.start_z
        end_y = self.end_y
        vel = self.target_velocity
        warmup_time = self.warmup_time
        
        # S-curve interpolation from initial pose to start pose during warmup
        if current_time < warmup_time:
            if not self.recording_active:
                progress = 0.0
            else:
                progress = current_time / max(warmup_time, 1e-9)
            s_curve = 0.5 * (1.0 - math.cos(math.pi * progress))
            target_x = self.initial_pose.translation.x + (start_x - self.initial_pose.translation.x) * s_curve
            target_y = self.initial_pose.translation.y + (start_y - self.initial_pose.translation.y) * s_curve
            target_z = self.initial_pose.translation.z + (start_z - self.initial_pose.translation.z) * s_curve
        else:
            target_x = start_x
            target_z = start_z
            move_time = current_time - warmup_time
            dist = vel * move_time
            target_y = start_y + dist
            if target_y > end_y:
                target_y = end_y

        msg.pose.position.x = target_x
        msg.pose.position.y = target_y
        msg.pose.position.z = target_z

        msg.pose.orientation = self.initial_pose.rotation

        self.pose_pub.publish(msg)
        
        if self.recording_active:
            t = time.time() - self.recording_start_time
            self.target_data.append((t, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = msg.header.frame_id
        tf_msg.child_frame_id = 'tracking_frame'

        tf_msg.transform.translation.x = msg.pose.position.x
        tf_msg.transform.translation.y = msg.pose.position.y
        tf_msg.transform.translation.z = msg.pose.position.z
        tf_msg.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)

    def rosout_cb(self, msg):
        if msg.name == 'motomini_feedback_stream':
            if not self.recording_active:
                if 'STATE_POSE_FOLLOW: tracking' in msg.msg:
                    self.get_logger().info('Robot is now in STATE_POSE_FOLLOW. Starting recording.')
                    self.recording_active = True
                    self.recording_start_time = time.time()
            
            # Continuously monitor for errors
            msg_text = msg.msg
            if 'limit → holding' in msg_text or 'limit -> holding' in msg_text:
                self.safety_fail = True
                self.safety_reason = 'Joint position limit hit'
            elif 'safety exceeded' in msg_text or 'exceeds limit' in msg_text:
                self.safety_fail = True
                self.safety_reason = 'Velocity safety exceeded'
            elif 'Pose input timeout' in msg_text:
                self.safety_fail = True
                self.safety_reason = 'Pose input timeout'

    def feedback_cb(self, msg):
        if not self.recording_active:
            return
        t = time.time() - self.recording_start_time
        self.feedback_data.append((t, msg.linear.x, msg.linear.y, msg.linear.z))

    def joint_cb(self, msg):
        if not self.recording_active:
            return
        t = time.time() - self.recording_start_time
        self.joint_vel_data.append((t, msg.velocity))

    def feedback_vel_cb(self, msg):
        if not self.recording_active:
            return
        t = time.time() - self.recording_start_time
        self.feedback_vel_data.append((t, msg.linear.x, msg.linear.y, msg.linear.z))

    def run_trial(self):
        # Spin the node in a background thread to ensure timers run smoothly
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # Wait for recording to become active
        try:
            while rclpy.ok() and not self.recording_active:
                if time.time() - self.start_time > self.ready_timeout:
                    self.get_logger().error('Timeout waiting for robot to enter STATE_POSE_FOLLOW')
                    break
                time.sleep(0.1)

            # Once active, record for 'duration' seconds
            if self.recording_active:
                while rclpy.ok() and (time.time() - self.recording_start_time) < self.duration:
                    if getattr(self, 'safety_fail', False):
                        self.get_logger().error(f"Trial aborted due to safety fail: {self.safety_reason}")
                        break
                    time.sleep(0.1)

        except Exception as e:
            self.safety_fail = True
            self.safety_reason = f"Exception: {str(e)}"
        finally:
            # Stop executor
            executor.shutdown()
            spin_thread.join(timeout=1.0)

            # Compute metrics
            metrics = self.compute_metrics()
            print(json.dumps(metrics))

    def compute_metrics(self):
        if getattr(self, 'safety_fail', False):
            return {'safety_fail': True, 'cost': 1e9, 'error': getattr(self, 'safety_reason', 'Unknown error')}

        if len(self.target_data) < 10 or len(self.feedback_data) < 10:
            return {'safety_fail': True, 'cost': 1e9, 'error': f'Not enough data points: target={len(self.target_data)}, feedback={len(self.feedback_data)}, joints={len(self.joint_vel_data)}, fvel={len(self.feedback_vel_data)}'}

        t_target = np.array([d[0] for d in self.target_data])
        y_target = np.array([d[2] for d in self.target_data])
        t_fb = np.array([d[0] for d in self.feedback_data])
        y_fb = np.array([d[2] for d in self.feedback_data])

        start_y = self.start_y
        end_y = self.end_y
        vel = self.target_velocity
        t_start_motion = self.warmup_time
        t_end_motion = self.warmup_time + abs(end_y - start_y) / max(abs(vel), 1e-9)

        # Phase 1: Approach (t=1.0 to t_end_motion)
        # Rise time (time to reach 90% of distance or end_y - 0.025)
        rise_threshold = start_y + 0.9 * (end_y - start_y)
        rise_times = t_fb[(t_fb > t_start_motion) & (y_fb >= rise_threshold)]
        rise_time = (rise_times[0] - t_start_motion) if len(rise_times) > 0 else 10.0

        # Lag during motion
        motion_mask = (t_fb >= t_start_motion) & (t_fb <= t_end_motion)
        y_target_interp = np.interp(t_fb, t_target, y_target)
        lag_error = np.mean(np.abs(y_target_interp[motion_mask] - y_fb[motion_mask])) if np.sum(motion_mask) > 0 else 1.0

        limits = np.array([2.26, 1.57, 2.61, 3.49, 3.49, 6.28])
        vel_violation = 0.0
        max_joint_vel_ratio = 0.0

        if len(self.joint_vel_data) > 0:
            jv = np.array([d[1] for d in self.joint_vel_data], dtype=float)
            if jv.ndim == 2 and jv.shape[1] == 6:
                max_vels = np.max(np.abs(jv), axis=0)
                ratios = max_vels / limits
                max_joint_vel_ratio = float(np.max(ratios))
                if max_joint_vel_ratio > 0.95:
                    vel_violation = max_joint_vel_ratio

        J_approach = 2.0 * rise_time + 3.0 * lag_error + 50.0 * vel_violation

        # Phase 2: Near Target (t > t_end_motion)
        near_mask = t_fb > t_end_motion
        if np.sum(near_mask) > 0:
            y_near = y_fb[near_mask]
            overshoot = np.max(y_near) - end_y
            if overshoot < 0: overshoot = 0.0
            
            # Settling time (within 2mm)
            t_near = t_fb[near_mask]
            unsettled = t_near[np.abs(y_near - end_y) > 0.002]
            settling_time = (unsettled[-1] - t_end_motion) if len(unsettled) > 0 else 0.0
            
            # Final error (last 2 seconds)
            final_mask = t_fb > (self.duration - 2.0)
            final_error = np.mean(np.abs(y_fb[final_mask] - end_y)) if np.sum(final_mask) > 0 else 1.0
        arrival_jitter = 0.0
        if np.sum(near_mask) > 10:
            y_near = y_fb[near_mask]
            tail = y_near[-min(len(y_near), 100):]
            arrival_jitter = float(np.std(tail))
        else:
            overshoot = 1.0
            settling_time = 10.0
            final_error = 1.0
            arrival_jitter = 1.0

        J_near = 50.0 * overshoot + 2.0 * settling_time + 5.0 * final_error

        # Phase 3: Tracking
        tracking_rmse = np.sqrt(np.mean((y_target_interp[motion_mask] - y_fb[motion_mask])**2)) if np.sum(motion_mask) > 0 else 1.0
        phase_lag = lag_error / vel
        
        max_feedback_vel = 0.0
        velocity_noise = 0.0
        if len(self.feedback_vel_data) > 0:
            fvel = np.array([[d[1], d[2], d[3]] for d in self.feedback_vel_data], dtype=float)
            speed = np.linalg.norm(fvel, axis=1)
            max_feedback_vel = float(np.max(speed))
            
            y_vels = fvel[:, 1]
            if len(y_vels) > 10:
                velocity_noise = np.mean(np.abs(np.diff(y_vels)))

        J_tracking = 5.0 * tracking_rmse + 3.0 * phase_lag + 2.0 * velocity_noise

        # Total Cost
        J_total = J_approach + J_near + J_tracking

        # Reject constraints
        if overshoot > 0.001 or vel_violation > 1.0:
            return {'safety_fail': True, 'cost': 1e9, 'error': f"Constraints violated: overshoot={overshoot:.4f}, vel_ratio={vel_violation:.2f}"}

        return {
            'safety_fail': False,
            'cost': float(J_total),

            'J_approach': float(J_approach),
            'J_near': float(J_near),
            'J_tracking': float(J_tracking),

            'rmse': float(tracking_rmse),
            'ramp_lag': float(lag_error),
            'overshoot': float(overshoot),
            'arrival_jitter': float(arrival_jitter),
            'rise_time': float(rise_time),
            'settling_time': float(settling_time),
            'final_error': float(final_error),

            'max_feedback_vel': float(max_feedback_vel),
            'max_joint_vel_ratio': float(max_joint_vel_ratio),
            'vel_violation': float(vel_violation),
        }

def main(args=None):
    rclpy.init(args=args)
    node = TrackingTrialNode()
    try:
        node.run_trial()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
