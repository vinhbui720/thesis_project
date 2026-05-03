import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, WrenchStamped
from rcl_interfaces.msg import Log
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import time
import math
import numpy as np
import json
import threading

class CollisionTrialNode(Node):
    def __init__(self):
        super().__init__('collision_trial_node')

        self.declare_parameter('trial_index', 0)
        self.declare_parameter('duration', 15.0)

        self.trial_index = self.get_parameter('trial_index').value
        self.duration = self.get_parameter('duration').value

        self.declare_parameter('start_x', 0.25)
        self.declare_parameter('start_y', -0.15)
        self.declare_parameter('start_z', 0.1)
        self.declare_parameter('end_y', 0.1)
        self.declare_parameter('target_velocity', 0.15)
        self.declare_parameter('warmup_time', 1.0)
        self.declare_parameter('stop_distance', 0.001)

        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.start_z = float(self.get_parameter('start_z').value)
        self.end_y = float(self.get_parameter('end_y').value)
        self.target_velocity = float(self.get_parameter('target_velocity').value)
        self.warmup_time = float(self.get_parameter('warmup_time').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)

        self.target_data = []
        self.feedback_data = []
        self.feedback_vel_data = []
        self.wrench_data = []
        self.distance_data = []

        self.recording_active = False
        self.ready_timeout = 40.0
        self.start_time = time.time()
        self.recording_start_time = None
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.initial_pose = None
        self.safety_fail = False
        self.safety_reason = ""

        self.rosout_sub = self.create_subscription(Log, '/rosout', self.rosout_cb, 100)
        self.feedback_sub = self.create_subscription(Twist, '/motomini/feedback', self.feedback_cb, 10)
        self.feedback_vel_sub = self.create_subscription(Twist, '/motomini/feedback_vel', self.feedback_vel_cb, 10)
        self.wrench_sub = self.create_subscription(WrenchStamped, '/motomini/collision_wrench', self.wrench_cb, 10)
        self.distance_sub = self.create_subscription(Float64, '/motomini/collision_distance', self.distance_cb, 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_topic = '/motomini/target_pose'
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)

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
        
        if current_time < warmup_time:
            progress = 0.0 if not self.recording_active else current_time / max(warmup_time, 1e-9)
            s_curve = 0.5 * (1.0 - math.cos(math.pi * progress))
            target_x = self.initial_pose.translation.x + (start_x - self.initial_pose.translation.x) * s_curve
            target_y = self.initial_pose.translation.y + (start_y - self.initial_pose.translation.y) * s_curve
            target_z = self.initial_pose.translation.z + (start_z - self.initial_pose.translation.z) * s_curve
        else:
            target_x = start_x
            target_z = start_z
            move_time = current_time - warmup_time
            target_y = start_y + vel * move_time
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
        if not self.recording_active: return
        t = time.time() - self.recording_start_time
        self.feedback_data.append((t, msg.linear.x, msg.linear.y, msg.linear.z))

    def feedback_vel_cb(self, msg):
        if not self.recording_active: return
        t = time.time() - self.recording_start_time
        self.feedback_vel_data.append((t, msg.linear.x, msg.linear.y, msg.linear.z))

    def wrench_cb(self, msg):
        if not self.recording_active: return
        t = time.time() - self.recording_start_time
        fx, fy, fz = msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z
        self.wrench_data.append((t, fx, fy, fz))

    def distance_cb(self, msg):
        if not self.recording_active: return
        t = time.time() - self.recording_start_time
        self.distance_data.append((t, msg.data))

    def run_trial(self):
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        try:
            while rclpy.ok() and not self.recording_active:
                if time.time() - self.start_time > self.ready_timeout:
                    self.get_logger().error('Timeout waiting for robot to enter STATE_POSE_FOLLOW')
                    break
                time.sleep(0.1)

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
            executor.shutdown()
            spin_thread.join(timeout=1.0)
            metrics = self.compute_metrics()
            print(json.dumps(metrics))

    def compute_metrics(self):
        if getattr(self, 'safety_fail', False):
            return {'safety_fail': True, 'cost': 1e9, 'error': getattr(self, 'safety_reason', 'Unknown error')}

        if len(self.distance_data) < 10 or len(self.wrench_data) < 10 or len(self.feedback_data) < 10:
            return {'safety_fail': True, 'cost': 1e9, 'error': 'Not enough data points'}

        distances = np.array([d[1] for d in self.distance_data])
        forces = np.array([[w[1], w[2], w[3]] for w in self.wrench_data])
        force_mags = np.linalg.norm(forces, axis=1)
        
        fvels = np.array([[v[1], v[2], v[3]] for v in self.feedback_vel_data])
        fvel_mags = np.linalg.norm(fvels, axis=1) if len(fvels) > 0 else np.array([0.0])
        
        t_fb = np.array([d[0] for d in self.feedback_data])
        x_fb = np.array([d[1] for d in self.feedback_data])
        y_fb = np.array([d[2] for d in self.feedback_data])
        z_fb = np.array([d[3] for d in self.feedback_data])

        min_dist = float(np.min(distances))
        max_force = float(np.max(force_mags))
        
        # Calculate penetration depth
        penetration_depth = self.stop_distance - min_dist if min_dist < self.stop_distance else 0.0
        
        # Force jitter
        contact_mask = distances < 0.05
        force_jitter = 0.0
        if np.sum(contact_mask) > 5:
            forces_in_contact = force_mags[contact_mask]
            force_diffs = np.diff(forces_in_contact)
            force_jitter = float(np.std(force_diffs))
            
        # Velocity spikes
        velocity_spike = 0.0
        if len(fvel_mags) > 5:
            vel_diffs = np.abs(np.diff(fvel_mags))
            velocity_spike = float(np.max(vel_diffs))
            
        # Progress along Y (did it get stuck behind the obstacle?)
        max_y_reached = float(np.max(y_fb))
        progress_error = max(0.0, self.end_y - max_y_reached)
        
        # Recovery phase (last 3 seconds): measure jitter and final error
        final_mask = t_fb > (self.duration - 3.0)
        if np.sum(final_mask) > 10:
            y_final = y_fb[final_mask]
            x_final = x_fb[final_mask]
            arrival_jitter = float(np.std(y_final) + np.std(x_final))
            final_error = float(np.mean(np.abs(y_final - self.end_y) + np.abs(x_final - self.start_x)))
        else:
            arrival_jitter = 1.0
            final_error = 1.0

        # Khởi tạo target arrays để tính sai số động
        target_y_array = np.zeros_like(t_fb)
        target_x_array = np.full_like(t_fb, self.start_x)
        move_time = abs(self.end_y - self.start_y) / self.target_velocity
        t_stop = self.warmup_time + move_time
        
        for i, t in enumerate(t_fb):
            if t < self.warmup_time:
                target_y_array[i] = self.start_y
            else:
                t_move = t - self.warmup_time
                if t_move > move_time:
                    target_y_array[i] = self.end_y
                else:
                    target_y_array[i] = self.start_y + np.sign(self.end_y - self.start_y) * self.target_velocity * t_move
                    
        # Sai số động trong suốt quá trình
        tracking_error_y = np.abs(y_fb - target_y_array)
        tracking_error_x = np.abs(x_fb - target_x_array)
        dynamic_tracking_error = float(np.sqrt(np.mean(tracking_error_y**2 + tracking_error_x**2)))
        
        # Sai số TẠI THỜI ĐIỂM target DỪNG (ép robot bắt được target trước khi target dừng)
        if len(t_fb) > 0:
            idx_stop = np.argmin(np.abs(t_fb - t_stop))
            error_at_stop = float(tracking_error_y[idx_stop] + tracking_error_x[idx_stop])
        else:
            error_at_stop = 1.0

        return {
            'safety_fail': False,
            'min_distance': min_dist,
            'penetration_depth': penetration_depth,
            'force_jitter': force_jitter,
            'velocity_spike': velocity_spike,
            'max_force': max_force,
            'progress_error': progress_error,
            'arrival_jitter': arrival_jitter,
            'final_error': final_error,
            'dynamic_tracking_error': dynamic_tracking_error,
            'error_at_stop': error_at_stop
        }

def main(args=None):
    rclpy.init(args=args)
    node = CollisionTrialNode()
    try:
        node.run_trial()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
