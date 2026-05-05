#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import time
import tkinter as tk
from threading import Thread

class LatencyMonitorNode(Node):
    def __init__(self):
        super().__init__('latency_monitor_node')

        # State tracking
        self.last_target_time = None
        
        self.planning_latencies = []
        self.transmit_latencies = []
        self.loop_frequencies = []
        
        self.last_joint_state_time = time.time()

        # Subscriptions
        # 1. Target inputs (trigger for planning)
        self.create_subscription(PoseStamped, '/motomini/target_pose', self.target_pose_cb, 10)
        self.create_subscription(PoseArray, '/target_poses', self.target_array_cb, 10)
        
        # 2. Planning output (result of planning)
        self.create_subscription(JointTrajectory, '/path_command', self.path_command_cb, 10)
        
        # 3. Hardware feedback (to measure loop frequency and potential transmission delay)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        self.get_logger().info("Latency Monitor Node Started. Waiting for messages...")

    def target_pose_cb(self, msg):
        self.last_target_time = self.get_clock().now()

    def target_array_cb(self, msg):
        if len(msg.poses) > 0:
            self.last_target_time = self.get_clock().now()

    def path_command_cb(self, msg):
        now = self.get_clock().now()
        
        # Calculation Latency: Time from last target input to path command output
        if self.last_target_time is not None:
            diff = now - self.last_target_time
            latency = diff.nanoseconds / 1e9
            self.planning_latencies.append(latency)
            if len(self.planning_latencies) > 50:
                self.planning_latencies.pop(0)
            self.last_target_time = None 

        # Transmission Latency: ROS 2 overhead (stamp vs arrival)
        # This measures how long the message took to "travel" from the publisher's now() to here.
        msg_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if msg_stamp.nanoseconds > 0:
            t_lat = (now - msg_stamp).nanoseconds / 1e9
            self.transmit_latencies.append(t_lat)
            if len(self.transmit_latencies) > 50:
                self.transmit_latencies.pop(0)

    def joint_state_cb(self, msg):
        now = time.time()
        dt = now - self.last_joint_state_time
        if dt > 0:
            self.loop_frequencies.append(1.0 / dt)
            if len(self.loop_frequencies) > 100:
                self.loop_frequencies.pop(0)
        self.last_joint_state_time = now

class LatencyGUI:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("MotoMini Latency Monitor")
        self.root.geometry("450x350")
        self.root.configure(bg='#f0f0f0')

        title_font = ("Helvetica", 14, "bold")
        value_font = ("Courier", 12, "bold")
        label_font = ("Helvetica", 10)

        tk.Label(self.root, text="MotoMini Latency Monitor", font=title_font, bg='#f0f0f0', fg='#333').pack(pady=10)

        # Planning Latency
        self.plan_frame = tk.Frame(self.root, bg='#fff', bd=1, relief=tk.RIDGE)
        self.plan_frame.pack(fill=tk.X, padx=20, pady=5)
        tk.Label(self.plan_frame, text="Planning Latency (Target -> /path_command)", font=label_font, bg='#fff').pack(anchor=tk.W, padx=5)
        self.plan_val = tk.Label(self.plan_frame, text="--- ms", font=value_font, bg='#fff', fg='#007bff')
        self.plan_val.pack(anchor=tk.E, padx=10)

        # Transmit Latency
        self.trans_frame = tk.Frame(self.root, bg='#fff', bd=1, relief=tk.RIDGE)
        self.trans_frame.pack(fill=tk.X, padx=20, pady=5)
        tk.Label(self.trans_frame, text="Internal ROS Latency (Stamp -> Arrival)", font=label_font, bg='#fff').pack(anchor=tk.W, padx=5)
        self.trans_val = tk.Label(self.trans_frame, text="--- ms", font=value_font, bg='#fff', fg='#28a745')
        self.trans_val.pack(anchor=tk.E, padx=10)

        # Feedback Freq
        self.freq_frame = tk.Frame(self.root, bg='#fff', bd=1, relief=tk.RIDGE)
        self.freq_frame.pack(fill=tk.X, padx=20, pady=5)
        tk.Label(self.freq_frame, text="Joint State Frequency", font=label_font, bg='#fff').pack(anchor=tk.W, padx=5)
        self.freq_val = tk.Label(self.freq_frame, text="--- Hz", font=value_font, bg='#fff', fg='#dc3545')
        self.freq_val.pack(anchor=tk.E, padx=10)

        # Stats
        self.stats_text = tk.StringVar(value="Avg Planning: ---\nMax Planning: ---")
        tk.Label(self.root, textvariable=self.stats_text, justify=tk.LEFT, bg='#f0f0f0', font=label_font).pack(pady=10)

        self.update_gui()

    def update_gui(self):
        if self.node.planning_latencies:
            last_p = self.node.planning_latencies[-1] * 1000
            avg_p = (sum(self.node.planning_latencies) / len(self.node.planning_latencies)) * 1000
            max_p = max(self.node.planning_latencies) * 1000
            self.plan_val.config(text=f"{last_p:.2f} ms")
            self.stats_text.set(f"Avg Planning: {avg_p:.2f} ms\nMax Planning: {max_p:.2f} ms")
        
        if self.node.transmit_latencies:
            avg_t = (sum(self.node.transmit_latencies) / len(self.node.transmit_latencies)) * 1000
            self.trans_val.config(text=f"{avg_t:.2f} ms")

        if self.node.loop_frequencies:
            avg_f = sum(self.node.loop_frequencies) / len(self.node.loop_frequencies)
            self.freq_val.config(text=f"{avg_f:.1f} Hz")
            if avg_f > 15: self.freq_val.config(fg="green")
            elif avg_f > 5: self.freq_val.config(fg="orange")
            else: self.freq_val.config(fg="red")

        self.root.after(100, self.update_gui)

def main():
    rclpy.init()
    node = LatencyMonitorNode()
    
    thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    gui = LatencyGUI(node)
    try:
        gui.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
