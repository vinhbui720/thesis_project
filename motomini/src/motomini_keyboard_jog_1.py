#!/usr/bin/env python3
"""
motomini_keyboard_jog.py
Keyboard jog controller for MotoMini via /joint_command (POINT_STREAMING).

Design principles (derived from MotoPlus MotionServer.c source):
  - sequence=0  →  InitTrajPointFull: seed must match servo command pos within 10 pulses.
                   Sent ONCE at the start of every jog session. No motion yet.
  - sequence>0  →  AddTrajPointFull: BUSY reply = controller still interpolating last point.
                   Streamer automatically retries the same point; point is NOT lost.
  - time field  →  absolute ms from session start, used by cubic interpolator as
                   (endTime - startTime). Keep increments small (~40 ms = 1 publish interval).
  - validFields = 0x07  →  time + position + velocity must all be set.
  - command_positions must NEVER be more than a tiny lead ahead of actual (latency guard).
    If they are, the NEXT session's InitTrajPointFull will fail with INVALID_DATA_START_POS.

Latency strategy (WiFi / Ethernet):
  - Publish at 25 Hz (40 ms interval). Controller processes each point in ~24 ms (3x8 ms
    interpolPeriod). 40 ms > 24 ms -> BUSY replies are rare.
  - command_positions is clamped to within MAX_LEAD of actual_positions so the
    streaming buffer can never run away from the real robot regardless of latency.
  - On each cycle we publish: target = actual + speed_direction * DT.
    This is directly latency-tolerant: if the network is slow, actual just catches up.

Keys:
  1 2 3 4 5 6  ->  jog joint 1-6 forward  (+)
  q w e r t y  ->  jog joint 1-6 reverse  (-)
  -            ->  decrease speed
  =            ->  increase speed
  SPACE        ->  stop (release jog)
  ESC / Ctrl+C ->  quit
"""

import sys
import curses
import threading
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PUBLISH_RATE    = 25       # Hz  -- 40 ms between points; safely above ~24 ms controller
                            #       processing time (3 x 8 ms interpolPeriod)
SPEED_INITIAL   = 0.20     # rad/s
SPEED_MIN       = 0.02
SPEED_MAX       = 1.0
SPEED_STEP      = 0.02

# How far ahead of actual_position the command is allowed to run.
# 0.10 rad bounds the buffer to ~5.7 degrees. Prevents lingering while absorbing network lag.
MAX_LEAD        = 0.10     # rad

# dt we assume for position integration (== 1/PUBLISH_RATE).
# We do NOT use measured dt to avoid integrating runaway positions when a loop
# iteration is accidentally long (e.g. during BUSY retries).
DT              = 1.0 / PUBLISH_RATE   # 0.04 s

# Physical joint limits from URDF: (lower rad, upper rad, max_vel rad/s)
JOINT_LIMITS = [
    (-2.9670,  2.9670, 5.4977),   # J1 (s)
    (-1.4835,  1.5707, 5.4977),   # J2 (l)
    (-0.8726,  1.5707, 7.3304),   # J3 (u)
    (-2.4434,  2.4434, 10.4719),  # J4 (r)
    (-0.5235,  3.6651, 10.4719),  # J5 (b)
    (-6.2831,  6.2831, 10.4719),  # J6 (t)
]

FORWARD_KEYS = list("123456")
REVERSE_KEYS = list("qwerty")

STATE_IDLE     = 0   # not streaming; waiting for keypress
STATE_SEED     = 1   # one cycle: send seq=0 seed to InitTrajPointFull
STATE_JOGGING  = 2   # streaming motion points (seq=1,2,3,...)


class KeyboardJogger:
    def __init__(self):
        rospy.init_node("motomini_keyboard_jog", anonymous=False)

        # /joint_command  ->  POINT_STREAMING  (jointCommandCB)
        self.pub = rospy.Publisher("/joint_command", JointTrajectory, queue_size=5)
        # /joint_path_command  ->  used only to arm trajectory mode at startup
        self.init_pub = rospy.Publisher("/joint_path_command", JointTrajectory, queue_size=1)

        self.state_lock = threading.Lock()

        rospy.loginfo("Waiting for /motomini_joint_states ...")
        seed_msg = rospy.wait_for_message("/motomini_joint_states", JointState, timeout=10.0)

        self.joint_names  = list(seed_msg.name)
        self.n_joints     = len(self.joint_names)

        with self.state_lock:
            self.actual_positions  = list(seed_msg.position[:self.n_joints])
            self.actual_velocities = list(seed_msg.velocity[:self.n_joints]) \
                                     if len(seed_msg.velocity) >= self.n_joints \
                                     else [0.0] * self.n_joints

        # command_positions tracks the last commanded waypoint.
        # Initialised == actual so the seed check always passes.
        self.command_positions = list(self.actual_positions)

        self.speed            = SPEED_INITIAL
        # Latched jog command: set by keypress, cleared only by SPACE or same-key toggle.
        # This avoids the ~500 ms OS keyboard-repeat gap that made the robot stop
        # after one cycle when holding a key.
        self.latched_velocity = [0.0] * self.n_joints   # persists until SPACE/toggle
        self.desired_velocity = [0.0] * self.n_joints   # what we actually command

        self.state            = STATE_IDLE
        self.session_seq      = 0    # streaming_sequence within current jog session
        self.stream_time_sec  = 0.0  # monotonically increasing time within session

        self.joint_sub = rospy.Subscriber(
            "/motomini_joint_states", JointState,
            self._joint_state_cb, queue_size=1
        )
        rospy.loginfo("Seeded from: %s", [f"{p:.4f}" for p in self.actual_positions])
        self._arm_traj_mode()

    # ------------------------------------------------------------------
    def _arm_traj_mode(self):
        """Send a hold-in-place trajectory to arm the trajectory mode job."""
        rospy.loginfo("Arming trajectory mode ...")
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names  = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions       = list(self.actual_positions)
        pt.velocities      = [0.0] * self.n_joints
        pt.time_from_start = rospy.Duration(0.5)
        msg.points = [pt]
        rospy.sleep(0.5)
        self.init_pub.publish(msg)
        rospy.loginfo("Arm pose sent.")
        rospy.sleep(1.0)

    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg):
        if len(msg.position) < self.n_joints:
            return
        with self.state_lock:
            self.actual_positions  = list(msg.position[:self.n_joints])
            self.actual_velocities = list(msg.velocity[:self.n_joints]) \
                                     if len(msg.velocity) >= self.n_joints \
                                     else [0.0] * self.n_joints

    # ------------------------------------------------------------------
    def _send(self, positions, velocities, time_sec):
        """Publish a single JointTrajectoryPoint to /joint_command.

        jointCommandCB on the controller reads msg->points[0] only, so we
        always publish exactly ONE point per message.
        validFields will be 0x07 (time+pos+vel) because both lists are non-empty.
        """
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names  = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions       = list(positions)
        pt.velocities      = list(velocities)
        pt.time_from_start = rospy.Duration.from_sec(time_sec)
        msg.points = [pt]
        self.pub.publish(msg)

    # ------------------------------------------------------------------
    def run(self, stdscr):
        curses.cbreak()
        curses.noecho()
        stdscr.nodelay(True)
        stdscr.keypad(True)

        rate = rospy.Rate(PUBLISH_RATE)

        while not rospy.is_shutdown():
            # ---- Read all pending keystrokes this cycle ---------------
            # KEY LATCH MODEL: pressing a motion key latches that joint/direction
            # and keeps it active until SPACE is pressed or the same key is pressed
            # again (toggle off). This avoids depending on OS key-repeat timing
            # (~500 ms gap between first press and repeats) which caused the robot
            # to stop after every single cycle.

            try:
                while True:
                    ch = stdscr.getch()
                    if ch == -1:
                        break
                    if ch == 27:      # ESC
                        return
                    if ch < 32 or ch > 126:
                        continue

                    c = chr(ch)
                    if c in FORWARD_KEYS:
                        idx = FORWARD_KEYS.index(c)
                        self.latched_velocity = [0.0] * self.n_joints
                        self.latched_velocity[idx] = self.speed
                    elif c in REVERSE_KEYS:
                        idx = REVERSE_KEYS.index(c)
                        self.latched_velocity = [0.0] * self.n_joints
                        self.latched_velocity[idx] = -self.speed
                    elif c == "=":
                        self.speed = min(SPEED_MAX, round(self.speed + SPEED_STEP, 4))
                        # update magnitude of any active latch
                        for i in range(self.n_joints):
                            if self.latched_velocity[i] > 0:
                                self.latched_velocity[i] = self.speed
                            elif self.latched_velocity[i] < 0:
                                self.latched_velocity[i] = -self.speed
                    elif c == "-":
                        self.speed = max(SPEED_MIN, round(self.speed - SPEED_STEP, 4))
                        for i in range(self.n_joints):
                            if self.latched_velocity[i] > 0:
                                self.latched_velocity[i] = self.speed
                            elif self.latched_velocity[i] < 0:
                                self.latched_velocity[i] = -self.speed
                    elif c == " ":
                        self.latched_velocity = [0.0] * self.n_joints
            except Exception:
                pass

            self.desired_velocity = list(self.latched_velocity)

            is_moving = any(abs(v) > 1e-9 for v in self.desired_velocity)

            # Snapshot actual state (protected read)
            with self.state_lock:
                actual_pos = list(self.actual_positions)
                actual_vel = list(self.actual_velocities) 

            # ---- State Machine Transitions ----------------------------
            # Separating transitions from execution ensures seamless resumption.
            if self.state == STATE_IDLE:
                if is_moving:
                    self.session_seq     = 0
                    self.stream_time_sec = 0.0
                    self.state           = STATE_SEED
                else:
                    # Keep command_positions anchored to actual while idle 
                    # so the next seed check always passes.
                    self.command_positions = list(actual_pos)
                    self.latched_velocity = [0.0] * self.n_joints

            elif self.state == STATE_JOGGING:
                # FIX: We must NOT transition back to STATE_IDLE and stop sending data.
                # If the data stream stops, the Yaskawa controller's trajectory job will 
                # time out and drop out of streaming mode, ignoring any future key presses.
                # We simply remain in STATE_JOGGING and naturally stream 0.0 velocities.
                pass

            # ---- State Machine Execution ------------------------------
            if self.state == STATE_SEED:
                self._send(self.command_positions, [0.0] * self.n_joints, 0.0)
                self.session_seq     = 1
                self.stream_time_sec = DT   # first real point arrives in one DT
                self.state           = STATE_JOGGING

            elif self.state == STATE_JOGGING:
                new_cmd = list(self.command_positions)
                new_vel = [0.0] * self.n_joints

                for i in range(self.n_joints):
                    v = self.desired_velocity[i]

                    # clamp to joint velocity limit
                    max_v = JOINT_LIMITS[i][2]
                    v = max(-max_v, min(max_v, v))

                    new_cmd[i] += v * DT

                    # soft position limit
                    lo, hi = JOINT_LIMITS[i][0], JOINT_LIMITS[i][1]
                    new_cmd[i] = max(lo, min(hi, new_cmd[i]))

                    # latency lead clamp: command can be at most MAX_LEAD
                    # ahead of actual. Bounds the buffer regardless of WiFi lag.
                    lead = new_cmd[i] - actual_pos[i]
                    if abs(lead) > MAX_LEAD:
                        new_cmd[i] = actual_pos[i] + MAX_LEAD * (1.0 if lead > 0 else -1.0)
                        # Mathematically safe velocity reduction to maintain continuity 
                        # without causing an ALARM 4414 jump.
                        v = (new_cmd[i] - self.command_positions[i]) / DT

                    new_vel[i] = v

                self.command_positions = new_cmd
                self._send(self.command_positions, new_vel, self.stream_time_sec)
                self.session_seq     += 1
                self.stream_time_sec += DT

            # ---- UI ---------------------------------------------------
            try:
                state_str = "IDLE   "
                if self.state == STATE_SEED:
                    state_str = "SEEDING"
                elif self.state == STATE_JOGGING:
                    state_str = "JOGGING" if is_moving else "HOLDING"
                
                stdscr.erase()
                
                # Header & Global State
                stdscr.addstr(0, 0, "=== MotoMini Keyboard Jog ===")
                stdscr.addstr(2, 0, f"System State : [ {state_str} ]")
                stdscr.addstr(3, 0, f"Target Speed : {self.speed:.3f} rad/s")
                
                # Controls Guide
                stdscr.addstr(5, 0, "Controls:")
                stdscr.addstr(6, 0, "  [1-6] Fwd   |  [q-y] Rev   |  [SPACE] Stop All")
                stdscr.addstr(7, 0, "  [ = ] Fast  |  [ - ] Slow  |  [ ESC ] Quit")
                
                # Data Table Header
                stdscr.addstr(9,  0, "Joint | Keys |   Pos (Act / Cmd)   |   Vel (Act / Cmd)   | Status")
                stdscr.addstr(10, 0, "------+------+---------------------+---------------------+---------")
                
                # Data Rows
                for i in range(self.n_joints):
                    j_stat = "---"
                    if self.desired_velocity[i] > 0:
                        j_stat = ">>> FWD"
                    elif self.desired_velocity[i] < 0:
                        j_stat = "<<< REV"
                        
                    stdscr.addstr(11 + i, 0,
                        f"  J{i+1}  | {FORWARD_KEYS[i]}/{REVERSE_KEYS[i]}  "
                        f"|  {actual_pos[i]:+.4f} / {self.command_positions[i]:+.4f}  "
                        f"|  {actual_vel[i]:+.3f} / {self.desired_velocity[i]:+.3f}  "
                        f"| {j_stat}"
                    )
                stdscr.refresh()
            except curses.error:
                pass

            rate.sleep()


def main():
    try:
        jogger = KeyboardJogger()
        curses.wrapper(jogger.run)
    except rospy.ROSException as e:
        print(f"ROS error: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()