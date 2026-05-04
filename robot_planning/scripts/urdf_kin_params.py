#!/usr/bin/env python3
"""
urdf_kin_params.py
------------------
Extracts and prints the kinematic parameters for the MotoMini chain
(base_link -> tool0) directly from the URDF.

Outputs:
  1. Raw URDF joint table  (origin xyz/rpy, axis, limits)
  2. Equivalent 4x4 fixed transform for each joint (at q=0)
  3. FK result at q=0 and an arbitrary test pose
  4. Modified DH table (Craig convention) extracted from the URDF

This mirrors exactly what KDL does internally — KDL does NOT use DH
parameters; it uses the URDF origin+axis directly.

Usage:
    python3 urdf_kin_params.py [path/to/urdf]   (defaults to local path)
"""

import sys
import math
import xml.etree.ElementTree as ET
import numpy as np

# ─────────────────────────────────────────────────────────────────
# Helpers: rotation matrices from rpy / axis-angle
# ─────────────────────────────────────────────────────────────────

def rpy_to_mat(roll, pitch, yaw):
    """URDF convention: Rz(yaw) @ Ry(pitch) @ Rx(roll)"""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    Rx = np.array([[1, 0,  0 ],
                   [0, cr, -sr],
                   [0, sr,  cr]])
    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])
    return Rz @ Ry @ Rx


def axis_angle_to_mat(axis, theta):
    """Rodrigues' rotation formula  R = I + sin(θ)K + (1-cos(θ))K²"""
    ax = np.asarray(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    K = np.array([[ 0,     -ax[2],  ax[1]],
                  [ ax[2],  0,     -ax[0]],
                  [-ax[1],  ax[0],  0    ]])
    return np.eye(3) + math.sin(theta)*K + (1 - math.cos(theta))*(K @ K)


def make_T(xyz, rpy):
    """Build a 4×4 homogeneous transform from URDF origin tag."""
    T = np.eye(4)
    T[:3, :3] = rpy_to_mat(*rpy)
    T[:3,  3] = xyz
    return T


def joint_T(T_fixed, axis, q):
    """Full joint transform = T_fixed @ Rot(axis, q)."""
    R_q = axis_angle_to_mat(axis, q)
    T_q = np.eye(4)
    T_q[:3, :3] = R_q
    return T_fixed @ T_q


# ─────────────────────────────────────────────────────────────────
# URDF parser  (only reads the chain base_link -> tool0)
# ─────────────────────────────────────────────────────────────────

CHAIN_ORDER = [
    "joint_1_s",
    "joint_2_l",
    "joint_3_u",
    "joint_4_r",
    "joint_5_b",
    "joint_6_t",
    "joint_6_t-tool0",   # fixed offset to tool0
]

def parse_vec3(s):
    return [float(v) for v in s.strip().split()]

def parse_urdf(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    joints = {}
    for j in root.findall("joint"):
        name = j.get("name")
        jtype = j.get("type")
        origin = j.find("origin")
        xyz  = parse_vec3(origin.get("xyz", "0 0 0")) if origin is not None else [0,0,0]
        rpy  = parse_vec3(origin.get("rpy", "0 0 0")) if origin is not None else [0,0,0]
        axis_el = j.find("axis")
        axis = parse_vec3(axis_el.get("xyz", "0 0 1")) if axis_el is not None else [0,0,1]
        limit_el = j.find("limit")
        lower = float(limit_el.get("lower", "0")) if limit_el is not None else 0
        upper = float(limit_el.get("upper", "0")) if limit_el is not None else 0
        vel   = float(limit_el.get("velocity", "0")) if limit_el is not None else 0
        joints[name] = dict(type=jtype, xyz=xyz, rpy=rpy, axis=axis,
                            lower=lower, upper=upper, vel=vel)
    return joints


# ─────────────────────────────────────────────────────────────────
# FK using URDF transforms (matches KDL exactly)
# ─────────────────────────────────────────────────────────────────

def fk(joints_data, q_vec):
    """
    Forward kinematics: returns T_base_to_tool0 (4×4).
    q_vec = [q1, q2, q3, q4, q5, q6]
    """
    active = [n for n in CHAIN_ORDER if joints_data[n]["type"] != "fixed"]
    assert len(q_vec) == len(active), f"Need {len(active)} joint values"

    T = np.eye(4)
    qi = 0
    for name in CHAIN_ORDER:
        jd = joints_data[name]
        T_fixed = make_T(jd["xyz"], jd["rpy"])
        if jd["type"] == "fixed":
            T = T @ T_fixed
        else:
            T = T @ joint_T(T_fixed, jd["axis"], q_vec[qi])
            qi += 1
    return T


# ─────────────────────────────────────────────────────────────────
# Modified DH (Craig convention) extractor
# ─────────────────────────────────────────────────────────────────

def extract_modified_dh(joints_data):
    """
    Attempts to extract Modified DH parameters from the URDF joint frames.

    Modified DH convention (Craig):
      T_i = Rot_x(alpha_{i-1}) @ Trans_x(a_{i-1}) @ Rot_z(theta_i) @ Trans_z(d_i)

    This is only exact for robots whose joint axes align to Z after a single
    Rx/Tx correction. For the MotoMini the axes are NOT all Z, so we also
    print the raw 'axis in parent frame' so you can see what KDL stores.

    Returns list of dicts with keys: joint, a_prev, alpha_prev, d, theta_offset.
    """
    results = []
    # Build transforms between consecutive z-axes
    T_accum = np.eye(4)
    active_names = [n for n in CHAIN_ORDER if joints_data[n]["type"] != "fixed"]

    # We extract MDH by looking at consecutive joint axis pairs.
    # z_i = R_accum @ axis_i
    prev_z = np.array([0, 0, 1.0])   # world z before joint 1
    prev_origin = np.zeros(3)

    frame = np.eye(4)   # current frame accumulated up to current joint origin

    for i, name in enumerate(active_names):
        jd = joints_data[name]
        T_fixed = make_T(jd["xyz"], jd["rpy"])
        frame = frame @ T_fixed

        # z_i: current joint axis expressed in base frame
        z_i = frame[:3, :3] @ np.array(jd["axis"])
        origin_i = frame[:3, 3]

        # x_{i-1}: common normal between z_{i-1} and z_i
        x_i_minus_1 = np.cross(prev_z, z_i)
        x_norm = np.linalg.norm(x_i_minus_1)
        if x_norm < 1e-9:
            # parallel axes: x is arbitrary, set to prev x or world x
            x_i_minus_1 = np.array([1, 0, 0.0])
        else:
            x_i_minus_1 /= x_norm

        # a_{i-1}: distance between z_{i-1} and z_i along x_{i-1}
        d_vec = origin_i - prev_origin
        a_prev = float(np.dot(d_vec, x_i_minus_1))

        # alpha_{i-1}: angle from z_{i-1} to z_i about x_{i-1}
        cos_a = float(np.clip(np.dot(prev_z, z_i), -1, 1))
        cross_a = np.cross(prev_z, z_i)
        sin_a = float(np.dot(cross_a, x_i_minus_1))
        alpha_prev = math.atan2(sin_a, cos_a)

        # d_i: distance along z_i from x_{i-1} to x_i
        d_i = float(np.dot(d_vec, z_i))

        # theta offset (joint variable is added on top of this at runtime)
        theta_offset = 0.0  # for URDF with rpy=0 joints this is 0

        results.append(dict(
            joint=name,
            a_prev=a_prev,
            alpha_prev=alpha_prev,
            d=d_i,
            theta_offset=theta_offset,
            z_in_base=z_i,
            axis_in_parent=jd["axis"],
            origin_xyz=jd["xyz"],
        ))

        prev_z = z_i
        prev_origin = origin_i

    return results


# ─────────────────────────────────────────────────────────────────
# Pretty print helpers
# ─────────────────────────────────────────────────────────────────

def print_separator(char="─", width=80):
    print(char * width)

def mat4_str(T, label=""):
    lines = [f"  {label}" if label else ""]
    for row in T:
        lines.append("  [{:8.5f}  {:8.5f}  {:8.5f}  {:8.5f}]".format(*row))
    return "\n".join(lines)

def euler_from_matrix(R):
    """Extract roll-pitch-yaw (ZYX) from rotation matrix."""
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    if not singular:
        roll  = math.atan2( R[2,1], R[2,2])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = math.atan2( R[1,0], R[0,0])
    else:
        roll  = math.atan2(-R[1,2], R[1,1])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = 0
    return roll, pitch, yaw


# ─────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────

def main():
    urdf_path = sys.argv[1] if len(sys.argv) > 1 else (
        "/home/vinbui/vinh_ws/src/thesis_project/robot_model/urdf/motoman_motomini.urdf")

    print(f"\nReading URDF: {urdf_path}")
    jdata = parse_urdf(urdf_path)

    active = [n for n in CHAIN_ORDER if jdata[n]["type"] != "fixed"]

    # ── 1. Raw URDF joint table ──────────────────────────────────
    print_separator()
    print("1. RAW URDF JOINT PARAMETERS  (what KDL reads from the URDF)")
    print_separator()
    hdr = f"{'Joint':<20} {'type':<8}  {'x(m)':>8} {'y(m)':>8} {'z(m)':>8}  "
    hdr += f"{'roll':>8} {'pitch':>8} {'yaw':>8}  {'axis':>14}  {'lower':>8} {'upper':>8} {'vel(r/s)':>9}"
    print(hdr)
    print("─"*130)
    for name in CHAIN_ORDER:
        jd = jdata[name]
        ax = "[{:5.2f},{:5.2f},{:5.2f}]".format(*jd["axis"])
        print(f"{name:<20} {jd['type']:<8}  "
              f"{jd['xyz'][0]:>8.4f} {jd['xyz'][1]:>8.4f} {jd['xyz'][2]:>8.4f}  "
              f"{jd['rpy'][0]:>8.4f} {jd['rpy'][1]:>8.4f} {jd['rpy'][2]:>8.4f}  "
              f"{ax:>14}  "
              f"{jd['lower']:>8.4f} {jd['upper']:>8.4f} {jd['vel']:>9.4f}")

    # ── 2. Fixed transforms at q=0 ───────────────────────────────
    print()
    print_separator()
    print("2. PER-JOINT FIXED TRANSFORM T_fixed  (KDL stores this as KDL::Frame for each segment)")
    print("   = make_T(origin_xyz, origin_rpy)  — the part that does NOT change with joint angle")
    print_separator()
    for name in CHAIN_ORDER:
        jd = jdata[name]
        Tf = make_T(jd["xyz"], jd["rpy"])
        print(mat4_str(Tf, f"T_fixed[{name}]:"))
        print()

    # ── 3. FK results ────────────────────────────────────────────
    print_separator()
    print("3. FORWARD KINEMATICS  (base_link → tool0)")
    print("   KDL computes: T = T_fixed[j1] @ Rot(axis1,q1) @ T_fixed[j2] @ Rot(axis2,q2) @ ...")
    print_separator()

    test_cases = [
        ([0.0]*6,                       "q = [0, 0, 0, 0, 0, 0]  (home)"),
        ([0, math.pi/4, 0, 0, 0, 0],   "q = [0, π/4, 0, 0, 0, 0]"),
        ([0, 0, math.pi/4, 0, 0, 0],   "q = [0, 0, π/4, 0, 0, 0]"),
        ([math.pi/4]*6,                  "q = [π/4]*6"),
    ]

    for q_vec, label in test_cases:
        T = fk(jdata, q_vec)
        pos = T[:3, 3]
        r, p, y_ = euler_from_matrix(T[:3, :3])
        print(f"  {label}")
        print(f"    Position  xyz  = [{pos[0]:.6f},  {pos[1]:.6f},  {pos[2]:.6f}]  (m)")
        print(f"    Orientation rpy = [{math.degrees(r):.3f}°,  {math.degrees(p):.3f}°,  {math.degrees(y_):.3f}°]")
        print(mat4_str(T))
        print()

    # ── 4. Modified DH table ─────────────────────────────────────
    print_separator()
    print("4. MODIFIED DH PARAMETERS  (Craig convention, extracted from URDF frames)")
    print()
    print("   MDH convention for joint i:")
    print("     T_i = Rot_x(α_{i-1}) · Trans_x(a_{i-1}) · Rot_z(θ_i + θ_offset) · Trans_z(d_i)")
    print()
    print("   NOTE: KDL does NOT use DH internally — it uses the URDF origin+axis directly.")
    print("   These DH values are derived from the URDF geometry for reference only.")
    print_separator()

    dh = extract_modified_dh(jdata)
    print(f"  {'Joint':<14} {'a_prev(m)':>10} {'α_prev(°)':>10} {'d(m)':>10} {'θ_offset(°)':>12}  "
          f"{'axis in parent':>16}  {'z in base (at q=0)':>28}")
    print("  " + "─"*110)
    for row in dh:
        ax_str = "[{:5.2f},{:5.2f},{:5.2f}]".format(*row["axis_in_parent"])
        z_str  = "[{:6.3f},{:6.3f},{:6.3f}]".format(*row["z_in_base"])
        print(f"  {row['joint']:<14} "
              f"{row['a_prev']:>10.5f} "
              f"{math.degrees(row['alpha_prev']):>10.3f} "
              f"{row['d']:>10.5f} "
              f"{math.degrees(row['theta_offset']):>12.3f}  "
              f"{ax_str:>16}  {z_str:>28}")

    # ── 5. Symbolic FK chain ─────────────────────────────────────
    print()
    print_separator()
    print("5. SYMBOLIC FK CHAIN  — how each joint contributes")
    print_separator()
    print("""
  KDL FK algorithm (ChainFkSolverPos_recursive):
  ───────────────────────────────────────────────
  T_result = I₄

  For each segment i in the chain:
    T_result = T_result  ×  T_fixed_i  ×  Rot(axis_i, q_i)

  where:
    T_fixed_i  = homogeneous transform from URDF <origin xyz rpy>
               = [ R_rpy | t_xyz ]
                 [  0 0 0 |   1  ]

    Rot(axis, q) = [ Rodrigues(axis, q) | 0 ]   ← axis-angle rotation
                   [      0   0   0     | 1 ]

    Rodrigues:  R = I + sin(q)·[axis]× + (1−cos(q))·[axis]×²
                [axis]× = skew-symmetric matrix of the unit axis vector

  For the MotoMini chain:
    T_base→tool0 = T_fix(j1) · Rz(q1)       ← joint_1_s  axis=[0,0,1]
                 × T_fix(j2) · Ry(q2)        ← joint_2_l  axis=[0,1,0]
                 × T_fix(j3) · R(-y)(q3)     ← joint_3_u  axis=[0,-1,0]
                 × T_fix(j4) · R(-x)(q4)     ← joint_4_r  axis=[-1,0,0]
                 × T_fix(j5) · R(-y)(q5)     ← joint_5_b  axis=[0,-1,0]
                 × T_fix(j6) · Rz(q6)        ← joint_6_t  axis=[0,0,-1]  (wait: see actual)
                 × T_fix(j6-tool0)            ← fixed offset: xyz=[0,0,-0.04] rpy=[π,0,0]
""")

    print_separator()
    print("6. IK ALGORITHM  (KDLInvKinChainLMA — Levenberg-Marquardt)")
    print_separator()
    print("""
  Minimises the task-space error  e(q) = log( T_fk(q)^{-1} · T_target )
  (a 6-vector: 3 translation + 3 rotation)

  Update rule per iteration:
    Δq = −(JᵀWJ + λI)⁻¹ Jᵀ W e(q)

  where:
    J       = geometric Jacobian (6×6, see section below)
    W       = diagonal task-space weight matrix  [configured via kdl_config.task_weights]
              default all-ones: treats translation and rotation equally
    λ       = Levenberg-Marquardt damping (auto-tuned each iteration)
    e(q)    = 6D pose error vector in se(3) (twist coordinates)

  Convergence check:  ‖e(q)‖ < ε  (default ε = 1e-5)
  Max iterations: 500 (default)

  Then:  q_{k+1} = q_k + Δq   (clipped to joint limits by KinematicGroup)
""")

    print_separator()
    print("7. JACOBIAN ALGORITHM  (KDL::ChainJntToJacSolver)")
    print_separator()
    print("""
  KDL computes the GEOMETRIC Jacobian expressed in the BASE frame.

  For a 6-DOF revolute chain, column i of J (6×6):

    J_i = [ z_{i-1} × (p_e − p_{i-1}) ]   ← linear  velocity part
          [           z_{i-1}          ]   ← angular velocity part

  where:
    z_{i-1} = rotation axis of joint i, expressed in BASE frame
            = R_{base→i-1} · axis_i_urdf

    p_{i-1} = origin of joint i frame, in BASE coordinates
            = T_base→i · [0,0,0,1]ᵀ (translation column)

    p_e     = end-effector origin in BASE coordinates

  Full matrix:
    J(q) = [ z_0×(p_e-p_0)  z_1×(p_e-p_1)  ...  z_5×(p_e-p_5) ]
           [     z_0              z_1        ...       z_5        ]

  This is what your code calls as:
    manip_->calcJacobian(q, base_link_, ee_link_)   →  Eigen::MatrixXd 6×6
""")


if __name__ == "__main__":
    main()
