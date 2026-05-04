give me an .tex file that give me detail about full of the report of KF and IK and jacobian and the mathmatic of the craig connvention(vinbui@vinh-ubuntu:~/vinh_ws$  python3 src/thesis_project/robot_planning/scripts/urdf_kin_params.py

Reading URDF: /home/vinbui/vinh_ws/src/thesis_project/robot_model/urdf/motoman_motomini.urdf
────────────────────────────────────────────────────────────────────────────────
1. RAW URDF JOINT PARAMETERS  (what KDL reads from the URDF)
────────────────────────────────────────────────────────────────────────────────
Joint                type          x(m)     y(m)     z(m)      roll    pitch      yaw            axis     lower    upper  vel(r/s)
──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
joint_1_s            revolute    0.0000   0.0000   0.1030    0.0000   0.0000   0.0000  [ 0.00, 0.00, 1.00]   -2.9670   2.9670    5.4977
joint_2_l            revolute    0.0200   0.0000   0.0000    0.0000   0.0000   0.0000  [ 0.00, 1.00, 0.00]   -1.4835   1.5707    5.4977
joint_3_u            revolute    0.0000   0.0000   0.1650    0.0000   0.0000   0.0000  [ 0.00,-1.00, 0.00]   -0.8726   1.5707    7.3304
joint_4_r            revolute    0.1650   0.0000   0.0000    0.0000   0.0000   0.0000  [-1.00, 0.00, 0.00]   -2.4434   2.4434   10.4719
joint_5_b            revolute    0.0000   0.0000   0.0000    0.0000   0.0000   0.0000  [ 0.00,-1.00, 0.00]   -0.5235   3.6651   10.4719
joint_6_t            revolute    0.0000   0.0000   0.0000    0.0000   0.0000   0.0000  [ 0.00, 0.00, 1.00]   -6.2831   6.2831   10.4719
joint_6_t-tool0      fixed       0.0000   0.0000  -0.0400    3.1416   0.0000   0.0000  [ 0.00, 0.00, 1.00]    0.0000   0.0000    0.0000

────────────────────────────────────────────────────────────────────────────────
2. PER-JOINT FIXED TRANSFORM T_fixed  (KDL stores this as KDL::Frame for each segment)
   = make_T(origin_xyz, origin_rpy)  — the part that does NOT change with joint angle
────────────────────────────────────────────────────────────────────────────────
  T_fixed[joint_1_s]:
  [ 1.00000   0.00000   0.00000   0.00000]
  [ 0.00000   1.00000   0.00000   0.00000]
  [ 0.00000   0.00000   1.00000   0.10300]
  [ 0.00000   0.00000   0.00000   1.00000]

  T_fixed[joint_2_l]:
  [ 1.00000   0.00000   0.00000   0.02000]
  [ 0.00000   1.00000   0.00000   0.00000]
  [ 0.00000   0.00000   1.00000   0.00000]
  [ 0.00000   0.00000   0.00000   1.00000]

  T_fixed[joint_3_u]:
  [ 1.00000   0.00000   0.00000   0.00000]
  [ 0.00000   1.00000   0.00000   0.00000]
  [ 0.00000   0.00000   1.00000   0.16500]
  [ 0.00000   0.00000   0.00000   1.00000]

  T_fixed[joint_4_r]:
  [ 1.00000   0.00000   0.00000   0.16500]
  [ 0.00000   1.00000   0.00000   0.00000]
  [ 0.00000   0.00000   1.00000   0.00000]
  [ 0.00000   0.00000   0.00000   1.00000]

  T_fixed[joint_5_b]:
  [ 1.00000   0.00000   0.00000   0.00000]
  [ 0.00000   1.00000   0.00000   0.00000]
  [ 0.00000   0.00000   1.00000   0.00000]
  [ 0.00000   0.00000   0.00000   1.00000]

  T_fixed[joint_6_t]:
  [ 1.00000   0.00000   0.00000   0.00000]
  [ 0.00000   1.00000   0.00000   0.00000]
  [ 0.00000   0.00000   1.00000   0.00000]
  [ 0.00000   0.00000   0.00000   1.00000]

  T_fixed[joint_6_t-tool0]:
  [ 1.00000   0.00000   0.00000   0.00000]
  [ 0.00000  -1.00000  -0.00000   0.00000]
  [ 0.00000   0.00000  -1.00000  -0.04000]
  [ 0.00000   0.00000   0.00000   1.00000]

────────────────────────────────────────────────────────────────────────────────
3. FORWARD KINEMATICS  (base_link → tool0)
   KDL computes: T = T_fixed[j1] @ Rot(axis1,q1) @ T_fixed[j2] @ Rot(axis2,q2) @ ...
────────────────────────────────────────────────────────────────────────────────
  q = [0, 0, 0, 0, 0, 0]  (home)
    Position  xyz  = [0.185000,  0.000000,  0.228000]  (m)
    Orientation rpy = [180.000°,  -0.000°,  0.000°]

  [ 1.00000   0.00000   0.00000   0.18500]
  [ 0.00000  -1.00000  -0.00000   0.00000]
  [ 0.00000   0.00000  -1.00000   0.22800]
  [ 0.00000   0.00000   0.00000   1.00000]

  q = [0, π/4, 0, 0, 0, 0]
    Position  xyz  = [0.225061,  0.000000,  0.074716]  (m)
    Orientation rpy = [180.000°,  45.000°,  0.000°]

  [ 0.70711   0.00000  -0.70711   0.22506]
  [ 0.00000  -1.00000  -0.00000   0.00000]
  [-0.70711   0.00000  -0.70711   0.07472]
  [ 0.00000   0.00000   0.00000   1.00000]

  q = [0, 0, π/4, 0, 0, 0]
    Position  xyz  = [0.164957,  0.000000,  0.356388]  (m)
    Orientation rpy = [180.000°,  -45.000°,  0.000°]

  [ 0.70711  -0.00000   0.70711   0.16496]
  [ 0.00000  -1.00000  -0.00000   0.00000]
  [ 0.70711   0.00000  -0.70711   0.35639]
  [ 0.00000   0.00000   0.00000   1.00000]

  q = [π/4]*6
    Position  xyz  = [0.247457,  0.219173,  0.199673]  (m)
    Orientation rpy = [120.361°,  8.421°,  104.639°]

  [-0.25000   0.45711   0.85355   0.24746]
  [ 0.95711   0.25000   0.14645   0.21917]
  [-0.14645   0.85355  -0.50000   0.19967]
  [ 0.00000   0.00000   0.00000   1.00000]

────────────────────────────────────────────────────────────────────────────────
4. MODIFIED DH PARAMETERS  (Craig convention, extracted from URDF frames)

   MDH convention for joint i:
     T_i = Rot_x(α_{i-1}) · Trans_x(a_{i-1}) · Rot_z(θ_i + θ_offset) · Trans_z(d_i)

   NOTE: KDL does NOT use DH internally — it uses the URDF origin+axis directly.
   These DH values are derived from the URDF geometry for reference only.
────────────────────────────────────────────────────────────────────────────────
  Joint           a_prev(m)  α_prev(°)       d(m)  θ_offset(°)    axis in parent            z in base (at q=0)
  ──────────────────────────────────────────────────────────────────────────────────────────────────────────────
  joint_1_s         0.00000      0.000    0.10300        0.000  [ 0.00, 0.00, 1.00]        [ 0.000, 0.000, 1.000]
  joint_2_l        -0.02000     90.000    0.00000        0.000  [ 0.00, 1.00, 0.00]        [ 0.000, 1.000, 0.000]
  joint_3_u         0.00000    180.000    0.00000        0.000  [ 0.00,-1.00, 0.00]        [ 0.000,-1.000, 0.000]
  joint_4_r         0.00000     90.000   -0.16500        0.000  [-1.00, 0.00, 0.00]        [-1.000, 0.000, 0.000]
  joint_5_b         0.00000     90.000    0.00000        0.000  [ 0.00,-1.00, 0.00]        [ 0.000,-1.000, 0.000]
  joint_6_t         0.00000     90.000    0.00000        0.000  [ 0.00, 0.00, 1.00]        [ 0.000, 0.000, 1.000]

────────────────────────────────────────────────────────────────────────────────
5. SYMBOLIC FK CHAIN  — how each joint contributes
────────────────────────────────────────────────────────────────────────────────

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

────────────────────────────────────────────────────────────────────────────────
6. IK ALGORITHM  (KDLInvKinChainLMA — Levenberg-Marquardt)
────────────────────────────────────────────────────────────────────────────────

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

────────────────────────────────────────────────────────────────────────────────
7. JACOBIAN ALGORITHM  (KDL::ChainJntToJacSolver)
────────────────────────────────────────────────────────────────────────────────

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
    manip_->calcJacobian(q, base_link_, ee_link_)   →  Eigen::MatrixXd 6×6), write one tex file on (/home/vinbui/HCMUT/Thesis/LVTN/texdoc/chapter_ly_thuyet/) viết bằng tiếng việt và giải thích cụ thể 