MotoMini Advanced Motion Planner

This package implements a robust multi-target motion planner for the Yaskawa MotoMini robot using the Tesseract framework. It features advanced optimization techniques including Ifopt (Interface for Optimization), automatic trajectory subdivision, and robust Inverse Kinematics (IK) seeding.

🌟 Key Features

Multi-Target Planning: Accepts a list of Cartesian poses ($T_1, T_2, T_3...$) and plans a continuous path visiting all of them.

Hybrid Solver Support:

Legacy TrajOpt: Fast, standard SQP solver.

Ifopt (Advanced): Modern solver supporting higher-order derivatives (Jerk/Acceleration smoothing).

Robust Seeding Strategy:

Uses Inverse Kinematics (IK) to generate a valid initial guess.

Implements Random Restarts to find solutions even for difficult poses.

Subdivides sparse waypoints into dense steps to allow for fine-grained smoothing.

Manual Time Parameterization:

Calculates valid timestamps for ros2_control.

Enforces minimum time steps to prevent controller errors (dt = 0.0).

🛠️ Key Methods Explained

1. run()

The main execution loop. It orchestrates the entire pipeline:

Updates the internal environment to the robot's current state.

Generates the IK Seed.

Subdivides the seed into a dense program.

Configures optimization profiles (Weights, Costs, Constraints).

Executes the solver.

Post-processes the trajectory (Time Parameterization).

2. generateIKSeed(...)

Goal: Solves the "Where to go" problem.

Logic: Iterates through every target pose.

First, tries to find an IK solution close to the previous point ("Neighbor Seed").

If that fails, it tries 20 random joint configurations ("Random Restarts").

Benefit: Prevents the optimization from crashing due to bad initial guesses.

3. subdivideProgram(..., steps_per_segment)

Goal: Solves the "How to move smoothly" problem.

Logic: Linearly interpolates between the IK solutions.

Why? Optimization solvers need "variables" to optimize. If you move 50cm with only 1 step, the solver can't smooth the acceleration. By splitting it into 20 steps, the solver can adjust the middle 18 points to create a curve.

⚙️ Tuning Guide (How to Adjust)

You can modify these variables inside src/motomini_planning.cpp to change the robot's behavior.

1. Adjusting Speed 🚀

Located at the end of run() inside the Manual Time Parameterization block.

// Controls the maximum joint velocity (rad/s)
double max_velocity = 0.5;

Increase (e.g., 1.5): Robot moves faster.

Decrease (e.g., 0.1): Robot moves very slowly (good for testing).

2. Adjusting Smoothness / Resolution 🌊

Located inside run() where subdivideProgram is called.

// Controls how many "dots" are generated between waypoints
CompositeInstruction dense_program = subdivideProgram(sparse_seed, joint_names, 20);

Increase (e.g., 50): Much smoother path, better jerk optimization, but slower calculation time.

Decrease (e.g., 5): Faster calculation, but movement might look robotic/linear.

3. Adjusting Accuracy (Cartesian Constraints) 🎯

Located in the ifopt\_ profile configuration.

trajopt_ifopt_move->cartesian_constraint_config.coeff = Eigen::VectorXd::Constant(6, 1, 1.0);

Increase (e.g., 10.0 or 20.0): Robot hits the target EXACTLY. Solver might fail if the target is physically hard to reach.

Decrease (e.g., 0.1 or 1.0): Robot gets "close enough" to the target. Motion is smoother, solver rarely fails.

4. Adjusting Collision Safety 🛡️

Located in the trajopt_ifopt_composite configuration.

// Arg 1: Safety Margin (meters), Arg 2: Penalty Weight
trajopt_ifopt_composite->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.010, 20);

Safety Margin (0.010): The robot tries to stay 1cm away from obstacles.

Weight (20): How much the robot "fears" collisions.

If the robot is getting too close to obstacles, increase the weight to 50.

🐞 Troubleshooting

Issue

Cause

Fix

"Time between points is 0.000"

The robot isn't moving between two steps.

Increase min_dt in the time parameterization loop (currently 0.01).

"Optimization FAILED" (Immediate)

Start state is in collision.

Decrease safety_margin to 0.005 or 0.0 temporarily to debug.

"IK Failed for Target X"

Target is out of reach or orientation is impossible.

Check target_poses input. Ensure orientation is pointing DOWN (not Identity quaternion).

Jerky Motion

Resolution is too low.

Increase steps_per_segment in subdivideProgram to 30 or 40.
