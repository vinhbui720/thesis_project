# Tesseract QP Solver Notes For MotoMini Planning

This note traces the QP/SQP pieces used by:

`robot_planning/src/motomini_planning_run.cpp`

The goal is to show which dependency owns each part of the QP problem, where the constraints/costs are created, and where the actual matrices passed to OSQP are built.

## Dependency Surface

Your `CMakeLists.txt` already declares and links the direct QP/SQP libraries:

```cmake
find_package(trajopt_sqp REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Taskflow REQUIRED)

target_link_libraries(
  ${PROJECT_NAME}
  trajopt::trajopt_sqp
  trajopt::trajopt_ifopt
  OsqpEigen::OsqpEigen
  Taskflow::Taskflow)
```

`package.xml` must also declare the ROS package that provides the `OsqpEigen` CMake package. That dependency is:

```xml
<depend>osqp_eigen</depend>
```

`Taskflow` is a plain CMake dependency in this workspace, not a ROS package dependency in the same style as `trajopt_sqp`.

## Planner Branches In Your Run File

`motomini_planning_run.cpp` chooses the TaskComposer pipeline here:

```cpp
const std::string task_name = ompl_enabled ? "FreespacePipeline"
                                           : (ifopt_ ? "TrajOptIfoptPipeline" : "TrajOptPipeline");
```

That means the QP/SQP implementation depends on the runtime flags:

| Runtime flags | Pipeline | QP/SQP stack |
|---|---|---|
| `use_ompl_ == true` | `FreespacePipeline` | OMPL seed, then legacy TrajOpt: `trajopt_sco::BasicTrustRegionSQP` + `sco::OSQPModel` |
| `use_ompl_ == false`, `ifopt_ == true` | `TrajOptIfoptPipeline` | TrajOpt-IFOPT: `trajopt_sqp::TrajOptQPProblem` + `trajopt_sqp::TrustRegionSQPSolver` + `OSQPEigenSolver` |
| `use_ompl_ == false`, `ifopt_ == false` | `TrajOptPipeline` | Legacy TrajOpt: `trajopt_sco::BasicTrustRegionSQP` + `sco::OSQPModel` |
| `online_mode_ == true` | direct thread after offline result | Direct `trajopt_sqp::TrajOptQPProblem` + `TrustRegionSQPSolver` + `OSQPEigenSolver` |

TaskComposer pipeline definitions are in:

`/home/vinbui/vinh_ws/src/tesseract/tesseract_planning/tesseract_task_composer/config/task_composer_plugins.yaml`

Important entries:

- `TrajOptPipeline`: lines 600-638.
- `TrajOptIfoptPipeline`: lines 707-745.
- `FreespacePipeline`: lines 947-985.
- `FreespaceTask`: lines 866-946. It runs `OMPLMotionPlannerTask` then `TrajOptMotionPlannerTask`.

## General QP Form

Both OSQP paths solve the same mathematical shape:

```text
minimize    1/2 z^T P z + q^T z
subject to  l <= A z <= u
```

For the `trajopt_sqp` path, Tesseract internally stores an objective as:

```text
minimize    z^T H z + g^T z
subject to  l <= A z <= u
```

Then `OSQPEigenSolver` sends `P = 2H` to OSQP because OSQP applies the `1/2` factor itself.

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_optimizers/trajopt_sqp/src/osqp_eigen_solver.cpp`

```cpp
bool OSQPEigenSolver::updateHessianMatrix(const SparseMatrix& hessian)
{
  // Also multiply by 2 because OSQP is multiplying by (1/2) for the objective fuction
  const SparseMatrix cleaned = 2.0 * hessian.pruned(1e-7, 1);
  ...
}
```

## TrajOpt-IFOPT QP Path

This is the path used when `ifopt_ == true` and `use_ompl_ == false`.

### Tesseract Entry Point

Source:

`/home/vinbui/vinh_ws/src/tesseract/tesseract_planning/tesseract_motion_planners/trajopt_ifopt/src/trajopt_ifopt_motion_planner.cpp`

The planner creates a `TrajOptQPProblem`, adds one joint-position variable set per waypoint, adds waypoint and composite constraints/costs, then calls the SQP solver.

```cpp
std::shared_ptr<trajopt_sqp::TrajOptQPProblem> nlp =
    std::make_shared<trajopt_sqp::TrajOptQPProblem>();

for (int i = 0; i < move_instructions.size(); ++i)
{
  TrajOptIfoptWaypointInfo wp_info =
      cur_move_profile->create(move_instruction, composite_mi, request.env, i);

  vars.push_back(wp_info.var);
  nlp->addVariableSet(wp_info.var);

  for (const auto& cnt : wp_info.term_infos.constraints)
    nlp->addConstraintSet(cnt);

  for (const auto& squared_cost : wp_info.term_infos.squared_costs)
    nlp->addCostSet(squared_cost, trajopt_sqp::CostPenaltyType::SQUARED);

  for (const auto& hinge_cost : wp_info.term_infos.hinge_costs)
    nlp->addCostSet(hinge_cost, trajopt_sqp::CostPenaltyType::HINGE);
}

TrajOptIfoptTermInfos term_infos =
    cur_composite_profile->create(composite_mi, request.env, vars, fixed_steps);

nlp->setup();

std::unique_ptr<trajopt_sqp::TrustRegionSQPSolver> solver =
    solver_profile->create(request.verbose);

solver->solve(nlp);
```

### Solver Profile

Source:

`/home/vinbui/vinh_ws/src/tesseract/tesseract_planning/tesseract_motion_planners/trajopt_ifopt/src/profile/trajopt_ifopt_osqp_solver_profile.cpp`

This creates the OSQP-backed trust-region SQP solver:

```cpp
auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
copyOSQPEigenSettings(*qp_solver->solver_->settings(), *qp_settings);

auto solver = std::make_unique<trajopt_sqp::TrustRegionSQPSolver>(qp_solver);
solver->params = opt_params;
```

Your run file sets these parameters:

```cpp
trajopt_ifopt_solver->opt_params.max_iterations = planning_cfg_.ifopt_max_iter;
trajopt_ifopt_solver->opt_params.min_approx_improve = planning_cfg_.ifopt_min_approx_improve;
trajopt_ifopt_solver->opt_params.min_trust_box_size = planning_cfg_.ifopt_min_trust_box_size;
trajopt_ifopt_solver->opt_params.initial_trust_box_size = planning_cfg_.ifopt_initial_trust_box_size;
```

### QP Variable Vector

For a trajectory with `N` waypoints and `n` joints, the base NLP variables are:

```text
x = [q_0, q_1, ..., q_{N-1}]
q_i in R^n
```

`TrajOptQPProblem` augments this with slack variables:

```text
z = [x,
     hinge_cost_slacks,
     absolute_cost_positive_slacks,
     absolute_cost_negative_slacks,
     constraint_slacks]
```

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp`

```cpp
num_qp_vars_ = getNumNLPVars() + hinge_costs_.GetRows() + (2L * abs_costs_.GetRows());
num_qp_cnts_ = getNumNLPConstraints() + getNumNLPVars()
             + (2L * hinge_costs_.GetRows()) + (3L * abs_costs_.GetRows());

for (std::size_t i = 0; i < static_cast<std::size_t>(nlp_bounds_diff.size()); i++)
{
  if (std::abs(nlp_bounds_diff[static_cast<Eigen::Index>(i)]) < 1e-3)
  {
    constraint_types_[i] = ConstraintType::EQ;
    num_qp_vars_ += 2;
    num_qp_cnts_ += 2;
  }
  else
  {
    constraint_types_[i] = ConstraintType::INEQ;
    num_qp_vars_ += 1;
    num_qp_cnts_ += 1;
  }
}
```

### QP Matrix Construction

`TrustRegionSQPSolver::stepSQPSolver()` calls:

```cpp
qp_problem->convexify();
qp_solver->init(qp_problem->getNumQPVars(), qp_problem->getNumQPConstraints());
qp_solver->updateHessianMatrix(qp_problem->getHessian());
qp_solver->updateGradient(qp_problem->getGradient());
qp_solver->updateLinearConstraintsMatrix(qp_problem->getConstraintMatrix());
qp_solver->updateBounds(qp_problem->getBoundsLower(), qp_problem->getBoundsUpper());
```

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_optimizers/trajopt_sqp/src/trust_region_sqp_solver.cpp`

The matrix getters are:

```cpp
Eigen::Ref<const SparseMatrix> TrajOptQPProblem::getHessian() { return impl_->hessian_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getGradient() { return impl_->gradient_; }
Eigen::Ref<const SparseMatrix> TrajOptQPProblem::getConstraintMatrix() { return impl_->constraint_matrix_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsLower() { return impl_->bounds_lower_; }
Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsUpper() { return impl_->bounds_upper_; }
```

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp`

### Cost Convexification

Squared costs are converted from an affine approximation into a quadratic objective.

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_optimizers/trajopt_sqp/src/expressions.cpp`

```cpp
AffExprs createAffExprs(const Eigen::Ref<const Eigen::VectorXd>& func_error,
                        const Eigen::Ref<const SparseMatrix>& func_jacobian,
                        const Eigen::Ref<const Eigen::VectorXd>& x)
{
  AffExprs aff_expr;
  aff_expr.constants = func_error - (func_jacobian * x);
  aff_expr.linear_coeffs = func_jacobian;
  return aff_expr;
}

QuadExprs squareAffExprs(const AffExprs& aff_expr)
{
  quad_expr.constants = aff_expr.constants.array().square();
  quad_expr.linear_coeffs = (2 * aff_expr.constants).asDiagonal() * aff_expr.linear_coeffs;
  ...
  const SparseMatrix eq_quadexpr_coeffs =
      eq_affexpr_coeffs.transpose() * eq_affexpr_coeffs;
  ...
}
```

Equivalent math:

```text
f(x) ~= c + Jx
||f(x)||^2 ~= x^T J^T J x + 2 c^T J x + c^T c
```

Then `TrajOptQPProblem::convexifyCosts()` inserts:

```cpp
gradient_.head(getNumNLPVars()) = squared_objective_nlp_.objective_linear_coeffs;
hessian_.coeffRef(it.row(), it.col()) += it.value();
```

### Linearized Constraint Matrix

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp`

```cpp
const SparseMatrix nlp_cnt_jac = constraints_.GetJacobian();
const SparseMatrix hinge_cnt_jac = hinge_constraints_.GetJacobian();
const SparseMatrix abs_cnt_jac = abs_constraints_.GetJacobian();

// Add hinge, absolute, and NLP constraint Jacobians.
// Add slack-variable columns.
// Add an identity block for variable and slack bounds.
constraint_matrix_.resize(num_qp_cnts_, num_qp_vars_);
constraint_matrix_.setFromTriplets(tripletList.begin(), tripletList.end());
```

The constraint constants are:

```cpp
constraint_constant_ = cnt_initial_value - jac * x_initial;
```

So the local constraint model is:

```text
c(x) ~= c(x0) + J (x - x0)
     = (c(x0) - J x0) + J x
```

The bounds are shifted by that constant:

```cpp
linearized_cnt_lower = cnt_bound_lower - constraint_constant_;
linearized_cnt_upper = cnt_bound_upper - constraint_constant_;
bounds_lower_.topRows(total_num_cnt) = linearized_cnt_lower;
bounds_upper_.topRows(total_num_cnt) = linearized_cnt_upper;
```

### Trust Region And Joint Limits

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp`

```cpp
const Eigen::VectorXd var_bounds_lower_final =
    (x_initial.cwiseMin(var_bounds_upper - box_size_) - box_size_).cwiseMax(var_bounds_lower);
const Eigen::VectorXd var_bounds_upper_final =
    (x_initial.cwiseMax(var_bounds_lower + box_size_) + box_size_).cwiseMin(var_bounds_upper);
```

This enforces:

```text
max(joint_lower, x0 - trust_box) <= x <= min(joint_upper, x0 + trust_box)
```

## IFOPT Constraint Math Used By Your Profiles

Your `ifopt_` branch configures these terms in `motomini_planning_run.cpp`:

- Cartesian waypoint constraint: enabled by `ifopt_cart_constraint_enable`.
- Cartesian waypoint cost: optional, `ifopt_cart_cost_enable`.
- Joint waypoint cost: enabled by `ifopt_joint_cost_enable`.
- Collision constraint: optional, `ifopt_coll_constraint_enable`.
- Collision cost: enabled by `ifopt_coll_cost_enable`.
- Smooth velocity, acceleration, jerk costs.

### Profile Creates The Terms

Waypoint terms are created here:

`/home/vinbui/vinh_ws/src/tesseract/tesseract_planning/tesseract_motion_planners/trajopt_ifopt/src/profile/trajopt_ifopt_default_move_profile.cpp`

```cpp
if (cartesian_constraint_config.enabled)
{
  auto constraint = createCartesianPositionConstraint(..., cartesian_constraint_config.coeff);
  info.term_infos.constraints.push_back(constraint);
}

if (joint_cost_config.enabled)
{
  auto constraint = createJointPositionConstraint(jwp, info.var, joint_cost_config.coeff);
  info.term_infos.squared_costs.push_back(constraint);
}
```

Composite trajectory terms are created here:

`/home/vinbui/vinh_ws/src/tesseract/tesseract_planning/tesseract_motion_planners/trajopt_ifopt/src/profile/trajopt_ifopt_default_composite_profile.cpp`

```cpp
if (collision_cost_config.enabled)
{
  auto constraints =
      createCollisionConstraints(vars, env, composite_manip_info, collision_cost_config, fixed_indices, false);
  term_infos.hinge_costs.insert(term_infos.hinge_costs.end(), constraints.begin(), constraints.end());
}

if (smooth_velocities)
  term_infos.squared_costs.push_back(createJointVelocityConstraint(target, vars, velocity_coeff));

if (smooth_accelerations)
  term_infos.squared_costs.push_back(createJointAccelerationConstraint(target, vars, acceleration_coeff));

if (smooth_jerks)
  term_infos.squared_costs.push_back(createJointJerkConstraint(target, vars, jerk_coeff));
```

### Cartesian Pose Constraint

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_ifopt/src/constraints/cartesian_position_constraint.cpp`

Value:

```cpp
const Eigen::VectorXd err = error_function_(target_tf, source_tf);
return coeffs_.cwiseProduct(err);
```

Jacobian:

```cpp
triplet_list.emplace_back(i, j, coeffs_(i) * jac0(info_.indices[i], j));
```

Math:

```text
e_cart(q) = selected_axes(transform_error(T_source(q), T_target))
constraint value = W_cart e_cart(q)
linearized row = W_cart J_cart(q0) q
```

In your code, the coefficient vector is:

```text
[ifopt_cart_coeff_x,
 ifopt_cart_coeff_y,
 ifopt_cart_coeff_z,
 ifopt_cart_coeff_rx,
 ifopt_cart_coeff_ry,
 ifopt_cart_coeff_rz]
```

### Joint Position Constraint Or Cost

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_ifopt/src/constraints/joint_position_constraint.cpp`

Value and Jacobian:

```cpp
values << coeffs_.cwiseProduct(...GetValues());
triplet_list.emplace_back(i * n_dof_ * 0 + j, j, coeffs_[j] * 1.0);
```

Math:

```text
value = W_q q
bound = W_q q_target
Jacobian = W_q
```

### Velocity Smoothness Cost

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_ifopt/src/constraints/joint_velocity_constraint.cpp`

```cpp
single_step = vals2 - vals1;
velocity.block(...) = coeffs_.cwiseProduct(single_step);

triplet_list.emplace_back((i * n_dof_) + j, j, -1.0 * coeffs_[j]);
triplet_list.emplace_back(((i - 1) * n_dof_) + j, j, 1.0 * coeffs_[j]);
```

Math:

```text
v_i = W_v (q_{i+1} - q_i)
cost = ||v_i - target||^2
```

### Acceleration Smoothness Cost

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_ifopt/src/constraints/joint_acceleration_constraint.cpp`

```cpp
single_step = vals3 - 2 * vals2 + vals1;
acceleration.block(...) = coeffs_.cwiseProduct(single_step);
```

Math:

```text
a_i = W_a (q_{i+2} - 2 q_{i+1} + q_i)
cost = ||a_i - target||^2
```

### Jerk Smoothness Cost

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_ifopt/src/constraints/joint_jerk_constraint.cpp`

```cpp
single_step = (3.0 * vals2) - (3.0 * vals3) - vals1 + vals4;
```

Math:

```text
j_i = W_j (-q_i + 3 q_{i+1} - 3 q_{i+2} + q_{i+3})
cost = ||j_i - target||^2
```

### Collision Constraint Or Hinge Cost

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_ifopt/src/constraints/collision/discrete_collision_constraint.cpp`

The bounds are `<= 0`:

```cpp
bounds_ = std::vector<ifopt::Bounds>(..., ifopt::BoundSmallerZero);
```

Value:

```cpp
values(i) = r.coeff * r.getMaxErrorT0();
```

Jacobian:

```cpp
Eigen::VectorXd grad_vec =
    getWeightedAvgGradientT0(r, r.getMaxErrorWithBufferT0(), position_var_->GetRows());
jac_block.coeffRef(static_cast<int>(i), j) = -1.0 * grad_vec[j];
```

Collision gradient source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_common/src/collision_utils.cpp`

```cpp
results.error = (margin - contact_result.distance);
results.error_with_buffer = (margin + margin_buffer - contact_result.distance);
...
link_gradient.translation_vector = ((i == 0) ? -1.0 : 1.0) * contact_result.normal;
link_gradient.jacobian = jac.topRows(3);
link_gradient.gradient = link_gradient.translation_vector.transpose() * link_gradient.jacobian;
```

Math:

```text
e_col(q) = margin - distance(q)
constraint value = collision_coeff * e_col(q)
constraint bound = value <= 0
Jacobian = - weighted_collision_gradient
```

For collision cost mode, the same collision constraint object is put into `hinge_costs`, so the QP penalizes only the violated positive side.

## Legacy TrajOpt QP Path

This path is used by:

- `ifopt_ == false`, `use_ompl_ == false`: `TrajOptPipeline`.
- `use_ompl_ == true`: `FreespacePipeline`, after OMPL produces a seed.

### Tesseract Entry Point

Source:

`/home/vinbui/vinh_ws/src/tesseract/tesseract_planning/tesseract_motion_planners/trajopt/src/trajopt_motion_planner.cpp`

```cpp
trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

sco::BasicTrustRegionSQP::Ptr opt;
if (pci->opt_info.num_threads > 1)
  opt = std::make_shared<sco::BasicTrustRegionSQPMultiThreaded>(problem);
else
  opt = std::make_shared<sco::BasicTrustRegionSQP>(problem);

opt->setParameters(pci->opt_info);
opt->initialize(trajToDblVec(problem->GetInitTraj()));
opt->optimize();
```

### Legacy Trust-Region SQP

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_sco/src/optimizers.cpp`

```cpp
const std::vector<ConvexObjective::Ptr> cost_models =
    convexifyCosts(prob_->getCosts(), results_.x, model_.get());
const std::vector<ConvexConstraints::Ptr> cnt_models =
    convexifyConstraints(constraints, results_.x, model_.get());
const std::vector<ConvexObjective::Ptr> cnt_cost_models =
    cntsToCosts(cnt_models, merit_error_coeffs, model_.get());

QuadExpr objective;
for (const ConvexObjective::Ptr& co : cost_models)
  exprInc(objective, co->quad_);
for (const ConvexObjective::Ptr& co : cnt_cost_models)
  exprInc(objective, co->quad_);

model_->setObjective(objective);
setTrustBoxConstraints(results_.x);
const CvxOptStatus status = model_->optimize();
```

### Legacy OSQP Interface

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_sco/include/trajopt_sco/osqp_interface.hpp`

```cpp
 * min   1/2*x'Px + q'x
 * s.t.  l <= Ax <= u
```

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_sco/src/osqp_interface.cpp`

```cpp
exprToEigen(objective_, sm, q_, static_cast<int>(n), true);
triangular_sm = sm.triangularView<Eigen::Upper>();
eigenToCSC(triangular_sm, P_row_indices_, P_column_pointers_, P_csc_data_);

exprToEigen(cnt_exprs_, sm, v, n_int);
...
l_[i_cnt] = (cnt_types_[i_cnt] == INEQ) ? -OSQP_INFINITY : v[i_cnt];
u_[i_cnt] = v[i_cnt];
...
sm.insert(i_bnd + m, i_bnd) = 1.;
```

### Legacy Cost/Constraint Convexification Math

Source:

`/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_sco/src/modeling_utils.cpp`

Affine approximation:

```cpp
AffExpr affFromValGrad(double y, const Eigen::VectorXd& x,
                       const Eigen::VectorXd& dydx, const VarVector& vars)
{
  aff.constant = y - dydx.dot(x);
  aff.coeffs = trajopt_common::toDblVec(dydx);
  aff.vars = vars;
  return cleanupAff(aff);
}
```

Squared, absolute, and hinge cost conversion:

```cpp
AffExpr aff = affFromValGrad(y[i], x_eigen, jac.row(i), vars_);
...
case SQUARED:
  out->addQuadExpr(exprSquare(aff));
  break;
case ABS:
  out->addAbs(aff, 1);
  break;
case HINGE:
  out->addHinge(aff, 1);
  break;
```

Constraint conversion:

```cpp
AffExpr aff = affFromValGrad(y[i], x_eigen, jac.row(i), vars_);
if (type() == INEQ)
  out->addIneqCnt(aff);
else
  out->addEqCnt(aff);
```

## Direct Online SQP Block In Your Run File

When `online_mode_ == true`, your file bypasses TaskComposer after the offline trajectory is generated.

Source:

`robot_planning/src/motomini_planning_run.cpp`

```cpp
auto nlp = std::make_shared<trajopt_sqp::TrajOptQPProblem>();

for (int i = 0; i < num_steps; ++i)
{
  auto var = std::make_shared<trajopt_ifopt::JointPosition>(
      full_traj[i].position, joint_names, "Joint_Position_" + std::to_string(i));
  var->SetBounds(joint_limits);
  vars.push_back(var);
  nlp->addVariableSet(var);
}

auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(full_traj.size());
for (int i = 1; i < num_steps; ++i)
{
  auto evaluator = std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(
      collision_cache, manip, env_, collision_config, true);
  auto constraint = std::make_shared<trajopt_ifopt::DiscreteCollisionConstraint>(
      evaluator, vars[i], collision_config.max_num_cnt, false,
      "Collision_" + std::to_string(i));
  nlp->addConstraintSet(constraint);
}

nlp->setup();
auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
solver.init(nlp);
...
solver.stepSQPSolver();
```

The direct online QP currently contains:

- Decision variables: all trajectory joint positions `q_0 ... q_{N-1}`.
- Hard variable bounds: robot joint limits plus trust-region box.
- Collision constraints: one single-timestep discrete collision constraint for each `i = 1 ... N-1`.
- Slack penalties for violated constraints.
- No explicit squared cost to keep the optimized path close to `full_traj`.
- No explicit velocity, acceleration, or jerk smoothness cost.

This means the online problem is mainly a collision-feasibility problem, not a full trajectory-tracking objective. If online mode is important, add squared costs around the original `full_traj` and smoothness terms, or reuse the same TrajOpt-IFOPT profile path used offline.

Important implementation detail: this block constructs `SingleTimestepCollisionEvaluator`, whose constructor expects `CollisionEvaluatorType::DISCRETE`. The current file sets `LVS_DISCRETE` before creating the evaluator. If online mode is enabled, this mismatch can throw at runtime unless the collision type is changed to `DISCRETE` or the evaluator/constraint pair is changed to the continuous/LVS variant.

## Source File Map

Use these files to examine the complete source:

| Purpose | File |
|---|---|
| Your planner branch logic and direct online SQP | `robot_planning/src/motomini_planning_run.cpp` |
| TaskComposer pipeline graph | `/home/vinbui/vinh_ws/src/tesseract/tesseract_planning/tesseract_task_composer/config/task_composer_plugins.yaml` |
| TrajOpt-IFOPT planner entry | `/home/vinbui/vinh_ws/src/tesseract/tesseract_planning/tesseract_motion_planners/trajopt_ifopt/src/trajopt_ifopt_motion_planner.cpp` |
| TrajOpt-IFOPT waypoint term creation | `/home/vinbui/vinh_ws/src/tesseract/tesseract_planning/tesseract_motion_planners/trajopt_ifopt/src/profile/trajopt_ifopt_default_move_profile.cpp` |
| TrajOpt-IFOPT composite term creation | `/home/vinbui/vinh_ws/src/tesseract/tesseract_planning/tesseract_motion_planners/trajopt_ifopt/src/profile/trajopt_ifopt_default_composite_profile.cpp` |
| TrajOpt-IFOPT OSQP solver profile | `/home/vinbui/vinh_ws/src/tesseract/tesseract_planning/tesseract_motion_planners/trajopt_ifopt/src/profile/trajopt_ifopt_osqp_solver_profile.cpp` |
| QP matrix construction | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp` |
| Trust-region SQP loop | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_optimizers/trajopt_sqp/src/trust_region_sqp_solver.cpp` |
| OSQPEigen wrapper | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_optimizers/trajopt_sqp/src/osqp_eigen_solver.cpp` |
| Affine/quadratic expression math | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_optimizers/trajopt_sqp/src/expressions.cpp` |
| Cartesian constraint math | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_ifopt/src/constraints/cartesian_position_constraint.cpp` |
| Joint position constraint math | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_ifopt/src/constraints/joint_position_constraint.cpp` |
| Velocity smoothness math | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_ifopt/src/constraints/joint_velocity_constraint.cpp` |
| Acceleration smoothness math | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_ifopt/src/constraints/joint_acceleration_constraint.cpp` |
| Jerk smoothness math | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_ifopt/src/constraints/joint_jerk_constraint.cpp` |
| Collision constraint math | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_ifopt/src/constraints/collision/discrete_collision_constraint.cpp` |
| Collision evaluator and gradients | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_ifopt/src/constraints/collision/discrete_collision_evaluators.cpp` |
| Collision gradient utility | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_common/src/collision_utils.cpp` |
| Legacy TrajOpt planner entry | `/home/vinbui/vinh_ws/src/tesseract/tesseract_planning/tesseract_motion_planners/trajopt/src/trajopt_motion_planner.cpp` |
| Legacy SQP optimizer | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_sco/src/optimizers.cpp` |
| Legacy OSQP backend | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_sco/src/osqp_interface.cpp` |
| Legacy convexification math | `/home/vinbui/vinh_ws/src/tesseract/trajopt/trajopt_sco/src/modeling_utils.cpp` |

