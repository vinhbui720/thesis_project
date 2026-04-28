# OSQP: Operator Splitting Quadratic Program Solver

The OSQP (Operator Splitting Quadratic Program) solver is a numerical optimization package for solving convex quadratic programs (QPs). It is based on the Alternating Direction Method of Multipliers (ADMM).

## 1. Mathematical Problem Formulation

OSQP solves convex QPs in the following canonical form:

$$
\begin{array}{ll}
\text{minimize} & \frac{1}{2} x^T P x + q^T x \\
\text{subject to} & l \le A x \le u
\end{array}
$$

Where:
- $x \in \mathbb{R}^n$ is the optimization variable.
- $P \in \mathbb{S}^n_+$ is a positive semidefinite objective matrix (sparse).
- $q \in \mathbb{R}^n$ is the linear objective vector.
- $A \in \mathbb{R}^{m \times n}$ is the linear constraint matrix (sparse).
- $l, u \in \mathbb{R}^m$ are the lower and upper bound vectors. Elements can be $\pm \infty$.

## 2. The ADMM Algorithm

OSQP converts the problem into a form suitable for ADMM by introducing a new variable $z = Ax$:

$$
\begin{array}{ll}
\text{minimize} & \frac{1}{2} x^T P x + q^T x + \mathcal{I}_{l \le z \le u}(z) \\
\text{subject to} & Ax - z = 0
\end{array}
$$

Where $\mathcal{I}$ is the indicator function. The iterations are defined as:

### Step 1: Linear System Solve (KKT System)
Compute the next iterates $(\tilde{x}^{k+1}, \tilde{\nu}^{k+1})$ by solving:
$$
\begin{bmatrix} P + \sigma I & A^T \\ A & -\rho^{-1}I \end{bmatrix} \begin{bmatrix} \tilde{x}^{k+1} \\ \tilde{\nu}^{k+1} \end{bmatrix} = \begin{bmatrix} \sigma x^k - q \\ z^k - \rho^{-1} y^k \end{bmatrix}
$$
- $\sigma$: Regularization parameter.
- $\rho$: ADMM step-size parameter.

### Step 2: Projection
Update $x$ and $z$:
$$x^{k+1} = \tilde{x}^{k+1}$$
$$z^{k+1} = \Pi_{[l,u]} \left( \tilde{z}^{k+1} + \rho^{-1} y^k \right)$$
where $\Pi_{[l,u]}(v) = \min(\max(v, l), u)$ is the element-wise projection onto the hyperbox.

### Step 3: Dual Update
$$y^{k+1} = y^k + \rho (\tilde{z}^{k+1} - z^{k+1})$$

## 3. Convergence and Termination

### Residuals
The solver tracks primal and dual residuals to check for convergence:
- **Primal Residual:** $r_{\text{prim}}^k = Ax^k - z^k$
- **Dual Residual:** $r_{\text{dual}}^k = Px^k + q + A^T y^k$

### Termination Criteria
The algorithm stops when:
$$\| r_{\text{prim}}^k \|_\infty \le \epsilon_{\text{abs}} + \epsilon_{\text{rel}} \max \{ \|Ax^k\|_\infty, \|z^k\|_\infty \}$$
$$\| r_{\text{dual}}^k \|_\infty \le \epsilon_{\text{abs}} + \epsilon_{\text{rel}} \max \{ \|Px^k\|_\infty, \|A^T y^k\|_\infty, \|q\|_\infty \}$$

## 4. Infeasibility Detection

OSQP can detect if the problem is impossible to solve:

- **Primal Infeasibility:** Generates a vector $v \in \mathbb{R}^m$ such that:
  $$A^T v = 0, \quad u^T v_+ + l^T v_- < 0$$
- **Dual Infeasibility:** Generates a vector $s \in \mathbb{R}^n$ such that:
  $$P s = 0, \quad q^T s < 0, \quad (As)_i \text{ satisfies sign conditions relative to bounds.}$$

## 5. Core API Features

### Initialization and Workspace
- `osqp_setup`: Initializes the solver with $P, q, A, l, u$ and settings. It performs the matrix factorization of the KKT system (if using a direct solver).
- `OSQPWorkspace`: A structure holding the internal state, allowing for efficient memory management.

### Warm Starting
- `osqp_warm_start`: Allows providing initial guesses for $x$ and $y$. This is critical for MPC or trajectory optimization where the solution from the previous time step is a good starting point.

### Adaptive Parameters
- **Adaptive Rho:** OSQP automatically adjusts the step-size $\rho$ during iterations to balance primal and dual convergence.
- `osqp_update_rho`: Manually update the $\rho$ parameter if needed.

### Problem Updates
- `osqp_update_P`, `osqp_update_A`: Update the non-zero elements of the matrices without re-allocating memory.
- `osqp_update_lin_cost`, `osqp_update_bounds`: Efficiently update $q, l, u$.

### Polishing
- `settings.polish = 1`: After ADMM converges, the solver can perform a "polishing" step to get a high-accuracy solution by identifying active constraints and solving a smaller linear system.

### Embedded Features
- **Code Generation:** OSQP can generate tailored C code for a specific problem instance, removing the need for dynamic memory allocation, which is ideal for flight controllers or industrial robots.
- **Header-only / Library-free:** The generated code can be compiled directly into embedded targets.

---
*Generated for the Robot Planning Thesis Project.*
