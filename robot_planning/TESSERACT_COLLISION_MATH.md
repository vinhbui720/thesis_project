# Tesseract Collision Processing: Mathematics & API (Deep Dive)

Tesseract is a flexible collision checking framework that abstracts backend physics engines (primarily **Bullet Physics** and **FCL**). It uses a plugin-based architecture to provide consistent distance and penetration information for robot planning.

## 1. Core Mathematical Algorithms

### A. GJK (Gilbert-Johnson-Keerthi) - Distance Math
Used for convex shapes and the foundation of distance queries.
- **Support Function:** $S_A(\vec{d}) = \text{argmax}_{\vec{x} \in A} (\vec{x} \cdot \vec{d})$.
- **Minkowski Difference:** $C = A \ominus B$.
- GJK finds the point in $C$ closest to the origin. If the origin is inside, the shapes overlap.
- **API implementation:** In Bullet, this is handled via `btGjkPairDetector`.

### B. EPA (Expansion Polytope Algorithm) - Penetration Math
Triggered when GJK detects an overlap (distance $\le 0$).
- EPA expands the simplex from GJK to find the point on the boundary of the Minkowski Difference closest to the origin.
- This gives the **Penetration Depth** ($p$) and the **Normal Vector** ($\vec{n}$) required to separate the shapes.

### C. Mesh Processing & Acceleration
For non-convex meshes (robot STLs), Tesseract does not use GJK directly on the whole mesh. Instead, it uses Hierarchical Acceleration:

| Backend | Structure | Acceleration Method |
| :--- | :--- | :--- |
| **Bullet** | `btCompoundShape` | Uses a **Dynamic Bounding Volume Tree (DBVT)**. Each leaf is a `btTriangleShapeEx`. |
| **FCL** | `fcl::BVHModel` | Uses **OBBRSS** (Oriented Bounding Box & Rectangle Swept Sphere) hierarchy. |

- **Triangle-to-Triangle:** When the BVH/DBVT leaf is reached, a specialized narrow-phase check is performed between individual triangles.
- **Margin (Padding):** Tesseract adds a collision margin (default usually 0) to shapes. In Bullet, this is `btCollisionShape::setMargin()`.

---

## 2. Tesseract Environment Setup & Workflow

### A. Component Roles
1.  **`tesseract_urdf::URDFParser`**: Parses the robot's URDF to create a `SceneGraph`. It converts `<collision>` tags into `tesseract_geometry` objects (Box, Sphere, Mesh, etc.).
2.  **`tesseract_common::ResourceLocator`**: Resolves file paths (e.g., `package://robot_description/mesh.stl`) so the parser can load the raw data.
3.  **`tesseract_scene_graph::SceneState`**: A snapshot of the robot's joint positions and the resulting link transforms ($T_{world\_link}$).
4.  **`tesseract_environment::Environment`**: The master class. It maintains the `SceneGraph` and uses a `StateSolver` to update the `DiscreteContactManager`.

### B. Update Cycle
```cpp
// 1. Update the state (Joint Positions -> Link Transforms)
env->setState(joint_names, joint_values);

// 2. The environment pushes transforms to the manager
// Internally: manager->setCollisionObjectsTransform(env->getState().link_transforms);

// 3. Perform the check
tesseract_collision::ContactResultMap results;
env->getDiscreteContactManager()->contactTest(results, request);
```

---

## 3. The `ContactResult` API: Vector Math

When a collision or "near miss" is detected, Tesseract returns a `ContactResult`. Understanding the orientation of these vectors is critical for Jacobian-based planners (like TrajOpt).

### A. Vector Orientation
- **`normal` ($\vec{n}$):** This unit vector **points from Link 0 to Link 1**.
  - To move Link 1 *away* from Link 0, follow the normal.
  - To move Link 0 *away* from Link 1, follow the negative normal ($-\vec{n}$).
- **`distance` ($d$):** 
  - $d > 0$: Objects are separated by distance $d$.
  - $d \le 0$: Objects are penetrating by depth $|d|$.

### B. Nearest Points
- **`nearest_points[0]`**: The point on Link 0 closest to Link 1 (World Coordinates).
- **`nearest_points[1]`**: The point on Link 1 closest to Link 0 (World Coordinates).
- **`nearest_points_local[0/1]`**: Same as above, but in the Link's own coordinate frame.

### C. Identification
- **`link_names[0/1]`**: Names of the colliding links.
- **`shape_id[0/1]`**: Index of the geometry within the link (a link can have multiple collision meshes).
- **`subshape_id[0/1]`**: 
  - For **Meshes**: This is the **Triangle Index** in the STL.
  - For **Octomaps**: This is the internal ID of the voxel.

---

## 4. Implementation Details (Bullet Backend)

In `bullet_utils.cpp`, the result is populated as:
```cpp
contact.distance = cp.m_distance1;
contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);
contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
```
*Note: Bullet's internal `m_normalWorldOnB` points from B to A, so Tesseract multiplies by -1 to ensure the standard 0 -> 1 orientation.*
