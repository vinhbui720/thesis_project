# Tesseract Collision Processing: Mathematics & API

Tesseract uses **Bullet Physics** as its primary back-end for collision detection. The process relies on Computational Geometry to determine the minimum distance between complex meshes.

## 1. Mathematical Equations

### A. Minkowski Difference
The core of modern collision detection for convex shapes (like the convex hulls of your robot links) is the **Minkowski Difference**.

For two sets of points (links) $A$ and $B$, the Minkowski Difference $C$ is defined as:
$$C = A \ominus B = \{ \vec{a} - \vec{b} \mid \vec{a} \in A, \vec{b} \in B \}$$

*   If the origin $(0,0,0)$ is inside $C$, the shapes $A$ and $B$ are **colliding**.
*   The minimum distance between $A$ and $B$ is the distance from the origin to the closest point on the boundary of $C$.

### B. GJK Algorithm (Distance & Nearest Points)
The **Gilbert-Johnson-Keerthi (GJK)** algorithm finds the minimum distance without explicitly calculating the entire Minkowski set. It uses a **Support Function** $S_A(\vec{d})$:
$$S_A(\vec{d}) = \text{argmax}_{\vec{x} \in A} (\vec{x} \cdot \vec{d})$$

The algorithm iteratively finds the point in $C$ closest to the origin:
1.  Search for the simplex (point, line, triangle, or tetrahedron) in $C$ that is closest to the origin.
2.  If the simplex contains the origin, distance is $0$ (collision).
3.  Otherwise, the distance $d$ is:
    $$d = \min \| \vec{a} - \vec{b} \| \quad \text{where } \vec{a} \in A, \vec{b} \in B$$

### C. EPA (Penetration Depth)
If GJK detects a collision (distance < 0), the **Expansion Polytope Algorithm (EPA)** is triggered. It expands the simplex found by GJK to find the **Penetration Vector** $\vec{v}$ and depth $p$:
$$p = \min \{ \|\vec{v}\| \mid \vec{v} \in \partial C \}$$
This vector $\vec{v}$ represents the "shortest path" to move the objects apart to stop the collision.

### D. BVH (Non-Convex Meshes)
For general meshes (your STL files), Tesseract uses a **Bounding Volume Hierarchy (BVH)**:
1.  Each mesh is wrapped in a tree of AABBs (Axis-Aligned Bounding Boxes).
2.  Tesseract prunes the search by ignoring branches of the tree that do not overlap.
3.  When leaves are reached, it performs a triangle-to-triangle distance calculation.

---

## 2. Tesseract API Detail

### A. Initialization & Setup
To perform collision checking, you must retrieve the Contact Manager from the environment:

```cpp
// 1. Get the manager
auto manager = env->getDiscreteContactManager();

// 2. Set Active Links (Optimization: only check things that move)
manager->setActiveCollisionObjects(env->getActiveLinkNames());

// 3. Set Margin (Padding)
manager->setDefaultCollisionMargin(0.01); // 1cm padding
```

### B. The Contact Request
The `ContactRequest` object configures what the math engine should calculate:

```cpp
tesseract_collision::ContactRequest request(tesseract_collision::ContactTestType::ALL);
request.calculate_distance = true;  // Trigger GJK/BVH Distance math
request.calculate_gradient = true;  // Trigger normal vector calculation
```

### C. The Execution Call
```cpp
tesseract_collision::ContactResultMap results;
manager->contactTest(results, request);
```

### D. Extracted Information (ContactResult)
For every pair of links found within the threshold, a `ContactResult` is generated. Here is the information you get:

| Field | Data Type | Description |
| :--- | :--- | :--- |
| `distance` | `double` | The absolute minimum distance (m). Negative if penetrating. |
| `nearest_points[0]` | `Eigen::Vector3d` | Closest point on the surface of **Link A**. |
| `nearest_points[1]` | `Eigen::Vector3d` | Closest point on the surface of **Link B**. |
| `normal` | `Eigen::Vector3d` | Unit vector pointing from point 1 to point 0. |
| `link_names[0/1]` | `std::string` | Names of the two links involved. |
| `cc_type` | `CastCollisionType` | Used for continuous collision (time of contact). |

### E. Getting the "Collision Vector"
In your code, you can derive the direction of the potential collision like this:
```cpp
for (const auto& pair : results) {
    for (const auto& res : pair.second) {
        // This vector points Link B -> Link A
        Eigen::Vector3d collision_vector = res.nearest_points[0] - res.nearest_points[1];
        double actual_dist = res.distance;
    }
}
```
