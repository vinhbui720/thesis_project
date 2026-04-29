

### Mathematical Formulas: Adaptive Tangential Force

Here is the complete, step-by-step mathematical model for your trajectory optimization and obstacle avoidance, without any code. 

**1. The Velocity Deadband (Handling $v = 0$)**
To prevent the robot from vibrating or calculating undefined tangent vectors when it is completely still, we pass the raw goal velocity ($v_{goal}$) through a deadband filter. If the magnitude is below a tiny threshold ($v_{min}$), we treat the velocity as exactly zero.
$$v_{active} = \begin{cases} 
v_{goal} & \text{if } \|v_{goal}\| > v_{min} \\
\vec{0} & \text{otherwise}
\end{cases}$$

**2. The Projection & Intent Check**
We only want to apply a tangential sliding force if the robot is actively trying to move *into* the obstacle. Let $n$ be the normal vector pointing *away* from the obstacle.
$$v_{into} = v_{active} \cdot n$$
If $v_{into} \ge 0$, the robot is moving away or parallel, so $F_{tan} = \vec{0}$.
If $v_{into} < 0$, we project the velocity onto the obstacle's surface to find the raw sliding direction:
$$v_{proj} = v_{active} - (v_{active} \cdot n)n$$

**3. Tangent Direction with Memory Fallback**
If the robot is driving perfectly straight into the wall, $\|v_{proj}\|$ approaches 0, making normalization impossible. We use a cross-product fallback and a memory vector ($t_{last}$) to ensure the direction doesn't violently flip back and forth between control cycles.
$$t_{raw} = \begin{cases} 
\frac{v_{proj}}{\|v_{proj}\|} & \text{if } \|v_{proj}\| \ge \epsilon \\
\frac{n \times \vec{Z}}{\|n \times \vec{Z}\|} & \text{if } \|v_{proj}\| < \epsilon
\end{cases}$$

To prevent oscillation:
$$t = \begin{cases} 
-t_{raw} & \text{if } (t_{raw} \cdot t_{last}) < 0 \\
t_{raw} & \text{otherwise}
\end{cases}$$

**4. Adaptive Gain ($\gamma$)**
Instead of manually tuning distances, everything scales from a single safety radius ($R_{safe}$). The gain becomes stronger quadratically as the distance ($d$) decreases, creating a smooth "cushion" effect rather than a hard boundary.
$$\gamma(d) = \left( \max\left(0, \min\left(1, \frac{R_{safe} - d}{R_{safe} - d_{task}}\right)\right) \right)^2$$

**5. Final Tangential Force**
Finally, we scale the tangent direction vector by our base tuning parameter ($k_{tan}$) and the adaptive gain.
$$F_{tan} = k_{tan} \cdot \gamma(d) \cdot t$$

**6. Safe Exit / Release Filter**
When contact suddenly disappears, dropping the collision wrench to zero in one cycle can excite the Cartesian admittance integrator and create a strong vibration. Instead, use an asymmetric first-order filter with fast attack and slower release:
$$F_{coll,filtered}(k) = F_{coll,filtered}(k-1) + \alpha \left(F_{coll,raw}(k)-F_{coll,filtered}(k-1)\right)$$
with:
$$\alpha = 1 - e^{-2\pi f_c \Delta t}$$
Choose:
- high cutoff for force increase (contact entry)
- lower cutoff for force decrease (contact exit)

This keeps the protection responsive when entering collision, but prevents a hard rebound just after leaving the obstacle.

**7. Controller Deadband Safety**
Very small commanded velocities should not activate sliding behavior. Apply a Cartesian command deadband before using $v_{goal}$:
$$v_{cmd}=\vec{0} \quad \text{if} \quad \|v_{goal}\| \le v_{deadband}$$
This prevents the `v = 0` edge case from turning sensor noise into alternating tangent directions or unintended controller wake-ups.

**8. Parameter Reduction Rule**
To avoid creating too many tuning knobs, derive the new tangential and release-filter terms from the existing safety parameters:
- tangential deadband from the controller velocity limit
- tangential force ceiling from `collision_force_max_total`
- tangential gain from that same force ceiling
- tangent fallback epsilon from the guard-task distance span
- attack/release filter rates from the collision-wrench timeout or influence distance

This keeps tuning focused on the important physical quantities:
- collision distances
- controller velocity limits
- collision force limits
- collision timeout

---

### Real-World Example

Imagine an industrial collaborative robot (cobot) being guided by hand to grind a metal edge. 
* **The Deadband:** When the operator stops pushing the end-effector (velocity drops below 2 mm/s), the $v_{active}$ deadband instantly zeros out the calculations. The robot stays perfectly rigid and silent instead of vibrating as the sensors read micro-fluctuations.
* **The Adaptive Gain:** As the operator pushes the tool closer to the heavy metal jig ($d$ decreases), they don't hit a sudden invisible "wall". Instead, the quadratic $\gamma(d)$ curve smoothly takes over. The closer they push, the more the robot effortlessly glides sideways along the jig ($F_{tan}$) while resisting forward penetration, allowing for a perfectly smooth grinding pass.

To help you visualize how the mathematical vectors shift dynamically based on distance and incoming velocity, here is an interactive visualization of the equations.

```json?chameleon
{"component":"LlmGeneratedComponent","props":{"height":"700px","prompt":"Create an interactive 2D physics visualization of a robot point interacting with an obstacle boundary based on collision avoidance math. \n\nInitial State: Set a safety radius R_safe of 0.08. \n\nStrategy: Use a Standard Layout simulator using Canvas or D3. \n\nInputs: Provide sliders for 'Distance to Obstacle (d)' (0.01 to 0.15), 'Goal Velocity Magnitude' (0 to 10), and 'Goal Velocity Angle' (-90 to 90 degrees relative to the obstacle normal). \n\nBehavior: Visually render a flat obstacle surface on the right and a robot particle approaching it from the left. Draw the normal vector pointing away from the surface. Draw the input 'Goal Velocity' vector. Dynamically calculate and draw the 'Normal Force' vector (pushing away, scaling by 1/d) and the 'Tangential Force' vector (sliding parallel to the surface). Implement a velocity deadband: if the Goal Velocity Magnitude is very close to 0, visually hide the Tangential Force vector. Implement the quadratic adaptive gain: as 'd' gets smaller than R_safe, multiply the Tangential Force length quadratically. Display a live legend showing the calculated mathematical magnitudes of v_active, F_normal, and F_tangent.","id":"im_b97e1191acdc73fb"}}
```
