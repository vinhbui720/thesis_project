# Lý Thuyết Bayesian Optimization — Robot Planning

---

## Phần 1: Bayesian Optimization Tổng Quát

### 1.1 Bài Toán

Bayesian Optimization (BO) giải bài toán tối ưu **hộp đen** (black-box): hàm mục tiêu $f(\mathbf{x})$ không có dạng giải tích rõ ràng, tốn kém để đánh giá (mỗi lần gọi = chạy một thí nghiệm thực), và không có gradient.

$$\mathbf{x}^* = \arg\min_{\mathbf{x} \in \mathcal{X}} f(\mathbf{x})$$

Trong đó $\mathcal{X} \subset \mathbb{R}^d$ là không gian tham số có giới hạn (bounded).

**Tại sao cần BO?** Thay vì random search (không học từ quá khứ) hay grid search (tổ hợp bùng nổ), BO xây dựng một **mô hình xác suất** về $f$ từ các quan sát đã có, rồi dùng mô hình đó để quyết định **điểm nào nên thử tiếp theo** sao cho vừa khai thác vùng tốt đã biết (exploitation), vừa khám phá vùng chưa biết (exploration).

---

### 1.2 Surrogate Model: Gaussian Process (GP)

Mô hình nền tảng của BO là **Gaussian Process**. Sau khi quan sát $n$ điểm $\mathcal{D}_n = \{(\mathbf{x}_i, y_i)\}_{i=1}^n$ với $y_i = f(\mathbf{x}_i) + \epsilon$:

**Prior:**
$$f(\mathbf{x}) \sim \mathcal{GP}\bigl(\mu_0(\mathbf{x}),\, k(\mathbf{x}, \mathbf{x}')\bigr)$$

**Posterior tại điểm mới $\mathbf{x}$:**
$$\mu_n(\mathbf{x}) = \mu_0(\mathbf{x}) + \mathbf{k}^\top (\mathbf{K} + \sigma^2 \mathbf{I})^{-1} (\mathbf{y} - \boldsymbol{\mu}_0)$$

$$\sigma_n^2(\mathbf{x}) = k(\mathbf{x}, \mathbf{x}) - \mathbf{k}^\top (\mathbf{K} + \sigma^2 \mathbf{I})^{-1} \mathbf{k}$$

Trong đó:
- $\mathbf{k} = [k(\mathbf{x}, \mathbf{x}_1), \dots, k(\mathbf{x}, \mathbf{x}_n)]^\top$ — vector covariance giữa điểm mới và tất cả điểm đã quan sát
- $\mathbf{K}_{ij} = k(\mathbf{x}_i, \mathbf{x}_j)$ — ma trận covariance $n \times n$
- $\sigma^2$ — phương sai nhiễu quan sát
- $\mu_n(\mathbf{x})$ — **dự đoán trung bình** tại $\mathbf{x}$
- $\sigma_n^2(\mathbf{x})$ — **độ không chắc chắn** tại $\mathbf{x}$ (cao ở vùng chưa khám phá)

---

### 1.3 Sampler Thực Tế: Tree-structured Parzen Estimator (TPE)

Trong hai script này, Optuna sử dụng **TPESampler** thay cho GP thuần túy. TPE hoạt động trực tiếp trên phân phối tham số thay vì ước lượng $f(\mathbf{x})$.

Sau $n$ lần quan sát, TPE phân chia kết quả thành hai nhóm:

$$\mathcal{D}^+ = \{(\mathbf{x}_i, y_i) \mid y_i \leq y_\gamma\}, \quad \mathcal{D}^- = \{(\mathbf{x}_i, y_i) \mid y_i > y_\gamma\}$$

Trong đó $y_\gamma$ là ngưỡng phân vị thứ $\gamma$ (mặc định $\gamma = 0.25$, tức top 25% tốt nhất).

Hai mô hình mật độ được xây dựng bằng Kernel Density Estimation (KDE):

$$l(\mathbf{x}) = p(\mathbf{x} \mid y \leq y_\gamma), \quad g(\mathbf{x}) = p(\mathbf{x} \mid y > y_\gamma)$$

**Acquisition function (Expected Improvement theo TPE):**

$$\text{EI}(\mathbf{x}) \propto \frac{l(\mathbf{x})}{g(\mathbf{x})}$$

Điểm tiếp theo được chọn:
$$\mathbf{x}_{n+1} = \arg\max_{\mathbf{x}} \frac{l(\mathbf{x})}{g(\mathbf{x})}$$

Trực giác: **tử số** $l(\mathbf{x})$ lớn nghĩa là $\mathbf{x}$ hay xuất hiện trong vùng tốt — **mẫu số** $g(\mathbf{x})$ lớn nghĩa là $\mathbf{x}$ hay xuất hiện trong vùng tệ. Tối đa hóa tỉ số này = chọn điểm vừa phổ biến ở vùng tốt, vừa hiếm ở vùng tệ.

**Multivariate TPE** (dùng trong cả hai script với `multivariate=True`): thay vì xây $l, g$ độc lập cho từng chiều, TPE xây dựng joint density để nắm bắt **tương quan giữa các tham số**.

---

### 1.4 Vòng Lặp Tổng Quát Mỗi Iteration

```
Khởi tạo: chạy n_startup_trials = 10 lần Random Search → thu được D_10

Với mỗi trial t = 11, 12, ..., N:
  ┌────────────────────────────────────────────────────────────┐
  │ 1. FIT surrogate: cập nhật l(x) và g(x) từ D_{t-1}        │
  │                                                            │
  │ 2. OPTIMIZE acquisition:                                   │
  │       x_t = argmax  l(x) / g(x)                           │
  │                                                            │
  │ 3. EVALUATE: chạy thí nghiệm thực → y_t = f(x_t)          │
  │    (= khởi động ROS, chạy robot, tính cost)                │
  │                                                            │
  │ 4. UPDATE: D_t = D_{t-1} ∪ {(x_t, y_t)}                  │
  │                                                            │
  │ 5. RECORD: lưu best nếu y_t < y*                          │
  └────────────────────────────────────────────────────────────┘
```

**Chuẩn hóa không gian tham số** (dùng trong cả hai script để theo dõi exploration distance):

$$\tilde{x}_k = \frac{x_k - x_k^{\min}}{x_k^{\max} - x_k^{\min}} \in [0, 1]$$

**Exploration distance** giữa hai lần liên tiếp:

$$d_t = \| \tilde{\mathbf{x}}_t - \tilde{\mathbf{x}}_{t-1} \|_2$$

Nếu $d_t$ nhỏ dần theo $t$ → TPE đang **hội tụ** vào vùng tốt. Nếu $d_t$ vẫn lớn → vẫn đang **khám phá**.

---

## Phần 2: Feedback Controller Optimizer (`bayes_optimize_feedback.py`)

### 2.1 Không Gian Tham Số

7 tham số điều khiển bộ admittance/tracking:

| Tham số | Ý nghĩa vật lý | Miền tìm kiếm |
|---|---|---|
| $m_{\min}$ (`m_pos_min`) | Khối lượng ảo tối thiểu (inertia thấp → phản ứng nhanh) | $[0.001,\ 0.1]$ |
| $m_{\max}$ (`m_pos_max`) | Khối lượng ảo tối đa (inertia cao → mượt, chậm) | $[0.1,\ 0.5]$ |
| $k_{\min}$ (`k_pos_min`) | Độ cứng ảo tối thiểu (stiffness thấp → linh hoạt) | $[100,\ 500]$ |
| $k_{\max}$ (`k_pos_max`) | Độ cứng ảo tối đa (stiffness cao → bám chặt) | $[1000,\ 3000]$ |
| $\zeta$ (`zeta_pos`) | Hệ số giảm chấn (damping ratio) | $[0.5,\ 1.2]$ |
| $\lambda$ (`adaptive_lambda`) | Cường độ thích nghi (điều chỉnh tốc độ cập nhật $m, k$) | $[10,\ 100]$ |
| $\alpha$ (`adaptive_alpha_pos`) | Độ nhạy sai số trong thích nghi | $[5,\ 50]$ |

**Ràng buộc cứng:**
$$m_{\min} < m_{\max}, \quad k_{\min} < k_{\max}$$
Nếu vi phạm: $f(\mathbf{x}) = 10^9$ (loại ngay, không chạy thí nghiệm).

---

### 2.2 Thí Nghiệm Một Vòng Lặp (`run_tracking_trial.py`, 10 giây)

Robot bám theo quỹ đạo tuyến tính dọc trục Y với vận tốc $v = 0.1$ m/s. Quỹ đạo chia 3 pha:

**Pha Warmup** ($0 \leq t < t_w$, S-curve):

$$y_{\text{target}}(t) = y_{\text{init}} + (y_{\text{start}} - y_{\text{init}}) \cdot \frac{1 - \cos(\pi \cdot t/t_w)}{2}$$

**Pha Di Chuyển** ($t_w \leq t < t_{\text{end}}$, tuyến tính):

$$y_{\text{target}}(t) = y_{\text{start}} + v \cdot (t - t_w)$$

**Pha Giữ** ($t \geq t_{\text{end}}$):

$$y_{\text{target}}(t) = y_{\text{end}}$$

---

### 2.3 Các Metric Đo Lường

**Phase 1 — Approach (bắt kịp target đang di chuyển):**

$$t_{\text{rise}} = \min\bigl\{t : y_{\text{fb}}(t) \geq y_{\text{start}} + 0.9 \cdot (y_{\text{end}} - y_{\text{start}})\bigr\} - t_w$$

$$e_{\text{lag}} = \frac{1}{|T_m|} \int_{T_m} |y_{\text{target}}(t) - y_{\text{fb}}(t)|\, dt \quad \text{(mean lag trong pha di chuyển)}$$

$$r_{\text{vel}} = \max_j \frac{|\dot{q}_j|}{\dot{q}_j^{\max}} \quad \text{(tỉ lệ vận tốc khớp lớn nhất so với giới hạn)}$$

$$J_{\text{approach}} = 2.0 \cdot t_{\text{rise}} + 3.0 \cdot e_{\text{lag}} + 50.0 \cdot r_{\text{vel}}^{\text{vio}}$$

**Phase 2 — Near Target (ổn định sau khi đến đích):**

$$e_{\text{overshoot}} = \max\bigl(0,\ \max_{t > t_{\text{end}}} y_{\text{fb}}(t) - y_{\text{end}}\bigr)$$

$$t_{\text{settle}} = \max\bigl\{t : |y_{\text{fb}}(t) - y_{\text{end}}| > 0.002\bigr\} - t_{\text{end}}$$

$$e_{\text{final}} = \frac{1}{|T_f|} \int_{T_f} |y_{\text{fb}}(t) - y_{\text{end}}|\, dt \quad (T_f = \text{2 giây cuối})$$

$$\sigma_{\text{jitter}} = \text{std}\bigl(y_{\text{fb}}(t)\ \text{trong 100 mẫu cuối}\bigr)$$

$$J_{\text{near}} = 50.0 \cdot e_{\text{overshoot}} + 2.0 \cdot t_{\text{settle}} + 5.0 \cdot e_{\text{final}}$$

**Phase 3 — Tracking (chất lượng bám trong pha di chuyển):**

$$\text{RMSE} = \sqrt{\frac{1}{|T_m|}\int_{T_m} \bigl(y_{\text{target}}(t) - y_{\text{fb}}(t)\bigr)^2 dt}$$

$$\tau_{\text{lag}} = \frac{e_{\text{lag}}}{v} \quad \text{(độ trễ pha, tính bằng giây)}$$

$$\nu_{\text{noise}} = \frac{1}{N-1}\sum_{i=1}^{N-1} |\dot{y}_{\text{fb}}(t_{i+1}) - \dot{y}_{\text{fb}}(t_i)| \quad \text{(nhiễu vi phân vận tốc)}$$

$$J_{\text{tracking}} = 5.0 \cdot \text{RMSE} + 3.0 \cdot \tau_{\text{lag}} + 2.0 \cdot \nu_{\text{noise}}$$

---

### 2.4 Hàm Chi Phí Tổng (Feedback)

$$J(\mathbf{x}) = \underbrace{1.5 \cdot t_{\text{rise}}}_{\text{tốc độ}} + \underbrace{2.0 \cdot \frac{\text{RMSE}}{0.001} + 2.0 \cdot \frac{e_{\text{lag}}}{0.001}}_{\text{độ chính xác bám}} + \underbrace{50.0 \cdot \frac{e_{\text{overshoot}}}{0.001}}_{\text{không vọt lố}} + \underbrace{4.0 \cdot \frac{\sigma_{\text{jitter}}}{0.0002}}_{\text{ổn định}} + \underbrace{1.0 \cdot t_{\text{settle}}}_{\text{hội tụ}}$$

**Các hạng phạt cứng (soft constraints qua penalty lớn):**

$$P_1 = \begin{cases} 10^6 + e_{\text{overshoot}} \cdot 10^8 & \text{nếu } e_{\text{overshoot}} > 0.001\ \text{m} \\ 0 & \text{ngược lại} \end{cases}$$

$$P_2 = \begin{cases} 10^5 + (v_{\text{fb}} - 0.85) \cdot 10^6 & \text{nếu } v_{\text{fb}} > 0.85\ \text{m/s} \\ 0 & \text{ngược lại} \end{cases}$$

$$P_3 = \begin{cases} 10^6 + (r_{\text{vel}} - 0.95) \cdot 10^6 & \text{nếu } r_{\text{vel}} > 0.95 \\ 0 & \text{ngược lại} \end{cases}$$

$$\boxed{f(\mathbf{x}) = J(\mathbf{x}) + P_1 + P_2 + P_3}$$

**Diễn giải:** Hệ số $50.0$ trước $e_{\text{overshoot}}$ lớn gấp 25 lần hệ số RMSE — điều này ưu tiên tuyệt đối **không vọt lố** (an toàn) trước khi tối ưu tốc độ bám. Penalty $10^8$ biến overshoot thành tường lửa: bất kỳ bộ tham số nào để robot vọt qua đích đều bị loại khỏi competition.

---

## Phần 3: Collision Controller Optimizer (`bayes_optimize_collision.py`)

### 3.1 Không Gian Tham Số — 4 Pha Tách Biệt

Bộ tối ưu này chia tham số va chạm thành 4 pha có thể tune độc lập hoặc cùng nhau (`--phase 1/2/3/4/all`):

**Phase 1 — Normal Force** (phản ứng pháp tuyến khi chạm vật cản):

| Tham số | Ý nghĩa | Miền |
|---|---|---|
| $F_n^{\max}$ (`collision_normal_force_max`) | Lực pháp tuyến tối đa cho phép | $[2, 10]$ N |
| $b_n$ (`collision_normal_damping`) | Hệ số giảm chấn pháp tuyến | $[2, 25]$ |
| $F_c^{\max}$ (`collision_force_max_per_contact`) | Lực tối đa mỗi điểm tiếp xúc | $[2, 10]$ N |
| $f_a$ (`collision_force_attack_hz`) | Tần số tăng lực (attack rate) | $[8, 35]$ Hz |
| $f_r$ (`collision_force_release_hz`) | Tần số giảm lực (release rate) | $[3, 20]$ Hz |
| $s_r$ (`collision_force_slew_rate`) | Tốc độ biến thiên lực | $[20, 150]$ N/s |

**Phase 2 — Tangent Force** (lực tiếp tuyến để trượt qua cạnh vật cản):

| Tham số | Ý nghĩa | Miền |
|---|---|---|
| $k_t$ (`collision_tangent_gain`) | Gain lực tiếp tuyến | $[0.5, 10]$ |
| $F_t^{\max}$ (`collision_tangent_force_max`) | Lực tiếp tuyến tối đa | $[1, 8]$ N |
| $\rho$ (`collision_tangent_force_ratio`) | Tỉ lệ lực tiếp tuyến / pháp tuyến | $[0.15, 0.60]$ |
| $\gamma$ (`collision_tangent_gamma_power`) | Số mũ hàm phi tuyến | $[1.0, 3.5]$ |
| $s_t$ (`collision_tangent_speed_scale`) | Hệ số tỉ lệ vận tốc tiếp tuyến | $[0.02, 0.15]$ |
| $d_{\text{db}}$ (`collision_tangent_velocity_deadband`) | Vùng chết vận tốc tiếp tuyến | $[0.003, 0.03]$ m/s |

**Phase 3 — Release** (nhả lực khi rời vật cản, tránh bouncing):

| Tham số | Ý nghĩa | Miền |
|---|---|---|
| $f_r'$ (`collision_force_release_hz`) | Release rate tinh chỉnh | $[4, 25]$ Hz |
| $s_r'$ (`collision_force_slew_rate`) | Slew rate tinh chỉnh | $[20, 150]$ N/s |
| $p_{\text{fade}}$ (`collision_normal_fade_power`) | Số mũ hàm fade lực khi rời | $[0.7, 3.0]$ |
| $s_f$ (`collision_force_scale`) | Hệ số scale lực tổng | $[0.3, 1.5]$ |
| $F^{\max}$ (`collision_force_max`) | Giới hạn lực toàn cục | $[2, 8]$ N |
| $\tau$ (`collision_constraint_timeout_sec`) | Timeout ràng buộc va chạm | $[0.08, 0.35]$ s |

**Phase 4 — Distances** (các ngưỡng khoảng cách phân vùng):

| Tham số | Ý nghĩa | Miền |
|---|---|---|
| $d_{\text{inf}}$ (`collision_influence_distance`) | Bắt đầu ảnh hưởng (detection) | $[0.02, 0.08]$ m |
| $d_{\text{safe}}$ (`collision_safe_distance`) | Vùng an toàn | $[0.01, 0.04]$ m |
| $d_{\text{guard}}$ (`collision_guard_distance`) | Vùng cảnh giác | $[0.005, 0.03]$ m |
| $d_{\text{task}}$ (`collision_task_distance`) | Vùng tác vụ (tiếp xúc) | $[0.002, 0.015]$ m |

**Ràng buộc cứng về cấu trúc phân vùng:**

$$d_{\text{inf}} > d_{\text{safe}} > d_{\text{guard}} > d_{\text{task}}$$

Nếu vi phạm: $f(\mathbf{x}) = 10^9$ (không chạy thí nghiệm).

---

### 3.2 Thí Nghiệm Một Vòng Lặp (`run_collision_trial.py`, 15 giây)

Robot bám quỹ đạo tuyến tính dọc Y với $v = 0.15$ m/s, nhưng có vật cản nằm chắn trên đường. Kịch bản kiểm tra:

1. **Phát hiện** vật cản qua distance sensor
2. **Né/trượt** qua vật cản bằng lực tiếp tuyến
3. **Tiếp tục** tiến đến đích $y_{\text{end}}$
4. **Ổn định** sau khi qua vật cản

**Quỹ đạo target** giống bộ feedback (S-curve warmup + tuyến tính):

$$y_{\text{ref}}(t) = \begin{cases} y_{\text{start}} + (y_{\text{end}} - y_{\text{start}}) \cdot \frac{1-\cos(\pi t/t_w)}{2} & 0 \leq t < t_w \\ \min\bigl(y_{\text{start}} + v \cdot (t - t_w),\ y_{\text{end}}\bigr) & t \geq t_w \end{cases}$$

---

### 3.3 Các Metric Đo Lường

**Penetration depth** (xuyên sâu vào vật cản — ràng buộc an toàn):

$$\delta_p = \max\bigl(0,\ d_{\text{stop}} - \min_t d(t)\bigr)$$

Trong đó $d(t)$ là khoảng cách tức thời đến vật cản, $d_{\text{stop}} = 0.001$ m.

**Force jitter** (nhiễu lực trong vùng tiếp xúc):

$$\sigma_F = \text{std}\Bigl(\Delta \|F(t)\|\ \text{khi}\ d(t) < 0.05\ \text{m}\Bigr)$$

**Velocity spike** (đột biến vận tốc khi va chạm/thoát):

$$s_v = \max_t \bigl|\, \|\dot{p}(t+1)\| - \|\dot{p}(t)\|\, \bigr|$$

**Progress error** (không qua được vật cản — ưu tiên cao nhất):

$$e_{\text{prog}} = \max\bigl(0,\ y_{\text{end}} - \max_t y_{\text{fb}}(t)\bigr)$$

**Dynamic tracking error** (sai số bám trong toàn bộ quỹ đạo):

$$e_{\text{dyn}} = \sqrt{\frac{1}{N}\sum_{i=1}^N \bigl[(y_{\text{fb},i} - y_{\text{ref},i})^2 + (x_{\text{fb},i} - x_{\text{ref},i})^2\bigr]}$$

**Error at stop** (sai số TẠI THỜI ĐIỂM target dừng — robot phải bắt kịp trước khi target dừng):

$$e_{\text{stop}} = |y_{\text{fb}}(t_{\text{stop}}) - y_{\text{ref}}(t_{\text{stop}})| + |x_{\text{fb}}(t_{\text{stop}}) - x_{\text{ref}}(t_{\text{stop}})|$$

Trong đó $t_{\text{stop}} = t_w + |y_{\text{end}} - y_{\text{start}}| / v$.

**Arrival jitter** (dao động sau khi qua vật cản, 3 giây cuối):

$$\sigma_{\text{arr}} = \text{std}(y_{\text{fb}}) + \text{std}(x_{\text{fb}}) \quad (t > T - 3)$$

**Final error** (sai số vị trí cuối):

$$e_{\text{final}} = \frac{1}{|T_f|} \sum_{T_f} \bigl(|y_{\text{fb}} - y_{\text{end}}| + |x_{\text{fb}} - x_{\text{ref}}|\bigr)$$

---

### 3.4 Hàm Chi Phí Tổng (Collision)

$$J(\mathbf{x}) = \underbrace{1000 \cdot \frac{e_{\text{prog}}}{0.1}}_{\text{phải qua được đích}} + \underbrace{800 \cdot \frac{e_{\text{stop}}}{0.01}}_{\text{bắt kịp target}} + \underbrace{100 \cdot \frac{\delta_p}{0.01}}_{\text{không xuyên}} + \underbrace{50 \cdot \frac{\sigma_F}{1.0}}_{\text{lực mượt}} + \underbrace{20 \cdot \frac{s_v}{0.1}}_{\text{vận tốc mượt}} + \underbrace{300 \cdot \frac{\sigma_{\text{arr}}}{0.002}}_{\text{ổn định sau né}} + \underbrace{100 \cdot \frac{e_{\text{final}}}{0.01}}_{\text{sai số cuối}} + \underbrace{50 \cdot \frac{e_{\text{dyn}}}{0.05}}_{\text{bám tổng thể}}$$

**Các hạng phạt cứng (theo thứ tự ưu tiên):**

$$P_1 = \begin{cases} 10^5 + e_{\text{prog}} \cdot 10^6 & e_{\text{prog}} > 0.05\ \text{m} \quad \text{(không qua vật cản)} \\ 0 & \text{ngược lại} \end{cases}$$

$$P_2 = \begin{cases} 10^4 + (e_{\text{stop}} - 0.02) \cdot 10^5 & e_{\text{stop}} > 0.02\ \text{m} \quad \text{(chậm hơn target)} \\ 0 & \text{ngược lại} \end{cases}$$

$$P_3 = \begin{cases} 10^6 + \delta_p \cdot 10^8 & \delta_p > 0.015\ \text{m} \quad \text{(xuyên sâu quá)} \\ 0 & \text{ngược lại} \end{cases}$$

$$P_4 = \begin{cases} 10^4 + (\sigma_F - 2.0) \cdot 10^5 & \sigma_F > 2.0\ \text{N} \quad \text{(lực rung) } \\ 0 & \text{ngược lại} \end{cases}$$

$$P_5 = \begin{cases} 10^4 + (\sigma_{\text{arr}} - 0.005) \cdot 10^6 & \sigma_{\text{arr}} > 0.005\ \text{m} \quad \text{(dao động sau né)} \\ 0 & \text{ngược lại} \end{cases}$$

$$\boxed{f(\mathbf{x}) = J(\mathbf{x}) + P_1 + P_2 + P_3 + P_4 + P_5}$$

**Diễn giải thứ tự ưu tiên:** Hệ số $10^8$ trong $P_3$ lớn hơn mọi thứ khác — **không xuyên vật cản là tuyệt đối**. Sau đó $P_1$ ($10^6$) đảm bảo robot **phải qua được đích** trước khi xét các tiêu chí mượt mà. Đây là thứ bậc ưu tiên an toàn → nhiệm vụ → chất lượng.

---

## Tóm Tắt So Sánh

| Tiêu chí | Feedback Optimizer | Collision Optimizer |
|---|---|---|
| **Số tham số** | 7 | 4 pha (6 + 6 + 6 + 4 = 22 tối đa) |
| **Thời gian 1 trial** | 10 giây | 15 giây |
| **Metric cốt lõi** | RMSE bám quỹ đạo, overshoot, rise time | Progress qua vật, penetration depth, jitter |
| **Ưu tiên tối thượng** | Không vọt lố ($P_1$, hệ số $10^8$) | Không xuyên vật cản ($P_3$, hệ số $10^8$) |
| **Ràng buộc cấu trúc** | $m_{\min} < m_{\max},\ k_{\min} < k_{\max}$ | $d_{\text{inf}} > d_{\text{safe}} > d_{\text{guard}} > d_{\text{task}}$ |
| **Startup trials** | 10 (random) | 10 (random) |
| **Sampler** | TPE multivariate + group | TPE multivariate + group |
| **Phased tuning** | Không (1 pha duy nhất) | Có (4 pha tách biệt hoặc all) |
