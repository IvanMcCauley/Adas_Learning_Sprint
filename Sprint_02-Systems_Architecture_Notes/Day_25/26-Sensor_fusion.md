# Day 25 - Sensor Fusion

## Big-Picture Overview (what “sensor fusion” means)
- **Goal:** Build a coherent, timely, uncertainty-aware world model from multiple imperfect sensors.
- **Levels:** Feature/measurement → **Track level (today’s focus)** → Map/ego.
- **Time & frames:** Every message has a timestamp. Keep a state time `t_state`. Predict to stamp; drop **OOSM** (`t_meas < t_state`) and **stale** (`now - t_meas` > budget). Use consistent frames (sensor → `base_link` → map).
- **Estimation toolbox (minimal):** KF/EKF with `F(Δt), Q(Δt), H, R, P, K, S, ν, NIS`. Linear → KF; nonlinear radar (range/bearing/Doppler) → EKF (Jacobian).
- **Association:** Gate with NIS, then **Nearest Neighbor (NN)** (smallest Mahalanobis). Update **once** per timestamp.
- **Lifecycle (defaults):** Birth (tentative) → Confirm (M-of-N, e.g., 2/3) → Coast (inflate `P`) → Delete (C misses, e.g., 3).
- **Planner contract:** Tracks age ≤ **60 ms**, states **predicted to publish time**, include pose/vel **with covariance** and timestamps; sensible QoS.
- **Tuning:** `R` = sensor noise (trust). `Q` = model flex (maneuvers). Watch **NIS/NEES** and **gated_out%**.

---

## Worked core - CV model, async updates, NIS, NN

### State model (CV)

**State:** `x = [x, y, vx, vy]^T`

**F(Δt):**

$$
\begin{bmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

**Process noise (simple start):** `Q = diag(0.1, 0.1, 1.0, 1.0)` in `[x, y, vx, vy]` units.

*Classic CV alternative:*

$$
Q(\Delta t)=q
\begin{bmatrix}
\frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2} & 0 \\
0 & \frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2} \\
\frac{\Delta t^3}{2} & 0 & \Delta t^2 & 0 \\
0 & \frac{\Delta t^3}{2} & 0 & \Delta t^2
\end{bmatrix}.
$$

### Sensors (today)

- **Camera (position only)**  
  `H_cam = [[1,0,0,0]; [0,1,0,0]]`  
  `R_cam = diag(0.25, 0.25)  m^2`  (σ ≈ 0.5 m)

- **Radar (position-only to start)**  
  `H_rad = H_cam`  
  `R_rad = diag(1.00, 1.00)  m^2`  (σ ≈ 1.0 m)

_Note: since `R_cam < R_rad`, then `K_cam > K_rad` (camera pulls more)._

### Gating (NIS)

Measurement dim **df = 2** (x,y). Use χ² @ **99% → 9.21** (start).

$$
\text{NIS} = \nu^\top S^{-1}\nu,\quad
\nu=z-Hx^-,\quad
S=HP^-H^\top + R.
$$

### Asynchronous sensors - how to process

- Keep a single state with **`t_state`** (the timestamp the estimate is valid for).
- On any measurement (camera or radar) with stamp **`t_meas`**:
  1) **Predict to `t_meas`** using Δt and add Q.
  2) **NIS-gate** the measurement (reject if above threshold).
  3) **Update** if passed (use that sensor’s \(H\) and \(R\)).
  4) **Set `t_state = t_meas`**.

**Late or too old:**
- **OOSM:** if `t_meas < t_state` → **drop** & count `oosm_drops`.
- **Stale:** if `now - t_meas` > budget (e.g., 60 ms) → **drop** & count `stale_drops`.

**Tiny example (sequence only):**
- Start `t_state=1.000 s`. Radar @ 1.016 → Predict(1.000→1.016) → Gate → Update → `t_state=1.016`.  
- Camera @ 1.024 → Predict(1.016→1.024) → Gate → Update → `t_state=1.024`.

### Data association (same timestamp) - Gate then NN

- **Rule:** gate each detection with NIS; from those that pass, pick **min Mahalanobis** \(d^2=\nu^\top S^{-1}\nu\). **Update once.**
- **Counters:** `assoc_none` (no gated detections), `assoc_nn` (updated via NN).


**Paper mini-example**
- Predicted pos `p̂ = [10, 5]`, `P_pp = diag(0.50, 0.50)`, `R = diag(0.25, 0.25)`  
  ⇒ `S = diag(0.75, 0.75)`, `S^{-1} = diag(1.333, 1.333)`  
- Detections: `z1 = [10.4, 4.6]`, `z2 = [12.0, 8.0]`, `z3 = [9.6, 5.1]`  
- NIS: `d1^2 ≈ 0.426` (pass), `d2^2 >> 9.21` (fail), `d3^2 ≈ 0.227` (pass) → **pick z3**.


---

### Worked hand-calc (kept brief; see [Day_25-Async_Sensor_Kalman_Math_Workings](https://github.com/IvanMcCauley/Adas_Learning_Sprint/blob/main/Sprint_02-Systems_Architecture_Notes/Sprint_02_Files/Day_25_Files/Day_25-Async_Sensor_Kalman_Math_Workings.pdf) for full workings on paper) 

Initial @ `t0 = 1.000 s`: `x̂0 = [10, 5, 1, 0]^T`, `P0 = diag(0.50, 0.50, 0.20, 0.20)`  
Radar @ `t1 = 1.016`: predict `Δt = 0.016`, gate with `R_rad`, update  
Camera @ `t2 = 1.024`: predict `Δt = 0.008`, gate with `R_cam`, update

---

## Output contract for downstream modules

- **Freshness:** track age ≤ **60 ms** at publish.  
- **Time alignment:** state **predicted to publish time** (not just measurement time).  
- **Fields:** pose, velocity, covariance P, track ID, timestamps, frame IDs.  
- **QoS:** raw topics = SensorData; fused output = **Reliable**, **KeepLast(5)**.

---

## Observability / health metrics (print occasionally)

- **gated_out% (per sensor):** % of measurements rejected by NIS gate.  
  - *Compute:* rejected / total_received.
  - *Watch:* too high ⇒ R too small or bad calibration; too low ⇒ gate too wide.

- **assoc_none%:** % of updates where no detection passed the gate.  
  - *Compute:* frames_with_no_match / total_update_frames.
  - *Watch:* high ⇒ missing detections or over-gating.

- **coast_rate%:** % of track steps that were “predict-only” (no update).  
  - *Compute:* coast_steps / total_steps per track (or averaged).
  - *Watch:* high ⇒ poor coverage; planner confidence drops.

- **oosm_drops:** count of out-of-sequence measurements dropped (`t_meas < t_state`).  
  - *Watch:* >0 means timestamping/transport issues; check clocks, queues, and TF.

- **stale_drops:** count of measurements older than freshness budget (e.g., >60 ms) that were dropped.  
  - *Watch:* sustained nonzero ⇒ upstream latency; adjust QoS or budgets.

- **age_ms p50/p95:** median and 95th-percentile age of published tracks.  
  - *Compute:* `publish_time - header.stamp`.
  - *Target:* p95 ≤ budget (e.g., 60 ms).

- **avg dt_predict_ms:** average Δt you predict over before each update/publish.  
  - *Watch:* growing values ⇒ sensors lagging or lower update rate; covariance should reflect it (Q scale).

---

## Quick recall

- **x̂** = estimate; **x̂⁻/x̂⁺** = before/after update.  
- **P** covariance; **Q** process noise; **R** measurement noise.  
- **H** state→measurement map; **K** Kalman gain; **ν** innovation; **S** innovation covariance.  
- **NIS** ν^T S^{-1} ν` gate; **OOSM** late packet (`t_meas < t_state`).
