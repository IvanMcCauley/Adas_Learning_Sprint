# Day 24 - Kalman_Filter

---

- Want the best current guess of an object’s **state** (e.g., position & velocity) using:
  - a **prediction** (physics: how things move), and
  - a **measurement** (sensor: what we observed),
- …and we **weight** them by how **trustworthy** they are.

> One-sentence: *Predict with a motion model, correct with a measurement, weight by uncertainty.*

## "State” vs “measurement”?
- **State (what I estimate):** `[x, y, vx, vy]^T` → where it is and how fast it’s moving.
- **Measurement (what a sensor gives now):**
  - **Camera (mono):** usually **position only** in some frame (after geometry) → `[x, y]`.
  - **Radar:** **range, bearing, radial speed `v_r`** (speed along the line-of-sight).
  - **LiDAR:** accurate **position** from point cloud, velocity via tracking over time.
  - **GNSS/IMU:** ego vehicle pose/velocity (used to put everything in the same frame).

**Key point:** The KF can **estimate velocity** even if the sensor doesn’t measure it directly, by watching how position changes over time. If radar gives `v_r`, we include it for faster/better speed.

## The loop each tick (predict → update)
**Predict (use physics):**
- Position moves by `v * Δt`. Velocity stays the same under a constant-velocity model.
- Uncertainty **grows** a bit during this step (because the world can accelerate/turn).

**Update (use sensor):**
- Compare the measurement to what we *expected* to see (`residual`).
- Blend prediction and measurement by the **Kalman gain** (computed from uncertainties).
- Uncertainty **shrinks** if the measurement is informative.

## The 5 small matrices (what they mean, no heavy math)
- `F` - **state transition**: “how state evolves over one step.”

$$
F =
\begin{bmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$ 

  For constant velocity with step `Δt`: `F * [x, y, vx, vy]^T = [x + vx*Δt, y + vy*Δt, vx, vy]^T`

- `H` - **measurement model**: “which parts of the state does this sensor see?”

$$
H_{\text{radar}} =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & \cos\theta & \sin\theta
\end{bmatrix}
$$

Camera sees `[x,y]`; radar can contribute `[x,y]` (after conversion) and `v_r = cosθ*vx + sinθ*vy`.
- `Q` - **process noise**: “how much reality deviates from our model” (unmodelled accel/turns).

Helper \(G\):

$$
G =
\begin{bmatrix}
\frac{1}{2}\Delta t^2 & 0 \\
0 & \frac{1}{2}\Delta t^2 \\
\Delta t & 0 \\
0 & \Delta t
\end{bmatrix}
$$

Which comes from `x' = x + vx*Δt + 0.5*ax*Δt²`
                 `y' = y + vy*Δt + 0.5*ay*Δt²`

Then:

$$
Q = G\
\begin{bmatrix}
\sigma_a^2 & 0 \\
0 & \sigma_a^2
\end{bmatrix}
G^{\top}
$$

How to pick σₐ: Think “typical RMS accel I need to tolerate”:
~1–3 m/s² city; higher for aggressive/highway.

- `R` - **measurement noise**: “how noisy the sensor is” (variance, not std).

Camera example (σ = 0.2 m):

$$
R_{\text{cam}} =
\begin{bmatrix}
0.04 & 0 \\
0 & 0.04
\end{bmatrix}
$$

Radar example (σ_x=σ_y=0.5 m, σ_{v_r}=0.3 m/s):

$$
R_{\text{radar}} =
\begin{bmatrix}
0.25 & 0 & 0 \\
0 & 0.25 & 0 \\
0 & 0 & 0.09
\end{bmatrix}
$$

- `P` - **state covariance**: our current **confidence**; larger values = less sure.

$$
P =
\begin{bmatrix}
0.25 & 0 & 0 & 0 \\
0 & 0.25 & 0 & 0 \\
0 & 0 & 25 & 0 \\
0 & 0 & 0 & 25
\end{bmatrix}
$$

> Intuition: **Bigger `R` ⇒ trust the model more. Bigger `Q` ⇒ allow more maneuvering.**

## Gating: throw out unlikely measurements
- Compute a **residual** (measurement − predicted measurement) and normalize by expected uncertainty → **NIS** (a squared distance).
- For a 2D position measurement, typical thresholds (chi-square):
- **95% gate**: `5.99`
- **99% gate**: `9.21`
- If `NIS > gate` → treat as **outlier** (reject this detection for the track).

## Multiple sensors (how to add radar nicely)
- Keep the **same KF**. Just add rows to the measurement model when you have more info.
- Example for one target:
- Camera update: use `[x, y]` with its `R_cam`.
- Radar update (same tick): add `v_r` row (and/or `[x,y]` from radar) with `R_radar`.
- Order doesn’t matter for linear KFs; do sequential updates in the same tick.

## Tiny numeric example (feel it once)
- **Setup:** `Δt=0.5 s`, current estimate = `[x,y,vx,vy] = [0,0, 2,0]`
- **Predict:** `x=1.0, y=0.0, vx=2, vy=0`
- **Camera measurement:** `[x,y] = [1.1, 0.1]`, noise std = `0.2 m` → `R = diag(0.04, 0.04)`
- Residual ≈ `[0.1, 0.1]` (close to prediction) → small correction toward the measurement.
- **Radar velocity (line-of-sight):** θ = 30°, `u=[cosθ,sinθ]≈[0.866, 0.5]`
- Predicted radial `v_r_pred = u·[vx,vy] ≈ 1.732 m/s`
- Radar reads `v_r_meas = 2.3 m/s` (residual ≈ `+0.57`) → KF increases speed **along** the LOS direction.

Takeaway: position and velocity both get nudged; uncertainty `P` updates so the filter “knows” how confident it is.

## How to pick/tune `Q` and `R` (simple rules)
- **Start with honest `R`** from sensor specs, confidence scores, or geometry (farther = noisier).
- **Use NIS to adjust `R`:**
- Many big residuals (NIS high) → `R` was **too small** → **increase** it.
- Residuals tiny (NIS very low) → `R` was **too big** → **decrease** it.
- **Pick `Q`** via a guessed acceleration std (e.g., `σ_a ≈ 1-3 m/s²` for urban).  
Larger `Q` = faster reaction, more jitter; smaller `Q` = smoother, may lag during maneuvers.

## Data association (when you have multiple objects)
- **Gate** measurements per track (NIS threshold).
- **Nearest Neighbor**: for each track, pick the in-gate detection with the smallest NIS.
- Unmatched detections → **new tracks**. Unmatched tracks → **miss count++**, delete after M misses.

## What NOT to forget in code
- **Use timestamps** to compute `Δt` (don’t assume a fixed period).
- **Work in one frame** (e.g., `odom`/`map`); transform measurements first.
- **Variance vs std dev:** `R` uses **variance** (σ²), not σ.
- **Drop stale** inputs (`now - stamp > threshold`) before updating.
- **Check consistency** (log NIS), not only the pretty output.

## Quick cheat sheet (tiny)
### KF Loop:

**Predict**
- `x = F x`
- `P = F P Fᵀ + Q`

**Update (with measurement z)**
- `y = z − H x`              # innovation / residual
- `S = H P Hᵀ + R`           # innovation covariance
- `K = P Hᵀ S⁻¹`             # Kalman gain
- `x = x + K y`              # updated state
- `P = (I − K H) P`          # updated covariance

- **Gates (2D):** 95%→5.99, 99%→9.21
- **Sensor roles:** camera → `[x,y]` (great lateral, weak depth); radar → `v_r` + range/bearing (great speed/range); LiDAR → strong geometry.

## What to say in an interview (2 lines)
- “A Kalman filter blends a physics **prediction** with sensor **measurements** using their **covariances**, giving the **best state estimate + uncertainty** in real time.”
- “We gate outliers with NIS, tune `R` from residual stats, and include radar `v_r` to lock velocity fast; `Q` sets maneuver responsiveness.”

