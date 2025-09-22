# Day 25 - Sensor Fusion

- State model I'm using today: 
  - State: `[x, y, vx, vy]`
  - F(Δt)=  

  $$
  \begin{bmatrix}
  1 & 0 & \Delta t & 0 \\
  0 & 1 & 0 & \Delta t \\
  0 & 0 & 1 & 0 \\
  0 & 0 & 0 & 1
  \end{bmatrix}
  $$

- Sensors
  - Camera (position only):
    -  H_cam = [1, 0, 0, 0; 0, 1, 0, 0]
  
    - R_cam = diag(0.5²,0.5²)m²

  - Radar (use position-only to start):
    - H_rad = H_cam
    - R_rad = diag (1.0²,1.0²)m²
    - *Note: since R_cam < R_rad, then K_cam > R_rad, so the camera is trusted more than the radar (because a higher R results in a lower K)*

- Gating (NIS)
  - df=2(x,y). Use χ² 99% -> 9.21 as the initial gate.

- Process noise 
  - Start with Q = diag(0.1, 0.1, 1.0, 1.0) in [x, y, vx, vy] units

## Side note - Asynchronous sensors

You keep one estimate of the target (position + velocity) and a point in time that estimate is valid for: `t_state`.
Each sensor message (camera, radar, etc.) comes with its own timestamp: `t_meas` and it's own **H** and **R**.

**What to do when any measurement arrives**
1) **Predict to the measurement time**  
   Roll your estimate forward from `t_state` to `t_meas` using the motion model (constant velocity).
2) **Plausibility check (NIS gate)**  
   Compare what the sensor says to what you expected at `t_meas`. If it’s too unlikely, **ignore** the measurement.
3) **Update (blend in)**  
   If it passes the gate, nudge the estimate toward the measurement (Kalman gain handles “how much”).
4) **Advance the estimate’s time**  
   Set `t_state = t_meas` (your estimate now describes the world at the measurement’s timestamp).

**Late or too old?**
- **Out-of-sequence (OOSM):** if `t_meas < t_state`, it’s older than your current estimate → **drop it** and count it (`oosm_drops`).
- **Stale:** if `now - t_meas` exceeds your freshness budget (e.g., 60 ms) → **drop it** and count it (`stale_drops`).

**Why this works**
- You always align the estimate to the time of the evidence before using it.
- This avoids jittery velocity (“sawtooth”) and keeps the filter consistent with real timestamps.
- Same single Kalman filter for all sensors; only `H` and `R` differ per sensor. `Δt` just changes each time.

**Tiny example**
- Current `t_state = 1.000 s`.  
- Radar arrives stamped `1.016 s` → Predict (1.000→1.016) → Gate → Update → set `t_state = 1.016`.  
- Camera arrives stamped `1.024 s` → Predict (1.016→1.024) → Gate → Update → set `t_state = 1.024`.

**What to log occasionally**
- `% gated_out` per sensor, `oosm_drops`, `stale_drops`, and average `dt` you predict over.

- `gated_%`, `oosm_drops`, `stale_drops`, `avg_dt_predict_ms`.

**Notes**
- Linear + independent noises ⇒ sequential ≈ joint update.
- Don’t inflate **R** for lateness; handle lateness with predict-to-stamp + freshness guard.

