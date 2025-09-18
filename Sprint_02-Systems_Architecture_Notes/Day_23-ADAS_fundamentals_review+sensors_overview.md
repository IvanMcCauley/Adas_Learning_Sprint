# Day 23 - ADAS fundamentals review and sensors overview
ADAS (Advanced Driver Assistance Systems) are vehicle technologies designed to support the driver by increasing safety and reducing workload. They use sensors and control logic to provide functions like adaptive cruise control, lane keeping and automatic emergency braking.

---

### Warning-only ADAS:
- Forward Collision Warning (FCW)
- Lane Departure Warning (LDW)
- Blind Spot Warning (BSW)
- Driver Monitoring / Drowsiness Detection
- Traffic Sign Regognition

### Active Control ADAS
- Automatic Emergency Braking (AEB)
- Adaptive Cruise Control (ACC)
- Lane Keeping Assist (LKA) / Lane Centering
- Automatic High Beams
- Park Assist / Automatic Parking
- Electronic Stability Control (ESC)

---
### Why ADAS are mainly level 1-2
Most ADAS are Level 1-2 because theyre designed as driver aids, not replacements. Current sensors and algorithms cant yet garuntee safety in all conditions, and strict standards/regulations mean responsibility remains with the human.

---
## Sensor Suite
- **Cameras:** Uses optical lenses and image sensors to capture the enviornment as pixels. Best at recognising *what* an object is (lanes, signs, pedestrians).
- **Radar:** Emits radio waves and measures reflections + Doppler shift to calculate distance and speed. Best at knowing *how far* and *how fast* things are, even in bad weather.
- **LiDAR:** Fires laser pulses and measures return time (time-of-flight) to build a dense 3D "point cloud". Best at knowing *where* objects are in 3D space.
### Comparison table
<table>
  <tr>
    <th>Sensor</th>
    <th>Resolution / Detail</th>
    <th>Range</th>
    <th>Cost</th>
    <th>Weather Robustness</th>
    <th>Key Strength</th>
    <th>Key Weakness</th>
  </tr>
  <tr>
    <td>Camera</td>
    <td>High (color, texture, classification)</td>
    <td>Medium–long (limited at night/fog)</td>
    <td>Low (cheap, mass-produced)</td>
    <td>Weak (bad in rain, fog, glare, dirt)</td>
    <td>Rich semantic detail (what things are)</td>
    <td>Reliability drops in poor visibility</td>
  </tr>
  <tr>
    <td>Radar</td>
    <td>Low (blobs, no texture)</td>
    <td>Long (hundreds of meters)</td>
    <td>Low–medium</td>
    <td>Strong (works in dark, fog, rain)</td>
    <td>Robust range + velocity detection</td>
    <td>Poor object/lane shape info</td>
  </tr>
  <tr>
    <td>LiDAR</td>
    <td>Very high (cm-level 3D geometry)</td>
    <td>Medium–long (100–200m)</td>
    <td>High (expensive, complex)</td>
    <td>Weak–medium (rain/fog scatter lasers)</td>
    <td>Precise 3D mapping</td>
    <td>Cost + weather limitations</td>
  </tr>
</table>

### Why sensor fusion?
- Cars combine camera + radar + LiDAR because no single sensor is reliable and informative in all conditions. Cameras give rich semantics (what things are) but struggle with depth/velocity and bad weather; radar gives robust range + relative speed in rain/fog but low shape detail; LiDAR gives precise 3D geometry but costs more and degrades in heavy rain/fog. Fusing them boosts coverage, confidence, and fault tolerance so the system maintains performance when one modality is weak.

---

## Pipeline
```
SENSORS
  Exteroceptive:
    • Camera  (~30 Hz)  → images
    • Radar   (~10–20 Hz) → targets (range, Doppler)
    • LiDAR   (~10 Hz)  → point clouds
  Proprioceptive:
    • IMU     (~100–200 Hz) → accel/gyro
    • GPS     (~1–10 Hz)    → GNSS fixes

        (exteroceptive path)              (proprioceptive path)
                  |                                 |
                  v                                 v
PERCEPTION (~15–30 Hz, camera-driven)         LOCALIZATION (IMU(~100-200 Hz) + GPS(~1-10 Hz))
  • Object & lane detection, freespace          • IMU preintegration, GNSS fusion,
  • (optional) stereo/mono depth                  map matching
  • Outputs: detections (bboxes/classes/3D),    • Output: ego pose (x, y, θ) + covariance
             lanes

                 \                               /
                  \                             /
                   v                           v
FUSION & TRACKING (~10–20 Hz)
  • Data association (cam/radar/lidar + ego pose)
  • Kalman/UKF/JPDA tracking
  • Output: tracked objects (ID, pose, vel, covariance) + ego state

                  |
                  v
PLANNING (~10–20 Hz)
  • Behavior: follow / yield / stop / overtake
  • Path/trajectory: A* / graph / RRT; polynomials/MPC refs
  • Output: time-stamped trajectory (2–5 s horizon)

                  |
                  v
CONTROL (~50–100 Hz)
  • Lateral (steering) & Longitudinal (speed/brake) loops
  • Output: actuator commands

                  |
                  v
ACTUATORS / CAN
  • Steering, throttle, brake; diagnostics & limits
```
### A couple of definitions of terms above that confused me:
- #### Covariance (how sure are we?)
  - **Idea:** A value/shape that says how uncertain an estimate is.
  - **Picture:** An ellipse around the estimated position; bigger ellipse = less sure.
  - **Why it matters:** The system trusts uncertain measurements less and uses a wider matching window for them.
  - **One-liner:** Covariance = confidence. Big = we’re unsure.

- #### Data association (matching detections to tracks)
  - **Idea:** Decide which new detection belongs to which existing ID/track.
  - **If wrong:** ID swaps, duplicate tracks, “ghost” objects, lost objects.
  - **One-liner:** First gate by “close enough”, then match.

- #### Gating (pre-filter before matching)
  - **Idea:** Throw out detections that are too far from where a track was predicted to be.
  - **Why:** Cuts false matches and reduces compute.
  - **One-liner:** Keep only plausible candidates, ignore the rest.

- #### UKF (vs EKF)
  - **Both** blend predictions + measurements.
  - **EKF:** approximates with straight-line math (linearization).
  - **UKF:** pushes a few sample points through the real (curvy) math → usually steadier for turning targets and angle/range sensors (radar).
  - **One-liner:** UKF handles curves/angles better; EKF is a straight-line approximation.

- #### JPDA (crowded scenes)
  - **Idea:** In clutter, don’t force one hard match; share a detection across tracks probabilistically.
  - **Why:** Avoids flip-flopping IDs when objects overlap.
  - **One-liner:** In crowds, share credit instead of guessing one match.

- #### Ego state (your car’s own state)
  - **What:** Your car’s position, heading, speed, acceleration, yaw-rate (+ how sure we are).
  - **Who:** Localization publishes it; Fusion, Planning, Control consume it.
  - **One-liner:** Our car’s best-known pose/speed (+ confidence) for everyone else to use.

- #### Polynomials vs MPC (making/following a path)
  - **Polynomials:** Precompute a smooth curve between start & goal (e.g., lane change). Fast & simple when constraints are mild.
  - **MPC:** Re-optimizes each control tick to follow a reference while respecting limits (steer/brake, comfort, lane bounds).
  - **One-liner:** Polynomials for quick smooth moves; MPC when constraints matter and you must re-plan each moment.

---

## Rates and QoS per hop
- #### Sensors -> perception
  - **Typical rates:** Camera ~30Hz, Radar ~10-20Hz, Lidar ~10Hz
  - **QoS (publisher):** `SensorDataQoS` (`BEST_EFFORT`, `VOLATILE`, `KEEP_LAST 1`, `lifespan ~1 frame` (eg. 33-50ms))
  - **Why:** Min latency; never replay old frames; stale frames auto expire

- #### Perception -> Fusion
  - **Paylosd:** Detections (bboxes/classes/3D), lanes
  - **Rate:** ~15-30Hz (camera-driven)
  - **QoS (publisher):** `RELIABLE`, `VOLATILE`, `KEEP_LAST 5-10`, `deadline 40-60ms`, `lifespan 100ms`
  - **Why:** Dont'drop detections, but also dont let old ones linger

- #### Localization (publications)
  - **Pose/Odom:** 30–100 Hz - `RELIABLE`, `VOLATILE`, `KEEP_LAST 10`, `deadline ~50 ms`
  - **TF (odom→base_link):** 50–100 Hz - `RELIABLE`, `VOLATILE`, `KEEP_LAST 10`
  - **TF_STATIC (sensor extrinsics, maps/calib):** `RELIABLE`, `TRANSIENT_LOCAL`, `KEEP_LAST 1`
  - **Why:** Fast, fresh pose; static frames/maps must be latched for late joiners.

- #### Fusion -> Planning
  - **Payload:** Tracked objects (ID, pose, vel, covariance) + ego state
  - **Rate:** ~10-20Hz
  - **QoS (publisher):** `RELIABLE`, `VOLATILE`, `KEEP_LAST ~10`, `deadline ~50 ms`, `lifespan 120 ms`
  - **Why:** Planner needs complete, non-stale tracks on time.

- #### Planning → Control
  - **Payload:** Time-stamped trajectory (2–5 s horizon)
  - **Rate:** ~10–20 Hz
  - **QoS (publisher):** `RELIABLE`, `VOLATILE`, `KEEP_LAST 2–5`, `deadline 60–80 ms`, `lifespan 150 ms`
  - **Why:** Keep only the most recent trajectories; drop anything late.

- #### Control → Actuators
  - **Payload:** Throttle/Brake %, Steering angle/rate
  - **Rate:** ~50–100 Hz
  - **QoS (publisher):** `RELIABLE`, `VOLATILE`, `KEEP_LAST 1–3`, `deadline ~20 ms`, `lifespan 40–60 ms`
  - **Why:** Tight loop, tiny queue, strict deadlines.
 

#### Quick rules of thumb
- **Live streams:** `VOLATILE` + small `KEEP_LAST`; consider **lifespan** to auto-drop stale.
- **Static/slow config:** `TRANSIENT_LOCAL` (e.g., `tf_static`, calibration, maps).
- **Deadlines:** Set on periodic topics (pose/tracks/trajectory) so downstream can detect staleness.
- **Targets (sensor→command):** Aim ~**≤50 ms** total (Perception ≤20, Fusion ≤10, Planning ≤15, Control ≤5, Transport ≤10). Measure, then tighten.

#### Timing
- **Latency budget (E2E):** Max sensor→command age. (example ≤50 ms total)
- **Timing budgets (per-stage):** Max compute per module so the sum ≤ E2E.
- **Transport budget:** Part of the E2E for DDS/ROS comms (serialization, queues, copies, NIC) (example: ≤ 10 ms total)
- **Worst-case vs average:** Budget to **worst-case** (with jitter), measure both; leave 10–20% **slack**.
- **How to enforce:** 
  - Set **deadlines** per stage near its timing budget (e.g., tracks 50–60 ms).
  - Use **lifespan** + **small KEEP_LAST** to prevent stale backlog.
  - Instrument `age_at_use = now − msg.header.stamp` at each hop.


#### Latency vs Timing Budgets (Just a quick mental model to grasp it fully)
- **Latency (E2E):** sensor timestamp → actuator command timestamp (includes compute + transport + waits).
- **Timing budgets:** per-stage compute targets; must sum (with transport) ≤ E2E latency target.
- **Transport budget:** non-compute time between modules (queues, copies, network); keep small to preserve freshness.

Measure:
- At publish: set `msg.header.stamp = now()`.
- At subscribe: `age_at_use = now() - msg.header.stamp`.
- Also log per-node **compute_ms** (start/stop timers) to separate compute vs transport.

Design:
- Set **deadlines** near each stage’s timing budget (detect late).
- Use **small KEEP_LAST** and **lifespan** to avoid backlog.
- Co-locate hot nodes / enable **intra-process** (meaning torun two chatty nodes in the same process so ROS can pass pointers (like directly in memory) instead of serializing (like packing it up and sending it through the OS). Cuts per-hop delay to ~sub-ms.)

---
## Two quick scenario questions:
### (1) Radar = 20 Hz (every 50 ms). Planning = 20 Hz. Fusion adds 30 ms delay. Worst case, how many planning cycles might reuse the same radar sample?
- Timeline: radar at 0,50,100… → arrives to planner at 30,80,130…; planner runs at 0,50,100,150…
- First tick (t=0): no radar yet → planner uses nothing (or last/held).
- After that: every planner tick has a new radar sample (30→used at 50, 80→used at 100, …).
- Answer: No reuse after the first tick (rates match; fixed phase shift just offsets).

### (2) Planner requires inputs < 60 ms old. Which stages threaten that budget, and one mitigation each?
- Perception compute can blow the age.
  - Fix: speed up (smaller input, TensorRT, ROI, GPU), set deadline ~60 ms.
- Fusion delay/queueing (big KEEP_LAST, RELIABLE backlog).
  - Fix: small queues (5–10), lifespan ~120 ms, deadline ~50–60 ms, drop-stale in code.
- Transport/backlog (serialization, retransmits, slow subs).
  -Fix: tiny queues, split consumers (planner RELIABLE, viz BEST_EFFORT), avoid slow RELIABLE subs; later: intra-process.
- Sensor QoS mistakes (RELIABLE on camera/LiDAR).
  - Fix: sensors BEST_EFFORT + VOLATILE + KEEP_LAST 1.
- Clock/timestamp issues (unsynced clocks, wrong stamps).
  - Fix: NTP/PTP sync; always stamp at publish; log age_at_use.
