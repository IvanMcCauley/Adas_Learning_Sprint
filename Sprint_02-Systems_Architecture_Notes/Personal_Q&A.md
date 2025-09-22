# Quick flashcard questions:
## Day 20 - ROS 2 Graph, QoS, Domains, Bagging (operator core)
- #### What does QoS Reliability control? 
  - RELIABILITY is delivery guarantee: RELIABLE retries/no drops (adds latency); BEST_EFFORT favors freshness and can drop frames.
- #### Give two topics that should be RELIABLE. 
  - /brake_cmd, /trajectory (planner→controller), mode/mission commands.
- #### Give two topics that should be BEST_EFFORT. 
  - /camera/image_raw, /lidar/points, /imu/data.
- #### Why is RELIABLE risky on high-rate sensors? 
  - It can queue under load → latency grows → bursts of stale frames → “driving into the past.”
- #### What does QoS Durability control? 
  - Late-joiner behavior: TRANSIENT_LOCAL stores last N for new subscribers; VOLATILE does not.
- #### Give two topics that should be TRANSIENT_LOCAL. 
  - /tf_static, static /map, camera intrinsics/calibration blob.
- #### Is Reader RELIABLE ← Writer BEST_EFFORT compatible? 
  - No - reader demands a guarantee the writer can’t provide.
- #### Is Reader BEST_EFFORT ← Writer RELIABLE compatible? 
  - Yes - effective delivery becomes best-effort for that reader.
- #### Is Reader TRANSIENT_LOCAL ← Writer VOLATILE compatible? 
  - No - reader wants a latched sample; writer keeps none.
- #### Do History/Depth affect matching? 
  - No - they’re local buffers (latency/drop trade-off), not a matching criterion.
- #### What does ROS_DOMAIN_ID do? 
  - Sets a DDS/ROS discovery island; nodes only see others with the same domain ID.
- #### Two reasons to use different domains. 
  - Run multiple sims/tests in parallel; separate robots on one LAN; isolate noisy debugging.
- #### How do I prove a bag is publishing /X? 
  - ros2 topic info /X -v → Publishers list shows /rosbag2_play_*; then ros2 node info /rosbag2_play_* confirms it publishes /X.
- #### Minimal commands to record and loop-play a bag. 
  - ros2 bag record -o out /X and ros2 bag play --loop out
- #### Subscriber is silent - first 3 checks? 
  - Topic exists? (ros2 topic list); Publishers + QoS? (ros2 topic info -v); Domain matches? (echo $ROS_DOMAIN_ID)

---

## Day 21 - (ROS 2 Python, QoS, Params, Launch)

- ####  What does `super().__init__('name')` do in a ROS 2 node?
  -  Runs the parent **`Node`** constructor so *this object* becomes a real ROS node (graph registration, logger, params, timers).
- ####  `create_subscription(String, '/chatter', self.on_msg, qos)` - name the four args.
  -  **Message type**, **topic name**, **callback function**, **QoS**.
- ####  What can the 4th arg (`qos`) be?
  -  Either an **int depth** (shorthand for `KEEP_LAST depth`, default RELIABLE/VOLATILE) or a full **`QoSProfile`**.
- ####  Why pass `self.on_msg` without `()`?
  -  It’s a **callback**; you pass the function itself now so ROS can call it later. `self.on_msg()` would call it immediately.
- ####  What does `10` as the 4th arg mean?
  -  `KEEP_LAST 10` with defaults: **RELIABLE** + **VOLATILE**.
- ####  What does `self` let you access?
  -  **This node’s** state (`self.count`) and tools (`self.get_logger()`, `self.create_timer()`, `self.create_subscription()`).
- ####  Define **RELIABLE** vs **BEST_EFFORT** (one-liners).
  -  RELIABLE = retries, no intentional drops (can add latency).  
BEST_EFFORT = freshest wins; drops OK.
- ####  Define **TRANSIENT_LOCAL** vs **VOLATILE** (one-liners).
  -  TRANSIENT_LOCAL = writer keeps last N; late joiners get it.  
VOLATILE = no history for late joiners.
- ####  Compat: Reader **RELIABLE** ← Writer **BEST_EFFORT** ?
  -  **Incompatible** (reader demands guaranteed delivery the writer can’t provide).
- ####  Compat: Reader **TRANSIENT_LOCAL** ← Writer **VOLATILE** ?
  -  **Incompatible** (reader wants a latched sample; writer stores none).
- ####  Common sensor QoS pattern?
  -  `BEST_EFFORT + KEEP_LAST (small 5–10) + VOLATILE` (favor freshness, avoid lag).
- ####  Common command/trajectory QoS pattern?
  -  `RELIABLE + KEEP_LAST ≥1 + VOLATILE` (don’t miss commands).
- ####  Where do **launch** and **param** files live in the package?
  -  `launch/` and `config/` at the **package root** (next to `setup.py`), not inside the Python module folder.
- ####  Two `setup.py` sections you must know and why?
  -  `entry_points['console_scripts']` → enables `ros2 run pkg exe`.  
`data_files` → installs `launch/` + `config/` to `share/pkg/...` for `ros2 launch` and YAML.
- ####  Why `colcon build --symlink-install` for Python?
  -  Installed files **symlink** to source, so `.py` edits are picked up **without rebuild**.
- ####  Correct source order (underlay/overlay) and why absolute path?
  -  Àbsolute path is used as it will work in any directory
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ```
- ####  How to load params from a file with `ros2 run`?
  - `ros2 run planning101 qos_probe --ros-args --params-file /full/path/qos_probe.yaml`
- ####  YAML vs CLI `-p key:=val` - who wins?
  - **CLI overrides** YAML at startup.
- ####  How does a launch file find your YAML at runtime?
  - Via `get_package_share_directory('pkg')` then `os.path.join(..., 'config', 'file.yaml')` because `data_files` installed it under `share/pkg/config/`.
- ####  Quick command to prove subscriber QoS on `/chatter`?
  - `ros2 topic info /chatter -v` (check the **Subscribers** block for your node’s Reliability/Durability/Depth).
- ####  Why can RELIABLE on high-rate sensors be risky?
  - Queues can build under load → **latency grows** → bursts of **stale frames** (acting on old data).
- ####  One-liner: list a node’s params & read one
  - 
    ```bash
    ros2 param list /qos_probe
    ros2 param get  /qos_probe reliability
    ```
- ####  Minimal node skeleton to remember?
  - 
    ```python
    from rclpy.node import Node

    class MyNode(Node):
        def __init__(self):
            super().__init__('my_node')
            self.timer = self.create_timer(0.5, self.tick)
        def tick(self):
            self.get_logger().info("tick")
    ```
- ####  Subscriber callback signature?
  - `def on_msg(self, msg):` - ROS passes `msg`; Python supplies `self`.
- ####  Fast proof that a bag is publishing `/X`?
  - Play the bag, then check publishers:
  ```bash
  ros2 topic info /X -v   # Publishers list should show /rosbag2_play_*
  ```

---

## Day 22 - Timing: Latency & Jiiter
- #### Why must the message carry a send timestamp to measure E2E latency?
  - Without the send time, you can’t compute <code>receive_now − send_time</code> → no age-of-information.
- #### Define E2E latency vs period jitter in one line each.
  - Latency = age of info at consumption (now − msg.stamp).  
  - Jitter = variability of inter-arrival intervals (Δt between messages).
- #### One common cause that increases latency but doesn’t necessarily change mean period?
  - RELIABLE back-pressure/retries causing queueing (messages get older before delivery).
- #### Why use a sliding window (e.g., last 100 samples) for stats?
  - It reflects current behavior and exposes recent spikes; lifetime averages hide problems.
- #### What does <code>create_timer(0.05, cb)</code> actually do?
  - Schedules <code>cb</code> to run every 50 ms using the node’s clock; executor fires it like any other event.
- #### Convert ROS time (sec + nanosec) to one number safely - why prefer integers first?
  - Use integers (ns or ms) to avoid float rounding, then format at the end (e.g., <code>:.2f</code>).
- #### What QoS setting tends to minimize age-of-info under load and why?
  - BEST_EFFORT - drops instead of retrying, avoiding backlog and stale deliveries.
- #### Does changing a “reliability” parameter at runtime update an existing subscription’s QoS?
  - No, QoS is fixed at creation; you must destroy and recreate the endpoint to apply new QoS.
- #### A clean way to verify publisher vs subscriber QoS on a topic?
  - <code>ros2 topic info /X -v</code> (check each endpoint’s Reliability/Durability/Depth).
- #### Your `/chatter` shows avg ~1.0 Hz with min 0.990 s, max 1.025 s. What is that telling you?
  - About ±25 ms period jitter - normal timer/OS variability at 1 Hz.

---

## Day 23 - ADAS fundamentals review & sensors overview + stack overview & E2E timing
- #### Why do AVs use camera + radar + LiDAR together?
  - To combine complementary strengths: camera = semantics; radar = range/velocity + bad-weather robustness; LiDAR = precise 3D geometry. Fusion covers each sensor’s weaknesses.
- #### What is “ego state”?
  - The car’s own pose, velocity, acceleration, yaw/yaw-rate (+ uncertainty). Published by Localization; used by Fusion, Planning, and Control.
- #### Covariance (plain meaning)?
  - A confidence “bubble” around an estimate; **bigger = less sure**. Filters use it to weight updates and to set matching gates.
- #### Data association (in fusion/tracking)?
  - Matching new detections to existing tracks/IDs. Errors cause ID swaps, ghosts, or lost tracks. Gate by distance/uncertainty, then assign (NN/Hungarian) or use JPDA in clutter.
- #### Gating (what & why)?
  - A pre-filter that discards detections too far (statistically) from a track’s prediction (e.g., Mahalanobis gate). Cuts false matches and compute.
- #### UKF vs EKF (one-liner)?
  - **EKF** linearizes with Jacobians (straight-line approximation). **UKF** propagates sigma points through the true nonlinearity - steadier for turns and angle/range sensors (radar).
- #### JPDA (when to use)?
  - In crowded scenes, it assigns detections to tracks **probabilistically** to avoid flip-flopping IDs when objects overlap.
- #### TF `odom → base_link` means?
  - The rigid transform giving the car’s pose of `base_link` expressed in the `odom` frame. (`odom` = smooth local frame that may drift; `base_link` = vehicle body frame.)
- #### `TRANSIENT_LOCAL` vs `VOLATILE` durability?
  - **TRANSIENT_LOCAL:** keep last sample for late joiners (static frames/maps/calibration).
  - **VOLATILE:** don’t keep history (live streams).
- #### RELIABLE vs BEST_EFFORT?
  - **RELIABLE** guarantees delivery (can backlog). **BEST_EFFORT** drops on loss (no retries). Use BEST_EFFORT for firehose sensors; RELIABLE (with anti-stale settings) for tracks/trajectories.
- #### What does `KEEP_LAST N` control?
  - Queue depth at pub/sub. Small values absorb micro-hiccups without building stale backlog; pub depth >1 (with RELIABLE) gives the writer headroom.
- #### Deadline vs Lifespan?
  - **Deadline:** watchdog for expected cadence (detect late). **Lifespan:** publisher auto-expires samples older than a set age (prevents acting on stale).
- #### Freshness controls besides BEST_EFFORT?
  - Small queues (`KEEP_LAST`), **lifespan**, **deadlines**, and app-level **drop-stale** (`now - stamp` guard). Co-locate hot nodes / intra-process to cut transport.
- #### Intra-process comms (plain English)?
  - Put chatty nodes in the **same program** so they pass **pointers** in memory instead of serializing - per-hop delay drops to sub-ms.
- #### E2E latency vs timing budgets?
  - **Latency (E2E)** is total sensor→command delay. **Timing budgets** are per-stage compute targets that, plus a transport slice, must sum ≤ the E2E target.
- #### Example E2E timing split (50 ms)?
  - Perception 18 ms + Fusion 7 ms + Planning 12 ms + Control 5 ms + Transport 8 ms = **50 ms** (example; tune per stack/speed).
- #### Why RELIABLE (not BEST_EFFORT) for Fusion → Planning (10–20 Hz)?
  - Planner decisions are stateful; missing a 50–100 ms update can cause jitter/late actions. Use **RELIABLE + small depth + lifespan + deadlines** to get **fresh & complete** data.
- #### When does the planner reuse the same sensor sample?
  - When the **consumer runs faster** than the producer (e.g., planner 20 Hz, radar 12.5 Hz) or when backlog exists. If rates match and delay is fixed, after the first tick there’s typically **no reuse**.
- #### What is “transport time”?
  - Non-compute delay between nodes: serialization, DDS queues/ACKs, OS scheduling, network/IPC. Measure as `age_at_use = now − msg.header.stamp` minus your compute.
- #### Is QoS always symmetrical (pub=sub)?
  - No. Intentional mismatches can help: **Pub RELIABLE × Sub BEST_EFFORT** (viz won’t back-pressure), **Pub TRANSIENT_LOCAL × Sub VOLATILE** (no replay for that sub). It just needs to remain **compatible**. Mismatches can be used when one node is publishing messages to more than one destination, one destination may have different QoS requirements than the other.
- #### Sensors → Perception QoS (typical)?
  - `BEST_EFFORT, VOLATILE, KEEP_LAST 1` (optionally lifespan ≈ 1 frame). Minimize latency; never replay old frames.
- #### Planning → Control QoS (typical)?
  - `RELIABLE, VOLATILE, KEEP_LAST 2–5`, `deadline 60–80 ms`, `lifespan ~150 ms`. Keep only the latest trajectories; drop anything late.
 
## Day 23 - Kalman Filters
- #### What is the purpose of a Kalman Filter in AV tracking?
  - To estimate hidden true states (e.g. position and velocity) by blending predictions from a motion model with noisy sensor measurements.
- #### What does the state vector [x, y, vx, vy] represent?
  - The object’s estimated position (x, y) and velocity (vx, vy) at the current time.
- #### Why do we include velocity in the state even if a camera only measures position?
  - Because velocity can be inferred from changes in position over time, enabling smoother and more accurate predictions.
- #### What does the F matrix do?
  - It projects the previous state forward in time using the motion model (e.g. constant velocity: position = position + velocity·Δt).
- #### What does the H matrix do?
  - It maps the full state into the measurement space - selecting which elements of the state the sensor can actually observe.
- #### What does the R matrix represent?
  - Measurement noise covariance: how noisy or uncertain the sensor readings are.
- #### What does the Q matrix represent?
  - Process noise covariance: uncertainty in the prediction due to unmodeled accelerations or dynamics.
- #### What does the P matrix represent?
  - State covariance: the current confidence in the state estimate; larger values mean more uncertainty.
- #### What happens if R is increased?
  - The filter trusts measurements less and leans more on predictions, giving smoother but slower response.
- #### What happens if Q is increased?
  - The filter trusts predictions less and adapts more quickly to sensor measurements, allowing for maneuvering but more jitter.
- #### What is the innovation (residual)?
  - The difference between the actual measurement and the predicted measurement: y = z – Hx.
- #### What does the innovation covariance S represent?
  - The expected uncertainty of the innovation, combining prediction uncertainty and measurement noise (S = HPHᵀ + R).
- #### What is the Kalman gain K?
  - A weighting factor that decides how much to lean towards the measurement versus the prediction.
- #### What happens if K is very small?
  - The filter trusts predictions much more than measurements, which may happen if measurements are very noisy or failing.
- #### What happens if K is very large?
  - The filter trusts measurements strongly, updating the state heavily each tick.
- #### What is the gating step with NIS?
  - Normalized Innovation Squared checks if a measurement is consistent with the prediction given expected uncertainty; if NIS > threshold, reject the measurement.
- #### What does a high NIS value mean?
  - The measurement is far from what was predicted relative to expected noise → likely an outlier or false detection.
- #### What does a very low NIS value mean?
  - The residuals are smaller than expected → may indicate R is set too high (overestimating sensor noise).
- #### What’s an example gating threshold for 2D position?
  - Around 9.21 (95% confidence level for χ² with 2 degrees of freedom).
- #### How is radar different from camera in KF measurements?
  - Radar provides range and radial velocity directly, while a monocular camera typically only provides bearing/position information.
- #### What role does Q play if the vehicle brakes or accelerates?
  - Q accounts for unmodeled acceleration; a higher Q allows the filter to adjust faster when the motion deviates from constant velocity.
- #### What is initial covariance P₀ used for?
  - It encodes initial uncertainty in the state; position variance reflects sensor noise, velocity variance is often large if unknown.

