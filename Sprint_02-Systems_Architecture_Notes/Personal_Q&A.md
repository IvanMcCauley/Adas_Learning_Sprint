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


- ####  `create_subscription(String, '/chatter', self.on_msg, qos)` — name the four args.
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
  -  
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ```

- ####  How to load params from a file with `ros2 run`?
  - `ros2 run planning101 qos_probe --ros-args --params-file /full/path/qos_probe.yaml`


- ####  YAML vs CLI `-p key:=val` — who wins?
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
  - `def on_msg(self, msg):` — ROS passes `msg`; Python supplies `self`.


- ####  Fast proof that a bag is publishing `/X`?
  - Play the bag, then check publishers:
  ```bash
  ros2 topic info /X -v   # Publishers list should show /rosbag2_play_*
```

