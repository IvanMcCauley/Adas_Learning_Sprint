# Day 20 - ROS 2 Graph, QoS, Domains, Bagging (study notes)
*(Purely personal notes to come back to, not a tutorial.)*

## Core ideas to remember
- **Reliability = delivery guarantee.**
  - **RELIABLE**: retries, no intentional drops → use for `/brake_cmd`, `/trajectory`.
  - **BEST_EFFORT**: freshest wins, drops OK → use for `/camera/image_raw`, `/lidar/points`, `/imu/data`.
- **Durability = late-joiner behavior.**
  - **TRANSIENT_LOCAL**: keep last N; late subscribers get it → `/tf_static`, static `/map`, calibration blobs.
  - **VOLATILE**: no history → live streams.
- **Compatibility (reader ← writer):**
  - RELIABLE ← RELIABLE ✅ | RELIABLE ← BEST_EFFORT ❌
  - BEST_EFFORT ← RELIABLE ✅ | BEST_EFFORT ← BEST_EFFORT ✅
  - TRANSIENT_LOCAL ← TRANSIENT_LOCAL ✅ | TRANSIENT_LOCAL ← VOLATILE ❌
  - VOLATILE works with either VOLATILE or TRANSIENT_LOCAL writer ✅
- **History/Depth**: `KEEP_LAST N` is a local queue (bigger = smoother bursts, but risk latency).
- **Domains**: `ROS_DOMAIN_ID` creates discovery “islands” (nodes see only same ID).
- **Rosbag2**: record topics to DB3; playback republishes them as `/rosbag2_play_*`.

---

## CLI I should know 
```bash
# List topics with types - first graph sanity check.
ros2 topic list -t

# Deep-dive a topic: endpoints + QoS (my go-to when a sub is silent).
ros2 topic info /chatter -v

# List active nodes - copy exact names from here.
ros2 node list

# Show a node’s pubs/subs/services/actions - prove who talks to who.
ros2 node info /talker

# Switch this shell to a separate DDS island - isolate tests/sims; prevents crosstalk.
export ROS_DOMAIN_ID=17

# Confirm which domain this shell is on (unset = default 0).
echo ${ROS_DOMAIN_ID:-unset}

# Record a minimal reproducible bag (DB3) into folder 'out'.
ros2 bag record -o out /chatter

# Inspect contents of a bag - topics, count, duration, start/end times.
ros2 bag info out

# Replay with original timing; --loop keeps it publishing forever.
ros2 bag play --loop out

# Echo that ignores common QoS mismatches (only sets the subscriber, not the publisher) (good “is anything arriving?” probe).
ros2 topic echo /chatter --qos-reliability best_effort --qos-durability volatile

# Measure effective publish rate - spot lag/backlog/drops quickly.
ros2 topic hz /chatter
```

## Pitfalls → fixes
- Subscriber silent:
  - ros2 topic info /X -v (check QoS); echo ${ROS_DOMAIN_ID:-unset} (domain); try a relaxed echo:
  ```  
  ros2 topic echo /X --qos-reliability best_effort --qos-durability volatile
  ```

- Laggy sensors (stale frames burst):
  - Likely RELIABLE on a high-rate stream → switch subscriber to BEST_EFFORT with small depth.

- `Unknown topic` after play:
  - Bag finished → `use ros2 bag play --loop`.

- PowerShell syntax errors (| |):
  - I'm not in bash; open Ubuntu (WSL) and rerun.

---
# Quick flashcard questions:
- What does QoS Reliability control? 
  - RELIABILITY is delivery guarantee: RELIABLE retries/no drops (adds latency); BEST_EFFORT favors freshness and can drop frames.
- Give two topics that should be RELIABLE. 
  - /brake_cmd, /trajectory (planner→controller), mode/mission commands.
- Give two topics that should be BEST_EFFORT. 
  - /camera/image_raw, /lidar/points, /imu/data.
- Why is RELIABLE risky on high-rate sensors? 
  - It can queue under load → latency grows → bursts of stale frames → “driving into the past.”
- What does QoS Durability control? 
  - Late-joiner behavior: TRANSIENT_LOCAL stores last N for new subscribers; VOLATILE does not.
- Give two topics that should be TRANSIENT_LOCAL. 
  - /tf_static, static /map, camera intrinsics/calibration blob.
- Is Reader RELIABLE ← Writer BEST_EFFORT compatible? 
  - No - reader demands a guarantee the writer can’t provide.
- Is Reader BEST_EFFORT ← Writer RELIABLE compatible? 
  - Yes - effective delivery becomes best-effort for that reader.
- Is Reader TRANSIENT_LOCAL ← Writer VOLATILE compatible? 
  - No - reader wants a latched sample; writer keeps none.
- Do History/Depth affect matching? 
  - No - they’re local buffers (latency/drop trade-off), not a matching criterion.
- What does ROS_DOMAIN_ID do? 
  - Sets a DDS/ROS discovery island; nodes only see others with the same domain ID.
- Two reasons to use different domains. 
  - Run multiple sims/tests in parallel; separate robots on one LAN; isolate noisy debugging.
- How do I prove a bag is publishing /X? 
  - ros2 topic info /X -v → Publishers list shows /rosbag2_play_*; then ros2 node info /rosbag2_play_* confirms it publishes /X.
- Minimal commands to record and loop-play a bag. 
  - ros2 bag record -o out /X and ros2 bag play --loop out
- Subscriber is silent - first 3 checks? 
  - Topic exists? (ros2 topic list); Publishers + QoS? (ros2 topic info -v); Domain matches? (echo $ROS_DOMAIN_ID)
