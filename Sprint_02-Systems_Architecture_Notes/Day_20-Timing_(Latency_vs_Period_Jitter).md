# Day 22 - Timing (Latency vs. Period Jitter)

## What I built (tiny but important)
- **`pinger`**: publishes `std_msgs/Header` on `/ping` at a fixed rate. The header’s `stamp` = **send time**.
- **`qos_probe`**: subscribes to `/ping`, computes **latency_ms = now − msg.stamp**, logs per-message + **sliding-window** mean/max.

## Core ideas (remember)
- **E2E latency (age of info)**: time between when the message was **stamped** and when I **consume** it. Measured in my callback.
- **Period jitter**: variability of **inter-arrival intervals** (Δt between messages). Measured with `ros2 topic hz`.
- **They’re different**: jitter can look fine while data is stale (high latency), or latency fine while arrivals are uneven.
- **QoS trade**:  
  - **RELIABLE** → retries/back-pressure → can **increase latency** under load.  
  - **BEST_EFFORT** → fresher data, may **drop** samples.
- **ROS timers**: `create_timer(T, cb)` schedules callbacks with the node’s clock (wall-time here). Jitter in timer wakeups shows up in `hz`.
- **Timestamps**: ROS time is `sec` + `nanosec` (like 1 second + 1 nanosecond = 1.000000001). Convert to a single unit before subtraction.

## Minimal patterns (mental templates)
#### Publisher
```python
self.pub = self.create_publisher(Header, '/ping', 10)  # KEEP_LAST 10, RELIABLE, VOLATILE
self.timer = self.create_timer(0.05, self.tick)        # 20 Hz
def tick(self):
    msg = Header()
    msg.stamp = self.get_clock().now().to_msg()
    self.pub.publish(msg)
```
#### Subscriber + latency calc
```python
def time_to_ns(t): return t.sec * 1_000_000_000 + t.nanosec
self.sub_ping = self.create_subscription(Header, '/ping', self.on_ping, qos)  # same QoS style as before
def on_ping(self, msg):
    now_ns = self.get_clock().now().nanoseconds
    lat_ms = (now_ns - time_to_ns(msg.stamp)) / 1e6
    self.get_logger().info(f"ping latency_ms={lat_ms:.2f}")
```
#### Sliding window stats (recent behavior, not lifetime)
```python
from collections import deque
self.lat_window = deque(maxlen=100)                   # ~5s at 20 Hz
self.stats_timer = self.create_timer(1.0, self.print_latency_stats)
def print_latency_stats(self):
    if not self.lat_window: return
    w = list(self.lat_window)
    mean_ms, max_ms = sum(w)/len(w), max(w)
    self.get_logger().info(f"[latency] window={len(w)} mean={mean_ms:.2f} ms max={max_ms:.2f} ms")
```
#### Commands I used (muscle memory)
```python
# Topic rate / period jitter
ros2 topic hz -w 200 /ping
ros2 topic hz -w 200 /chatter

# See QoS endpoints for a topic
ros2 topic info /ping -v

# Record / replay if needed
ros2 bag record -o ~/ros2_ws/bags/lab /ping
ros2 bag play ~/ros2_ws/bags/lab
```

## Pitfalls → fixes
- No `/ping` → pinger isn’t running or wrong `ROS_DOMAIN_ID`.
- “Param changed but QoS didn’t” → QoS is fixed at creation; you must destroy+recreate the sub/pub to actually change it.
- Crazy hz max/min → you measured across restarts; use -w 200 for a fresh short window.

## Why it works 
- Stamp at creation to capture the full pipeline delay.
- Subtract at consumption to get the age the controller/planner actually sees.
- Sliding window shows what’s happening now, not averaged over ancient history.

## Interview bullets (ready to say)
- “Latency is age-of-information; jitter is inter-arrival variability. Controllers hate jitter; planners hate stale data.”
- “RELIABLE can raise latency under load (retries/back-pressure); BEST_EFFORT keeps data fresh but may drop.”
- “QoS endpoints must match; I verify with ros2 topic info -v. Params from CLI override YAML, but QoS doesn’t hot-swap unless I recreate the endpoint.”
