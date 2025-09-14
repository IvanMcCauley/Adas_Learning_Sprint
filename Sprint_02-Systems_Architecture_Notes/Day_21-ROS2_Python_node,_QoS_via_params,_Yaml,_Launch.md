# Day 21: ROS2 Python node, QoS via params, YAML, launch

## Main ideas to remember
- **Overlay vs underlay:** `/opt/ros/humble` (base) + `~/ros2_ws` (my code). Source both (base first).
- **Package = unit ROS can run.** `ros2 pkg create`, then `colcon build`, then `ros2 run`.
- **Entry points:** `setup.py -> console_scripts` makes `ros2 run planning101 qos_probe` work.
- **Install non-code files:** `setup.py -> data_files` installs `launch/` + `config/` to `share/planning101/...`.
- **`create_subscription` signature:** `(msg_type, topic, callback, qos)`  
  - `qos` can be **int depth** or a **QoSProfile**.
- **Params sources:** defaults in code → YAML file → **CLI overrides win**.
- **QoS gist:** BEST_EFFORT vs RELIABLE (delivery), VOLATILE vs TRANSIENT_LOCAL (late joiners). Mismatch → **no match**.

## Commands I’ll reuse 
```bash
# Build dev-friendly (symlinks for Python)
colcon build --symlink-install

# Make ROS see my overlay (use abs path so it works anywhere)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Create a package
ros2 pkg create --build-type ament_python planning101 --dependencies rclpy std_msgs

# Run my node
ros2 run planning101 qos_probe

# Run with params from file (no long CLI)
ros2 run planning101 qos_probe --ros-args --params-file ~/ros2_ws/src/planning101/config/qos_probe.yaml

# Launch (loads the YAML installed under share/)
ros2 launch planning101 qos_probe.launch.py
```

## Code patterns
### Node skeleton
```python
class QoSProbe(Node):
    def __init__(self):
        super().__init__('qos_probe')
        self.get_logger().info('Node started')
        self.count = 0
        self.timer = self.create_timer(0.5, self.tick)

    def tick(self):
        self.count +=1
        self.get_logger().info(f"tick: #{self.count}")
```
### Subscriber + QoS from params
```python
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
topic = self.declare_parameter('topic_name', '/chatter').value
rel = self.declare_parameter('reliability', 'reliable').value
dur = self.declare_parameter('durability', 'volatile').value
depth = self.declare_parameter('depth', 10).value
qos = QosPolicy(
    history=HistoryPolicy.KEEP_LAST, depth=depth
    reliability=ReliabilityPolicy.Reliable if rel=='reliable' else ReliabilityPolicy=BEST_EFFORT
    durability=DurabilityPolicy.TRANSIENT_LOCAL if dur=='transient_local' else DurabiltiyPolicy=VOLATILE
)
self.sub = self.create_subscription(String, '/chatter', self.on_msg, qos)
```
### YAML (params preset)
qos_probe:
    ros__parameters:
        topic_name: /chatter
        reliability: best_effort
        durability: volatile
        depth: 5

### Launch (loads YAML)
```python
Node(package='planning101, executable='qos_probe', name='qos_probe',
     parameters={os.path.join(get_package_share_directory('planning101'),'config'.'qps_probe.yam;')])
```

## Pitfalls → fixes
- No `/qos_probe` in `ros2 node list` → I didn’t `source ~/ros2_ws/install/setup.bash`.
- `file 'qos_probe.launch.py' not found` → add data_files entries in `setup.py`, rebuild.
- Silent subscriber with `durability: transient_local` vs talker default → incompatible (writer is VOLATILE).


