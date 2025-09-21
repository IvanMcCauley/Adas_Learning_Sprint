# qos_probe.py - minimal ROS2 node skeleton
import rclpy                  # the ros python client library
from rclpy.node import Node   # Base class for user nodes
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from collections import deque

def time_to_ns(t):
    return t.sec * 1_000_000_000 + t.nanosec

class QoSProbe(Node):
    def __init__(self):
        super().__init__('qos_probe')                # node name       
        self.get_logger().info('qos_probe started')  # one time log to know it ran
        self.count=0
        self.timer = self.create_timer(0.5, self.tick) #call tick() every 0.5s
        
        # ---- QOS VIA PARAMETERS ----
        topic = self.declare_parameter('topic_name', '/chatter').value
        rel = self.declare_parameter('reliability', 'reliable').value   # 'reliable' | 'best_effort'
        dur = self.declare_parameter('durability', 'volatile').value    # 'volatile' | 'transient_local'
        depth = self.declare_parameter('depth', 10).value
        # Forming QoS profile
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=ReliabilityPolicy.RELIABLE if rel == 'reliable' else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if dur == 'transient_local' else DurabilityPolicy.VOLATILE       
        )
        self.get_logger().info(f"Subscribing to {topic} with QoS rel={rel}, dur={dur}, depth={depth}")
        # Creating subscription to /chatter
        self.sub = self.create_subscription(String, 
                                            '/chatter', 
                                            self.on_msg, 
                                            qos)
        # Creating subscription to /ping
        self.sub_ping = self.create_subscription(Header, '/ping', self.on_ping, qos)
 
        self.lat_window = deque(maxlen=100) # last 100 pings (~5s at 20Hz)
        self.stats_timer = self.create_timer(1.0, self.print_latency_stats) # print each second

    # ---- CALLBACK FUNCTIONS --- 
    def tick(self):
        self.count +=1
        self.get_logger().info(f"tick #{self.count}")

    def on_msg(self, msg: String):
        self.get_logger().info(f"chatter: {msg.data}")

    def on_ping(self, msg:Header):
        now_ns = self.get_clock().now().nanoseconds
        send_ns = time_to_ns(msg.stamp)
        lat_ms = (now_ns - send_ns) / 1e6
        self.lat_window.append(lat_ms)
        self.get_logger().info(f"ping latency_ms={lat_ms:.2f}")
       

    def print_latency_stats(self):
        if not self.lat_window:
            return
        w = list(self.lat_window)
        mean_ms = sum(w) / len(w)
        max_ms = max(w)
        self.get_logger().info(f"[latency] window={len(w)} mean={mean_ms:.2f} ms max={max_ms:.2f} ms")


def main():
    rclpy.init()                 #initialize 
    node = QoSProbe()            # create node
    try:
        rclpy.spin(node)         #keep node alive
    finally:
        node.destroy_node()      #__clean exit
        rclpy.shutdown()         #

if __name__ == '__main__':
    main()





