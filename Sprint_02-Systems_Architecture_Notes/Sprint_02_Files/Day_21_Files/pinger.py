import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

class Pinger(Node):
    def __init__(self):
        super().__init__('pinger')
        self.pub = self.create_publisher(Header, '/ping', 10) ## default QoS (KEEP_LAST 10, RELIABLE, VOLATILE)
        self.timer = self.create_timer(0.05, self.tick) # 20Hz

    def tick(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg() # send time (ROS time; wall-time here)
        msg.frame_id = 'pinger'
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = Pinger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
