import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class simple_publisher(Node):
    def __init__(self, node_name: str, topic_name: str, buffer_size: int):
        super().__init__(node_name)
        self.publisher = self.create_publisher(String, topic_name, buffer_size)
        self.counter = 0
        self.frequency = 1.0
        self.get_logger().info(f"Publishing at {self.frequency} Hz")
        self.timer = self.create_timer(self.frequency, self.callback)

    def callback(self):
        msg = String()
        msg.data = f"Hello ROS 2 - counter: {self.counter}"
        self.publisher.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = simple_publisher("simple_publisher", "chatter_py", 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    