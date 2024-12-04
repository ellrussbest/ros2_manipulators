import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class simple_subscriber(Node):
    def __init__(self, node_name: str, topic: str, buffer_size: int):
        super().__init__(node_name)
        self.subscriber = self.create_subscription(String, topic, self.callback, buffer_size)
    
    def callback(self, msg: String):
        self.get_logger().info(f"I heard: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = simple_subscriber("simple_subscriber", "/chatter_py", 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()