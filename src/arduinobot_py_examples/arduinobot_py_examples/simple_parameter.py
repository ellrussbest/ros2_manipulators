import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from typing import List

class simple_parameter(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.declare_parameter("simple_int_param", 28)
        self.declare_parameter("simple_string_param", "hello_ros")

        self.add_on_set_parameters_callback(self.callback)

    def callback(self, params: List[Parameter]):
        res = SetParametersResult()

        for param in params:
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f"Param simple_int_param changed! New value is {param.value}")
                res.successful = True
            if param.name == "simple_string_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f"Param simple_string_param changed! New value is {param.value}")
                res.successful = True
        return res


def main(args=None):
    rclpy.init(args=args)
    node = simple_parameter("simple_parameter")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()