import rclpy
from rclpy.node import Node

class EnvironmentManager(Node):
    def __init__(self):
        super().__init__('environment_manager')
        self.get_logger().info("Environment Manager Started (Placeholder)")
        # In the future, publish wave/wind data here

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EnvironmentManager())
    rclpy.shutdown()