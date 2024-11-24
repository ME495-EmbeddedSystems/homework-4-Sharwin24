import rclpy
from rclpy.node import Node


class Explore(Node):
    def __init__(self):
        super().__init__('explore')
        self.get_logger().info("Explore Node Started")


def main(args=None):
    rclpy.init(args=args)
    explore = Explore()
    rclpy.spin(explore)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
