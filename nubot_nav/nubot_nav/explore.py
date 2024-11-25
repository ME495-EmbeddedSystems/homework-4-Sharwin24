import rclpy
from rclpy.node import Node


class Explore(Node):
    def __init__(self):
        super().__init__('explore')
        self.get_logger().info("Explore Node Started")
        self.freq = 10  # [Hz]
        self.create_timer(1.0 / self.freq, self.timer_callback)

    def timer_callback(self):
        # Run a Frontier Exploration Algorithm
        pass


def main(args=None):
    rclpy.init(args=args)
    explore = Explore()
    rclpy.spin(explore)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
