import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Quaternion
import numpy as np
from enum import Enum, auto


class Frontier:
    def __init__(self, x, y):
        # (x,y ) defines the origin of the frontier
        self.x = x
        self.y = y
        self.min_radius = 0.8  # Defines the inner radius of the "donut" frontier
        self.max_radius = 2.0  # Defines the outer radius of the "donut" frontier
        self.explored = False

    def __eq__(self, value):
        if not isinstance(value, self.__class__):
            return False
        return self.x == value.x and self.y == value.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __repr__(self):
        return f'Frontier centered at ({self.x}, {self.y})'

    def visit(self):
        self.explored = True

    def contains(self, x, y):
        # Check if the point (x, y) is within the "donut" frontier
        distance = np.sqrt(((x - self.x) ** 2 + (y - self.y) ** 2))
        return self.min_radius <= distance <= self.max_radius

    def generate_random_pose(self) -> Pose:
        # -------------- Begin_Citation [1] --------------#
        # NOTE: I'm not randomly picking this point using polar coordinates
        # because it isn't uniformly distributed.
        # -------------- End_Citation [1] --------------#
        rand_x = np.random.uniform(
            self.x - self.max_radius,
            self.x + self.max_radius
        )
        rand_y = np.random.uniform(
            self.y - self.max_radius,
            self.y + self.max_radius
        )
        while not self.contains(rand_x, rand_y):
            rand_x = np.random.uniform(
                self.x - self.max_radius, self.x + self.max_radius
            )
            rand_y = np.random.uniform(
                self.y - self.max_radius,
                self.y + self.max_radius
            )
        # Select an orientation that points away from the center
        heading = np.arctan2(rand_y - self.y, rand_x - self.x)
        ori = Quaternion(
            x=0.0,
            y=0.0,
            z=np.sin(heading / 2),
            w=np.cos(heading / 2)
        )
        return Pose(x=rand_x, y=rand_y, orientation=ori)


class FrontierUnion:
    def __init__(self):
        self.frontiers: set[Frontier] = set()

    def add_frontier(self, frontier: Frontier):
        self.frontiers.add(frontier)

    def remove_frontier(self, frontier: Frontier):
        self.frontiers.remove(frontier)

    def is_empty(self) -> bool:
        return len(self.frontiers) == 0

    def in_union(self, x: float, y: float) -> bool:
        return any(frontier.contains(x, y) for frontier in self.frontiers)


# Frontier Algorithm:
    # Establish every visible point within some distance threshold as part of a frontier
    # Use both a minimum and maximum distance threshold to define a "donut" frontier.
    # Then select a random point in the frontier and set it as the goal_pose
    # If the robot fails to move/navigate, then select a new point in the frontier.
    # Next repeat this process but select a point in the new frontier that is not
    # in the union of all of the created frontiers.


class Explore(Node):
    def __init__(self):
        super().__init__('explore')
        self.get_logger().info('Explore Node Started')

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        # Change this to a transform listener from map to base_link
        self.robot_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.robot_pose_callback, 10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )

        # Robot Pose and Map
        self.robot_pose: PoseWithCovarianceStamped = None
        self.saved_map: OccupancyGrid = None
        self.frontier_map = FrontierUnion()
        self.current_goal: Pose = None

        # Timer for frontier exploration
        self.freq = 10  # [Hz]
        self.create_timer(1.0 / self.freq, self.timer_callback)

    def timer_callback(self):
        # Run a Frontier Exploration Algorithm
        if self.saved_map is None:
            self.get_logger().info('No Map Received Yet', once=True)
            return
        elif self.frontier_map.is_empty():
            self.get_logger().info('Explore Node Received Map', once=True)
            # Get the map's origin
            map_origin_x = self.saved_map.info.origin.position.x
            map_origin_y = self.saved_map.info.origin.position.y

            # Create a frontier at the map's origin since it is the first frontier
            first_frontier = Frontier(map_origin_x, map_origin_y)

            # Add the frontier to the frontier union
            self.frontier_map.add_frontier(first_frontier)

            # Pick a random point from the frontier
            goal_pose = first_frontier.generate_random_pose()

            # Publish the goal pose
            self.send_robot(goal_pose)
        else:
            # If the frontier_map has the first frontier
            # Check if the robot is at the goal pose
            if self.robot_at_goal():
                # The robot is at the goal pose (or nearby)
                # Get the robot's current position
                robot_x = self.robot_pose.pose.pose.position.x
                robot_y = self.robot_pose.pose.pose.position.y

                # Create a new frontier here
                new_frontier = Frontier(robot_x, robot_y)

                # Select a random pose from the new frontier
                # that isn't already in our Union of frontiers
                new_goal = new_frontier.generate_random_pose()
                while self.frontier_map.in_union(new_goal.position.x, new_goal.position.y):
                    new_goal = new_frontier.generate_random_pose()
                # After we've found our new goal, we can add the frontier to the union
                self.frontier_map.add_frontier(new_frontier)

                # Publish the new goal pose
                self.send_robot(new_goal)

    def robot_pose_callback(self, msg):
        self.robot_pose = msg

    def map_callback(self, msg):
        self.saved_map = msg

    def send_robot(self, pose: Pose):
        # Check if the robot is travelling towards the goal pose
        goal_pose_msg = PoseStamped()
        goal_pose_msg.pose = pose
        goal_pose_msg.header.frame_id = 'map'
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.current_goal = pose
        self.pose_pub.publish(goal_pose_msg)

    def robot_at_goal(self, epsilon: float = 0.1) -> bool:
        robot_x = self.robot_pose.pose.pose.position.x
        robot_y = self.robot_pose.pose.pose.position.y
        goal_x = self.current_goal.position.x
        goal_y = self.current_goal.position.y
        return np.sqrt((robot_x - goal_x) ** 2 + (robot_y - goal_y) ** 2) <= epsilon


def main(args=None):
    rclpy.init(args=args)
    explore = Explore()
    rclpy.spin(explore)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
