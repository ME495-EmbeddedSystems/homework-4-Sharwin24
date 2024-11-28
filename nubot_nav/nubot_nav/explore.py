import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Quaternion
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import Buffer, TransformListener
import itertools


class Frontier:
    # -------------- Begin_Citation [2] --------------#
    id_iter = itertools.count()

    def __init__(self, x, y):
        self.id = next(self.id_iter)
    # -------------- End_Citation [2] --------------#
        # (x,y ) defines the origin of the frontier
        self.x = x
        self.y = y
        self.min_radius = 3.0  # Defines the inner radius of the "donut" frontier
        self.max_radius = 10.0  # Defines the outer radius of the "donut" frontier
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
            z=np.sin(heading / 2.0),
            w=np.cos(heading / 2.0)
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
        self.pose_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
        self.frontier_pub = self.create_publisher(
            MarkerArray, '/explore/frontiers', 10
        )

        # Setup a Transform Listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Robot Pose and Map
        self.robot_pose: Pose = None
        self.saved_map: OccupancyGrid = None
        self.frontier_map = FrontierUnion()
        self.current_goal: Pose = None

        # Timer for frontier exploration
        self.freq = 10  # [Hz]
        self.create_timer(1.0 / self.freq, self.timer_callback)

    def timer_callback(self):
        # Get the transform from the map to the robot and save it
        self.update_robot_pose()
        # Run a Frontier Exploration Algorithm
        if self.saved_map is None:
            self.get_logger().info('No Map Received Yet', once=True)
            return
        elif self.frontier_map.is_empty():
            self.get_logger().info('Explore Node Received Map', once=True)
            # Get the robot's current position
            robot_x = self.robot_pose.position.x
            robot_y = self.robot_pose.position.y

            # Create a frontier at the map's origin since it is the first frontier
            first_frontier = Frontier(robot_x, robot_y)

            # Add the frontier to the frontier union
            self.frontier_map.add_frontier(first_frontier)

            # Visualize the first frontier
            frontier_markers = MarkerArray(
                markers=[self.get_frontier_marker(first_frontier, new=True)]
            )
            self.frontier_pub.publish(frontier_markers)

            # Pick a random point from the frontier
            goal_pose = first_frontier.generate_random_pose()

            # Publish the goal pose
            self.send_robot(goal_pose)
        else:  # The frontier_map has frontiers
            self.get_logger().info('Exploring Frontiers', once=True)
            # If the frontier_map has the first frontier
            # Check if the robot is at the goal pose
            if self.robot_at_goal():
                self.get_logger().info(
                    f'Robot at Goal Pose: {self.current_goal}'
                )
                # The robot is at the goal pose (or nearby)
                # Get the robot's current position
                if self.robot_pose is None:
                    self.get_logger().warn('No Robot Pose Received Yet')
                    return

                # Create a new frontier here
                goal_x = self.current_goal.position.x
                goal_y = self.current_goal.position.y
                new_frontier = Frontier(goal_x, goal_y)

                # Select a random pose from the new frontier
                # that isn't already in our Union of frontiers
                new_goal = new_frontier.generate_random_pose()
                while self.frontier_map.in_union(new_goal.position.x, new_goal.position.y):
                    new_goal = new_frontier.generate_random_pose()
                # After we've found our new goal, we can add the frontier to the union
                # Visualize the frontiers
                frontier_markers = MarkerArray(
                    markers=[self.get_frontier_marker(
                        f) for f in self.frontier_map.frontiers]
                )
                # Visualize the new frontier
                frontier_markers.markers.append(
                    self.get_frontier_marker(new_frontier, new=True)
                )
                self.frontier_pub.publish(frontier_markers)

                self.frontier_map.add_frontier(new_frontier)
                # Publish the new goal pose
                self.send_robot(new_goal)

    def update_robot_pose(self):
        map_to_robot = None
        try:
            map_to_robot = self.buffer.lookup_transform(
                source_frame='map',
                target_frame='base_link',
                time=rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to get transform: {e}')
            return
        if map_to_robot is None:
            self.get_logger().warn('No Transform Received')
            return
        self.robot_pose = Pose()
        self.robot_pose.position.x = map_to_robot.transform.translation.x
        self.robot_pose.position.y = map_to_robot.transform.translation.y
        self.robot_pose.position.z = map_to_robot.transform.translation.z
        self.robot_pose.orientation = map_to_robot.transform.rotation

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

    def get_frontier_marker(self, frontier: Frontier, new: bool = False) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontier'
        marker.id = frontier.id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = frontier.x
        marker.pose.position.y = frontier.y
        marker.pose.position.z = 0.0
        marker.scale.x = frontier.max_radius
        marker.scale.y = frontier.max_radius
        marker.scale.z = 0.1
        marker.color.r = 0.0 if new else 1.0
        marker.color.g = 1.0 if new else 0.0
        marker.color.b = 0.0
        marker.color.a = 0.75 if new else 0.5
        return marker


def main(args=None):
    rclpy.init(args=args)
    explore = Explore()
    rclpy.spin(explore)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
