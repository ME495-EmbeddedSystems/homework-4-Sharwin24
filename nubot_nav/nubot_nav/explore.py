import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import Buffer, TransformListener
import random
from nubot_nav.frontier import Frontier, FrontierUnion


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

        # Parameter "explore_type"
        self.explore_type = self.declare_parameter(
            'explore_type', 'frontier'
        ).value

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
        if self.explore_type == 'frontier':
            self.frontier_pub = self.create_publisher(
                MarkerArray, '/explore/frontiers', 10
            )

        # Setup a Transform Listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Robot Pose and Map
        self.prev_robot_pose: Pose = None
        self.robot_pose: Pose = None
        self.saved_map: OccupancyGrid = None
        self.frontier_map = FrontierUnion()
        self.current_goal: Pose = None
        # How many times we need to publish a goal pose without the robot moving
        # to deem it a rejected goal
        self.robot_unmoving_count = 0
        self.goal_failed_threshold = 25

        self.freq = 100  # [Hz]
        # Timer for frontier exploration
        if self.explore_type == 'frontier':
            self.create_timer(1.0 / self.freq, self.frontier_timer_cb)
        elif self.explore_type == 'random':
            self.create_timer(1.0 / self.freq, self.random_timer_cb)
        else:
            self.get_logger().error(
                f'Invalid Explore Type: {self.explore_type}'
            )
        self.get_logger().info(
            f'Exploring at {self.freq} Hz using ' +
            f'{self.explore_type} algorithm'
        )

    def random_timer_cb(self):
        self.update_robot_pose()
        # Randomly select a location for the robot to travel to
        if self.saved_map is None or self.prev_robot_pose is None:
            self.get_logger().info('No Map Received Yet', once=True)
            return
        # Get map information
        map_width = self.saved_map.info.width
        map_height = self.saved_map.info.height
        map_resolution = self.saved_map.info.resolution
        map_origin_x = self.saved_map.info.origin.position.x
        map_origin_y = self.saved_map.info.origin.position.y
        if self.current_goal is not None:
            # If we're traveling to a goal pose,
            # check if we've reached it before
            # sending to a new random goal pose
            if self.robot_at_goal():
                self.get_logger().info(
                    'Robot Reached Goal Pose: ' +
                    f'({self.current_goal.position.x:.3f}, ' +
                    f'{self.current_goal.position.y:.3f})'
                )
                random_x = random.random() * map_width * map_resolution + map_origin_x
                random_y = random.random() * map_height * map_resolution + map_origin_y
                goal_pose = Pose()
                goal_pose.position.x = random_x
                goal_pose.position.y = random_y
                goal_pose.orientation = self.current_goal.orientation
                self.send_robot(goal_pose)
        else:
            self.get_logger().info('First Random Goal Pose', once=True)
            # Current goal is None for first goal
            random_x = random.random() * map_width * map_resolution + map_origin_x
            random_y = random.random() * map_height * map_resolution + map_origin_y
            goal_pose = Pose()
            goal_pose.position.x = random_x
            goal_pose.position.y = random_y
            goal_pose.orientation = self.robot_pose.orientation
            self.send_robot(goal_pose)
        # Check if the robot is moving and if not
        # we need to select a new goal pose
        if not self.robot_is_moving():
            self.robot_unmoving_count += 1
            self.get_logger().info(
                f'Robot Unmoving Count: {self.robot_unmoving_count}'
            )
        if self.robot_unmoving_count >= self.goal_failed_threshold:
            self.get_logger().warn(
                'Robot Failed to Move ' +
                f'{self.robot_unmoving_count} Times'
            )
            self.robot_unmoving_count = 0
            random_x = random.random() * map_width * map_resolution + map_origin_x
            random_y = random.random() * map_height * map_resolution + map_origin_y
            goal_pose = Pose()
            goal_pose.position.x = random_x
            goal_pose.position.y = random_y
            self.send_robot(goal_pose)

    def frontier_timer_cb(self):
        # Get the transform from the map to the robot and save it
        self.update_robot_pose()
        # Run a Frontier Exploration Algorithm
        if self.saved_map is None or self.prev_robot_pose is None:
            self.get_logger().info('No Map Received Yet', once=True)
            return
        elif self.frontier_map.is_empty():
            self.get_logger().info('Explore Node Received Map', once=True)
            # Get the robot's current position
            if self.robot_pose is None:
                self.get_logger().warn('No Robot Pose Received Yet')
                return
            robot_x = self.robot_pose.position.x
            robot_y = self.robot_pose.position.y

            # Create a frontier at the map's origin since it is the first frontier
            first_frontier = Frontier(robot_x, robot_y)

            # Add the frontier to the frontier union
            self.frontier_map.add_frontier(first_frontier)

            # Visualize the first frontier
            frontier_markers = MarkerArray(
                markers=[self.get_frontier_marker(first_frontier)]
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
                    'Robot Reached Goal Pose: ' +
                    f'({self.current_goal.position.x:.3f}, ' +
                    f'{self.current_goal.position.y:.3f})'
                )
                # The robot is at the goal pose (or nearby)
                # Get the robot's current position
                if self.robot_pose is None:
                    self.get_logger().warn('No Robot Pose Received Yet')
                    return

                # Create a new frontier here (Robot current's position)
                new_frontier_x = self.robot_pose.position.x
                new_frontier_y = self.robot_pose.position.y
                new_frontier = Frontier(new_frontier_x, new_frontier_y)

                # Select a random pose from the new frontier
                # that isn't already in our Union of frontiers
                new_goal = self.generate_random_goal(new_frontier)
                # After we've found our new goal, we can add the frontier to the union
                # Visualize the frontiers
                frontier_markers = MarkerArray(
                    markers=[self.get_frontier_marker(
                        f) for f in self.frontier_map.frontiers]
                )
                # Visualize the new frontier
                frontier_markers.markers.append(
                    self.get_frontier_marker(new_frontier)
                )
                self.frontier_pub.publish(frontier_markers)
                self.frontier_map.add_frontier(new_frontier)
                self.send_robot(new_goal)
        # Check if the robot is moving and if not
        # we need to select a new goal pose
        self.get_logger().info(
            f'Robot Unmoving Count: {self.robot_unmoving_count}' +
            ' Checking if robot is moving...'
        )
        distance = np.sqrt(
            (self.robot_pose.position.x - self.prev_robot_pose.position.x) ** 2 +
            (self.robot_pose.position.y -
                self.prev_robot_pose.position.y) ** 2
        )
        if distance <= 0.0001:
            self.robot_unmoving_count += 1
        if self.robot_unmoving_count >= self.goal_failed_threshold:
            self.get_logger().warn(
                'Robot Failed to Move ' +
                f'{self.robot_unmoving_count} Times'
            )
            self.robot_unmoving_count = 0
            latest_frontier = self.frontier_map.get_latest_frontier()
            # Delete the latest frontier
            self.frontier_map.remove_frontier(latest_frontier)
            new_goal = self.generate_random_goal(latest_frontier)
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
        self.prev_robot_pose = self.robot_pose
        self.robot_pose = Pose()
        self.robot_pose.position.x = map_to_robot.transform.translation.x
        self.robot_pose.position.y = map_to_robot.transform.translation.y
        self.robot_pose.position.z = map_to_robot.transform.translation.z
        self.robot_pose.orientation = map_to_robot.transform.rotation
        self.get_logger().info(
            f'Robot Pose: ({self.robot_pose.position.x:.3f}, ' +
            f'{self.robot_pose.position.y:.3f})'
        )

    def map_callback(self, msg):
        self.saved_map = msg

    def pick_random_goal(self):
        random_x = random.random() * self.saved_map.info.width * \
            self.saved_map.info.resolution + self.saved_map.info.origin.position.x
        random_y = random.random() * self.saved_map.info.height * \
            self.saved_map.info.resolution + self.saved_map.info.origin.position.y
        while not self.is_goal_valid(random_x, random_y):
            random_x = random.random() * self.saved_map.info.width * \
                self.saved_map.info.resolution + self.saved_map.info.origin.position.x
            random_y = random.random() * self.saved_map.info.height * \
                self.saved_map.info.resolution + self.saved_map.info.origin.position.y
        goal_pose = Pose()
        goal_pose.position.x = random_x
        goal_pose.position.y = random_y
        goal_pose.orientation = self.robot_pose.orientation
        return goal_pose

    def is_goal_valid(self, x, y):
        map_data = np.array(self.saved_map.data).reshape(
            self.saved_map.info.height, self.saved_map.info.width
        )
        map_x = int((x - self.saved_map.info.origin.position.x) /
                    self.saved_map.info.resolution)
        map_y = int((y - self.saved_map.info.origin.position.y) /
                    self.saved_map.info.resolution)
        if 0 <= map_x < map_data.shape[1] and 0 <= map_y < map_data.shape[0]:
            # Free space is usually marked as 0 in occupancy grids
            return map_data[map_y, map_x] == 0
        return False

    def robot_is_moving(self):
        if self.robot_pose is None or self.prev_robot_pose is None:
            return False

        distance = np.sqrt(
            (self.robot_pose.position.x - self.prev_robot_pose.position.x) ** 2 +
            (self.robot_pose.position.y - self.prev_robot_pose.position.y) ** 2
        )

        # Calculate the angular difference between current and previous orientations
        def quat2yaw(quaternion):
            return math.atan2(2.0 * (quaternion.w * quaternion.z),
                              1.0 - 2.0 * (quaternion.z ** 2))

        current_yaw = quat2yaw(self.robot_pose.orientation)
        prev_yaw = quat2yaw(self.prev_robot_pose.orientation)
        orientation_diff = abs(current_yaw - prev_yaw)

        # Normalize orientation difference to the range [0, pi]
        orientation_diff = min(orientation_diff, 2 *
                               math.pi - orientation_diff)

        # Check if the robot has moved (position or orientation)
        return distance > 0.01 or orientation_diff > 0.05

    def send_robot(self, pose: Pose):
        # Check if the robot is travelling towards the goal pose
        goal_pose_msg = PoseStamped()
        goal_pose_msg.pose = pose
        goal_pose_msg.header.frame_id = 'map'
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            'Sending Robot to Goal Pose: ' +
            f'({pose.position.x:.3f}, {pose.position.y:.3f})'
        )
        self.pose_pub.publish(goal_pose_msg)
        # Update the current goal pose
        self.current_goal = pose

    def robot_at_goal(self, epsilon: float = 0.1) -> bool:
        if self.robot_pose is None or self.current_goal is None:
            return False
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        goal_x = self.current_goal.position.x
        goal_y = self.current_goal.position.y
        return np.sqrt((robot_x - goal_x) ** 2 + (robot_y - goal_y) ** 2) <= epsilon

    def generate_random_goal(self, new_frontier: Frontier) -> Pose:
        new_goal = new_frontier.generate_random_pose()
        while self.frontier_map.in_union(new_goal.position.x, new_goal.position.y):
            new_goal = new_frontier.generate_random_pose()
        return new_goal

    def get_frontier_marker(self, frontier: Frontier) -> Marker:
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
        if frontier.id == 0:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif frontier.explored:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        marker.color.a = 0.25
        return marker


def main(args=None):
    rclpy.init(args=args)
    explore = Explore()
    rclpy.spin(explore)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
