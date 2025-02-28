"""
Runs an exploration algorithm using nav2 and the slam_toolbox.

Publishers
    + /explore/goal_pose_marker (visualization_msgs.msg.MarkerArray) - Visualizing the goal pose.
    + /explore/frontiers (visualization_msgs.msg.MarkerArray) - Visualizing the frontiers.
    + /goal_pose (geometry_msgs.msg.PoseStamped) - The goal pose for the robot.

Subscribers
    + /goal_pose (geometry_msgs.msg.PoseStamped) - The goal pose for the robot, for visualizing.
    + /map (nav_msgs.msg.OccupancyGrid) - The map for exploration.

Parameters
----------
    + explore_type (str) - The exploration algorithm to use. [frontier, random]

"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from nubot_nav.frontier import Frontier, FrontierUnion
import math
import numpy as np
import random


class Explore(Node):
    """Exploration Node for a diff-drive robot."""

    def __init__(self):
        """Initialize the Explore Node."""
        super().__init__('explore')
        self.get_logger().info('Explore Node Started')

        # ROS Parameters, Publishers, and Subscribers
        self.explore_type = self.declare_parameter(
            'explore_type', 'frontier'
        ).value
        self.pose_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
        self.goal_marker_pub = self.create_publisher(
            MarkerArray, '/explore/goal_pose_marker', 10
        )
        self.frontier_pub = self.create_publisher(
            MarkerArray, '/explore/frontiers', 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10
        )

        # Setup a Transform Listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Robot Pose and Map
        self.prev_robot_pose: Pose = None
        self.robot_pose: Pose = None  # map -> base_link
        self.saved_map: OccupancyGrid = None
        self.frontier_map = FrontierUnion()
        self.current_goal: Pose = None
        # How many times we need to publish a goal pose without the robot moving
        # to deem it a rejected goal
        self.robot_unmoving_count = 0
        self.goal_failed_threshold = 500  # This is about 5 seconds at 100 Hz

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
            self.create_timer(1.0 / self.freq, self.telemetry_cb)
        self.get_logger().info(
            f'Exploring at {self.freq} Hz using ' +
            f'{self.explore_type} algorithm'
        )

    def telemetry_cb(self):
        """Update the robot's pose so we can debug this node."""
        self.update_robot_pose()

    def random_timer_cb(self):
        """Run a random exploration algorithm."""
        self.update_robot_pose()
        # Randomly select a location for the robot to travel to
        if self.saved_map is None or self.prev_robot_pose is None:
            self.get_logger().info('No Map Received Yet', once=True)
            return
        if self.current_goal is not None:
            # If we're traveling to a goal pose,
            # check if we've reached it before
            # sending to a new random goal pose
            if self.robot_at_goal():
                self.get_logger().info(
                    'Robot reached goal pose: ' +
                    f'({self.current_goal.position.x:.3f}, ' +
                    f'{self.current_goal.position.y:.3f})'
                )
                random_pose = self.random_goal_pose()
                self.get_logger().info(
                    'Sending Robot to random goal pose: ' +
                    f'({random_pose.position.x:.3f}, ' +
                    f'{random_pose.position.y:.3f})'
                )
                self.send_robot(random_pose)
        else:
            # Current goal is None for first goal
            first_goal = Pose(
                position=self.robot_pose.position,
                orientation=self.robot_pose.orientation
            )
            # First goal should just drive forward a little bit
            first_goal.position.x += 1.5
            self.send_robot(first_goal)
        # If the robot is not moving for a while, reselect a random goal
        if not self.robot_is_moving():
            self.robot_unmoving_count += 1
            if self.robot_unmoving_count >= self.goal_failed_threshold:
                self.get_logger().warn(
                    'Robot failed to move ' +
                    f'{self.robot_unmoving_count} times' +
                    ', selecting new random goal'
                )
                self.robot_unmoving_count = 0
                self.send_robot(self.random_goal_pose())
        else:
            self.robot_unmoving_count = 0

    def frontier_timer_cb(self):
        """Run a frontier exploration algorithm."""
        # Get the transform from the map to the robot and save it
        self.update_robot_pose()
        # Run a Frontier Exploration Algorithm
        if self.saved_map is None or self.prev_robot_pose is None:
            self.get_logger().info('No Map Received Yet', once=True)
            return
        elif self.frontier_map.is_empty():
            self.get_logger().info('Explore Node Received Map', once=True)
            # Get the robot's current position
            robot_x = self.robot_pose.position.x
            robot_y = self.robot_pose.position.y

            # Create the first frontier at the robot's position
            first_frontier = Frontier(robot_x, robot_y)

            # Add the frontier to the frontier union
            self.frontier_map.add_frontier(first_frontier)

            # Visualize the first frontier
            frontier_markers = MarkerArray(
                markers=[self.get_frontier_marker(first_frontier)]
            )
            self.frontier_pub.publish(frontier_markers)

            # Pick a random point from the frontier
            goal_pose = self.frontier_random_goal(first_frontier)

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

                # visit the latest frontier
                latest_frontier = self.frontier_map.get_latest()
                latest_frontier.visit()

                # Create a new frontier here (Robot current's position)
                new_frontier_x = self.robot_pose.position.x
                new_frontier_y = self.robot_pose.position.y
                new_frontier = Frontier(new_frontier_x, new_frontier_y)

                # Select a random pose from the new frontier
                # that isn't already in our Union of frontiers
                new_goal = self.frontier_random_goal(new_frontier)

                # Visualize the frontiers
                self.frontier_map.add_frontier(new_frontier)
                frontier_markers = MarkerArray(
                    markers=[self.get_frontier_marker(
                        f) for f in self.frontier_map.frontiers]
                )
                self.frontier_pub.publish(frontier_markers)
                self.send_robot(new_goal)
        # If the robot is not moving for a while, reselect the last frontier
        if not self.robot_is_moving():
            self.robot_unmoving_count += 1
            if self.robot_unmoving_count >= self.goal_failed_threshold:
                self.get_logger().warn(
                    'Robot Failed to Move ' +
                    f'{self.robot_unmoving_count} Times'
                )
                self.robot_unmoving_count = 0
                self.frontier_map.remove_frontier(
                    self.frontier_map.get_latest()
                )
                new_goal = self.frontier_random_goal(
                    self.frontier_map.get_latest()
                )
                self.send_robot(new_goal)
        else:
            self.robot_unmoving_count = 0

    async def map_callback(self, msg: OccupancyGrid):
        """
        Save the map for navigation.

        :param msg: The map as an OccupancyGrid
        :type msg: OccupancyGrid
        """
        self.saved_map = msg

    def goal_pose_callback(self, msg: PoseStamped):
        """
        Subscribe to the goal_pose topic and visualize it with an arrow.

        :param msg: The goal pose message
        :type msg: PoseStamped
        """
        markers = MarkerArray()
        goal = Marker()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.ns = 'goal_pose'
        goal.id = 0
        goal.type = Marker.SPHERE
        goal.action = Marker.ADD
        goal.pose = msg.pose
        goal.scale.x = 0.4
        goal.scale.y = 0.4
        goal.scale.z = 0.4
        goal.color.r = 1.0
        goal.color.g = 0.0
        goal.color.b = 0.0
        goal.color.a = 0.5
        # Clear the arrow after some time
        goal.lifetime = rclpy.duration.Duration(seconds=20).to_msg()
        arrow = Marker()
        arrow.header.frame_id = 'map'
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = 'goal_pose'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = msg.pose
        arrow.scale.x = 1.5
        arrow.scale.y = 0.25
        arrow.scale.z = 0.25
        arrow.color.r = 1.0
        arrow.color.g = 0.0
        arrow.color.b = 1.0
        arrow.color.a = 0.75
        arrow.lifetime = rclpy.duration.Duration(seconds=20).to_msg()
        markers.markers = [goal, arrow]
        self.goal_marker_pub.publish(markers)

    def update_robot_pose(self):
        """Update the robot's pose using a transform from the map to the robot."""
        map_to_robot = None
        try:
            map_to_robot = self.buffer.lookup_transform(
                source_frame='base_link',
                target_frame='map',
                time=rclpy.time.Time().to_msg()
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

    def random_goal_pose(self) -> Pose:
        """
        Generate a valid random goal pose. Orientation is derived from the current robot pose.

        :return: A random goal pose
        :rtype: Pose
        """
        random_x = random.random() * self.saved_map.info.width * \
            self.saved_map.info.resolution + self.saved_map.info.origin.position.x
        random_y = random.random() * self.saved_map.info.height * \
            self.saved_map.info.resolution + self.saved_map.info.origin.position.y
        while not self.is_goal_valid(random_x, random_y, new_area=True):
            random_x = random.random() * self.saved_map.info.width * \
                self.saved_map.info.resolution + self.saved_map.info.origin.position.x
            random_y = random.random() * self.saved_map.info.height * \
                self.saved_map.info.resolution + self.saved_map.info.origin.position.y
        goal_pose = Pose()
        goal_pose.position.x = random_x
        goal_pose.position.y = random_y
        goal_pose.position.z = self.robot_pose.position.z
        heading = np.arctan2(random_y - self.robot_pose.position.y,
                             random_x - self.robot_pose.position.x)
        goal_pose.orientation = self.yaw2quat(heading)
        return goal_pose

    def is_goal_valid(self, x: float, y: float, new_area: bool = False) -> bool:
        """
        Given a position, determine if it is a valid goal position within the occupancy grid.

        :param x: The x-coordinate of the goal position
        :type x: float
        :param y: The y-coordinate of the goal position
        :type y: float
        :param new_area: If the goal is in a new area, defaults to False
        :type new_area: bool, optional
        :return: True if the point is marked as free in the occupancy grid, False otherwise
        :rtype: bool
        """
        map_data = np.array(self.saved_map.data).reshape(
            self.saved_map.info.height, self.saved_map.info.width
        )
        map_x = int((x - self.saved_map.info.origin.position.x) /
                    self.saved_map.info.resolution)
        map_y = int((y - self.saved_map.info.origin.position.y) /
                    self.saved_map.info.resolution)
        if 0 <= map_x < map_data.shape[1] and 0 <= map_y < map_data.shape[0]:
            # Free space is 0
            # Encourage the robot to explore new areas by choosing free spaces that are near
            # unexplored areas
            neighbors = [
                (map_x + 1, map_y), (map_x - 1, map_y),
                (map_x, map_y + 1), (map_x, map_y - 1),
                (map_x + 1, map_y + 1), (map_x - 1, map_y - 1),
                (map_x + 1, map_y - 1), (map_x - 1, map_y + 1)
            ]
            if map_data[map_y, map_x] == 0:
                if not new_area:
                    return True
                for nx, ny in neighbors:
                    if 0 <= nx < map_data.shape[1] and 0 <= ny < map_data.shape[0]:
                        if map_data[ny, nx] == -1:
                            return True
        return False

    def robot_is_moving(self) -> bool:
        """
        Determine if the robot is moving by comparing the current and previous poses.

        :return: True if the robot is moving, False otherwise
        :rtype: bool
        """
        if self.robot_pose is None or self.prev_robot_pose is None:
            return False

        distance = np.sqrt(
            (self.robot_pose.position.x - self.prev_robot_pose.position.x) ** 2 +
            (self.robot_pose.position.y - self.prev_robot_pose.position.y) ** 2
        )

        # Calculate the angular difference between current and previous orientations
        current_yaw = self.quat2yaw(self.robot_pose.orientation)
        prev_yaw = self.quat2yaw(self.prev_robot_pose.orientation)
        orientation_diff = abs(current_yaw - prev_yaw)

        # Normalize orientation difference to the range [0, pi]
        orientation_diff = min(orientation_diff, 2 *
                               math.pi - orientation_diff)
        return distance > 0.001 or orientation_diff > 0.001

    def send_robot(self, pose: Pose):
        """
        Send the robot to the given pose by publishing to goal_pose.

        :param pose: The pose to send the robot to
        :type pose: Pose
        """
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

    def robot_at_goal(self, epsilon: float = 0.75) -> bool:
        """
        Determine if the robot is at the current goal within some epsilon.

        :param epsilon: The tolerance for the distance function, defaults to 0.75
        :type epsilon: float, optional
        :return: True if the robot is at the goal, False otherwise
        :rtype: bool
        """
        if self.robot_pose is None or self.current_goal is None:
            return False
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        goal_x = self.current_goal.position.x
        goal_y = self.current_goal.position.y
        distance = np.sqrt((robot_x - goal_x) ** 2 + (robot_y - goal_y) ** 2)
        return distance <= epsilon

    def frontier_random_goal(self, new_frontier: Frontier) -> Pose:
        """
        Given a new frontier, select a random goal pose within the frontier.

        The selected goal pose is both valid (free in the occupancy grid) and
        not in the union of frontiers. This should encourage the selection
        of new areas to explore.

        :param new_frontier: The new frontier to select a goal pose from
        :type new_frontier: Frontier
        :return: The goal pose generated from the frontier
        :rtype: Pose
        """
        new_goal = new_frontier.random_pose_polar()
        while self.frontier_map.in_union(new_goal.position.x, new_goal.position.y) and \
                not self.is_goal_valid(new_goal.position.x, new_goal.position.y):
            new_goal = new_frontier.random_pose_polar()
        return new_goal

    def get_frontier_marker(self, frontier: Frontier) -> Marker:
        """
        Given a frontier, build a marker that represents it.

        :param frontier: The frontier to visualize
        :type frontier: Frontier
        :return: The marker with the correct color and size of the frontier
        :rtype: Marker
        """
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
        marker.scale.x = 2.0 * frontier.max_radius
        marker.scale.y = 2.0 * frontier.max_radius
        marker.scale.z = 0.15
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
        marker.color.a = 0.4
        return marker

    def yaw2quat(self, yaw: float) -> Quaternion:
        """
        Convert a yaw angle (in radians) to a quaternion.

        :param yaw: The yaw angle [rad]
        :paramtype yaw: float
        :return: The quaternion representing the yaw angle
        """
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )

    def quat2yaw(self, quaternion: Quaternion) -> float:
        """
        Convert a quaternion to a yaw angle (in radians).

        :param quaternion: quaternion to convert
        :type quaternion: Quaternion
        :return: The yaw angle [rad]
        :rtype: float
        """
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z),
            1.0 - 2.0 * (quaternion.z ** 2)
        )


def main(args=None):
    """Entry Point for the Explore Node."""
    rclpy.init(args=args)
    explore = Explore()
    rclpy.spin(explore)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
