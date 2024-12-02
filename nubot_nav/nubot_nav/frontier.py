"""Contains the Frontier and FrontierUnion classes for the frontier exploration algorithm."""
import itertools
import numpy as np
import random
from geometry_msgs.msg import Pose, Quaternion


# Frontier Algorithm:
# Donut frontiers are easier to implement
# Establish every visible point within some distance threshold as part of a frontier
# Use both a minimum and maximum distance threshold to define a "donut" frontier.
# Then select a random point in the frontier and set it as the goal_pose
# If the robot fails to move/navigate, then select a new point in the frontier.
# Next repeat this process but select a point in the new frontier that is not
# in the union of all of the created frontiers.


class Frontier:
    """Represents a donut frontier that can generate random poses."""

    # -------------- Begin_Citation [2] --------------#
    id_iter = itertools.count()

    def __init__(self, x: float, y: float):
        """
        Initialize a frontier at the given (x, y) location.

        :param x: The x-coordinate of the frontier.
        :type x: float
        :param y: The y-coordinate of the frontier.
        :type y: float
        """
        self.id = next(self.id_iter)
    # -------------- End_Citation [2] --------------#
        self.x = x
        self.y = y
        self.min_radius = 3.0
        self.max_radius = 5.0
        self.explored = False

    def __eq__(self, value: object) -> bool:
        """
        Determine if the given object is equal to this frontier.

        :param value: Another object to compare to.
        :type value: object
        :return: Whether the given frontier is equal to this frontier.
        :rtype: bool
        """
        if not isinstance(value, self.__class__):
            return False
        return self.id == value.id

    def __hash__(self) -> int:
        """
        Generate a hash code for this frontier using its id, x, and y.

        This enables frontiers to be stored in sets and dictionaries.

        :return: The hash code for this frontier.
        :rtype: int
        """
        return hash((self.id, self.x, self.y))

    def __repr__(self) -> str:
        """
        Create a string representing the frontier.

        :return: The frontier's id and location as a string.
        :rtype: str
        """
        return f'Frontier [{self.id}] centered at ({self.x}, {self.y})'

    def visit(self):
        """Visit this frontier and mark as explored."""
        self.explored = True

    def unvisit(self):
        """Mark this frontier as unexplored."""
        self.explored = False

    def contains(self, x: float, y: float) -> bool:
        """
        Determine if the given point is within the frontier.

        :param x: The x-coordinate of the point.
        :type x: float
        :param y: The y-coordinate of the point.
        :type y: float
        :return: True if the point is within the frontier, False otherwise.
        :rtype: bool
        """
        distance = np.sqrt(((x - self.x) ** 2 + (y - self.y) ** 2))
        return self.min_radius <= distance <= self.max_radius

    def random_pose_cart(self) -> Pose:
        """
        Generate a random pose within this frontier using Cartesian coordinates.

        :return: A pose within this frontier that points away from the center.
        :rtype: Pose
        """
        # -------------- Begin_Citation [1] --------------#
        # NOTE: Supposedly polar coordinates aren't uniformly distributed
        # so this function can be used instead
        # -------------- End_Citation [1] --------------#
        random_pose = Pose()
        rand_x = random.random() * 2 * self.max_radius + self.x - self.max_radius
        rand_y = random.random() * 2 * self.max_radius + self.y - self.max_radius
        while not self.contains(rand_x, rand_y):
            rand_x = random.random() * 2 * self.max_radius + self.x - self.max_radius
            rand_y = random.random() * 2 * self.max_radius + self.y - self.max_radius
        # This pose is in the frame of the frontier, so we need to transform it to the map frame
        random_pose.position.x = rand_x
        random_pose.position.y = rand_y
        # Select an orientation that points away from the center
        heading = np.arctan2(rand_y - self.y, rand_x - self.x)
        random_pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=np.sin(heading / 2.0),
            w=np.cos(heading / 2.0)
        )
        return random_pose

    def random_pose_polar(self) -> Pose:
        """
        Generate a random pose within this frontier using polar coordinates.

        :return: A pose within this frontier that points away from the center.
        :rtype: Pose
        """
        # -------------- Begin_Citation [3] --------------#
        # Generate a random radius with uniform area weighting
        rand_radius = np.sqrt(
            random.uniform(self.min_radius ** 2, self.max_radius ** 2)
        )
        # -------------- End_Citation [3] --------------#
        # Random angle
        rand_angle = random.uniform(0, 2 * np.pi)
        # Convert polar coordinates to Cartesian
        rand_x = self.x + rand_radius * np.cos(rand_angle)
        rand_y = self.y + rand_radius * np.sin(rand_angle)
        # Set the position of the pose
        random_pose = Pose()
        random_pose.position.x = rand_x
        random_pose.position.y = rand_y
        # Set the orientation to face outward from the center
        random_pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=np.sin(rand_angle / 2.0),
            w=np.cos(rand_angle / 2.0)
        )
        return random_pose


class FrontierUnion:
    """A union of frontiers that represent an explored area."""

    def __init__(self):
        """Initialize the union as an empty set."""
        self.frontiers: set[Frontier] = set()

    def add_frontier(self, frontier: Frontier):
        """
        Add the frontier to the union, if it already exists, it won't be added.

        :param frontier: The frontier to add
        :type frontier: Frontier
        """
        self.frontiers.add(frontier)

    def remove_frontier(self, frontier: Frontier):
        """
        Remove the given frontier from the union.

        :param frontier: The frontier to remove
        :type frontier: Frontier
        """
        self.frontiers.remove(frontier)

    def get_latest(self) -> Frontier:
        """
        Get the latest frontier added to the union.

        :return: The frontier with the highest id, since ids are generated sequentially
        :rtype: Frontier
        """
        return max(self.frontiers, key=lambda frontier: frontier.id)

    def is_empty(self) -> bool:
        """
        Determine if the union is empty.

        :return: True if empty, False otherwise
        :rtype: bool
        """
        return len(self.frontiers) == 0

    def in_union(self, x: float, y: float) -> bool:
        """
        Determine if the given point is within any of the frontiers in the union.

        :param x: The x-coordinate of the point
        :type x: float
        :param y: The y-coordinate of the point
        :type y: float
        :return: True if the point is in the union, False otherwise
        :rtype: bool
        """
        return any(frontier.contains(x, y) for frontier in self.frontiers)


if __name__ == '__main__':
    frontier = Frontier(0, 0)
    print(frontier)
    print('Generating 10 random poses (cartesian):')
    for _ in range(10):
        rand_pose = frontier.random_pose_cart()
        print(
            f'Random Pose: ({rand_pose.position.x:.3f}, ' +
            f'{rand_pose.position.y:.3f})'
        )
        # verify that the random pose is within the frontier
        assert frontier.contains(rand_pose.position.x, rand_pose.position.y)
    print('Generating 10 random poses (polar):')
    for _ in range(10):
        rand_pose = frontier.random_pose_polar()
        print(
            f'Random Pose: ({rand_pose.position.x:.3f}, ' +
            f'{rand_pose.position.y:.3f})'
        )
        # verify that the random pose is within the frontier
        assert frontier.contains(rand_pose.position.x, rand_pose.position.y)
