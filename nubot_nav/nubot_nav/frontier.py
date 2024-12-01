import itertools
import numpy as np
import random

from geometry_msgs.msg import Pose, Quaternion


class Frontier:
    # -------------- Begin_Citation [2] --------------#
    id_iter = itertools.count()

    def __init__(self, x, y, min_radius=5.0, max_radius=6.0):
        self.id = next(self.id_iter)
    # -------------- End_Citation [2] --------------#
        self.x = x
        self.y = y
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.explored = False

    def __eq__(self, value):
        if not isinstance(value, self.__class__):
            return False
        return self.id == value.id

    def __hash__(self):
        return hash((self.id, self.x, self.y))

    def __repr__(self):
        return f'Frontier [{self.id}] centered at ({self.x}, {self.y})'

    def visit(self):
        self.explored = True

    def unvisit(self):
        self.explored = False

    def contains(self, x, y):
        # Check if the point (x, y) is within the "donut" frontier
        distance = np.sqrt(((x - self.x) ** 2 + (y - self.y) ** 2))
        return distance > self.min_radius and distance < self.max_radius

    def generate_random_pose(self) -> Pose:
        # -------------- Begin_Citation [1] --------------#
        # NOTE: I'm not randomly picking this point using polar coordinates
        # because it isn't uniformly distributed.
        # -------------- End_Citation [1] --------------#
        random_pose = Pose()
        rand_x = random.random() * 2 * self.max_radius + self.x - self.max_radius
        rand_y = random.random() * 2 * self.max_radius + self.y - self.max_radius
        while not self.contains(rand_x, rand_y):
            rand_x = random.random() * 2 * self.max_radius + self.x - self.max_radius
            rand_y = random.random() * 2 * self.max_radius + self.y - self.max_radius
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


class FrontierUnion:
    def __init__(self):
        self.frontiers: set[Frontier] = set()

    def add_frontier(self, frontier: Frontier):
        frontier.visit()
        self.frontiers.add(frontier)

    def remove_frontier(self, frontier: Frontier):
        frontier.unvisit()
        self.frontiers.remove(frontier)

    def get_latest_frontier(self):
        return max(self.frontiers, key=lambda frontier: frontier.id)

    def is_empty(self) -> bool:
        return len(self.frontiers) == 0

    def in_union(self, x: float, y: float) -> bool:
        return any(frontier.contains(x, y) for frontier in self.frontiers)


if __name__ == '__main__':
    frontier = Frontier(0, 0)
    print(frontier)
    print('Generating 10 random poses:')
    for _ in range(10):
        rand_pose = frontier.generate_random_pose()
        print(
            f'Random Pose: ({rand_pose.position.x:.3f}, ' +
            f'{rand_pose.position.y:.3f})'
        )
