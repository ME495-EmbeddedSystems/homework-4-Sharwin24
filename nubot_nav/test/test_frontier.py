"""Test the frontier module."""
import pytest
from nubot_nav.frontier import Frontier, FrontierUnion
# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


def test_contains():
    """Test if contains method works."""
    frontier = Frontier(0, 0)
    r_min = frontier.min_radius
    r_max = frontier.max_radius
    assert not frontier.contains(0, 0)
    assert not frontier.contains(r_min - 0.1, 0)
    assert frontier.contains(r_min, 0)
    assert frontier.contains(r_max, 0)
    assert frontier.contains(r_max - 0.5, 0)
    assert frontier.contains(r_min + 0.5, 0)
    assert frontier.contains(r_min + 0.1, r_min - 0.2)


def test_random_pose():
    """Test if random_pose method works."""
    frontier = Frontier(0, 0)
    random_pose = frontier.random_pose_polar()
    assert frontier.contains(random_pose.position.x, random_pose.position.y)
    random_pose = frontier.random_pose_cart()
    assert frontier.contains(random_pose.position.x, random_pose.position.y)


def test_union():
    """Test if frontier union works."""
    union = FrontierUnion()
    frontier1 = Frontier(0, 0)
    min_r = frontier1.min_radius
    assert frontier1.contains(min_r + 0.1, 0)
    frontier2 = Frontier(min_r + 0.1, 0)
    union.frontiers.add(frontier1)
    union.frontiers.add(frontier2)
    assert len(union.frontiers) == 2
    assert union.in_union(0, 0)
    assert union.in_union(min_r + 0.1, 0)
    assert not union.in_union(min_r - 0.1, 0)
