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
    frontier = Frontier(0, 0)
    assert not frontier.contains(0, 0)
    assert not frontier.contains(3, 0)
    assert not frontier.contains(7, 0)
    assert frontier.contains(3.5, 0)
    assert frontier.contains(4, 0)
