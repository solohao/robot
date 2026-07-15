# Copyright 2026 Rodriguez Sage
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

import importlib.util
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from launch.actions import DeclareLaunchArgument
from nav_msgs.msg import Path as PathMessage


def launch_arguments():
    launch_path = (
        Path(__file__).parents[1] / 'launch' / 'simulation.launch.py'
    )
    spec = importlib.util.spec_from_file_location(
        'simulation_launch',
        launch_path,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    return {
        action.name
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }


def test_namespace_is_available_to_delayed_controller_actions():
    arguments = launch_arguments()
    assert 'namespace' in arguments


def test_localization_parameters_can_be_overridden():
    arguments = launch_arguments()
    assert 'localization_params' in arguments


def test_simulation_spawn_pose_can_be_overridden():
    arguments = launch_arguments()
    assert {'x', 'y', 'z', 'yaw'} <= arguments


def test_extended_mapping_route_uses_velocity_smoother_input():
    script = (
        Path(__file__).parents[1]
        / 'scripts'
        / 'run_extended_mapping_route.py'
    )
    assert "create_publisher(Twist, '/cmd_vel_nav', 10)" in script.read_text()


def test_long_navigation_measures_obstacle_detour():
    script = (
        Path(__file__).parents[1]
        / 'scripts'
        / 'run_long_navigation.py'
    )
    spec = importlib.util.spec_from_file_location('run_long_navigation', script)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    path = PathMessage()
    for x, y in ((0.0, 0.0), (1.0, 2.0), (2.0, 0.0)):
        pose = PoseStamped()
        path.poses.append(pose)
        pose.pose.position.x = x
        pose.pose.position.y = y

    length, direct, ratio, deviation = module.path_detour_metrics(path, 2.0, 0.0)
    assert math.isclose(length, 2.0 * math.sqrt(5.0))
    assert math.isclose(direct, 2.0)
    assert math.isclose(ratio, math.sqrt(5.0))
    assert math.isclose(deviation, 2.0)
