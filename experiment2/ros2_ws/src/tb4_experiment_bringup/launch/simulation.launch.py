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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    bringup_share = get_package_share_directory('tb4_experiment_bringup')
    navigation_share = get_package_share_directory('turtlebot4_navigation')
    simulator_share = get_package_share_directory('turtlebot4_ignition_bringup')
    viz_share = get_package_share_directory('turtlebot4_viz')

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    gz_args = LaunchConfiguration('gz_args')
    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [simulator_share, 'launch', 'turtlebot4_ignition.launch.py']
            )
        ),
        launch_arguments={
            'world': world,
            'model': model,
            'rviz': 'false',
            'gz_args': gz_args,
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [navigation_share, 'launch', 'localization.launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_file,
        }.items(),
        condition=UnlessCondition(slam),
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([navigation_share, 'launch', 'slam.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'sync': 'true',
        }.items(),
        condition=IfCondition(slam),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([navigation_share, 'launch', 'nav2.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': PathJoinSubstitution(
                [bringup_share, 'config', 'nav2_astar.yaml']
            ),
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([viz_share, 'launch', 'view_robot.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'model': model,
        }.items(),
    )

    delayed_navigation = TimerAction(
        period=5.0,
        actions=[localization, mapping, nav2, rviz],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value='warehouse',
                choices=['warehouse', 'depot', 'maze'],
            ),
            DeclareLaunchArgument(
                'model',
                default_value='standard',
                choices=['standard', 'lite'],
            ),
            DeclareLaunchArgument(
                'gz_args',
                default_value='',
                description='Override arguments passed to Gazebo Sim.',
            ),
            DeclareLaunchArgument(
                'slam',
                default_value='false',
                choices=['true', 'false'],
                description='Run SLAM instead of localization on an existing map.',
            ),
            DeclareLaunchArgument(
                'map',
                default_value=PathJoinSubstitution(
                    [navigation_share, 'maps', 'warehouse.yaml']
                ),
                description='Map YAML used when slam is false.',
            ),
            simulator,
            delayed_navigation,
        ]
    )
