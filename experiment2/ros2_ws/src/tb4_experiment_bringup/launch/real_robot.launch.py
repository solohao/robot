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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    bringup_share = get_package_share_directory('tb4_experiment_bringup')
    navigation_share = get_package_share_directory('turtlebot4_navigation')
    viz_share = get_package_share_directory('turtlebot4_viz')

    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')
    model = LaunchConfiguration('model')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [navigation_share, 'launch', 'localization.launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_file,
        }.items(),
        condition=UnlessCondition(slam),
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([navigation_share, 'launch', 'slam.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'sync': 'true',
        }.items(),
        condition=IfCondition(slam),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([navigation_share, 'launch', 'nav2.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'false',
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
            'use_sim_time': 'false',
            'model': model,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'slam',
                default_value='false',
                choices=['true', 'false'],
                description='Run SLAM instead of localization on an existing map.',
            ),
            DeclareLaunchArgument(
                'map',
                default_value='map.yaml',
                description='Absolute path to the saved map YAML.',
            ),
            DeclareLaunchArgument(
                'model',
                default_value='standard',
                choices=['standard', 'lite'],
            ),
            localization,
            mapping,
            nav2,
            rviz,
        ]
    )
