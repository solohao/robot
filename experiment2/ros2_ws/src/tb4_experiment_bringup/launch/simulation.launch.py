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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('tb4_experiment_bringup')
    navigation_share = get_package_share_directory('turtlebot4_navigation')
    simulator_share = get_package_share_directory('turtlebot4_ignition_bringup')

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')
    gz_args = LaunchConfiguration('gz_args')
    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')
    localization_params = LaunchConfiguration('localization_params')
    software_rendering = LaunchConfiguration('software_rendering')
    start_nav2 = LaunchConfiguration('start_nav2')
    start_rviz = LaunchConfiguration('start_rviz')
    validate_simulation = LaunchConfiguration('validate_simulation')

    simulator = GroupAction(
        scoped=True,
        actions=[
            SetEnvironmentVariable(
                name='LIBGL_ALWAYS_SOFTWARE',
                value='1',
                condition=IfCondition(software_rendering),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [simulator_share, 'launch', 'turtlebot4_ignition.launch.py']
                    )
                ),
                launch_arguments={
                    'world': world,
                    'model': model,
                    'namespace': namespace,
                    'rviz': 'false',
                    'gz_args': gz_args,
                    'localization': 'false',
                    'slam': 'false',
                    'nav2': 'false',
                }.items(),
            )
        ],
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
            'params': localization_params,
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
        condition=IfCondition(start_nav2),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([bringup_share, 'config', 'experiment.rviz']),
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    simulation_validator = Node(
        package='tb4_experiment_bringup',
        executable='validate_vmware_simulation',
        name='vmware_simulation_validator',
        output='screen',
        condition=IfCondition(validate_simulation),
    )

    delayed_navigation = TimerAction(
        period=5.0,
        actions=[localization, mapping, nav2, rviz],
    )

    delayed_validation = TimerAction(
        period=10.0,
        actions=[simulation_validator],
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
                'namespace',
                default_value='',
                description='Robot namespace retained for delayed controllers.',
            ),
            DeclareLaunchArgument(
                'gz_args',
                default_value='',
                description='Override arguments passed to Gazebo Sim.',
            ),
            DeclareLaunchArgument(
                'software_rendering',
                default_value='false',
                choices=['true', 'false'],
                description='Use Mesa software rendering for Gazebo only.',
            ),
            DeclareLaunchArgument(
                'start_nav2',
                default_value='true',
                choices=['true', 'false'],
                description='Start the Nav2 planning and control servers.',
            ),
            DeclareLaunchArgument(
                'start_rviz',
                default_value='true',
                choices=['true', 'false'],
                description='Start RViz with the experiment configuration.',
            ),
            DeclareLaunchArgument(
                'validate_simulation',
                default_value='false',
                choices=['true', 'false'],
                description='Check that simulation time and laser ranges are valid.',
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
            DeclareLaunchArgument(
                'localization_params',
                default_value=PathJoinSubstitution(
                    [bringup_share, 'config', 'localization_stable.yaml']
                ),
                description='AMCL parameters used when slam is false.',
            ),
            simulator,
            delayed_navigation,
            delayed_validation,
        ]
    )
