#!/usr/bin/env python3

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

import argparse
import json
import math
from pathlib import Path
import sys
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args


STATUS_LABELS = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def path_length(message: PathMessage) -> float:
    return sum(
        math.hypot(
            current.pose.position.x - previous.pose.position.x,
            current.pose.position.y - previous.pose.position.y,
        )
        for previous, current in zip(message.poses, message.poses[1:])
    )


def path_detour_metrics(
    message: PathMessage,
    goal_x: float,
    goal_y: float,
) -> tuple[float, float, float, float]:
    length = path_length(message)
    start = message.poses[0].pose.position
    direct_distance = math.hypot(goal_x - start.x, goal_y - start.y)
    if direct_distance <= 1.0e-6:
        return length, direct_distance, math.inf, 0.0

    line_x = goal_x - start.x
    line_y = goal_y - start.y
    lateral_deviation = max(
        abs(
            line_y * (pose.pose.position.x - start.x)
            - line_x * (pose.pose.position.y - start.y)
        ) / direct_distance
        for pose in message.poses
    )
    return (
        length,
        direct_distance,
        length / direct_distance,
        lateral_deviation,
    )


def grid_line(
    start_x: int,
    start_y: int,
    goal_x: int,
    goal_y: int,
):
    delta_x = abs(goal_x - start_x)
    delta_y = -abs(goal_y - start_y)
    step_x = 1 if start_x < goal_x else -1
    step_y = 1 if start_y < goal_y else -1
    error = delta_x + delta_y
    x = start_x
    y = start_y

    while True:
        yield x, y
        if x == goal_x and y == goal_y:
            break
        doubled_error = 2 * error
        if doubled_error >= delta_y:
            error += delta_y
            x += step_x
        if doubled_error <= delta_x:
            error += delta_x
            y += step_y


def direct_obstacle_segments(
    occupancy_grid: OccupancyGrid,
    start_x: float,
    start_y: float,
    goal_x: float,
    goal_y: float,
) -> int:
    origin = occupancy_grid.info.origin
    yaw = math.atan2(
        2.0 * origin.orientation.w * origin.orientation.z,
        1.0 - 2.0 * origin.orientation.z * origin.orientation.z,
    )
    cosine = math.cos(yaw)
    sine = math.sin(yaw)

    def world_to_grid(x: float, y: float) -> tuple[int, int]:
        delta_x = x - origin.position.x
        delta_y = y - origin.position.y
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        return (
            math.floor(local_x / occupancy_grid.info.resolution),
            math.floor(local_y / occupancy_grid.info.resolution),
        )

    start = world_to_grid(start_x, start_y)
    goal = world_to_grid(goal_x, goal_y)
    segments = 0
    inside_obstacle = False
    for x, y in grid_line(*start, *goal):
        in_bounds = (
            0 <= x < occupancy_grid.info.width
            and 0 <= y < occupancy_grid.info.height
        )
        occupied = (
            in_bounds
            and occupancy_grid.data[y * occupancy_grid.info.width + x] >= 65
        )
        if occupied and not inside_obstacle:
            segments += 1
        inside_obstacle = occupied
    return segments


class LongNavigationRunner(Node):

    def __init__(self, arguments: argparse.Namespace) -> None:
        super().__init__('long_navigation_runner')
        self.arguments = arguments
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )
        path_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PathMessage, '/plan', self.path_callback, path_qos)
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos,
        )
        self.static_map = None
        self.goal_sent = False
        self.path_pose_count = 0
        self.path_length = None
        self.direct_distance = None
        self.detour_ratio = None
        self.max_lateral_deviation = None
        self.direct_obstacle_segments = None

    def map_callback(self, message: OccupancyGrid) -> None:
        self.static_map = message

    def path_callback(self, message: PathMessage) -> None:
        if not self.goal_sent or not message.poses or self.static_map is None:
            return
        endpoint = message.poses[-1].pose.position
        endpoint_error = math.hypot(
            endpoint.x - self.arguments.x,
            endpoint.y - self.arguments.y,
        )
        if endpoint_error > 0.5:
            return
        (
            candidate_length,
            direct_distance,
            detour_ratio,
            lateral_deviation,
        ) = path_detour_metrics(message, self.arguments.x, self.arguments.y)
        start = message.poses[0].pose.position
        obstacle_segments = direct_obstacle_segments(
            self.static_map,
            start.x,
            start.y,
            self.arguments.x,
            self.arguments.y,
        )
        if self.path_length is None or candidate_length > self.path_length:
            self.path_length = candidate_length
            self.path_pose_count = len(message.poses)
            self.direct_distance = direct_distance
            self.detour_ratio = detour_ratio
            self.max_lateral_deviation = lateral_deviation
            self.direct_obstacle_segments = obstacle_segments
            self.get_logger().info(
                f'path received: poses={self.path_pose_count}, '
                f'length={self.path_length:.3f} m, '
                f'detour_ratio={self.detour_ratio:.3f}, '
                f'lateral_deviation={self.max_lateral_deviation:.3f} m, '
                f'direct_obstacles={self.direct_obstacle_segments}'
            )

    def goal_message(self) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = self.arguments.x
        goal.pose.pose.position.y = self.arguments.y
        goal.pose.pose.orientation.z = math.sin(self.arguments.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.arguments.yaw / 2.0)
        return goal


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=0.4)
    parser.add_argument('--y', type=float, default=-10.5)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--min-path-length', type=float, default=22.0)
    parser.add_argument('--min-detour-ratio', type=float, default=1.8)
    parser.add_argument('--min-lateral-deviation', type=float, default=6.0)
    parser.add_argument('--min-direct-obstacles', type=int, default=2)
    parser.add_argument('--timeout', type=float, default=1200.0)
    parser.add_argument(
        '--evidence',
        type=Path,
        default=Path('long-navigation-result.json'),
    )
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def write_evidence(
    arguments: argparse.Namespace,
    runner: LongNavigationRunner,
    wall_duration: float,
    status: int,
) -> None:
    evidence = {
        'goal': {
            'x': arguments.x,
            'y': arguments.y,
            'yaw': arguments.yaw,
        },
        'minimum_path_length': arguments.min_path_length,
        'path_pose_count': runner.path_pose_count,
        'path_length': runner.path_length,
        'direct_distance': runner.direct_distance,
        'detour_ratio': runner.detour_ratio,
        'minimum_detour_ratio': arguments.min_detour_ratio,
        'max_lateral_deviation': runner.max_lateral_deviation,
        'minimum_lateral_deviation': arguments.min_lateral_deviation,
        'direct_obstacle_segments': runner.direct_obstacle_segments,
        'minimum_direct_obstacles': arguments.min_direct_obstacles,
        'wall_duration_seconds': wall_duration,
        'terminal_status': status,
        'terminal_status_label': STATUS_LABELS.get(status, str(status)),
    }
    arguments.evidence.parent.mkdir(parents=True, exist_ok=True)
    arguments.evidence.write_text(
        json.dumps(evidence, indent=2, ensure_ascii=False) + '\n'
    )


def main() -> None:
    arguments = parse_arguments()
    rclpy.init(args=sys.argv)
    runner = LongNavigationRunner(arguments)
    status = GoalStatus.STATUS_UNKNOWN
    wall_start = time.monotonic()

    try:
        runner.get_logger().info('waiting for /navigate_to_pose')
        if not runner.action_client.wait_for_server(timeout_sec=120.0):
            raise SystemExit('NavigateToPose action server is unavailable')

        map_deadline = time.monotonic() + 30.0
        while runner.static_map is None and time.monotonic() < map_deadline:
            rclpy.spin_once(runner, timeout_sec=0.2)
        if runner.static_map is None:
            raise SystemExit('Static occupancy map is unavailable')

        send_future = runner.action_client.send_goal_async(runner.goal_message())
        rclpy.spin_until_future_complete(runner, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise SystemExit('Navigation goal was rejected')

        runner.goal_sent = True
        wall_start = time.monotonic()
        runner.get_logger().info(
            f'goal accepted: x={arguments.x:.3f}, y={arguments.y:.3f}'
        )
        result_future = goal_handle.get_result_async()
        cancel_requested = False

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(runner, timeout_sec=0.2)
            path_failed = (
                runner.path_length is not None
                and (
                    runner.path_length < arguments.min_path_length
                    or runner.detour_ratio < arguments.min_detour_ratio
                    or runner.max_lateral_deviation
                    < arguments.min_lateral_deviation
                    or runner.direct_obstacle_segments
                    < arguments.min_direct_obstacles
                )
            )
            if path_failed and not cancel_requested:
                runner.get_logger().error(
                    'path does not demonstrate the required obstacle detour: '
                    f'length={runner.path_length:.3f} m, '
                    f'detour_ratio={runner.detour_ratio:.3f}, '
                    'lateral_deviation='
                    f'{runner.max_lateral_deviation:.3f} m, '
                    f'direct_obstacles={runner.direct_obstacle_segments}'
                )
                goal_handle.cancel_goal_async()
                cancel_requested = True
            if time.monotonic() - wall_start > arguments.timeout:
                runner.get_logger().error('navigation wall-clock timeout')
                goal_handle.cancel_goal_async()
                cancel_requested = True
                break

        if result_future.done():
            status = result_future.result().status
        wall_duration = time.monotonic() - wall_start
        write_evidence(arguments, runner, wall_duration, status)
    finally:
        runner.destroy_node()
        rclpy.shutdown()

    path_passed = (
        runner.path_length is not None
        and runner.path_length >= arguments.min_path_length
        and runner.detour_ratio >= arguments.min_detour_ratio
        and runner.max_lateral_deviation >= arguments.min_lateral_deviation
        and runner.direct_obstacle_segments >= arguments.min_direct_obstacles
    )
    if status != GoalStatus.STATUS_SUCCEEDED or not path_passed:
        raise SystemExit(
            'Long navigation failed: '
            f'status={STATUS_LABELS.get(status, status)}, '
            f'path_length={runner.path_length}'
        )
    print(
        '[PASS] Long navigation: '
        f'path_length={runner.path_length:.3f} m, '
        f'detour_ratio={runner.detour_ratio:.3f}, '
        f'lateral_deviation={runner.max_lateral_deviation:.3f} m, '
        f'direct_obstacles={runner.direct_obstacle_segments}, '
        f'wall_duration={wall_duration:.1f} s, status=SUCCEEDED'
    )


if __name__ == '__main__':
    main()
