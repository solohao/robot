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
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
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
        self.create_subscription(PathMessage, '/plan', self.path_callback, path_qos)
        self.goal_sent = False
        self.path_pose_count = 0
        self.path_length = None
        self.direct_distance = None
        self.detour_ratio = None
        self.max_lateral_deviation = None

    def path_callback(self, message: PathMessage) -> None:
        if not self.goal_sent or not message.poses:
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
        if self.path_length is None or candidate_length > self.path_length:
            self.path_length = candidate_length
            self.path_pose_count = len(message.poses)
            self.direct_distance = direct_distance
            self.detour_ratio = detour_ratio
            self.max_lateral_deviation = lateral_deviation
            self.get_logger().info(
                f'path received: poses={self.path_pose_count}, '
                f'length={self.path_length:.3f} m, '
                f'detour_ratio={self.detour_ratio:.3f}, '
                f'lateral_deviation={self.max_lateral_deviation:.3f} m'
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
    parser.add_argument('--x', type=float, default=8.5)
    parser.add_argument('--y', type=float, default=-12.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--min-path-length', type=float, default=18.0)
    parser.add_argument('--min-detour-ratio', type=float, default=2.5)
    parser.add_argument('--min-lateral-deviation', type=float, default=5.0)
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
                )
            )
            if path_failed and not cancel_requested:
                runner.get_logger().error(
                    'path does not demonstrate the required obstacle detour: '
                    f'length={runner.path_length:.3f} m, '
                    f'detour_ratio={runner.detour_ratio:.3f}, '
                    'lateral_deviation='
                    f'{runner.max_lateral_deviation:.3f} m'
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
        f'wall_duration={wall_duration:.1f} s, status=SUCCEEDED'
    )


if __name__ == '__main__':
    main()
