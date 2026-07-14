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
import math
import sys
import time

from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args


class SlamCoverageValidator(Node):

    def __init__(self) -> None:
        super().__init__('slam_coverage_validator')
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.initial_known_cells = None
        self.known_cells = None
        self.previous_position = None
        self.travelled_distance = 0.0

    def map_callback(self, message: OccupancyGrid) -> None:
        self.known_cells = sum(value >= 0 for value in message.data)
        if self.initial_known_cells is None:
            self.initial_known_cells = self.known_cells

    def odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        current = (position.x, position.y)
        if self.previous_position is not None:
            delta = math.hypot(
                current[0] - self.previous_position[0],
                current[1] - self.previous_position[1],
            )
            if delta < 1.0:
                self.travelled_distance += delta
        self.previous_position = current

    @property
    def known_cell_delta(self) -> int:
        if self.initial_known_cells is None or self.known_cells is None:
            return 0
        return self.known_cells - self.initial_known_cells


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--min-distance', type=float, default=5.0)
    parser.add_argument('--min-known-cell-delta', type=int, default=15000)
    parser.add_argument('--timeout', type=float, default=1200.0)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main() -> None:
    arguments = parse_arguments()
    rclpy.init(args=sys.argv)
    validator = SlamCoverageValidator()
    start = time.monotonic()
    last_report = start
    passed = False

    try:
        while rclpy.ok() and time.monotonic() - start < arguments.timeout:
            rclpy.spin_once(validator, timeout_sec=0.2)
            now = time.monotonic()
            if now - last_report >= 5.0:
                validator.get_logger().info(
                    f'distance={validator.travelled_distance:.2f} m, '
                    f'known_cell_delta={validator.known_cell_delta}'
                )
                last_report = now
            if (
                validator.travelled_distance >= arguments.min_distance
                and validator.known_cell_delta >= arguments.min_known_cell_delta
            ):
                passed = True
                break
    finally:
        validator.destroy_node()
        rclpy.shutdown()

    if not passed:
        raise SystemExit(
            'SLAM coverage failed: '
            f'distance={validator.travelled_distance:.2f} m, '
            f'known_cell_delta={validator.known_cell_delta}'
        )
    print(
        '[PASS] SLAM coverage: '
        f'distance={validator.travelled_distance:.2f} m, '
        f'known_cell_delta={validator.known_cell_delta}'
    )


if __name__ == '__main__':
    main()
