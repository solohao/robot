#!/usr/bin/env python3
#
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

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


class SimulationValidator(Node):
    def __init__(self) -> None:
        super().__init__('vmware_simulation_validator')
        self.clock_messages = 0
        self.scan: LaserScan | None = None
        self.create_subscription(
            Clock,
            '/clock',
            self.receive_clock,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self.receive_scan,
            qos_profile_sensor_data,
        )

    def receive_clock(self, _: Clock) -> None:
        self.clock_messages += 1

    def receive_scan(self, scan: LaserScan) -> None:
        self.scan = scan

    def scan_is_valid(self) -> bool:
        if self.scan is None:
            return False
        finite_ranges = [
            distance
            for distance in self.scan.ranges
            if math.isfinite(distance)
        ]
        if not finite_ranges:
            return False
        threshold = self.scan.range_min + 0.05
        has_environment_return = any(
            distance > threshold for distance in finite_ranges
        )
        has_range_variation = (
            max(finite_ranges) - min(finite_ranges) > 0.05
        )
        has_infinite_return = any(
            math.isinf(distance) and distance > 0.0
            for distance in self.scan.ranges
        )
        return (
            has_environment_return
            and (has_range_variation or has_infinite_return)
        )

    def report_scan(self) -> None:
        if self.scan is None:
            return
        finite_ranges = [
            distance
            for distance in self.scan.ranges
            if math.isfinite(distance)
        ]
        if not finite_ranges:
            return
        self.get_logger().info(
            'VMware preset validated: /clock is active and /scan contains '
            f'environment returns from {min(finite_ranges):.3f} m to '
            f'{max(finite_ranges):.3f} m.'
        )


def main() -> None:
    rclpy.init()
    validator = SimulationValidator()
    deadline = time.monotonic() + 180.0

    while time.monotonic() < deadline:
        rclpy.spin_once(validator, timeout_sec=1.0)
        if validator.clock_messages >= 3 and validator.scan_is_valid():
            validator.report_scan()
            validator.destroy_node()
            rclpy.shutdown()
            return

    if validator.clock_messages == 0:
        validator.get_logger().error(
            'VMware preset validation failed: /clock did not advance.'
        )
    elif validator.scan is None:
        validator.get_logger().error(
            'VMware preset validation failed: no /scan message was received.'
        )
    else:
        validator.get_logger().error(
            'VMware preset validation failed: laser ranges remained at the '
            'minimum distance. Keep software_rendering enabled.'
        )

    validator.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1)


if __name__ == '__main__':
    main()
