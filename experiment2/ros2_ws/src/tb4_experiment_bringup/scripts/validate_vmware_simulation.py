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


VALIDATION_TIMEOUT_SECONDS = 180.0
STABILITY_DURATION_SECONDS = 120.0
MAX_CLOCK_GAP_SECONDS = 15.0


class SimulationValidator(Node):
    def __init__(self) -> None:
        super().__init__('vmware_simulation_validator')
        self.clock_messages = 0
        self.clock_advances = 0
        self.first_clock_wall_time: float | None = None
        self.last_clock_advance_wall_time: float | None = None
        self.last_clock_stamp: int | None = None
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

    def receive_clock(self, clock: Clock) -> None:
        wall_time = time.monotonic()
        stamp = clock.clock.sec * 1_000_000_000 + clock.clock.nanosec
        self.clock_messages += 1
        if self.first_clock_wall_time is None:
            self.first_clock_wall_time = wall_time
        if self.last_clock_stamp is None or stamp > self.last_clock_stamp:
            if self.last_clock_stamp is not None:
                self.clock_advances += 1
            self.last_clock_stamp = stamp
            self.last_clock_advance_wall_time = wall_time

    def receive_scan(self, scan: LaserScan) -> None:
        self.scan = scan

    def clock_has_stalled(self, wall_time: float) -> bool:
        if self.last_clock_advance_wall_time is None:
            return False
        return (
            wall_time - self.last_clock_advance_wall_time
            > MAX_CLOCK_GAP_SECONDS
        )

    def clock_is_stable(self, wall_time: float) -> bool:
        if (
            self.first_clock_wall_time is None
            or self.last_clock_advance_wall_time is None
        ):
            return False
        return (
            self.clock_advances >= 3
            and wall_time - self.first_clock_wall_time
            >= STABILITY_DURATION_SECONDS
            and not self.clock_has_stalled(wall_time)
        )

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
            'VMware preset validated: /clock advanced for 120 wall seconds '
            'and /scan contains '
            f'environment returns from {min(finite_ranges):.3f} m to '
            f'{max(finite_ranges):.3f} m.'
        )


def main() -> None:
    rclpy.init()
    validator = SimulationValidator()
    deadline = time.monotonic() + VALIDATION_TIMEOUT_SECONDS

    while time.monotonic() < deadline:
        rclpy.spin_once(validator, timeout_sec=1.0)
        wall_time = time.monotonic()
        if validator.clock_has_stalled(wall_time):
            validator.get_logger().error(
                'VMware preset validation failed: /clock stopped advancing '
                f'for more than {MAX_CLOCK_GAP_SECONDS:.0f} wall seconds.'
            )
            validator.destroy_node()
            rclpy.shutdown()
            raise SystemExit(1)
        if validator.clock_is_stable(wall_time) and validator.scan_is_valid():
            validator.report_scan()
            validator.destroy_node()
            rclpy.shutdown()
            return

    if validator.clock_messages == 0 or validator.clock_advances == 0:
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
