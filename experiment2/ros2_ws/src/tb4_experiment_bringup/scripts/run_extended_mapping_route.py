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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odometry(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )


class ExtendedMappingRoute(Node):

    def __init__(self, waypoints: list[tuple[float, float]]) -> None:
        super().__init__('extended_mapping_route')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control)
        self.relative_waypoints = waypoints
        self.waypoints = []
        self.position = None
        self.yaw = None
        self.previous_yaw = None
        self.scan_rotation = 0.0
        self.phase = 'initial_scan'
        self.waypoint_index = 0
        self.front_clearance = math.inf
        self.finished = False
        self.failed = False

    def odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        if self.position is None:
            self.waypoints = [
                (position.x + x, position.y + y)
                for x, y in self.relative_waypoints
            ]
        self.position = (position.x, position.y)
        current_yaw = yaw_from_odometry(message)
        if self.previous_yaw is not None and self.phase.endswith('scan'):
            self.scan_rotation += abs(normalize_angle(current_yaw - self.previous_yaw))
        self.previous_yaw = current_yaw
        self.yaw = current_yaw

    def scan_callback(self, message: LaserScan) -> None:
        front = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if (
                abs(normalize_angle(angle)) <= math.radians(25.0)
                and math.isfinite(distance)
                and distance > message.range_min
            ):
                front.append(distance)
        self.front_clearance = min(front, default=math.inf)

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())

    def control_scan(self) -> None:
        if self.scan_rotation >= 2.0 * math.pi:
            self.publish_stop()
            self.scan_rotation = 0.0
            if self.phase == 'initial_scan':
                self.phase = 'drive'
            else:
                self.finished = True
            return
        command = Twist()
        command.angular.z = 0.45
        self.publisher.publish(command)

    def control_drive(self) -> None:
        if self.waypoint_index >= len(self.waypoints):
            self.phase = 'final_scan'
            return
        target_x, target_y = self.waypoints[self.waypoint_index]
        delta_x = target_x - self.position[0]
        delta_y = target_y - self.position[1]
        distance = math.hypot(delta_x, delta_y)
        if distance <= 0.18:
            self.publish_stop()
            self.get_logger().info(
                f'reached waypoint {self.waypoint_index + 1}/{len(self.waypoints)}'
            )
            self.waypoint_index += 1
            return

        heading = math.atan2(delta_y, delta_x)
        heading_error = normalize_angle(heading - self.yaw)
        command = Twist()
        if abs(heading_error) > 0.18:
            command.angular.z = max(-0.45, min(0.45, 0.9 * heading_error))
        else:
            if self.front_clearance < 0.55:
                self.publish_stop()
                self.failed = True
                self.get_logger().error(
                    f'obstacle detected at {self.front_clearance:.2f} m'
                )
                return
            command.linear.x = max(0.06, min(0.18, 0.15 * distance))
            command.angular.z = max(-0.25, min(0.25, 0.8 * heading_error))
        self.publisher.publish(command)

    def control(self) -> None:
        if self.finished or self.failed or self.position is None or self.yaw is None:
            self.publish_stop()
            return
        if self.phase.endswith('scan'):
            self.control_scan()
        else:
            self.control_drive()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=1500.0)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main() -> None:
    arguments = parse_arguments()
    rclpy.init(args=sys.argv)
    route = ExtendedMappingRoute([(-3.0, 0.0), (-3.0, -10.0), (-7.0, -10.0)])
    start = time.monotonic()
    try:
        while (
            rclpy.ok()
            and not route.finished
            and not route.failed
            and time.monotonic() - start < arguments.timeout
        ):
            rclpy.spin_once(route, timeout_sec=0.2)
    finally:
        route.publish_stop()
        route.destroy_node()
        rclpy.shutdown()

    if not route.finished:
        raise SystemExit('Extended mapping route did not finish')
    print('[PASS] Extended mapping route completed')


if __name__ == '__main__':
    main()
