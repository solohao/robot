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
from collections import deque
import math
from statistics import median
import sys
import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y ** 2 + z ** 2))


def angular_distance(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(second - first), math.cos(second - first)))


class LocalizationStabilityValidator(Node):

    def __init__(self, arguments: argparse.Namespace) -> None:
        super().__init__('localization_stability_validator')
        self.arguments = arguments
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map = None
        self.poses = deque()
        self.alignment_samples = deque()
        self.create_timer(0.2, self.sample_pose)

    def map_callback(self, message: OccupancyGrid) -> None:
        self.map = message

    def lookup_transform(self, source_frame: str, stamp: Time):
        try:
            return self.tf_buffer.lookup_transform(
                'map',
                source_frame,
                stamp,
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            return self.tf_buffer.lookup_transform(
                'map',
                source_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )

    def scan_callback(self, message: LaserScan) -> None:
        if self.map is None:
            return
        try:
            transform = self.lookup_transform(
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
            )
        except TransformException:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform_yaw = quaternion_yaw(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        aligned = 0
        considered = 0
        stride = max(1, len(message.ranges) // self.arguments.max_beams)

        for index in range(0, len(message.ranges), stride):
            distance = message.ranges[index]
            if (
                not math.isfinite(distance)
                or distance <= message.range_min + 0.05
                or distance >= message.range_max
            ):
                continue
            angle = message.angle_min + index * message.angle_increment
            laser_x = distance * math.cos(angle)
            laser_y = distance * math.sin(angle)
            map_x = (
                translation.x
                + math.cos(transform_yaw) * laser_x
                - math.sin(transform_yaw) * laser_y
            )
            map_y = (
                translation.y
                + math.sin(transform_yaw) * laser_x
                + math.cos(transform_yaw) * laser_y
            )
            considered += 1
            if self.endpoint_is_occupied(map_x, map_y):
                aligned += 1

        if considered >= 10:
            now = time.monotonic()
            self.alignment_samples.append((now, aligned / considered))
            self.prune(now)

    def endpoint_is_occupied(self, map_x: float, map_y: float) -> bool:
        info = self.map.info
        origin = info.origin
        origin_yaw = quaternion_yaw(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        delta_x = map_x - origin.position.x
        delta_y = map_y - origin.position.y
        local_x = math.cos(origin_yaw) * delta_x + math.sin(origin_yaw) * delta_y
        local_y = -math.sin(origin_yaw) * delta_x + math.cos(origin_yaw) * delta_y
        cell_x = int(local_x / info.resolution)
        cell_y = int(local_y / info.resolution)
        radius = max(1, int(self.arguments.endpoint_tolerance / info.resolution))

        for y_index in range(cell_y - radius, cell_y + radius + 1):
            if y_index < 0 or y_index >= info.height:
                continue
            for x_index in range(cell_x - radius, cell_x + radius + 1):
                if x_index < 0 or x_index >= info.width:
                    continue
                value = self.map.data[y_index * info.width + x_index]
                if value >= 65:
                    return True
        return False

    def sample_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        now = time.monotonic()
        self.poses.append((now, translation.x, translation.y, yaw))
        self.prune(now)

    def prune(self, now: float) -> None:
        cutoff = now - self.arguments.stable_seconds
        while self.poses and self.poses[0][0] < cutoff:
            self.poses.popleft()
        while self.alignment_samples and self.alignment_samples[0][0] < cutoff:
            self.alignment_samples.popleft()

    def metrics(self):
        if len(self.poses) < 2 or len(self.alignment_samples) < 5:
            return None
        window = self.poses[-1][0] - self.poses[0][0]
        if window < self.arguments.stable_seconds - 0.5:
            return None
        first = self.poses[0]
        position_drift = max(
            math.hypot(pose[1] - first[1], pose[2] - first[2])
            for pose in self.poses
        )
        yaw_drift = max(
            angular_distance(first[3], pose[3])
            for pose in self.poses
        )
        alignment = median(sample[1] for sample in self.alignment_samples)
        return position_drift, yaw_drift, alignment

    def is_stable(self) -> bool:
        metrics = self.metrics()
        if metrics is None:
            return False
        position_drift, yaw_drift, alignment = metrics
        return (
            position_drift <= self.arguments.max_position_drift
            and yaw_drift <= self.arguments.max_yaw_drift
            and alignment >= self.arguments.min_alignment
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--stable-seconds', type=float, default=15.0)
    parser.add_argument('--max-position-drift', type=float, default=0.10)
    parser.add_argument('--max-yaw-drift', type=float, default=0.10)
    parser.add_argument('--min-alignment', type=float, default=0.35)
    parser.add_argument('--endpoint-tolerance', type=float, default=0.20)
    parser.add_argument('--max-beams', type=int, default=120)
    parser.add_argument('--timeout', type=float, default=180.0)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main() -> None:
    arguments = parse_arguments()
    rclpy.init(args=sys.argv)
    validator = LocalizationStabilityValidator(arguments)
    start = time.monotonic()
    last_report = start
    passed = False
    final_metrics = None

    try:
        while rclpy.ok() and time.monotonic() - start < arguments.timeout:
            rclpy.spin_once(validator, timeout_sec=0.2)
            now = time.monotonic()
            if now - last_report >= 5.0:
                metrics = validator.metrics()
                if metrics is not None:
                    validator.get_logger().info(
                        'position_drift='
                        f'{metrics[0]:.3f} m, yaw_drift={metrics[1]:.3f} rad, '
                        f'scan_alignment={metrics[2]:.3f}'
                    )
                last_report = now
            if validator.is_stable():
                passed = True
                final_metrics = validator.metrics()
                break
    finally:
        validator.destroy_node()
        rclpy.shutdown()

    if not passed or final_metrics is None:
        raise SystemExit('Localization stability validation failed')
    print(
        '[PASS] Localization stable: '
        f'position_drift={final_metrics[0]:.3f} m, '
        f'yaw_drift={final_metrics[1]:.3f} rad, '
        f'scan_alignment={final_metrics[2]:.3f}'
    )


if __name__ == '__main__':
    main()
