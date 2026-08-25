#!/usr/bin/env python3

import argparse
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


DEFAULT_LEFT = (
    "/world/default/model/x500_stereo_vio_0/link/vio_sensor_link/"
    "sensor/left_camera/image"
)

DEFAULT_RIGHT = (
    "/world/default/model/x500_stereo_vio_0/link/vio_sensor_link/"
    "sensor/right_camera/image"
)

DEFAULT_IMU = "/vio/imu"


class TimingChecker(Node):
    def __init__(self, duration, left_topic, right_topic, imu_topic):
        super().__init__("stereo_timing_checker")

        self.duration = duration
        self.start_wall = None
        self.reported = False

        self.left_stamps = []
        self.right_stamps = []
        self.imu_stamps = []

        self.left_arrivals = []
        self.right_arrivals = []
        self.imu_arrivals = []

        self.create_subscription(
            Image,
            left_topic,
            lambda msg: self.record(
                msg, self.left_stamps, self.left_arrivals
            ),
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Image,
            right_topic,
            lambda msg: self.record(
                msg, self.right_stamps, self.right_arrivals
            ),
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Imu,
            imu_topic,
            lambda msg: self.record(
                msg, self.imu_stamps, self.imu_arrivals
            ),
            qos_profile_sensor_data,
        )

        self.create_timer(0.2, self.check_duration)

        print(f"Collecting data for {duration:.1f} wall-clock seconds...")

    def record(self, msg, stamps, arrivals):
        now = time.monotonic()

        if self.start_wall is None:
            self.start_wall = now

        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )

        stamps.append(stamp_ns)
        arrivals.append(now)

    def check_duration(self):
        if self.start_wall is None:
            return

        if time.monotonic() - self.start_wall >= self.duration:
            self.report()
            rclpy.shutdown()

    @staticmethod
    def stream_report(name, stamps, arrivals):
        print(f"\n{name}")
        print(f"  Messages: {len(stamps)}")

        if len(stamps) < 2:
            print("  Not enough messages")
            return

        stamp_deltas = [
            (b - a) / 1e9 for a, b in zip(stamps, stamps[1:])
        ]

        stamp_span = (stamps[-1] - stamps[0]) / 1e9
        wall_span = arrivals[-1] - arrivals[0]

        stamp_rate = (
            (len(stamps) - 1) / stamp_span if stamp_span > 0 else 0.0
        )

        wall_rate = (
            (len(arrivals) - 1) / wall_span if wall_span > 0 else 0.0
        )

        nonmonotonic = sum(delta <= 0 for delta in stamp_deltas)

        print(f"  Header-stamp rate: {stamp_rate:.3f} Hz")
        print(f"  Wall-arrival rate: {wall_rate:.3f} Hz")
        print(
            f"  Average header interval: "
            f"{statistics.mean(stamp_deltas) * 1000:.3f} ms"
        )
        print(
            f"  Maximum header interval: "
            f"{max(stamp_deltas) * 1000:.3f} ms"
        )
        print(f"  Duplicate/nonmonotonic stamps: {nonmonotonic}")

    def stereo_report(self):
        left = sorted(self.left_stamps)
        right = sorted(self.right_stamps)

        exact_pairs = len(set(left).intersection(right))

        # Less than half of one 30 Hz frame interval.
        tolerance_ns = 20_000_000

        i = 0
        j = 0
        differences = []

        while i < len(left) and j < len(right):
            difference = left[i] - right[j]

            if abs(difference) <= tolerance_ns:
                differences.append(difference)
                i += 1
                j += 1
            elif difference < 0:
                i += 1
            else:
                j += 1

        print("\nSTEREO PAIRING")
        print(f"  Exact timestamp pairs: {exact_pairs}")
        print(f"  Paired within 20 ms: {len(differences)}")
        print(f"  Unpaired left frames: {len(left) - len(differences)}")
        print(f"  Unpaired right frames: {len(right) - len(differences)}")

        if differences:
            differences_ms = [value / 1e6 for value in differences]

            print(
                f"  Mean signed difference: "
                f"{statistics.mean(differences_ms):.6f} ms"
            )
            print(
                f"  Maximum absolute difference: "
                f"{max(abs(value) for value in differences_ms):.6f} ms"
            )

    def report(self):
        if self.reported:
            return

        self.reported = True

        print("\n========== TIMING REPORT ==========")

        self.stream_report(
            "LEFT CAMERA", self.left_stamps, self.left_arrivals
        )
        self.stream_report(
            "RIGHT CAMERA", self.right_stamps, self.right_arrivals
        )
        self.stream_report(
            "MODULE IMU", self.imu_stamps, self.imu_arrivals
        )

        self.stereo_report()

        print("\n===================================")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--left-topic", default=DEFAULT_LEFT)
    parser.add_argument("--right-topic", default=DEFAULT_RIGHT)
    parser.add_argument("--imu-topic", default=DEFAULT_IMU)

    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    node = TimingChecker(
        args.duration,
        args.left_topic,
        args.right_topic,
        args.imu_topic,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()