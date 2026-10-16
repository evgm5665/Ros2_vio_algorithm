#!/usr/bin/env python3

import os
import sqlite3
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


BAG_DIR = os.path.expanduser("~/Ovio_ws/bags/LivePX4Conv")
OUTPUT_DIR = os.path.expanduser("~/Ovio_ws/eval/LivePX4Conv")

GT_TOPIC = "/ground_truth/odometry"
VIO_TOPIC = "/ov_msckf/poseimu"

GT_OUTPUT = os.path.join(OUTPUT_DIR, "ground_truth_imu.txt")
VIO_OUTPUT = os.path.join(OUTPUT_DIR, "openvins_estimate.txt")

# Recorded transform:
# base_link -> camera_imu_frame
GT_IMU_OFFSET = np.array([0.12, 0.03, 0.242], dtype=float)


def rotate_vector_by_quaternion(vector, quaternion):
    """Rotate a 3D vector using a geometry_msgs Quaternion."""

    q = np.array(
        [
            quaternion.w,
            quaternion.x,
            quaternion.y,
            quaternion.z,
        ],
        dtype=float,
    )

    norm = np.linalg.norm(q)

    if not np.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("Invalid quaternion in ground-truth message")

    q /= norm

    qw = q[0]
    q_xyz = q[1:4]
    vector = np.asarray(vector, dtype=float)

    return (
        vector
        + 2.0 * qw * np.cross(q_xyz, vector)
        + 2.0 * np.cross(q_xyz, np.cross(q_xyz, vector))
    )


def find_database():
    database_files = sorted(
        filename
        for filename in os.listdir(BAG_DIR)
        if filename.endswith(".db3") or filename.endswith(".sqlite3")
    )

    if not database_files:
        raise FileNotFoundError(
            f"No .db3 or .sqlite3 file found in {BAG_DIR}"
        )

    return os.path.join(BAG_DIR, database_files[0])


def get_topic_id(cursor, topic_name):
    cursor.execute(
        "SELECT id FROM topics WHERE name = ?",
        (topic_name,),
    )

    result = cursor.fetchone()

    if result is None:
        raise RuntimeError(f"Topic not found in bag: {topic_name}")

    return result[0]


def message_timestamp(message, bag_timestamp_ns):
    sec = message.header.stamp.sec
    nanosec = message.header.stamp.nanosec

    timestamp = sec + nanosec * 1.0e-9

    if timestamp <= 0.0:
        timestamp = bag_timestamp_ns * 1.0e-9

    return timestamp


def write_pose_line(output_file, timestamp, position, orientation):
    output_file.write(
        f"{timestamp:.9f} "
        f"{position[0]:.9f} "
        f"{position[1]:.9f} "
        f"{position[2]:.9f} "
        f"{orientation.x:.9f} "
        f"{orientation.y:.9f} "
        f"{orientation.z:.9f} "
        f"{orientation.w:.9f}\n"
    )


def extract_ground_truth(cursor):
    topic_id = get_topic_id(cursor, GT_TOPIC)

    cursor.execute(
        """
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = ?
        ORDER BY timestamp ASC
        """,
        (topic_id,),
    )

    count = 0

    with open(GT_OUTPUT, "w", encoding="utf-8") as output:
        output.write("# time x y z qx qy qz qw\n")

        for bag_timestamp_ns, serialized_data in cursor:
            message = deserialize_message(
                serialized_data,
                Odometry,
            )

            pose = message.pose.pose
            timestamp = message_timestamp(
                message,
                bag_timestamp_ns,
            )

            base_position = np.array(
                [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ],
                dtype=float,
            )

            # Convert the Gazebo base_link position into the position
            # of camera_imu_frame.
            rotated_offset = rotate_vector_by_quaternion(
                GT_IMU_OFFSET,
                pose.orientation,
            )

            imu_position = base_position + rotated_offset

            write_pose_line(
                output,
                timestamp,
                imu_position,
                pose.orientation,
            )

            count += 1

    print(f"Wrote {count} ground-truth poses to:")
    print(GT_OUTPUT)


def extract_openvins(cursor):
    topic_id = get_topic_id(cursor, VIO_TOPIC)

    cursor.execute(
        """
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = ?
        ORDER BY timestamp ASC
        """,
        (topic_id,),
    )

    count = 0

    with open(VIO_OUTPUT, "w", encoding="utf-8") as output:
        output.write("# time x y z qx qy qz qw\n")

        for bag_timestamp_ns, serialized_data in cursor:
            message = deserialize_message(
                serialized_data,
                PoseWithCovarianceStamped,
            )

            pose = message.pose.pose
            timestamp = message_timestamp(
                message,
                bag_timestamp_ns,
            )

            position = np.array(
                [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ],
                dtype=float,
            )

            write_pose_line(
                output,
                timestamp,
                position,
                pose.orientation,
            )

            count += 1

    print(f"Wrote {count} OpenVINS poses to:")
    print(VIO_OUTPUT)


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    database_path = find_database()
    print(f"Reading bag database: {database_path}")

    connection = sqlite3.connect(database_path)

    try:
        extract_ground_truth(connection.cursor())
        extract_openvins(connection.cursor())
    finally:
        connection.close()

    print("Extraction complete.")


if __name__ == "__main__":
    main()
