#!/usr/bin/env python3

# Imports

import math
import rclpy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

#subscribes to OpenVINS imu pose then publishes the estimated baselink pose as /vio.pose_base
#changes the point on the drone being reported

# Helper function for rotating a vector with a quaternion

def rotate_vector(qx, qy, qz, qw, vx, vy, vz):
    sqx = qx ** 2
    sqy = qy ** 2
    sqz = qz ** 2
    sqw = qw ** 2

    quaternion_norm = math.sqrt(sqx + sqy + sqz + sqw)

    if quaternion_norm < 1.0e-12:
        raise ValueError("Received zero-length quaternion")

    # Normalize the quaternion
    qx /= quaternion_norm
    qy /= quaternion_norm
    qz /= quaternion_norm
    qw /= quaternion_norm

    # Rotate the vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    # Return rotated x, y, and z
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


class SPNode(Node):

    def __init__(self):
        super().__init__("subscriber_node")

        self.subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            "/ov_msckf/poseimu",
            self.pose_callback,
            10,
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            "/vio/pose_base",
            10,
        )

        self.baseTimu = (
            float(self.declare_parameter("BtI_x", 0.12).value),
            float(self.declare_parameter("BtI_y", 0.03).value),
            float(self.declare_parameter("BtI_z", 0.242).value),
        )

        self.get_logger().info("Subscriber node has been started.")

    def pose_callback(self, msg):
        pose = msg.pose.pose
        q = pose.orientation

        try:
            global_offset = rotate_vector(
                q.x,
                q.y,
                q.z,
                q.w,
                *self.baseTimu,
            )
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        quaternion_norm = math.sqrt(
            q.x ** 2 +
            q.y ** 2 +
            q.z ** 2 +
            q.w ** 2
        )

        out_pose_stamp = PoseStamped()
        out_pose_stamp.header = msg.header

        out_pose_stamp.pose.position.x = (
            pose.position.x - global_offset[0]
        )
        out_pose_stamp.pose.position.y = (
            pose.position.y - global_offset[1]
        )
        out_pose_stamp.pose.position.z = (
            pose.position.z - global_offset[2]
        )

        out_pose_stamp.pose.orientation.x = q.x / quaternion_norm
        out_pose_stamp.pose.orientation.y = q.y / quaternion_norm
        out_pose_stamp.pose.orientation.z = q.z / quaternion_norm
        out_pose_stamp.pose.orientation.w = q.w / quaternion_norm

        self.publisher.publish(out_pose_stamp)


def main(args=None):
    rclpy.init(args=args)

    node = SPNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

    #!/usr/bin/env python3

# Imports

import math

import numpy as np
import rclpy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node


# ----------------------------------------------------------
# Pure math helper functions
# ----------------------------------------------------------

def rotate_vector(qw, qx, qy, qz, vx, vy, vz):
    quaternion_norm = math.sqrt(
        qw ** 2 +
        qx ** 2 +
        qy ** 2 +
        qz ** 2
    )

    if quaternion_norm < 1.0e-12:
        raise ValueError("Received zero-length quaternion")

    qw /= quaternion_norm
    qx /= quaternion_norm
    qy /= quaternion_norm
    qz /= quaternion_norm

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def rotate_covariance(C, R):
    C = np.asarray(C, dtype=float)
    R = np.asarray(R, dtype=float)

    if C.size != 9:
        raise ValueError("Covariance must contain exactly 9 values")

    if R.size != 9:
        raise ValueError("Rotation matrix must contain exactly 9 values")

    C = C.reshape((3, 3))
    R = R.reshape((3, 3))

    if not np.all(np.isfinite(C)):
        raise ValueError("Covariance contains non-finite values")

    if not np.all(np.isfinite(R)):
        raise ValueError("Rotation matrix contains non-finite values")

    if not np.allclose(C, C.T):
        raise ValueError("Input covariance matrix is not symmetric")

    C_rotated = R @ C @ R.T

    # Remove small floating-point asymmetry.
    C_rotated = 0.5 * (C_rotated + C_rotated.T)

    if np.any(np.diag(C_rotated) < -1.0e-12):
        raise ValueError(
            "Rotated covariance contains a negative variance"
        )

    return C_rotated


def normalize_quaternion(qw, qx, qy, qz):
    quaternion_norm = math.sqrt(
        qw ** 2 +
        qx ** 2 +
        qy ** 2 +
        qz ** 2
    )

    if quaternion_norm < 1.0e-12:
        raise ValueError("Received zero-length quaternion")

    return (
        qw / quaternion_norm,
        qx / quaternion_norm,
        qy / quaternion_norm,
        qz / quaternion_norm,
    )


def multiply_quaternions(q, r):
    wq, xq, yq, zq = q
    wr, xr, yr, zr = r

    w = wq * wr - xq * xr - yq * yr - zq * zr
    x = wq * xr + xq * wr + yq * zr - zq * yr
    y = wq * yr - xq * zr + yq * wr + zq * xr
    z = wq * zr + xq * yr - yq * xr + zq * wr

    return (w, x, y, z)


def quaternion_to_rotation_matrix(qw, qx, qy, qz):
    qw, qx, qy, qz = normalize_quaternion(
        qw,
        qx,
        qy,
        qz,
    )

    qx2 = qx * qx
    qy2 = qy * qy
    qz2 = qz * qz

    R = np.array([
        [
            1.0 - 2.0 * (qy2 + qz2),
            2.0 * (qx * qy - qw * qz),
            2.0 * (qx * qz + qw * qy),
        ],
        [
            2.0 * (qx * qy + qw * qz),
            1.0 - 2.0 * (qx2 + qz2),
            2.0 * (qy * qz - qw * qx),
        ],
        [
            2.0 * (qx * qz - qw * qy),
            2.0 * (qy * qz + qw * qx),
            1.0 - 2.0 * (qx2 + qy2),
        ],
    ])

    if not np.allclose(R @ R.T, np.identity(3)):
        raise ValueError("Rotation matrix is not orthogonal")

    if not np.isclose(np.linalg.det(R), 1.0):
        raise ValueError("Rotation matrix determinant is not 1")

    return R


# ----------------------------------------------------------
# ROS 2 node
# ----------------------------------------------------------

class SPNode(Node):

    def __init__(self):
        super().__init__("vio_px4_converter")

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------

        self.declare_parameter(
            "openvins_topic",
            "/ov_msckf/odomimu",
        )

        self.declare_parameter(
            "gazebo_topic",
            "/ground_truth/odometry",
        )

        self.declare_parameter(
            "px4_topic",
            "/fmu/in/vehicle_visual_odometry",
        )

        self.declare_parameter(
            "source",
            "openvins",
        )

        self.declare_parameter(
            "frame_mode",
            "local_frd",
        )

        self.declare_parameter(
            "publish_rate_hz",
            50.0,
        )

        self.declare_parameter(
            "stale_timeout",
            0.25,
        )

        # --------------------------------------------------
        # Read parameter values
        # --------------------------------------------------

        self.openvins_topic = str(
            self.get_parameter("openvins_topic").value
        )

        self.gazebo_topic = str(
            self.get_parameter("gazebo_topic").value
        )

        self.px4_topic = str(
            self.get_parameter("px4_topic").value
        )

        self.source = str(
            self.get_parameter("source").value
        )

        self.frame_mode = str(
            self.get_parameter("frame_mode").value
        )

        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )

        self.stale_timeout = float(
            self.get_parameter("stale_timeout").value
        )

        # --------------------------------------------------
        # Parameter validation
        # --------------------------------------------------

        if self.source not in ("openvins", "gazebo_test"):
            raise ValueError(
                "source must be 'openvins' or 'gazebo_test'"
            )

        if self.frame_mode not in ("local_frd", "ned"):
            raise ValueError(
                "frame_mode must be 'local_frd' or 'ned'"
            )

        if self.publish_rate_hz <= 0.0:
            raise ValueError(
                "publish_rate_hz must be greater than zero"
            )

        if self.stale_timeout <= 0.0:
            raise ValueError(
                "stale_timeout must be greater than zero"
            )

        # --------------------------------------------------
        # Fixed coordinate rotations
        # --------------------------------------------------

        # ROS body FLU to PX4 body FRD.
        self.R_frd_flu = np.array([
            [1.0,  0.0,  0.0],
            [0.0, -1.0,  0.0],
            [0.0,  0.0, -1.0],
        ])

        # ROS/Gazebo world ENU to PX4 world NED.
        self.R_ned_enu = np.array([
            [0.0, 1.0,  0.0],
            [1.0, 0.0,  0.0],
            [0.0, 0.0, -1.0],
        ])

        # --------------------------------------------------
        # OpenVINS global-frame alignment
        # --------------------------------------------------

        self.R_output_openvins = None
        self.t_output_openvins = None
        self.alignment_valid = False

        # --------------------------------------------------
        # Stored input messages
        # --------------------------------------------------

        self.latest_openvins_msg = None
        self.latest_gazebo_msg = None

        self.openvins_message_pending = False
        self.gazebo_message_pending = False

        # --------------------------------------------------
        # Output continuity and reset state
        # --------------------------------------------------

        self.previous_sample_timestamp_us = None
        self.previous_output_quaternion = None
        self.reset_counter = 0

        # --------------------------------------------------
        # Subscriptions
        # --------------------------------------------------

        self.openvins_subscription = self.create_subscription(
            Odometry,
            self.openvins_topic,
            self.openvins_callback,
            10,
        )

        self.gazebo_subscription = self.create_subscription(
            Odometry,
            self.gazebo_topic,
            self.gazebo_callback,
            10,
        )

        # --------------------------------------------------
        # PX4 publisher
        # --------------------------------------------------

        self.px4_publisher = self.create_publisher(
            VehicleOdometry,
            self.px4_topic,
            10,
        )

        # --------------------------------------------------
        # Rate-limited timer
        # --------------------------------------------------

        timer_period_sec = 1.0 / self.publish_rate_hz

        self.publish_timer = self.create_timer(
            timer_period_sec,
            self.publish_latest_measurement,
        )

        self.get_logger().info(
            f"VIO-to-PX4 converter started: "
            f"source={self.source}, "
            f"frame_mode={self.frame_mode}, "
            f"rate={self.publish_rate_hz:.1f} Hz"
        )

    # ------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------

    def openvins_callback(self, msg):
        if self.source != "openvins":
            return

        self.latest_openvins_msg = msg
        self.openvins_message_pending = True

    def gazebo_callback(self, msg):
        if self.source != "gazebo_test":
            return

        self.latest_gazebo_msg = msg
        self.gazebo_message_pending = True

    # ------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------

    def publish_latest_measurement(self):
        # Select and consume the newest message.
        if self.source == "openvins":
            if not self.openvins_message_pending:
                return

            msg = self.latest_openvins_msg
            self.openvins_message_pending = False

        elif self.source == "gazebo_test":
            if not self.gazebo_message_pending:
                return

            msg = self.latest_gazebo_msg
            self.gazebo_message_pending = False

        else:
            return

        if msg is None:
            return

        # --------------------------------------------------
        # Extract covariance matrices
        # --------------------------------------------------

        CM_pose = np.asarray(
            msg.pose.covariance,
            dtype=float,
        ).reshape((6, 6))

        CM_twist = np.asarray(
            msg.twist.covariance,
            dtype=float,
        ).reshape((6, 6))

        c_position = CM_pose[:3, :3]
        c_orientation = CM_pose[3:6, 3:6]
        c_velocity = CM_twist[:3, :3]
        c_angular = CM_twist[3:6, 3:6]

        # c_angular is extracted for completeness, but PX4
        # VehicleOdometry has no angular-velocity variance field.

        # --------------------------------------------------
        # Convert and validate sample timestamp
        # --------------------------------------------------

        sample_timestamp_us = (
            msg.header.stamp.sec * 1_000_000
            + msg.header.stamp.nanosec // 1_000
        )

        if sample_timestamp_us <= 0:
            self.get_logger().warning(
                "Rejected measurement with zero or negative timestamp"
            )
            return

        if self.previous_sample_timestamp_us is not None:
            if sample_timestamp_us == self.previous_sample_timestamp_us:
                self.get_logger().warning(
                    "Rejected repeated sample timestamp"
                )
                return

            if sample_timestamp_us < self.previous_sample_timestamp_us:
                self.get_logger().error(
                    "Rejected backward sample timestamp"
                )
                return

        now_us = self.get_clock().now().nanoseconds // 1_000
        message_age_us = now_us - sample_timestamp_us
        stale_timeout_us = int(
            self.stale_timeout * 1_000_000
        )

        if message_age_us < 0:
            self.get_logger().warning(
                "Measurement timestamp is ahead of the node clock"
            )
            return

        if message_age_us > stale_timeout_us:
            self.get_logger().warning(
                "Rejected stale measurement"
            )
            return

        # The current version intentionally stops here.
        #
        # The next implementation stage will:
        # - extract pose and twist values
        # - establish/apply frame alignment
        # - transform position and orientation
        # - transform velocity
        # - rotate covariance
        # - construct VehicleOdometry
        # - publish to PX4

        self.previous_sample_timestamp_us = sample_timestamp_us

        # Prevent unused-variable warnings while this stage
        # remains extraction/validation only.
        _ = (
            c_position,
            c_orientation,
            c_velocity,
            c_angular,
        )


# ----------------------------------------------------------
# Program entry point
# ----------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    node = SPNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()