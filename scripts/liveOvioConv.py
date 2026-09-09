#!/usr/bin/env python3

# Imports

import math
import rclpy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


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