#imports

import math
import rclpy
import numpy as np
from nav_msgs.msg import Odometry  
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node

def rotate_vector(qw, qx, qy, qz, vx, vy, vz):
    #square root the quats
    sqw = qw ** 2
    sqx = qx ** 2
    sqy = qy ** 2
    sqz = qz ** 2

    quaternion_norm = math.sqrt(sqw + sqx + sqy + sqz)

    if quaternion_norm < 1.0e-12:
        raise ValueError("Received zero-length quaternion")

    #nomarlize the quaternion
    qw/= quaternion_norm
    qx/= quaternion_norm
    qy/= quaternion_norm
    qz/= quaternion_norm

    #rotate the vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    #return rotated x, y, and z
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )

def rotate_covariance(C, R):
    # # using msg.pose.covariance and msg.twist.covariance
    # # both output [x, y, z rotation x, rotation, y, rotation z] in a 6x6 matrix  with float64[36] cov
    
    # C_pose = np.array(C).msgs.pose.covariance.reshape((6, 6))
    # C_twist = np.array(R).msgs.twist.covariance.reshape((6, 6))

    # #assigning the pose covariance matrix to the appropriate variables
    # c_position = C_pose[:3, :3]
    # c_orientation = C_pose[3:6, 3:6]
    # c_position_orientation = C_pose[:3, 3:6]
    # c_orientation_position = C_pose[3:6, :3]

    # #assigning the twist covariance matrix to the appropriate variables
    # c_velocity = C_twist[:3, :3]
    # c_angular = C_twist[3:6, 3:6]


    #input 
    C = np.array(C).reshape((3, 3))
    R = np.array(R).reshape((3, 3))

    C_rotated = R @ C @ R.T
    C_rotated = 1/2 * (C_rotated + C_rotated.T)  # Ensure symmetry

    if not np.allclose(C_rotated, C_rotated.T):
        raise ValueError("Covariance matrix is not symmetric")
    return (C_rotated) 

    


def normalize_quaternion(qw, qx, qy, qz):
    quaternion_norm = math.sqrt(qw ** 2 + qx ** 2 + qy ** 2 + qz ** 2)

    if quaternion_norm < 1.0e-12:
        raise ValueError("Received zero-length quaternion")

    qw /= quaternion_norm
    qx /= quaternion_norm
    qy /= quaternion_norm
    qz /= quaternion_norm   
    return( qw, qx, qy, qz )

def multiply_quaternions(q, r):
    wq, xq, yq, zq = q
    wr, xr, yr, zr = r

    w = wq*wr - xq*xr - yq*yr - zq*zr
    x = wq*xr + xq*wr + yq*zr - zq*yr
    y = wq*yr - xq*zr + yq*wr + zq*xr
    z = wq*zr + xq*yr - yq*xr + zq*wr

    return (w, x, y, z)

def quaternion_to_rotation_matrix(qw, qx, qy, qz):
    qw, qx, qy, qz = normalize_quaternion(qw, qx, qy, qz)

    qx2 = qx * qx
    qy2 = qy * qy
    qz2 = qz * qz

    R = np.array([
        [(1-2*(qy2 + qz2)), (2*((qx*qy) - (qw*qz))), (2*((qx*qz) + (qw*qy)))],
        [(2*((qx*qy) + (qw*qz))), (1-2*(qx2 + qz2)), (2*((qy*qz) - (qw*qx)))], 
        [(2*((qx*qz)- (qw*qy))), (2*((qy*qz) + (qw*qx))), (1-2*(qx2 + qy2))]
    ])

    transpose_R = R.T    
    
    if not np.allclose(R @ transpose_R, np.identity(3)):
        raise ValueError("Rotation matrix is not orthogonal")
    if not np.isclose(np.linalg.det(R), 1):
        raise ValueError("Rotation matrix determinant is not 1")
    else:
        return (R)



class SPNode(Node):
    def __init__(self): 
        super().__init__("Converter_node")
        # parameters

        #input topics
        self.declare_parameter("openvins_topic","/ov_msckf/odomimu")
        self.declare_parameter("gazebo_topic","/ground_truth/odometry")
        self.declare_parameter("px4_topic","/fmu/in/vehicle_visual_odometry")
        self.declare_parameter("source","openvins")
        self.declare_parameter("frame_mode","local_frd")
        self.declare_parameter("publish_rate_hz",50.0)
        self.declare_parameter('stale_timeout', 0.25)

        #read param values
        self.openvins_topic = str(self.get_parameter("openvins_topic").value)
        self.gazebo_topic = str(self.get_parameter("gazebo_topic").value)
        self.px4_topic = str(self.get_parameter("px4_topic").value)
        self.source = str(self.get_parameter("source").value)
        self.frame_mode = str(self.get_parameter("frame_mode").value)
        self.publisher_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.stale_timeout = float(self.get_parameter("stale_timeout").value)

        #validation of parameters
        if self.source not in ["openvins", "gazebo_test"]:
            raise ValueError("Invalid source parameter. Must be 'openvins' or 'gazebo_test'.")  
        if self.frame_mode not in ["local_frd", "ned"]:
            raise ValueError("Invalid frame_mode parameter. Must be 'local_frd' or 'ned'.")  
        if self.publisher_rate <= 0:
            raise ValueError("Invalid publish_rate_hz parameter. Must be a positive number.")  
        if self.stale_timeout <= 0:
            raise ValueError("Invalid stale_timeout parameter. Must be a positive number.")

        #Fixed Coordinate rotation

        #vector in ros body flu to frd body px4
        self.R_frd_flu = np.array([
            [1, 0, 0], 
            [0, -1, 0],
            [0, 0, -1]
        ])

        #converts vector  in enu to ned
        self.R_ned_enu = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])

        #OVIO gloabal alignment 
        # not configured yet until alignment has been established
        self.R_output_openvins = None
        self.t_output_openvins = None
        self.alignment_valid = False

        #stored input messages
        self.latest_openvins_msg = None
        self.latest_gazebo_msg = None

        #true only when a new message arived since last publication attempt
        self.openvins_message_pending = True
        self.gazebo_message_pending = False

        #output cont and rest state
        self.previous_sample_timestamp_us = None
        self.previous_output_quaternion = None
        self.reset_counter = 0

        #subscriptions

        #subscribe to /ov_msckf/odomimu
        self.subscriber = self.create_subscription(
            Odometry,
            self.openvins_topic,
            self.openvins_callback,
            10
        )
        self.gazebo_subscription = self.create_subscription(
                    Odometry,
                    self.gazebo_topic,
                    self.gazebo_callback,
                    10,
                )
        
        #publish to /fmu/in/vehicle_visual_odometry
        self.publisher = self.create_publisher(
            VehicleOdometry,
            self.px4_topic,
            10
        )


        #Rate limited publication timer
        timer_period_sec = 1.0 / self.publish_rate_hz
        self.publish_timer = self.create_timer(
            timer_period_sec,
            self.publish_latest_measurement,
        )

        self.get_logger().info(
            f"VIO-to-PX4 converter started: "
            f"source={self.source}, "
            f"frame_mode={self.frame_mode}, "
            f"rate={self.publish_rate_hz:.1f} Hz")



def pose_callback(self, msg):
    #extract position and orientation
    pose = msg.pose.pose
    position = pose.position
    orientation = pose.orientation

def openvins_callback(self, msg):
    self.source == "openvins"
    if self.source != "openvins":
        return
    self.latest_openvins_msg = msg
    

def gazebo_callback(self, msg):
    self.source == "gazebo_test"
    if self.source != "gazebo_test":
        return
    self.latest_gazebo_msg = msg

def publish_latest_measurement(self):
    if self.source is openvins_callback:
        if not msg or self.


def main(args = None):
    rclpy.init(args = args)

    node = SPNode

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.detroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

        

       


 