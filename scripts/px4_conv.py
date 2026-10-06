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
        self.declare_parameter("frame_mode","local frd")
        self.declare_parameter("publish_rate",50.0)
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
        if self.frame_mode not in ["local frd", "global ned"]:
            raise ValueError("Invalid frame_mode parameter. Must be 'local frd' or 'global ned'.")  
        if self.publisher_rate <= 0:
            raise ValueError("Invalid publish_rate parameter. Must be a positive number.")  
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
        self.openvins_message_pending = False
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
        pose=msg.pose.pose
        position=pose.position
        orientation=pose.orientation


        

       


    

# Copy and use this as the continuation prompt:

# ```text
# Continue helping me build my ROS 2 OpenVINS-to-PX4 odometry converter. Work in read-only mode: do not create, edit, or overwrite files. Review the code I paste, explain errors, and guide me step-by-step. Do not write the complete node for me unless I explicitly request it.

# Environment and goal:

# - Ubuntu 22.04 / ROS 2 Humble.
# - OpenVINS input: /ov_msckf/odomimu
# - Input type: nav_msgs/msg/Odometry
# - Optional Gazebo test input: /ground_truth/odometry
# - PX4 output: /fmu/in/vehicle_visual_odometry
# - Output type: px4_msgs/msg/VehicleOdometry
# - Normal source: OpenVINS
# - Gazebo is only a selectable ground-truth plumbing test source.
# - Never publish OpenVINS and Gazebo to PX4 simultaneously.
# - Target publication rate: 50 Hz.
# - Start with PX4 POSE_FRAME_FRD because OpenVINS has arbitrary global yaw.
# - Do not use POSE_FRAME_NED until a real OpenVINS-global-to-NED alignment has been established.
# - OpenVINS publishes the camera-IMU reference point.
# - PX4 will perform the sensor lever-arm correction.
# - Do not subtract the sensor offset in the OpenVINS converter.

# PX4 sensor offset, expressed in body FRD:

# - EKF2_EV_POS_X = 0.12
# - EKF2_EV_POS_Y = -0.03
# - EKF2_EV_POS_Z = -0.242

# Current design:

# 1. Subscribe to both OpenVINS and Gazebo odometry topics.
# 2. Each subscription callback only stores its latest message and sets a pending flag.
# 3. A 50 Hz timer selects the configured source, processes one new sample, creates VehicleOdometry, validates it, and publishes it.
# 4. A source parameter selects either "openvins" or "gazebo_test".
# 5. A frame_mode parameter selects "local_frd" or "ned".
# 6. NED publication must be rejected while alignment_valid is false.

# Math helpers currently planned:

# - normalize_quaternion(qw,qx,qy,qz)
# - multiply_quaternions(q,r), using Hamilton products and returning (w,x,y,z)
# - quaternion_to_rotation_matrix(qw,qx,qy,qz)
# - rotate_vector(qw,qx,qy,qz,vx,vy,vz)
# - rotate_covariance(C,R)

# Quaternion conventions:

# - ROS input fields are x,y,z,w.
# - Internal quaternion order is w,x,y,z.
# - PX4 output q is [w,x,y,z].
# - Final orientation must include both the world-frame conversion and body FLU-to-FRD conversion.
# - Maintain quaternion sign continuity by comparing each output quaternion with the previous output quaternion.

# Fixed coordinate rotations:

# R_FRD_FLU =
# [ 1,  0,  0
#   0, -1,  0
#   0,  0, -1 ]

# R_NED_ENU =
# [ 0,  1,  0
#   1,  0,  0
#   0,  0, -1 ]

# The complete OpenVINS world alignment is not established yet:

# - R_output_openvins = None
# - t_output_openvins = None
# - alignment_valid = False

# Immediate code corrections still required:

# 1. rotate_covariance(C,R) must accept one 3×3 covariance block and one 3×3 rotation. It must not reshape C to 6×6. Its operation is:
#    C_output = R C R^T
#    Then symmetrize for floating-point cleanup.

# 2. Covariance extraction belongs in the timer processing:
#    C_pose = msg.pose.covariance reshaped to 6×6
#    C_twist = msg.twist.covariance reshaped to 6×6

#    Extract:
#    C_position = C_pose[0:3,0:3]
#    C_orientation = C_pose[3:6,3:6]
#    C_velocity = C_twist[0:3,0:3]

#    Rotate:
#    C_position_output = rotate_covariance(C_position,R_world)
#    C_orientation_output = rotate_covariance(C_orientation,R_FRD_FLU)
#    C_velocity_output = rotate_covariance(C_velocity,R_FRD_FLU)

#    PX4 receives only the diagonals of those three output matrices.

# 3. Correct the Gazebo topic default to /ground_truth/odometry.

# 4. Correct the state initialization typo:
#    previous_output_quaternion must be assigned None, not subtracted from None.

# 5. Use one publication-rate attribute name consistently, preferably publish_rate_hz.

# 6. Rename the ENU-to-NED matrix R_ned_enu, not R_frd_enu.

# 7. Use consistent frame strings, preferably:
#    local_frd
#    ned

# 8. Implement the methods referenced by initialization:
#    openvins_callback(msg)
#    gazebo_callback(msg)
#    publish_latest_measurement()

# 9. Remove or rename the old unused pose_callback.

# 10. Add the ROS 2 main entry point that initializes rclpy, creates the node, spins, destroys the node, and shuts down.

# OpenVINS callback responsibilities:

# - Return unless source == "openvins".
# - Store the newest Odometry message.
# - Set openvins_message_pending = True.
# - Do not perform the main conversion in this callback.

# Gazebo callback responsibilities:

# - Return unless source == "gazebo_test".
# - Store the newest Odometry message.
# - Set gazebo_message_pending = True.
# - Do not perform the main conversion in this callback.

# Timer processing still required:

# 1. Select the correct stored message based on source.
# 2. Return unless a new message is pending.
# 3. Extract pose, twist, timestamp, and covariance.
# 4. Convert the sample timestamp to microseconds:
#    sec*1,000,000 + nanosec/1,000
# 5. Reject zero, duplicate, backward, stale, or non-finite measurements.
# 6. Normalize the input quaternion.
# 7. Establish or apply local-FRD/global-NED alignment.
# 8. Transform position into the selected output world frame.
# 9. Transform orientation from OpenVINS IMU/FLU into PX4 FRD.
# 10. Convert body linear velocity:
#     vx_frd = vx_flu
#     vy_frd = -vy_flu
#     vz_frd = -vz_flu
# 11. Apply the same FLU-to-FRD conversion to angular velocity.
# 12. Rotate position, orientation, and velocity covariance.
# 13. Maintain quaternion sign continuity.
# 14. Construct VehicleOdometry.
# 15. Validate all output values.
# 16. Publish.
# 17. Store the accepted sample timestamp and quaternion.
# 18. Clear the corresponding pending flag.

# VehicleOdometry fields that must be populated:

# - timestamp: current compatible ROS/PX4 clock in microseconds
# - timestamp_sample: original OpenVINS/Gazebo sample time in microseconds
# - pose_frame: initially POSE_FRAME_FRD
# - position[3]
# - q[4] in [w,x,y,z]
# - velocity_frame: VELOCITY_FRAME_BODY_FRD
# - velocity[3]
# - angular_velocity[3]
# - position_variance[3]
# - orientation_variance[3]
# - velocity_variance[3]
# - reset_counter
# - quality: use 0 until a legitimate quality metric exists

# Reset handling:

# - Start reset_counter at zero.
# - Increment it when the VIO origin/alignment changes, OpenVINS reinitializes, or simulation time moves backward.
# - Do not silently continue across a frame reset.

# Gazebo test path:

# - Gazebo ground truth represents base_link.
# - PX4 is configured to receive the camera-IMU/sensor reference point.
# - Therefore the Gazebo test path must create a synthetic sensor pose:
#   p_world_sensor =
#       p_world_base
#       + R_world_base * p_base_sensor
# - Use p_base_sensor_FLU = [0.12,0.03,0.242].
# - Then convert the synthetic sensor pose from ENU/FLU to NED/FRD.
# - This keeps the same EKF2_EV_POS values for both OpenVINS and Gazebo testing.

# Validation required before enabling PX4 fusion:

# 1. Test quaternion identity and Hamilton multiplication.
# 2. Test quaternion-to-rotation-matrix orthogonality and determinant.
# 3. Test covariance conversion:
#    ENU diag(1,2,3) should become NED diag(2,1,3).
# 4. Confirm the node starts without exceptions.
# 5. Confirm both subscriptions receive their expected messages.
# 6. Confirm only the selected source can publish.
# 7. Confirm output rate is approximately 50 Hz.
# 8. Confirm timestamps are increasing.
# 9. Confirm quaternion norm is approximately one.
# 10. Test positive X/Y/Z translations and roll/pitch/yaw separately.
# 11. Confirm upward movement produces negative NED Z.
# 12. Inspect /fmu/in/vehicle_visual_odometry and PX4 vehicle_visual_odometry before enabling fusion.
# 13. Start PX4 fusion with position only, then add velocity, and add yaw only after its signs and alignment are proven.

# Package/runtime work still required after the node is complete:

# - Ensure package.xml includes rclpy, nav_msgs, px4_msgs, and the NumPy runtime dependency.
# - Add the new script to CMakeLists.txt installation.
# - Make the script executable.
# - Rebuild and source the ROS workspace.
# - Run with use_sim_time=true in Gazebo.
# - Confirm px4_msgs matches the PX4 checkout.
# - Confirm Micro XRCE-DDS connectivity.
# - Do not enable EKF2 external-vision fusion until the published message is validated.

# When reviewing my next code version, first identify syntax/runtime errors, then mathematical/frame errors, then missing functionality. Explain one stage at a time so I understand the implementation rather than receiving a completed solution.
# ```



