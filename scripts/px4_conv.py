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
   
    #input 
    C = np.array(C, dtype = float)
    R = np.array(R, dtype = float)

    if C.size != 9:
        raise ValueError("C is not a 3x3 matrix")
    if R.size != 9:
        raise ValueError("R is not a 3x3 matrix")

    

    if not np.all(np.isfinite(C)):
        raise ValueError("C does not have at least one finite value")
    if not np.all(np.isfinite(R)):
        raise ValueError("R does not have at least one finite value")
    if not np.allclose(C_rotated, C_rotated.T):
        raise ValueError("Covariance matrix is not symmetric")
    
    C_rotated = R @ C @ R.T

    CM_rotated = 0.5 * (C_rotated + C_rotated.T)  # Ensure symmetry

    if np.array(np.diag(CM_rotated) < -1.0e-12):
        raise ValueError("Rotated conv contatains a negative variance")
    
    return (CM_rotated) 

    


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
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.stale_timeout = float(self.get_parameter("stale_timeout").value)

        #validation of parameters
        if self.source not in ["openvins", "gazebo_test"]:
            raise ValueError("Invalid source parameter. Must be 'openvins' or 'gazebo_test'.")  
        if self.frame_mode not in ["local_frd", "ned"]:
            raise ValueError("Invalid frame_mode parameter. Must be 'local_frd' or 'ned'.")  
        if self.publish_rate_hz <= 0:
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
        self.openvins_message_pending = False
        self.gazebo_message_pending = False

        #output cont and rest state
        self.previous_sample_timestamp_us = None
        self.previous_output_quaternion = None
        self.reset_counter = 0

        #subscriptions

        #subscribe to /ov_msckf/odomimu
        self.openvins_subscription = self.create_subscription(
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
        self.px4_publisher = self.create_publisher(
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

    def publish_latest_measurement(self):
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


        #extract matricies 
        # # using msg.pose.covariance and msg.twist.covariance
        # both output [x, y, z rotation x, rotation, y, rotation z] in a 6x6 matrix  with float64[36] cov

        CM_pose = np.asarray(msg.pose.covariance, dtype = float,).reshape((6, 6))

        #Position of the OV IMU expressed in OV global 6
        p_G = np.array([CM_pose.position.x, CM_pose.position.y, CM_pose.position.z], dtype = float)
        q_G_I = normalize_quaternion(CM_pose.orientation.w, CM_pose.orientation.x, CM_pose.orientation.y, CM_pose.orientation.z)

        q_G_I = normalize_quaternion(CM_pose.orientation.w, CM_pose.orientation.x, CM_pose.orientation.y, CM_pose.orientation.z)

        #Rotation from OV imu FLU I into OV global G
        R_G_I = quaternion_to_rotation_matrix(q_G_I[0], q_G_I[1], q_G_I[2], q_G_I[3], q_G_I[4])

    
        CM_twist = np.asarray(msg.twist.covariance, dtype = float,).reshape((6, 6))

       #Linear velocity expressed in OV IMU FLU I 
        v_I = np.array([CM_twist.linear.x, CM_twist.linear.y, CM_twist.linear.z], dtype = float)

        #Angular velocity expressed in OV IMU FLU I
        omega_I = np.array([CM_twist.angular.x, CM_twist.angular.y, CM_twist.angular.z], dtype = float)

        if not np.all(np.isfinite(p_G)):
            self.get_logger().warning("Rejected non finite position")
            return

        if not np.all(np.isfinite(q_G_I)):
            self.get_logger().warning("Rejected non finite position")
            return

        if not np.all(np.isfinite(v_I)):
            self.get_logger().warning("Rejeted non finite velocity")
            return

        if not np.all(np.isfinite(omega_I)):
            self.get_logger().warning("Rejected non finite velocity")
            return

        if not self.alignment_valid:

            #first column of R_G_I
            forward_G = R_G_I[:, 0].copy()

            #IMU forward x axis expressed in OV global 6
            forward_G[2] = 0.0

            forward_norm = np.linalg.norm(forward_G)

        if not np.isfinite(forward_norm):
            self.get_logged().warning("cant establish alignment from non-finite forward direction")
            return

        if forward_norm < 1.0e-6:
            self.get_logger().warning("Cant establish alignment because forward direction is vertical")
            return

        forward_G = forward_G / forward_norm

        #output local frd has pos z pointing down
        down_G = np.array([0.0, 0.0, -1.0], dtype = float)

        right_G = np.cross(down_G, forward_G)

        right_norm = np.linal.noarm(right_G)

        if right_norm < 1.0e-6:
            self.get_logger().warning("cant establish a valid local right direction")
            return

        right_G = right_G / right_norm

        #G to O rotation
        #row 0 = forward; row 1 = right, row 2 = down
        R_O_G = np.vstack([forward_G, right_G, down_G])

        if not np.allclose(R_O_G @ R_O_G.T, np.identity(3), atol = 1.0e-6):
            self.get_logger().warning("OV output alignment is not orthogonal")
            return

        self.R_output_openvins = R_O_G

        #local origin as the first accepted position


        #assigning the pose covariance matrix to the appropriate variables
        c_position = CM_pose[:3, :3]
        c_orientation = CM_pose[3:6, 3:6]

        #assigning the twist covariance matrix to the appropriate variables
        c_velocity = CM_twist[:3, :3]
        c_angular = CM_twist[3:6, 3:6]

        

        #convert sample timestmamp to int ms
        sample_timestamp_us = (msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1_000)

        #sample errors
        if sample_timestamp_us <= 0:
            self.get_logger().warning("rejected measurement with 0 or neg timestamp")
            return
        
        if self.previous_sample_timestamp_us is not None:
            if sample_timestamp_us == self.previous_sample_timestamp_us:
                self.get_logger().warning("Sample timestamp repeats")
                return
        
            if sample_timestamp_us < self.previous_sample_timestamp_us:
                self.get_logger().warning("timing error: sample time is less than previous timestamp")
                return
            
        now_us = self.get_clock().now().nanoseconds // 1_000
        message_age_us = now_us - sample_timestamp_us
        stale_timeout_us = int(self.stale_timeout * 1_000_000)

        if message_age_us < 0:
            self.get_logger().warning("measurement timestamp is ahead of the node clock")
            return

        if message_age_us > stale_timeout_us:
            self.get_logger().warning("Rejected stale measurement")
            return
        
    

        c_position_input = np.array([c_position, c_position * c_orientation], [c_orientation* c_position, c_orientation])

        NrE = self.R_ned_enu
        NrE_T = self.R_ned_enu.T

        BrI = self.R_frd_flu
        BrI_T = self.R_frd_flu.T
        
        #transform linear velocity
        C_velocity_FRD = BrI @ c_velocity @ BrI_T
        #transform orientation
        C_orientation_FRD = BrI @ c_angular @ BrI_T

        C_P_G = C_orientation_FRD

        if np.array(np.diag(IDK) < -1.0e-12):
            raise ValueError("Transformed position contains a negative variance")
        if np.array(np.diag(C_velocity_FRD) < -1.0e-12):
            raise ValueError("Transformed velocity contains a negative variance")
        if np.array(np.diag(C_orientation_FRD) < -1.0e-12):
            raise  ValueError("Transformed orientation contains a negative variance")

        t_O_G = -(R_O_G) @ p_G

        p_O = R_O_G @ p_G + t_O_G

        self.t_output_openvins = t_O_G

        self.alignment_valid = True

        self.get_logger().info("Established OV local-frd alignment and local origin")

        R_O_G = self.R_output_openvins
        t_O_G = self.t_output_openvins

        #output position
        position_output = p_O

        #output orientation
        R_B_I = self.R_frd_flu
        R_I_B = R_B_I.T
        R_O_B = R_O_G @ R_G_I @ R_I_B 

        if not np.allclose(R_O_B @ R_O_B.T, np.identity(3), atol = 1.0e-6,):
            self.get_logger().warning("Output orientation matric is not orthogonal")
            return

        if not np.isclose(np.linalg.det(R_O_B), 1.0, atol = 1.0e-6):
            self.get_logger().warning("Output orientation matrix had invalid determinant")
            return

        orientation_output = np.array(normalize_quaternion(orientation_output[0], orientation_output[1], orientation_output[2], orientation_output[3]), dtype = float)

        if self.previous_output_quaternion is not None:

            previous_q = np.array(self.previous_output_quaternion, dtype = float,)

            if np.dot(orientation_output, previous_q) < 0.0:
                orientation_output = -orientation_output

        #linear and angular velocity

        #OV IMU FLU I to PX4 body FRD B

        linear_velocity_frd = R_B_I @ v_I
        angular_velocity_frd = R_B_I @ omega_I


        #TF each covariance 

        #position covariance: OV global G to output local FRD O
        C_position_output = rotate_covariance(c_position, R_O_G,)

        #Orientation- error covariance:
        #OV IMU FLU I to PX4 body FRD B
        C_orientation_output = rotate_covariance(c_orientation, R_B_I,)

        C_velocity_output = rotate_covariance(c_velocity, R_B_I)
        #Extract three covariance diagnals

        position_variance = np.diag(C_position_output).astype(float)

        orientation_variance = np.diag(C_orientation_output).astype(float)

        velocity_variance = np.diag(C_velocity_output).astype(float)

        #validate
        if not np.all(np.isfinite(position_output)):
            self.get_logger().warning("Rejected non-finite output position")
            return
        if not np.all(np.isfinite(orientation_output)):
            self.get_logger().warning("Rejected non-finite output quaternion")
            return
        if not np.isclose(np.linalg.norm(orientation_output), 1.0, atol = 1.0e-5,):
            self.get_logger().warning("Rejected non unit output quaternion")
            return
        if not np.all(np.isfinite(linear_velocity_frd)):
            self.get_logger().warning("Rejected non-finite output velocity")
            return

        if not np.all(np.isfinite(angular_velocity_frd)):
            self.get_logger().warning("Rejected non finite output angular velocity")
            return
        if not np.all(np.isfintie(position_variance)):
            self.get_logger().warning("Rejected non finite position variance")
            return
        if not np.all(np.isfinite(orientation_variance)):
            self.get_logger().warning("Rejected non finite orientation variance")
            return
        if not np.all(np.isfinite(velocity_variance)):
            self.get_logger().warning("Rejected non finite orientation variance")
            return
        if not np.any(orientation_variance < -1.0e-12):
            self.get_logger().warning("Rejected negative orientation variance")
            return
        if not np.any(position_variance < -1.0e-12):
            self.get_logger().warning("Rejected negative position variance")
            return
        if not np.any(velocity_variance < -1.0e-12):
            self.get_logger().warning("Rejected negative velocity variance")
            return


        #VEHICLE ODOMETRY MESSAGES

        output_msg = VehicleOdometry()

        #OG OV time
        output_msg.timestamp_sample = int(sample_timestamp_us)

        #OV published in arb heading local-frd world
        output_msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD

        output_msg.position = [float(position_output[0]), float(position_output[1]), float(position_output[2]),]

        output_msg.q = [ float(orientation_output[0]),float(orientation_output[1]),float(orientation_output[2]),float(orientation_output[3]),]

        output_msg.velocity_frame = (VehicleOdometry.VELOCITY_FRAME_BODY_FRD)

        output_msg.velocity = [float(linear_velocity_frd[0]), float(linear_velocity_frd[1]),float(linear_velocity_frd[2]),]

        output_msg.angular_velocity = [float(angular_velocity_frd[0]),float(angular_velocity_frd[1]),float(angular_velocity_frd[2]),]

        output_msg.position_variance = [float(position_variance[0]), float(position_variance[1]), float(position_variance[2]),]

        output_msg.orientation_variance = [float(orientation_variance[0]), float(orientation_variance[1]), float(orientation_variance[2]),]

        output_msg.velocity_variance = (float(velocity_variance[0]), float(velocity_variance[1]), float(velocity_variance[2]),)

        output_msg.reset_counter = int(self.reset_counter)

        output_msg.quality = 0 

        #TODO VALIDATE


        self.px4_publisher.publish(output_msg)
        self.previous_sample_timestamp_us = sample_timestamp_us
        self.previous_output_quaternion = (orientation_output.copy())
        self.openvins_message_pending = False







            


def main(args = None):
    rclpy.init(args = args)

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

        

       


 