from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_vio_sensor",
            arguments=[
                "--x", "0.12",
                "--y", "0.03",
                "--z", "0.242",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "base_link",
                "--child-frame-id", "vio_sensor_link",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="vio_sensor_to_camera_imu",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "vio_sensor_link",
                "--child-frame-id", "camera_imu_frame",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="vio_sensor_to_left_camera",
            arguments=[
                "--x", "0",
                "--y", "0.04",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "vio_sensor_link",
                "--child-frame-id", "left_camera_sensor_frame",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="vio_sensor_to_right_camera",
            arguments=[
                "--x", "0",
                "--y", "-0.04",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "vio_sensor_link",
                "--child-frame-id", "right_camera_sensor_frame",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="left_camera_to_optical",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "-1.57079632679",
                "--pitch", "0",
                "--yaw", "-1.57079632679",
                "--frame-id", "left_camera_sensor_frame",
                "--child-frame-id", "left_camera_frame",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="right_camera_to_optical",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "-1.57079632679",
                "--pitch", "0",
                "--yaw", "-1.57079632679",
                "--frame-id", "right_camera_sensor_frame",
                "--child-frame-id", "right_camera_frame",
            ],
        ),
    ])