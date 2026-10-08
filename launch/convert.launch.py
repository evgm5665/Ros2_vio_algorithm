from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    source = LaunchConfiguration("source")
    frame_mode = LaunchConfiguration("frame_mode")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    stale_timeout = LaunchConfiguration("stale_timeout")
    openvins_topic = LaunchConfiguration("openvins_topic")
    gazebo_topic = LaunchConfiguration("gazebo_topic")
    px4_topic = LaunchConfiguration("px4_topic")

    return LaunchDescription([
        DeclareLaunchArgument("source", default_value="gazebo_test", description="Input source: gazebo_test or openvins"),
        DeclareLaunchArgument("frame_mode", default_value="ned", description="Output frame: ned or local_frd"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use the Gazebo simulation clock"),
        DeclareLaunchArgument("publish_rate_hz", default_value="50.0", description="Maximum converter publication rate"),
        DeclareLaunchArgument("stale_timeout", default_value="0.25", description="Maximum accepted measurement age in seconds"),
        DeclareLaunchArgument("openvins_topic", default_value="/ov_msckf/odomimu", description="OpenVINS odometry input topic"),
        DeclareLaunchArgument("gazebo_topic", default_value="/ground_truth/odometry", description="Gazebo ground-truth input topic"),
        DeclareLaunchArgument("px4_topic", default_value="/fmu/in/vehicle_visual_odometry", description="PX4 vehicle-odometry output topic"),

        Node(
            package="vio_alg",
            executable="px4_conv.py",
            output="screen",
            parameters=[{
                "source": source,
                "frame_mode": frame_mode,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                "stale_timeout": ParameterValue(stale_timeout, value_type=float),
                "openvins_topic": openvins_topic,
                "gazebo_topic": gazebo_topic,
                "px4_topic": px4_topic,
            }],
        ),
    ])