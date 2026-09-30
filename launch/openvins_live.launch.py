import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

#starts openvins 

def generate_launch_description():
    config_directory = os.path.join(
        get_package_share_directory("vio_alg"),
        "config",
        "openvins",
        "x500_stereo_vio",
    )

    estimator_config = os.path.join(
        config_directory,
        "estimator_config.yaml",
    )

    required_files = [
        estimator_config,
        os.path.join(config_directory, "kalibr_imu_chain.yaml"),
        os.path.join(config_directory, "kalibr_imucam_chain.yaml"),
    ]

    missing_files = [
        path for path in required_files
        if not os.path.isfile(path)
    ]

    if missing_files:
        raise FileNotFoundError(
            "Missing OpenVINS configuration files: "
            + ", ".join(missing_files)
        )

    openvins_node = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        namespace="ov_msckf",
        output="screen",
        parameters=[
            {
                "config_path": estimator_config,
                "verbosity": "INFO",
                "use_stereo": True,
                "max_cameras": 2,
                "save_total_state": False,
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription([
        LogInfo(msg=f"OpenVINS configuration: {estimator_config}"),
        openvins_node,
    ])