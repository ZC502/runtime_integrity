#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    profile = LaunchConfiguration("profile")
    use_demo_cmd = LaunchConfiguration("use_demo_cmd")
    slip_probability = LaunchConfiguration("slip_probability")
    publish_tf = LaunchConfiguration("publish_tf")
    use_static_tf = LaunchConfiguration("use_static_tf")

    control_rate_hz = LaunchConfiguration("control_rate_hz")
    odom_rate_hz = LaunchConfiguration("odom_rate_hz")

    yellow_threshold = LaunchConfiguration("yellow_threshold")
    red_threshold = LaunchConfiguration("red_threshold")

    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value="bad_wifi",
            description="Jitter profile: clean, mild_wifi, bad_wifi, 5g_burst, wifi_collapse",
        ),
        DeclareLaunchArgument(
            "use_demo_cmd",
            default_value="true",
            description="Let jitter_injector generate demo /cmd_vel_raw commands.",
        ),
        DeclareLaunchArgument(
            "slip_probability",
            default_value="0.01",
            description="Probability of synthetic odom slip disturbance per odom tick.",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description="Let synthetic_odom_provider publish odom -> base_link TF.",
        ),
        DeclareLaunchArgument(
            "use_static_tf",
            default_value="true",
            description="Publish static map -> odom TF for RViz.",
        ),
        DeclareLaunchArgument(
            "control_rate_hz",
            default_value="20.0",
            description="NARH Guard evaluation/control rate.",
        ),
        DeclareLaunchArgument(
            "odom_rate_hz",
            default_value="100.0",
            description="Synthetic odom publish rate.",
        ),
        DeclareLaunchArgument(
            "yellow_threshold",
            default_value="2.5",
            description="R_NAR threshold for YELLOW_SLOWDOWN.",
        ),
        DeclareLaunchArgument(
            "red_threshold",
            default_value="5.0",
            description="R_NAR threshold for RED_BRAKE.",
        ),

        # 1. Bad-WiFi jitter injector
        Node(
            package="ros2_kinematic_guard",
            executable="jitter_injector_node",
            name="jitter_injector",
            output="screen",
            parameters=[{
                "profile": profile,
                "use_demo_cmd": use_demo_cmd,

                "demo_raw_topic": "/cmd_vel_raw",
                "input_topic": "/cmd_vel_raw",
                "output_topic": "/cmd_vel_jittered",

                "status_topic": "/jitter_injector/status",
                "demo_rate_hz": 20.0,
                "tick_hz": 100.0,
                "status_hz": 2.0,
                "log_events": True,
            }],
        ),

        # 2. NARH Guard main gate
        Node(
            package="ros2_kinematic_guard",
            executable="kinematic_guard_node",
            name="kinematic_guard",
            output="screen",
            parameters=[{
                "cmd_input_topic": "/cmd_vel_jittered",
                "cmd_output_topic": "/kinematic_guard/safe_cmd_vel",
                "odom_topic": "/odom",

                "status_topic": "/kinematic_guard/status",
                "residual_topic": "/kinematic_guard/residual",

                "cmd_input_type": "twist",
                "cmd_output_type": "twist",

                "control_rate_hz": control_rate_hz,
                "status_rate_hz": 5.0,

                "yellow_threshold": yellow_threshold,
                "red_threshold": red_threshold,

                "default_dt": 0.05,
                "min_dt": 0.001,
                "max_dt": 0.50,
                "cmd_ttl": 0.25,
                "phase_tolerance": 0.08,

                "max_linear_accel": 0.8,
                "max_angular_accel": 1.5,
                "max_linear_jerk": 3.0,
                "max_angular_jerk": 6.0,

                "position_tolerance": 0.03,
                "yaw_tolerance": 0.08,
                "lateral_tolerance": 0.03,

                "enable_brake_and_resync": True,
                "flush_buffers_on_red": True,
                "publish_zero_until_ready": True,
                "node_resync_good_frames": 5,

                "slowdown_scale": 0.45,
                "resync_required_good_frames": 5,

                "use_receive_time_for_twist": True,
                "replace_zero_header_stamp": True,
            }],
        ),

        # 3. Synthetic odometry provider
        Node(
            package="ros2_kinematic_guard",
            executable="synthetic_odom_provider",
            name="synthetic_odom",
            output="screen",
            parameters=[{
                "input_topic": "/kinematic_guard/safe_cmd_vel",
                "input_type": "twist",
                "odom_topic": "/odom",
                "status_topic": "/synthetic_odom/status",

                "rate_hz": odom_rate_hz,
                "cmd_timeout": 0.30,

                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_tf": publish_tf,

                "max_linear_accel": 1.0,
                "max_angular_accel": 2.0,

                "slip_probability": slip_probability,
                "linear_noise_std": 0.005,
                "angular_noise_std": 0.01,
            }],
        ),

        # 4. Command integrity reporter
        Node(
            package="ros2_kinematic_guard",
            executable="command_integrity_reporter_node",
            name="command_integrity_reporter",
            output="screen",
            parameters=[{
                "guard_status_topic": "/kinematic_guard/status",
                "guard_residual_topic": "/kinematic_guard/residual",

                "diagnostics_topic": "/diagnostics",
                "vda_state_topic": "/command_integrity/vda5050_state",
                "summary_topic": "/command_integrity/summary",

                "vehicle_id": "demo_amr_001",
                "manufacturer": "ros2_kinematic_guard_demo",
                "serial_number": "demo",

                "yellow_threshold": yellow_threshold,
                "red_threshold": red_threshold,

                "cmd_ttl": 0.25,
                "nominal_dt": 0.05,
                "status_timeout": 1.0,
                "publish_rate_hz": 5.0,
            }],
        ),

        # 5. Optional static transform map -> odom
        Node(
            condition=IfCondition(use_static_tf),
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_to_odom",
            output="screen",
            arguments=[
                "0", "0", "0",
                "0", "0", "0",
                "map", "odom",
            ],
        ),
    ])
