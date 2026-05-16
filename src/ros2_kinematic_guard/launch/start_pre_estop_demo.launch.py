#!/usr/bin/env python3
"""
start_pre_estop_demo.launch.py

5-minute pre-E-stop demo for ros2_kinematic_guard.

Profiles:
  wheel_slip
  localization_jump
  normal

Typical use:
  ros2 launch ros2_kinematic_guard start_pre_estop_demo.launch.py profile:=wheel_slip mode:=guard

Then publish a command:
  ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8}, angular: {z: 0.0}}"

Watch:
  ros2 topic echo /kinematic_guard/status
  ros2 topic echo /safe_cmd_vel
  ros2 topic echo /mock_robot/status
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    profile = LaunchConfiguration("profile")
    mode = LaunchConfiguration("mode")
    control_rate_hz = LaunchConfiguration("control_rate_hz")
    odom_rate_hz = LaunchConfiguration("odom_rate_hz")
    lookback_window_ms = LaunchConfiguration("lookback_window_ms")
    yellow_threshold = LaunchConfiguration("yellow_threshold")
    red_threshold = LaunchConfiguration("red_threshold")
    publish_tf = LaunchConfiguration("publish_tf")

    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value="wheel_slip",
            description="Demo fault profile: normal, wheel_slip, localization_jump.",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="guard",
            description="Guard mode: observe, passthrough, guard.",
        ),
        DeclareLaunchArgument(
            "control_rate_hz",
            default_value="20.0",
            description="Kinematic Guard evaluation rate.",
        ),
        DeclareLaunchArgument(
            "odom_rate_hz",
            default_value="50.0",
            description="Mock robot odometry rate.",
        ),
        DeclareLaunchArgument(
            "lookback_window_ms",
            default_value="200.0",
            description="Sliding command/odom history window in milliseconds.",
        ),
        DeclareLaunchArgument(
            "yellow_threshold",
            default_value="2.5",
            description="Residual threshold for YELLOW_SLOWDOWN.",
        ),
        DeclareLaunchArgument(
            "red_threshold",
            default_value="5.0",
            description="Residual threshold for BRAKE_AND_RESYNC.",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description="Whether mock robot publishes odom -> base_link TF.",
        ),

        # --------------------------------------------------------
        # 1. Kinematic Guard
        # --------------------------------------------------------
        Node(
            package="ros2_kinematic_guard",
            executable="kinematic_guard_node",
            name="kinematic_guard",
            output="screen",
            parameters=[{
                "cmd_input_topic": "/cmd_vel",
                "cmd_output_topic": "/safe_cmd_vel",
                "odom_topic": "/odom",

                "status_topic": "/kinematic_guard/status",
                "residual_topic": "/kinematic_guard/residual",

                "cmd_input_type": "twist",
                "cmd_output_type": "twist",

                "mode": mode,
                "lookback_window_ms": lookback_window_ms,

                "control_rate_hz": control_rate_hz,
                "status_rate_hz": 10.0,

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

                "slowdown_scale": 0.35,
                "resync_required_good_frames": 5,
                "node_resync_good_frames": 5,

                "enable_brake_and_resync": True,
                "flush_buffers_on_red": True,
                "publish_zero_until_ready": True,

                "use_receive_time_for_twist": True,
                "replace_zero_header_stamp": True,

                # Demo-friendly cause labeling.
                "wheel_slip_expected_min_m": 0.03,
                "wheel_slip_measured_ratio": 0.25,
                "localization_jump_min_m": 0.50,
                "localization_jump_ratio": 5.0,
            }],
        ),

        # --------------------------------------------------------
        # 2. Lightweight virtual AMR/AGV
        # --------------------------------------------------------
        Node(
            package="ros2_kinematic_guard",
            executable="mock_robot_simulator",
            name="mock_robot",
            output="screen",
            parameters=[{
                "cmd_topic": "/safe_cmd_vel",
                "odom_topic": "/odom",
                "status_topic": "/mock_robot/status",

                "profile": profile,
                "rate_hz": odom_rate_hz,
                "cmd_timeout": 0.50,

                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_tf": publish_tf,

                "max_linear_accel": 1.2,
                "max_angular_accel": 2.5,

                # wheel_slip profile
                "slip_start_sec": 3.0,
                "slip_duration_sec": 3.0,
                "slip_ratio": 0.05,
                "slip_lateral_drift_mps": 0.05,

                # localization_jump profile
                "jump_time_sec": 4.0,
                "jump_distance_m": 2.5,
                "jump_yaw_rad": 0.0,
            }],
        ),
    ])
