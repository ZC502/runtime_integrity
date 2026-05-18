#!/usr/bin/env python3
"""
mock_robot_simulator.py

Lightweight AMR/AGV mock robot for ros2_kinematic_guard v0.2.

No Gazebo.
No Isaac Sim.
No real robot.

It publishes /odom from a simple differential-drive style SE(2) integration
and can inject industrial failure profiles:

normal
wheel_slip
localization_jump
command_burst_placeholder

Typical loop
------------
/cmd_vel
   ↓
kinematic_guard_node
   ↓
/safe_cmd_vel
   ↓
mock_robot_simulator
   ↓
/odom
   ↑
kinematic_guard_node
"""

from __future__ import annotations

import math
import json
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

try:
    from tf2_ros import TransformBroadcaster
except Exception:
    TransformBroadcaster = None


def finite_or(x, fallback: float = 0.0) -> float:
    try:
        x = float(x)
        return x if math.isfinite(x) else fallback
    except Exception:
        return fallback


def yaw_to_quaternion(yaw: float):
    half = 0.5 * yaw
    z = math.sin(half)
    w = math.cos(half)
    return 0.0, 0.0, z, w


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class MockRobotSimulator(Node):
    def __init__(self) -> None:
        super().__init__("mock_robot_simulator")

        # --------------------------------------------------------
        # Topics
        # --------------------------------------------------------
        self.declare_parameter("cmd_topic", "/safe_cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("status_topic", "/mock_robot/status")

        # --------------------------------------------------------
        # Frames
        # --------------------------------------------------------
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        # --------------------------------------------------------
        # Simulation
        # --------------------------------------------------------
        self.declare_parameter("profile", "normal")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("cmd_timeout", 0.50)

        # First-order actuator limits
        self.declare_parameter("max_linear_accel", 1.2)
        self.declare_parameter("max_angular_accel", 2.5)

        # Wheel slip profile
        self.declare_parameter("slip_start_sec", 3.0)
        self.declare_parameter("slip_duration_sec", 3.0)
        self.declare_parameter("slip_ratio", 0.05)
        self.declare_parameter("slip_lateral_drift_mps", 0.05)

        # Localization jump profile
        self.declare_parameter("jump_time_sec", 4.0)
        self.declare_parameter("jump_distance_m", 2.5)
        self.declare_parameter("jump_yaw_rad", 0.0)

        # Optional noise
        self.declare_parameter("linear_noise_std", 0.0)
        self.declare_parameter("angular_noise_std", 0.0)

        # --------------------------------------------------------
        # Read params
        # --------------------------------------------------------
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.profile = str(self.get_parameter("profile").value).lower().strip()
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.max_linear_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_angular_accel = float(self.get_parameter("max_angular_accel").value)

        self.slip_start_sec = float(self.get_parameter("slip_start_sec").value)
        self.slip_duration_sec = float(self.get_parameter("slip_duration_sec").value)
        self.slip_ratio = float(self.get_parameter("slip_ratio").value)
        self.slip_lateral_drift_mps = float(self.get_parameter("slip_lateral_drift_mps").value)

        self.jump_time_sec = float(self.get_parameter("jump_time_sec").value)
        self.jump_distance_m = float(self.get_parameter("jump_distance_m").value)
        self.jump_yaw_rad = float(self.get_parameter("jump_yaw_rad").value)

        # --------------------------------------------------------
        # State
        # --------------------------------------------------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.cmd_vx = 0.0
        self.cmd_wz = 0.0

        self.actual_vx = 0.0
        self.actual_wz = 0.0

        self.last_cmd_time: Optional[float] = None
        self.first_cmd_time: Optional[float] = None

        self.start_time = self._now_sec()
        
        self.last_loop_time = self.start_time
        self.jump_done = False
        self.last_fault_state = "NONE"

        # --------------------------------------------------------
        # ROS interfaces
        # --------------------------------------------------------
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_topic,
            self._cmd_callback,
            10,
        )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.tf_broadcaster = None
        if self.publish_tf and TransformBroadcaster is not None:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            1.0 / max(self.rate_hz, 1.0),
            self._physics_loop,
        )

        self.get_logger().info(
            "Mock Robot Simulator started | "
            f"profile={self.profile} | "
            f"cmd={self.cmd_topic} -> odom={self.odom_topic} | "
            f"rate={self.rate_hz:.1f}Hz"
        )

    # ============================================================
    # Time
    # ============================================================

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ============================================================
    # Callbacks
    # ============================================================

    def _cmd_callback(self, msg: Twist) -> None:
        now = self._now_sec()

        self.cmd_vx = finite_or(msg.linear.x)
        self.cmd_wz = finite_or(msg.angular.z)
        self.last_cmd_time = now

        # Do not start the fault timer on the initial zero /safe_cmd_vel.
        # Start only when the mock robot receives a real motion command.
        motion_cmd = abs(self.cmd_vx) > 1e-3 or abs(self.cmd_wz) > 1e-3

        if self.first_cmd_time is None and motion_cmd:
            self.first_cmd_time = now
            self.get_logger().info(
                "[MOCK_ROBOT] First non-zero /safe_cmd_vel received. Fault timer starts now."
            )
          
    # ============================================================
    # Simulation
    # ============================================================

    def _physics_loop(self) -> None:
        now = self._now_sec()
        dt = clamp(now - self.last_loop_time, 1e-4, 0.20)
        self.last_loop_time = now

        elapsed = now - self.start_time
        if self.first_cmd_time is None:
            fault_elapsed = None
        else:
            fault_elapsed = now - self.first_cmd_time
       

        # If command is stale, the mock robot's internal driver stops.
        if self.last_cmd_time is None or (now - self.last_cmd_time) > self.cmd_timeout:
            target_vx = 0.0
            target_wz = 0.0
        else:
            target_vx = self.cmd_vx
            target_wz = self.cmd_wz

        # First-order actuator response
        self.actual_vx = self._approach(
            current=self.actual_vx,
            target=target_vx,
            max_delta=self.max_linear_accel * dt,
        )
        self.actual_wz = self._approach(
            current=self.actual_wz,
            target=target_wz,
            max_delta=self.max_angular_accel * dt,
        )

        fault_state = "NONE"

        # --------------------------------------------------------
        # Fault profiles
        # --------------------------------------------------------
        vx_for_motion = self.actual_vx
        wz_for_motion = self.actual_wz
        lateral_drift = 0.0

        if self.profile == "wheel_slip":
            if (
                fault_elapsed is not None
                and self.slip_start_sec <= fault_elapsed <= self.slip_start_sec + self.slip_duration_sec
            ):
                fault_state = "WHEEL_SLIP"
                vx_for_motion = self.actual_vx * self.slip_ratio
                lateral_drift = self.slip_lateral_drift_mps

        elif self.profile == "localization_jump":
            if (
                fault_elapsed is not None
                and (not self.jump_done)
                and fault_elapsed >= self.jump_time_sec
            ):
                fault_state = "LOCALIZATION_JUMP"
                self.x += self.jump_distance_m * math.cos(self.yaw)
                self.y += self.jump_distance_m * math.sin(self.yaw)
                self.yaw += self.jump_yaw_rad
                self.jump_done = True
                self.get_logger().warn(
                    "[MOCK_ROBOT] SIMULATING LOCALIZATION_JUMP | "
                    f"jump_distance={self.jump_distance_m:.2f}m"
                )

        elif self.profile in {"command_burst", "wifi_collapse"}:
            # Command burst is better simulated at command-message level
            # using jitter_injector_node.py. This mock keeps normal physics.
            fault_state = "COMMAND_BURST_PROFILE_PLACEHOLDER"

        # --------------------------------------------------------
        # SE(2) integration
        # --------------------------------------------------------
        self.x += vx_for_motion * math.cos(self.yaw) * dt
        self.y += vx_for_motion * math.sin(self.yaw) * dt

        if lateral_drift != 0.0:
            self.x += -math.sin(self.yaw) * lateral_drift * dt
            self.y += math.cos(self.yaw) * lateral_drift * dt

        self.yaw += wz_for_motion * dt
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        self._publish_odom(now, vx_for_motion, wz_for_motion)
        self._publish_status(now, elapsed, fault_state, vx_for_motion, wz_for_motion)

        if fault_state != self.last_fault_state:
            if fault_state != "NONE":
                self.get_logger().warn(f"[MOCK_ROBOT] {fault_state}_START")
            elif self.last_fault_state != "NONE":
                self.get_logger().info(f"[MOCK_ROBOT] {self.last_fault_state}_END")
            self.last_fault_state = fault_state

    def _approach(self, current: float, target: float, max_delta: float) -> float:
        if target > current:
            return min(target, current + max_delta)
        return max(target, current - max_delta)

    # ============================================================
    # Publishing
    # ============================================================

    def _publish_odom(self, now: float, vx: float, wz: float) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = wz

        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

    def _publish_status(
        self,
        now: float,
        elapsed: float,
        fault_state: str,
        vx_for_motion: float,
        wz_for_motion: float,
    ) -> None:
        fault_elapsed = None
        if self.first_cmd_time is not None:
            fault_elapsed = now - self.first_cmd_time

        payload = {
            "timestamp": now,
            "profile": self.profile,
            "elapsedSec": elapsed,
            "faultElapsedSec": fault_elapsed,
            "faultState": fault_state,
            "pose": {
                "x": self.x,
                "y": self.y,
                "yaw": self.yaw,
            },
            "inputCmd": {
                "linear_vx": self.cmd_vx,
                "angular_wz": self.cmd_wz,
            },
            "actualMotion": {
                "linear_vx": vx_for_motion,
                "angular_wz": wz_for_motion,
            },
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MockRobotSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
