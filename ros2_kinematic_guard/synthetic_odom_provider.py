#!/usr/bin/env python3
"""
synthetic_odom_provider.py

Synthetic odometry provider for ros2_kinematic_guard.

Project:
    ros2_kinematic_guard

Formal name:
    NARH-based Kinematic Guard for ROS 2

Tagline:
    It does not fix the network.
    It prevents bad network timing from becoming dangerous robot motion.

Purpose
-------
This node provides a lightweight synthetic body for the Bad-WiFi test loop.

It subscribes to a command topic, integrates a differential-drive model,
and publishes nav_msgs/Odometry.

Typical closed loop
-------------------
/cmd_vel_raw
    -> jitter_injector_node.py
    -> /cmd_vel_jittered
    -> kinematic_guard_node.py
    -> /kinematic_guard/safe_cmd_vel
    -> synthetic_odom_provider.py
    -> /odom
    -> kinematic_guard_node.py

This allows NARH Guard to test command/feedback consistency without Gazebo,
Isaac Sim, or a real robot.
"""

from __future__ import annotations

import json
import math
import random
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

try:
    from tf2_ros import TransformBroadcaster
except Exception:
    TransformBroadcaster = None


# ============================================================
# Math helpers
# ============================================================

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def finite_or(x, fallback: float = 0.0) -> float:
    try:
        x = float(x)
        return x if math.isfinite(x) else fallback
    except Exception:
        return fallback


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quaternion(yaw: float):
    half = 0.5 * yaw
    z = math.sin(half)
    w = math.cos(half)
    return 0.0, 0.0, z, w


# ============================================================
# State
# ============================================================

@dataclass
class BodyState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    vx_cmd: float = 0.0
    wz_cmd: float = 0.0

    vx_eff: float = 0.0
    vy_eff: float = 0.0
    wz_eff: float = 0.0


@dataclass
class SlipState:
    active: bool = False
    end_time: float = 0.0
    linear_scale: float = 1.0
    angular_scale: float = 1.0
    lateral_velocity: float = 0.0
    yaw_bias_rate: float = 0.0


# ============================================================
# Node
# ============================================================

class SyntheticOdomProvider(Node):
    def __init__(self):
        super().__init__("synthetic_odom_provider")

        # --------------------------------------------------------
        # Topics
        # --------------------------------------------------------
        self.declare_parameter("input_topic", "/kinematic_guard/safe_cmd_vel")
        self.declare_parameter("input_type", "twist")  # twist | twist_stamped
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("status_topic", "/synthetic_odom/status")

        # --------------------------------------------------------
        # Frames
        # --------------------------------------------------------
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", False)

        # --------------------------------------------------------
        # Timing
        # --------------------------------------------------------
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("cmd_timeout", 0.30)

        # --------------------------------------------------------
        # Differential-drive kinematics
        # --------------------------------------------------------
        self.declare_parameter("max_linear_accel", 1.0)     # m/s^2
        self.declare_parameter("max_angular_accel", 2.0)    # rad/s^2

        # --------------------------------------------------------
        # Noise / disturbance
        # --------------------------------------------------------
        self.declare_parameter("seed", 502)
        self.declare_parameter("linear_noise_std", 0.0)     # m/s
        self.declare_parameter("angular_noise_std", 0.0)    # rad/s

        # Slip event model
        self.declare_parameter("slip_probability", 0.0)
        self.declare_parameter("slip_duration_min", 0.15)
        self.declare_parameter("slip_duration_max", 0.60)
        self.declare_parameter("slip_linear_scale_min", 0.20)
        self.declare_parameter("slip_linear_scale_max", 0.75)
        self.declare_parameter("slip_angular_scale_min", 0.60)
        self.declare_parameter("slip_angular_scale_max", 1.40)
        self.declare_parameter("slip_lateral_velocity_max", 0.15)
        self.declare_parameter("slip_yaw_bias_rate_max", 0.35)

        # --------------------------------------------------------
        # Parameters
        # --------------------------------------------------------
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.input_type = str(self.get_parameter("input_type").value).lower().strip()
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.max_linear_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_angular_accel = float(self.get_parameter("max_angular_accel").value)

        seed = int(self.get_parameter("seed").value)
        self.rng = random.Random(seed)

        self.linear_noise_std = float(self.get_parameter("linear_noise_std").value)
        self.angular_noise_std = float(self.get_parameter("angular_noise_std").value)

        self.slip_probability = float(self.get_parameter("slip_probability").value)
        self.slip_duration_min = float(self.get_parameter("slip_duration_min").value)
        self.slip_duration_max = float(self.get_parameter("slip_duration_max").value)
        self.slip_linear_scale_min = float(self.get_parameter("slip_linear_scale_min").value)
        self.slip_linear_scale_max = float(self.get_parameter("slip_linear_scale_max").value)
        self.slip_angular_scale_min = float(self.get_parameter("slip_angular_scale_min").value)
        self.slip_angular_scale_max = float(self.get_parameter("slip_angular_scale_max").value)
        self.slip_lateral_velocity_max = float(self.get_parameter("slip_lateral_velocity_max").value)
        self.slip_yaw_bias_rate_max = float(self.get_parameter("slip_yaw_bias_rate_max").value)

        # --------------------------------------------------------
        # Runtime state
        # --------------------------------------------------------
        self.state = BodyState()
        self.slip = SlipState()

        self.last_cmd_time: Optional[float] = None
        self.last_tick_time: Optional[float] = None

        self.stats = {
            "cmd_received": 0,
            "odom_published": 0,
            "timeout_brake_count": 0,
            "slip_events": 0,
        }

        # --------------------------------------------------------
        # ROS interfaces
        # --------------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        if self.publish_tf and TransformBroadcaster is not None:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None
            if self.publish_tf:
                self.get_logger().warn("publish_tf=true but tf2_ros is not available. TF disabled.")

        if self.input_type == "twist_stamped":
            self.sub = self.create_subscription(
                TwistStamped,
                self.input_topic,
                self._cmd_stamped_callback,
                10,
            )
        else:
            self.sub = self.create_subscription(
                Twist,
                self.input_topic,
                self._cmd_callback,
                10,
            )

        self.timer = self.create_timer(
            1.0 / max(self.rate_hz, 1.0),
            self._tick,
        )

        self.status_timer = self.create_timer(
            1.0,
            self._publish_status,
        )

        self.get_logger().info(
            f"Synthetic Odom Provider started | input={self.input_topic} "
            f"({self.input_type}) -> odom={self.odom_topic} | rate={self.rate_hz:.1f}Hz"
        )

    # ============================================================
    # Time
    # ============================================================

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ============================================================
    # Command callbacks
    # ============================================================

    def _cmd_callback(self, msg: Twist) -> None:
        self._set_command(msg.linear.x, msg.angular.z)

    def _cmd_stamped_callback(self, msg: TwistStamped) -> None:
        self._set_command(msg.twist.linear.x, msg.twist.angular.z)

    def _set_command(self, vx: float, wz: float) -> None:
        self.state.vx_cmd = finite_or(vx)
        self.state.wz_cmd = finite_or(wz)
        self.last_cmd_time = self._now_sec()
        self.stats["cmd_received"] += 1

    # ============================================================
    # Main integration
    # ============================================================

    def _tick(self) -> None:
        now = self._now_sec()

        if self.last_tick_time is None:
            self.last_tick_time = now
            return

        dt = now - self.last_tick_time
        self.last_tick_time = now

        if not math.isfinite(dt) or dt <= 0.0:
            return

        # Prevent one huge integration jump after pauses.
        dt = min(dt, 0.20)

        target_vx = self.state.vx_cmd
        target_wz = self.state.wz_cmd

        # If command is stale, synthetic body brakes to zero.
        if self.last_cmd_time is None or (now - self.last_cmd_time) > self.cmd_timeout:
            target_vx = 0.0
            target_wz = 0.0
            self.stats["timeout_brake_count"] += 1

        self._maybe_start_slip(now)
        self._update_slip(now)

        # Acceleration-limited command following.
        self.state.vx_eff = self._step_toward(
            self.state.vx_eff,
            target_vx,
            self.max_linear_accel * dt,
        )
        self.state.wz_eff = self._step_toward(
            self.state.wz_eff,
            target_wz,
            self.max_angular_accel * dt,
        )

        vx_body = self.state.vx_eff
        wz_body = self.state.wz_eff
        vy_body = 0.0

        if self.slip.active:
            vx_body *= self.slip.linear_scale
            wz_body = wz_body * self.slip.angular_scale + self.slip.yaw_bias_rate
            vy_body = self.slip.lateral_velocity

        if self.linear_noise_std > 0.0:
            vx_body += self.rng.gauss(0.0, self.linear_noise_std)

        if self.angular_noise_std > 0.0:
            wz_body += self.rng.gauss(0.0, self.angular_noise_std)

        self.state.vy_eff = vy_body

        # Differential-drive / planar body integration.
        c = math.cos(self.state.yaw)
        s = math.sin(self.state.yaw)

        dx_world = (vx_body * c - vy_body * s) * dt
        dy_world = (vx_body * s + vy_body * c) * dt

        self.state.x += dx_world
        self.state.y += dy_world
        self.state.yaw = wrap_angle(self.state.yaw + wz_body * dt)

        self._publish_odom(now, vx_body, vy_body, wz_body)
        self.stats["odom_published"] += 1

    def _step_toward(self, current: float, target: float, max_delta: float) -> float:
        return current + clamp(target - current, -abs(max_delta), abs(max_delta))

    # ============================================================
    # Slip model
    # ============================================================

    def _maybe_start_slip(self, now: float) -> None:
        if self.slip.active:
            return

        p = clamp(self.slip_probability, 0.0, 1.0)
        if self.rng.random() >= p:
            return

        duration = self.rng.uniform(
            min(self.slip_duration_min, self.slip_duration_max),
            max(self.slip_duration_min, self.slip_duration_max),
        )

        linear_scale = self.rng.uniform(
            min(self.slip_linear_scale_min, self.slip_linear_scale_max),
            max(self.slip_linear_scale_min, self.slip_linear_scale_max),
        )

        angular_scale = self.rng.uniform(
            min(self.slip_angular_scale_min, self.slip_angular_scale_max),
            max(self.slip_angular_scale_min, self.slip_angular_scale_max),
        )

        lateral_velocity = self.rng.uniform(
            -abs(self.slip_lateral_velocity_max),
            abs(self.slip_lateral_velocity_max),
        )

        yaw_bias_rate = self.rng.uniform(
            -abs(self.slip_yaw_bias_rate_max),
            abs(self.slip_yaw_bias_rate_max),
        )

        self.slip = SlipState(
            active=True,
            end_time=now + max(0.01, duration),
            linear_scale=linear_scale,
            angular_scale=angular_scale,
            lateral_velocity=lateral_velocity,
            yaw_bias_rate=yaw_bias_rate,
        )

        self.stats["slip_events"] += 1

        self.get_logger().warn(
            "[SYN_ODOM] SLIP_START | "
            f"duration={duration:.2f}s, "
            f"linear_scale={linear_scale:.2f}, "
            f"angular_scale={angular_scale:.2f}, "
            f"lateral_v={lateral_velocity:.3f}, "
            f"yaw_bias={yaw_bias_rate:.3f}"
        )

    def _update_slip(self, now: float) -> None:
        if self.slip.active and now >= self.slip.end_time:
            self.slip = SlipState(active=False)
            self.get_logger().info("[SYN_ODOM] SLIP_END")

    # ============================================================
    # Publishing
    # ============================================================

    def _publish_odom(self, now: float, vx_body: float, vy_body: float, wz_body: float) -> None:
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.state.x
        odom.pose.pose.position.y = self.state.y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(self.state.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.angular.z = wz_body

        # Lightweight covariance defaults.
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.03

        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame

            tf.transform.translation.x = self.state.x
            tf.transform.translation.y = self.state.y
            tf.transform.translation.z = 0.0

            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(tf)

    def _publish_status(self) -> None:
        status = {
            "input_topic": self.input_topic,
            "input_type": self.input_type,
            "odom_topic": self.odom_topic,
            "rate_hz": self.rate_hz,
            "state": {
                "x": self.state.x,
                "y": self.state.y,
                "yaw": self.state.yaw,
                "vx_cmd": self.state.vx_cmd,
                "wz_cmd": self.state.wz_cmd,
                "vx_eff": self.state.vx_eff,
                "vy_eff": self.state.vy_eff,
                "wz_eff": self.state.wz_eff,
            },
            "slip": {
                "active": self.slip.active,
                "linear_scale": self.slip.linear_scale,
                "angular_scale": self.slip.angular_scale,
                "lateral_velocity": self.slip.lateral_velocity,
                "yaw_bias_rate": self.slip.yaw_bias_rate,
            },
            "stats": self.stats,
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


# ============================================================
# Main
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)

    node = SyntheticOdomProvider()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
