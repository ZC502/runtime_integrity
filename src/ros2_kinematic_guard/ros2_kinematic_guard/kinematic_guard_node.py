#!/usr/bin/env python3
"""
kinematic_guard_node.py

ROS 2 adapter for NARH Guard.

Project:
    ros2_kinematic_guard

Formal name:
    NARH-based Kinematic Guard for ROS 2

Core line:
    ROS 2 transmits messages.
    NARH Guard ensures those messages are still executable.

Tagline:
    It does not fix the network.
    It prevents bad network timing from becoming dangerous robot motion.

Purpose
-------
This node is the "main gate" of ros2_kinematic_guard.

It subscribes to:
    /cmd_vel_in
    /odom

It publishes:
    /cmd_vel_out
    /kinematic_guard/status
    /kinematic_guard/residual

It wraps the ROS-free narh_lite_core.py.

Important design
----------------
- Rate decoupling:
  /cmd_vel is usually 10-20Hz.
  /odom is usually 50-100Hz.
  Therefore odom_callback only updates latest odom buffer.
  The guard runs in a timer and evaluates the latest two cmd + odom samples.

- Strict state maintenance:
  The core needs prev/curr command and prev/curr feedback.
  This node maintains cmd_buffer and odom_buffer.

- Brake & Resync:
  When RED_BRAKE occurs, old buffers are flushed so poisoned old data
  cannot trigger post-recovery residual shaking.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple
import json
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64

try:
    from narh_lite_core import (
        NarhLiteCore,
        NarhLiteConfig,
        KinematicCommand,
        KinematicFeedback,
        GuardStatus,
        GuardAction,
    )
except ImportError:
    # If used inside a Python package:
    from .narh_lite_core import (
        NarhLiteCore,
        NarhLiteConfig,
        KinematicCommand,
        KinematicFeedback,
        GuardStatus,
        GuardAction,
    )


# ============================================================
# Small helpers
# ============================================================

def finite_or(x, fallback: float = 0.0) -> float:
    try:
        x = float(x)
        return x if math.isfinite(x) else fallback
    except Exception:
        return fallback


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def is_zero_stamp(stamp) -> bool:
    return int(stamp.sec) == 0 and int(stamp.nanosec) == 0


def yaw_from_quaternion(q) -> float:
    """
    Convert geometry_msgs Quaternion to yaw.
    """
    x = finite_or(q.x)
    y = finite_or(q.y)
    z = finite_or(q.z)
    w = finite_or(q.w, 1.0)

    # yaw from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


# ============================================================
# Buffers
# ============================================================

@dataclass
class BufferedCommand:
    cmd: KinematicCommand
    receive_time: float
    source_type: str


@dataclass
class BufferedOdom:
    odom: KinematicFeedback
    receive_time: float


# ============================================================
# Node
# ============================================================

class KinematicGuardNode(Node):
    def __init__(self):
        super().__init__("kinematic_guard_node")

        # --------------------------------------------------------
        # Topics
        # --------------------------------------------------------
        self.declare_parameter("cmd_input_topic", "/cmd_vel_in")
        self.declare_parameter("cmd_output_topic", "/cmd_vel_out")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("status_topic", "/kinematic_guard/status")
        self.declare_parameter("residual_topic", "/kinematic_guard/residual")

        # twist | twist_stamped
        self.declare_parameter("cmd_input_type", "twist")
        self.declare_parameter("cmd_output_type", "twist")

        # --------------------------------------------------------
        # Loop timing
        # --------------------------------------------------------
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("status_rate_hz", 5.0)

        # --------------------------------------------------------
        # Core tuning
        # --------------------------------------------------------
        self.declare_parameter("default_dt", 0.05)
        self.declare_parameter("min_dt", 0.001)
        self.declare_parameter("max_dt", 0.50)
        self.declare_parameter("cmd_ttl", 0.25)
        self.declare_parameter("phase_tolerance", 0.08)

        self.declare_parameter("max_linear_accel", 0.8)
        self.declare_parameter("max_angular_accel", 1.5)
        self.declare_parameter("max_linear_jerk", 3.0)
        self.declare_parameter("max_angular_jerk", 6.0)

        self.declare_parameter("position_tolerance", 0.03)
        self.declare_parameter("yaw_tolerance", 0.08)
        self.declare_parameter("lateral_tolerance", 0.03)

        self.declare_parameter("yellow_threshold", 2.5)
        self.declare_parameter("red_threshold", 5.0)
        self.declare_parameter("slowdown_scale", 0.45)
        self.declare_parameter("resync_required_good_frames", 5)

        # --------------------------------------------------------
        # Node-level behavior
        # --------------------------------------------------------
        self.declare_parameter("enable_brake_and_resync", True)
        self.declare_parameter("publish_zero_until_ready", True)
        self.declare_parameter("flush_buffers_on_red", True)
        self.declare_parameter("replace_zero_header_stamp", True)
        self.declare_parameter("use_receive_time_for_twist", True)

        # During resync, require this many clean evaluations before release.
        self.declare_parameter("node_resync_good_frames", 5)

        # --------------------------------------------------------
        # Read params
        # --------------------------------------------------------
        self.cmd_input_topic = str(self.get_parameter("cmd_input_topic").value)
        self.cmd_output_topic = str(self.get_parameter("cmd_output_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.residual_topic = str(self.get_parameter("residual_topic").value)

        self.cmd_input_type = str(self.get_parameter("cmd_input_type").value).lower().strip()
        self.cmd_output_type = str(self.get_parameter("cmd_output_type").value).lower().strip()

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.status_rate_hz = float(self.get_parameter("status_rate_hz").value)

        self.enable_brake_and_resync = bool(self.get_parameter("enable_brake_and_resync").value)
        self.publish_zero_until_ready = bool(self.get_parameter("publish_zero_until_ready").value)
        self.flush_buffers_on_red = bool(self.get_parameter("flush_buffers_on_red").value)
        self.replace_zero_header_stamp = bool(self.get_parameter("replace_zero_header_stamp").value)
        self.use_receive_time_for_twist = bool(self.get_parameter("use_receive_time_for_twist").value)
        self.node_resync_good_frames = int(self.get_parameter("node_resync_good_frames").value)

        cfg = NarhLiteConfig(
            default_dt=float(self.get_parameter("default_dt").value),
            min_dt=float(self.get_parameter("min_dt").value),
            max_dt=float(self.get_parameter("max_dt").value),
            cmd_ttl=float(self.get_parameter("cmd_ttl").value),
            phase_tolerance=float(self.get_parameter("phase_tolerance").value),

            max_linear_accel=float(self.get_parameter("max_linear_accel").value),
            max_angular_accel=float(self.get_parameter("max_angular_accel").value),
            max_linear_jerk=float(self.get_parameter("max_linear_jerk").value),
            max_angular_jerk=float(self.get_parameter("max_angular_jerk").value),

            position_tolerance=float(self.get_parameter("position_tolerance").value),
            yaw_tolerance=float(self.get_parameter("yaw_tolerance").value),
            lateral_tolerance=float(self.get_parameter("lateral_tolerance").value),

            yellow_threshold=float(self.get_parameter("yellow_threshold").value),
            red_threshold=float(self.get_parameter("red_threshold").value),
            slowdown_scale=float(self.get_parameter("slowdown_scale").value),
            resync_required_good_frames=int(self.get_parameter("resync_required_good_frames").value),
        )

        self.core = NarhLiteCore(cfg)

        # --------------------------------------------------------
        # Buffers
        # --------------------------------------------------------
        self.cmd_buffer: Deque[BufferedCommand] = deque(maxlen=2)
        self.odom_buffer: Deque[BufferedOdom] = deque(maxlen=2)

        self.cmd_seq = 0

        # --------------------------------------------------------
        # Guard state
        # --------------------------------------------------------
        self.resync_gate_active = False
        self.resync_started_time: Optional[float] = None
        self.resync_good_count = 0

        self.last_status = "WAITING_FOR_DATA"
        self.last_action = "NONE"
        self.last_r_nar = 0.0
        self.last_safe_cmd = KinematicCommand(0.0, 0.0)
        self.last_status_payload: Dict = {}

        self.stats = {
            "cmd_received": 0,
            "odom_received": 0,
            "evaluations": 0,
            "safe_cmd_published": 0,
            "red_brake_count": 0,
            "resync_count": 0,
            "recovered_count": 0,
            "buffer_flush_count": 0,
            "waiting_count": 0,
        }

        # --------------------------------------------------------
        # Publishers
        # --------------------------------------------------------
        if self.cmd_output_type == "twist_stamped":
            self.safe_cmd_pub = self.create_publisher(
                TwistStamped,
                self.cmd_output_topic,
                10,
            )
        else:
            self.safe_cmd_pub = self.create_publisher(
                Twist,
                self.cmd_output_topic,
                10,
            )

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.residual_pub = self.create_publisher(Float64, self.residual_topic, 10)

        # --------------------------------------------------------
        # Subscribers
        # --------------------------------------------------------
        if self.cmd_input_type == "twist_stamped":
            self.cmd_sub = self.create_subscription(
                TwistStamped,
                self.cmd_input_topic,
                self._cmd_stamped_callback,
                10,
            )
        else:
            self.cmd_sub = self.create_subscription(
                Twist,
                self.cmd_input_topic,
                self._cmd_callback,
                10,
            )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            20,
        )

        # --------------------------------------------------------
        # Timers
        # --------------------------------------------------------
        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1.0),
            self._control_tick,
        )

        self.status_timer = self.create_timer(
            1.0 / max(self.status_rate_hz, 0.5),
            self._publish_status,
        )

        self.get_logger().info(
            "NARH Guard started | "
            f"{self.cmd_input_topic} ({self.cmd_input_type}) -> "
            f"{self.cmd_output_topic} ({self.cmd_output_type}) | "
            f"odom={self.odom_topic}"
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

        if self.use_receive_time_for_twist:
            stamp = now
        else:
            stamp = None

        self.cmd_seq += 1

        cmd = KinematicCommand(
            vx=finite_or(msg.linear.x),
            wz=finite_or(msg.angular.z),
            stamp=stamp,
            seq=self.cmd_seq,
        )

        self.cmd_buffer.append(
            BufferedCommand(
                cmd=cmd,
                receive_time=now,
                source_type="twist",
            )
        )

        self.stats["cmd_received"] += 1

    def _cmd_stamped_callback(self, msg: TwistStamped) -> None:
        now = self._now_sec()

        if self.replace_zero_header_stamp and is_zero_stamp(msg.header.stamp):
            stamp = now
        else:
            stamp = stamp_to_sec(msg.header.stamp)

        self.cmd_seq += 1

        cmd = KinematicCommand(
            vx=finite_or(msg.twist.linear.x),
            wz=finite_or(msg.twist.angular.z),
            stamp=stamp,
            seq=self.cmd_seq,
        )

        self.cmd_buffer.append(
            BufferedCommand(
                cmd=cmd,
                receive_time=now,
                source_type="twist_stamped",
            )
        )

        self.stats["cmd_received"] += 1

    def _odom_callback(self, msg: Odometry) -> None:
        now = self._now_sec()

        if self.replace_zero_header_stamp and is_zero_stamp(msg.header.stamp):
            stamp = now
        else:
            stamp = stamp_to_sec(msg.header.stamp)

        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        odom = KinematicFeedback(
            x=finite_or(msg.pose.pose.position.x),
            y=finite_or(msg.pose.pose.position.y),
            yaw=yaw,
            vx=finite_or(msg.twist.twist.linear.x),
            wz=finite_or(msg.twist.twist.angular.z),
            stamp=stamp,
        )

        self.odom_buffer.append(
            BufferedOdom(
                odom=odom,
                receive_time=now,
            )
        )

        self.stats["odom_received"] += 1

    # ============================================================
    # Main control loop
    # ============================================================

    def _control_tick(self) -> None:
        now = self._now_sec()

        if len(self.cmd_buffer) < 2 or len(self.odom_buffer) < 2:
            self.stats["waiting_count"] += 1
            self._set_status_waiting(now)

            if self.publish_zero_until_ready:
                self._publish_safe_cmd(KinematicCommand(0.0, 0.0, stamp=now))

            return

        cmd_prev_buf, cmd_curr_buf = self.cmd_buffer[0], self.cmd_buffer[1]
        odom_prev_buf, odom_curr_buf = self.odom_buffer[0], self.odom_buffer[1]

        # --------------------------------------------------------
        # Resync gate:
        # After RED_BRAKE, only accept samples that arrived after the red event.
        # --------------------------------------------------------
        if self.resync_gate_active and self.resync_started_time is not None:
            if (
                cmd_prev_buf.receive_time < self.resync_started_time
                or cmd_curr_buf.receive_time < self.resync_started_time
                or odom_prev_buf.receive_time < self.resync_started_time
                or odom_curr_buf.receive_time < self.resync_started_time
            ):
                self._set_status_resyncing(now, reason="WAITING_FOR_FRESH_WINDOW")
                self._publish_safe_cmd(KinematicCommand(0.0, 0.0, stamp=now))
                return

        result = self.core.evaluate(
            cmd_prev=cmd_prev_buf.cmd,
            cmd_curr=cmd_curr_buf.cmd,
            odom_prev=odom_prev_buf.odom,
            odom_curr=odom_curr_buf.odom,
            now=now,
        )

        self.stats["evaluations"] += 1

        # --------------------------------------------------------
        # If node is in resync mode, require several clean frames
        # before releasing control.
        # --------------------------------------------------------
        if self.resync_gate_active:
            if result.r_nar < self.core.cfg.yellow_threshold and result.status in (
                GuardStatus.GREEN,
                GuardStatus.RECOVERED,
            ):
                self.resync_good_count += 1
            else:
                self.resync_good_count = 0

            if self.resync_good_count < self.node_resync_good_frames:
                self._set_status_resyncing(
                    now,
                    reason="RESYNC_GOOD_FRAMES_REQUIRED",
                    r_nar=result.r_nar,
                    components=result.components,
                    debug=result.debug,
                )
                self._publish_safe_cmd(KinematicCommand(0.0, 0.0, stamp=now))
                return

            # Resync complete
            self.resync_gate_active = False
            self.resync_started_time = None
            self.resync_good_count = 0
            self.core.reset()
            self.stats["recovered_count"] += 1

            self._flush_buffers_keep_latest()
            self._set_status_recovered(now)
            self._publish_safe_cmd(KinematicCommand(0.0, 0.0, stamp=now))
            return

        # --------------------------------------------------------
        # Normal path
        # --------------------------------------------------------
        if (
            self.enable_brake_and_resync
            and result.status == GuardStatus.RED_BRAKE
            and result.action == GuardAction.BRAKE_AND_RESYNC
        ):
            self.stats["red_brake_count"] += 1
            self._enter_brake_and_resync(now, result)
            self._publish_safe_cmd(result.safe_cmd)
            return

        # GREEN / YELLOW path
        self.last_safe_cmd = result.safe_cmd
        self.last_r_nar = result.r_nar
        self.last_status = result.status.value
        self.last_action = result.action.value

        self._update_status_payload_from_result(now, result)
        self._publish_safe_cmd(result.safe_cmd)
        self._publish_residual(result.r_nar)

    # ============================================================
    # Brake & Resync
    # ============================================================

    def _enter_brake_and_resync(self, now: float, result) -> None:
        self.resync_gate_active = True
        self.resync_started_time = now
        self.resync_good_count = 0
        self.stats["resync_count"] += 1

        self.last_r_nar = result.r_nar
        self.last_status = GuardStatus.RED_BRAKE.value
        self.last_action = GuardAction.BRAKE_AND_RESYNC.value
        self.last_safe_cmd = result.safe_cmd

        self._update_status_payload_from_result(
            now,
            result,
            override_status=GuardStatus.RED_BRAKE.value,
            override_action=GuardAction.BRAKE_AND_RESYNC.value,
            extra={
                "brake_and_resync": True,
                "resync_started_time": now,
            },
        )

        if self.flush_buffers_on_red:
            self.cmd_buffer.clear()
            self.odom_buffer.clear()
            self.core.reset()
            self.stats["buffer_flush_count"] += 1

        self.get_logger().warn(
            f"[NARH_GUARD] RED_BRAKE -> RESYNCING | "
            f"R_NAR={result.r_nar:.3f} | reasons={result.reasons}"
        )

    def _flush_buffers_keep_latest(self) -> None:
        """
        Flush old samples but keep the most recent one if available.

        This avoids toxic pre-RED samples while allowing the node to restart
        quickly once a new sample arrives.
        """
        latest_cmd = self.cmd_buffer[-1] if len(self.cmd_buffer) > 0 else None
        latest_odom = self.odom_buffer[-1] if len(self.odom_buffer) > 0 else None

        self.cmd_buffer.clear()
        self.odom_buffer.clear()

        if latest_cmd is not None:
            self.cmd_buffer.append(latest_cmd)

        if latest_odom is not None:
            self.odom_buffer.append(latest_odom)

        self.stats["buffer_flush_count"] += 1

    # ============================================================
    # Publishing commands
    # ============================================================

    def _publish_safe_cmd(self, cmd: KinematicCommand) -> None:
        self.last_safe_cmd = cmd

        if self.cmd_output_type == "twist_stamped":
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = finite_or(cmd.vx)
            msg.twist.angular.z = finite_or(cmd.wz)
            self.safe_cmd_pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = finite_or(cmd.vx)
            msg.angular.z = finite_or(cmd.wz)
            self.safe_cmd_pub.publish(msg)

        self.stats["safe_cmd_published"] += 1

    def _publish_residual(self, r_nar: float) -> None:
        msg = Float64()
        msg.data = finite_or(r_nar)
        self.residual_pub.publish(msg)

    # ============================================================
    # Status
    # ============================================================

    def _set_status_waiting(self, now: float) -> None:
        self.last_status = "WAITING_FOR_DATA"
        self.last_action = "NONE"
        self.last_r_nar = 0.0

        self.last_status_payload = {
            "timestamp": now,
            "status": self.last_status,
            "action": self.last_action,
            "r_nar": self.last_r_nar,
            "safe_cmd": {
                "vx": 0.0,
                "wz": 0.0,
            },
            "reason": "Need at least two cmd samples and two odom samples.",
            "buffers": self._buffer_status(),
            "stats": self.stats,
        }

    def _set_status_resyncing(
        self,
        now: float,
        reason: str,
        r_nar: Optional[float] = None,
        components: Optional[Dict] = None,
        debug: Optional[Dict] = None,
    ) -> None:
        self.last_status = GuardStatus.RESYNCING.value
        self.last_action = GuardAction.BRAKE_AND_RESYNC.value
        self.last_r_nar = finite_or(r_nar, self.last_r_nar)

        self.last_status_payload = {
            "timestamp": now,
            "status": self.last_status,
            "action": self.last_action,
            "r_nar": self.last_r_nar,
            "safe_cmd": {
                "vx": 0.0,
                "wz": 0.0,
            },
            "reason": reason,
            "resync_gate_active": self.resync_gate_active,
            "resync_good_count": self.resync_good_count,
            "resync_good_required": self.node_resync_good_frames,
            "components": components or {},
            "debug": debug or {},
            "buffers": self._buffer_status(),
            "stats": self.stats,
        }

        self._publish_residual(self.last_r_nar)

    def _set_status_recovered(self, now: float) -> None:
        self.last_status = GuardStatus.RECOVERED.value
        self.last_action = GuardAction.PASS.value
        self.last_r_nar = 0.0

        self.last_status_payload = {
            "timestamp": now,
            "status": self.last_status,
            "action": self.last_action,
            "r_nar": self.last_r_nar,
            "safe_cmd": {
                "vx": 0.0,
                "wz": 0.0,
            },
            "reason": "Fresh command/odom window restored.",
            "buffers": self._buffer_status(),
            "stats": self.stats,
        }

    def _update_status_payload_from_result(
        self,
        now: float,
        result,
        override_status: Optional[str] = None,
        override_action: Optional[str] = None,
        extra: Optional[Dict] = None,
    ) -> None:
        status = override_status or result.status.value
        action = override_action or result.action.value

        self.last_status_payload = {
            "timestamp": now,
            "status": status,
            "action": action,
            "r_nar": float(result.r_nar),
            "velocity_scale": float(result.velocity_scale),
            "safe_cmd": {
                "vx": float(result.safe_cmd.vx),
                "wz": float(result.safe_cmd.wz),
            },
            "reasons": list(result.reasons),
            "components": {
                k: float(v) for k, v in result.components.items()
            },
            "debug": {
                k: float(v) for k, v in result.debug.items()
                if isinstance(v, (int, float))
            },
            "buffers": self._buffer_status(),
            "stats": self.stats,
        }

        if extra:
            self.last_status_payload.update(extra)

    def _buffer_status(self) -> Dict:
        return {
            "cmd_buffer_size": len(self.cmd_buffer),
            "odom_buffer_size": len(self.odom_buffer),
            "resync_gate_active": self.resync_gate_active,
            "resync_started_time": self.resync_started_time,
            "resync_good_count": self.resync_good_count,
        }

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self.last_status_payload)
        self.status_pub.publish(msg)

    # ============================================================
    # Main
    # ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)

    node = KinematicGuardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
