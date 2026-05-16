#!/usr/bin/env python3
"""
kinematic_guard_node.py

ros2_kinematic_guard v0.2

A pre-E-stop guard for ROS 2 AMR/AGV systems.

Core idea
---------
Monitor /cmd_vel and /odom.

If the robot's physical response no longer matches the command stream,
publish a compact KinematicStatus JSON and optionally intercept the command
stream by publishing /safe_cmd_vel.

Deployment modes
----------------
observe:
    Passive mode. Publish status only and pass commands through unchanged.
    Good for first-day deployment.

passthrough:
    Explicit inline test mode. Publish /safe_cmd_vel equal to raw /cmd_vel.

guard:
    Active guard mode. YELLOW clamps velocity. RED enters BRAKE_AND_RESYNC.

Main outputs
------------
/kinematic_guard/status      std_msgs/String JSON
/kinematic_guard/residual    std_msgs/Float64
/safe_cmd_vel                geometry_msgs/Twist or TwistStamped
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple, Any
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
    from .narh_lite_core import (
        NarhLiteCore,
        NarhLiteConfig,
        KinematicCommand,
        KinematicFeedback,
        GuardStatus,
        GuardAction,
    )


# ============================================================
# Helpers
# ============================================================

def finite_or(x: Any, fallback: float = 0.0) -> float:
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
    x = finite_or(q.x)
    y = finite_or(q.y)
    z = finite_or(q.z)
    w = finite_or(q.w, 1.0)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def json_safe(obj: Any) -> Any:
    if obj is None or isinstance(obj, (str, int, float, bool)):
        return obj

    if isinstance(obj, bytes):
        try:
            return obj.decode("utf-8")
        except Exception:
            return list(obj)

    if isinstance(obj, dict):
        return {str(k): json_safe(v) for k, v in obj.items()}

    if isinstance(obj, (list, tuple)):
        return [json_safe(v) for v in obj]

    return str(obj)


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
    def __init__(self) -> None:
        super().__init__("kinematic_guard_node")

        # --------------------------------------------------------
        # Topics
        # --------------------------------------------------------
        self.declare_parameter("cmd_input_topic", "/cmd_vel")
        self.declare_parameter("cmd_output_topic", "/safe_cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("status_topic", "/kinematic_guard/status")
        self.declare_parameter("residual_topic", "/kinematic_guard/residual")

        # twist | twist_stamped
        self.declare_parameter("cmd_input_type", "twist")
        self.declare_parameter("cmd_output_type", "twist")

        # --------------------------------------------------------
        # v0.2 deployment mode
        # --------------------------------------------------------
        # observe | passthrough | guard
        self.declare_parameter("mode", "observe")

        # Backward-compatible boolean. If mode is not explicitly set and this is
        # true, you can still switch to guard behavior.
        self.declare_parameter("is_guard_mode_enabled", False)

        self.declare_parameter("lookback_window_ms", 200.0)

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
        self.declare_parameter("slowdown_scale", 0.35)
        self.declare_parameter("resync_required_good_frames", 5)

        # --------------------------------------------------------
        # Node-level behavior
        # --------------------------------------------------------
        self.declare_parameter("enable_brake_and_resync", True)
        self.declare_parameter("publish_zero_until_ready", True)
        self.declare_parameter("flush_buffers_on_red", True)
        self.declare_parameter("replace_zero_header_stamp", True)
        self.declare_parameter("use_receive_time_for_twist", True)
        self.declare_parameter("node_resync_good_frames", 5)

        # Heuristics for demo-friendly cause labeling
        self.declare_parameter("wheel_slip_expected_min_m", 0.03)
        self.declare_parameter("wheel_slip_measured_ratio", 0.25)
        self.declare_parameter("localization_jump_min_m", 0.50)
        self.declare_parameter("localization_jump_ratio", 5.0)

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

        self.mode = str(self.get_parameter("mode").value).lower().strip()
        if self.mode not in {"observe", "passthrough", "guard"}:
            self.get_logger().warn(
                f"Unknown mode='{self.mode}', falling back to observe."
            )
            self.mode = "observe"

        if bool(self.get_parameter("is_guard_mode_enabled").value):
            self.mode = "guard"

        self.lookback_window_ms = float(self.get_parameter("lookback_window_ms").value)

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.status_rate_hz = float(self.get_parameter("status_rate_hz").value)

        self.enable_brake_and_resync = bool(self.get_parameter("enable_brake_and_resync").value)
        self.publish_zero_until_ready = bool(self.get_parameter("publish_zero_until_ready").value)
        self.flush_buffers_on_red = bool(self.get_parameter("flush_buffers_on_red").value)
        self.replace_zero_header_stamp = bool(self.get_parameter("replace_zero_header_stamp").value)
        self.use_receive_time_for_twist = bool(self.get_parameter("use_receive_time_for_twist").value)
        self.node_resync_good_frames = int(self.get_parameter("node_resync_good_frames").value)

        self.wheel_slip_expected_min_m = float(self.get_parameter("wheel_slip_expected_min_m").value)
        self.wheel_slip_measured_ratio = float(self.get_parameter("wheel_slip_measured_ratio").value)
        self.localization_jump_min_m = float(self.get_parameter("localization_jump_min_m").value)
        self.localization_jump_ratio = float(self.get_parameter("localization_jump_ratio").value)

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
        # Sliding window buffers
        # --------------------------------------------------------
        self.cmd_history: Deque[BufferedCommand] = deque(maxlen=512)
        self.odom_history: Deque[BufferedOdom] = deque(maxlen=1024)

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
        self.last_raw_cmd = KinematicCommand(0.0, 0.0)
        self.last_status_payload: Dict[str, Any] = {}

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
            "Kinematic Guard started | "
            f"mode={self.mode} | "
            f"{self.cmd_input_topic} ({self.cmd_input_type}) -> "
            f"{self.cmd_output_topic} ({self.cmd_output_type}) | "
            f"odom={self.odom_topic} | "
            f"lookback_window_ms={self.lookback_window_ms:.1f}"
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

        self.last_raw_cmd = cmd

        self.cmd_history.append(
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

        self.last_raw_cmd = cmd

        self.cmd_history.append(
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

        self.odom_history.append(
            BufferedOdom(
                odom=odom,
                receive_time=now,
            )
        )

        self.stats["odom_received"] += 1

    # ============================================================
    # Window utilities
    # ============================================================

    def _prune_history(self, now: float) -> None:
        cutoff = now - max(0.001, self.lookback_window_ms * 1e-3)

        while len(self.cmd_history) > 2 and self.cmd_history[0].receive_time < cutoff:
            self.cmd_history.popleft()

        while len(self.odom_history) > 2 and self.odom_history[0].receive_time < cutoff:
            self.odom_history.popleft()

    def _get_window_pair(self, history: Deque) -> Optional[Tuple[Any, Any]]:
        if len(history) < 2:
            return None
        return history[0], history[-1]

    # ============================================================
    # Main control loop
    # ============================================================

    def _control_tick(self) -> None:
        now = self._now_sec()
        self._prune_history(now)

        cmd_pair = self._get_window_pair(self.cmd_history)
        odom_pair = self._get_window_pair(self.odom_history)

        if cmd_pair is None or odom_pair is None:
            self.stats["waiting_count"] += 1
            self._set_status_waiting(now)

            output_cmd = self._select_output_cmd(
                raw_cmd=self.last_raw_cmd,
                computed_safe_cmd=KinematicCommand(0.0, 0.0, stamp=now),
                force_zero=False,
            )
            self._publish_safe_cmd(output_cmd)
            return

        cmd_prev_buf, cmd_curr_buf = cmd_pair
        odom_prev_buf, odom_curr_buf = odom_pair

        if self.resync_gate_active and self.resync_started_time is not None:
            if (
                cmd_prev_buf.receive_time < self.resync_started_time
                or cmd_curr_buf.receive_time < self.resync_started_time
                or odom_prev_buf.receive_time < self.resync_started_time
                or odom_curr_buf.receive_time < self.resync_started_time
            ):
                self._set_status_resyncing(
                    now,
                    reason="WAITING_FOR_FRESH_WINDOW",
                    r_nar=self.last_r_nar,
                    components={},
                    debug={},
                    dominant_cause="WAITING_FOR_FRESH_WINDOW",
                )

                output_cmd = self._select_output_cmd(
                    raw_cmd=self.last_raw_cmd,
                    computed_safe_cmd=KinematicCommand(0.0, 0.0, stamp=now),
                    force_zero=True,
                )
                self._publish_safe_cmd(output_cmd)
                return

        result = self.core.evaluate(
            cmd_prev=cmd_prev_buf.cmd,
            cmd_curr=cmd_curr_buf.cmd,
            odom_prev=odom_prev_buf.odom,
            odom_curr=odom_curr_buf.odom,
            now=now,
        )

        self.stats["evaluations"] += 1

        dominant_cause = self._classify_dominant_cause(
            result=result,
            cmd_prev=cmd_prev_buf.cmd,
            cmd_curr=cmd_curr_buf.cmd,
            odom_prev=odom_prev_buf.odom,
            odom_curr=odom_curr_buf.odom,
        )

        # --------------------------------------------------------
        # Resync mode
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
                    dominant_cause=dominant_cause,
                )

                output_cmd = self._select_output_cmd(
                    raw_cmd=self.last_raw_cmd,
                    computed_safe_cmd=KinematicCommand(0.0, 0.0, stamp=now),
                    force_zero=True,
                )
                self._publish_safe_cmd(output_cmd)
                return

            # Resync complete
            self.resync_gate_active = False
            self.resync_started_time = None
            self.resync_good_count = 0
            self.core.reset()
            self.stats["recovered_count"] += 1

            self._flush_buffers_keep_latest()
            self._set_status_recovered(now)

            output_cmd = self._select_output_cmd(
                raw_cmd=self.last_raw_cmd,
                computed_safe_cmd=KinematicCommand(0.0, 0.0, stamp=now),
                force_zero=False,
            )
            self._publish_safe_cmd(output_cmd)
            return

        # --------------------------------------------------------
        # RED path
        # --------------------------------------------------------
        if (
            self.enable_brake_and_resync
            and result.status == GuardStatus.RED_BRAKE
            and result.action == GuardAction.BRAKE_AND_RESYNC
        ):
            self.stats["red_brake_count"] += 1
            self._enter_brake_and_resync(now, result, dominant_cause)

            output_cmd = self._select_output_cmd(
                raw_cmd=self.last_raw_cmd,
                computed_safe_cmd=result.safe_cmd,
                force_zero=True,
            )
            self._publish_safe_cmd(output_cmd)
            return

        # --------------------------------------------------------
        # GREEN / YELLOW path
        # --------------------------------------------------------
        output_cmd = self._select_output_cmd(
            raw_cmd=self.last_raw_cmd,
            computed_safe_cmd=result.safe_cmd,
            force_zero=False,
        )

        self.last_safe_cmd = output_cmd
        self.last_r_nar = result.r_nar
        self.last_status = result.status.value
        self.last_action = result.action.value

        self._update_status_payload_from_result(
            now=now,
            result=result,
            output_cmd=output_cmd,
            dominant_cause=dominant_cause,
        )

        self._publish_safe_cmd(output_cmd)
        self._publish_residual(result.r_nar)

    # ============================================================
    # Cause classification
    # ============================================================

    def _classify_dominant_cause(
        self,
        result,
        cmd_prev: KinematicCommand,
        cmd_curr: KinematicCommand,
        odom_prev: KinematicFeedback,
        odom_curr: KinematicFeedback,
    ) -> str:
        expected_dist, measured_dist, measured_yaw = self._motion_gap(
            cmd_prev,
            cmd_curr,
            odom_prev,
            odom_curr,
        )

        # Localization jump: odom reports a physically large jump relative to command.
        jump_threshold = max(
            self.localization_jump_min_m,
            abs(expected_dist) * self.localization_jump_ratio + 0.20,
        )
        if measured_dist > jump_threshold:
            return "LOCALIZATION_JUMP"

        # Wheel slip: command expects forward motion, odom barely moves.
        if (
            abs(expected_dist) >= self.wheel_slip_expected_min_m
            and measured_dist <= abs(expected_dist) * self.wheel_slip_measured_ratio
        ):
            return "WHEEL_SLIP"

        # Timing / phase clues from core reasons.
        reasons = [str(r).upper() for r in getattr(result, "reasons", [])]
        joined = " ".join(reasons)

        if any(k in joined for k in ["DT", "TTL", "STALE", "TIME"]):
            return "TIMING"

        if any(k in joined for k in ["PHASE"]):
            return "PHASE"

        if any(k in joined for k in ["ODOM", "CMD"]):
            return "CMD_ODOM_MISMATCH"

        # Component fallback.
        components = getattr(result, "components", {}) or {}
        if components:
            try:
                key, value = max(
                    components.items(),
                    key=lambda item: abs(float(item[1])),
                )
                key_upper = str(key).upper()
                if "TIME" in key_upper or "STALE" in key_upper:
                    return "TIMING"
                if "PHASE" in key_upper:
                    return "PHASE"
                if "ODOM" in key_upper or "CMD" in key_upper:
                    return "CMD_ODOM_MISMATCH"
                return key_upper
            except Exception:
                pass

        if result.status == GuardStatus.YELLOW_SLOWDOWN:
            return "SOFT_DEGRADATION"

        if result.status == GuardStatus.RED_BRAKE:
            return "EXECUTION_COLLAPSE"

        return "NONE"

    def _motion_gap(
        self,
        cmd_prev: KinematicCommand,
        cmd_curr: KinematicCommand,
        odom_prev: KinematicFeedback,
        odom_curr: KinematicFeedback,
    ) -> Tuple[float, float, float]:
        dt = finite_or(odom_curr.stamp - odom_prev.stamp, self.core.cfg.default_dt)
        dt = clamp(dt, self.core.cfg.min_dt, self.core.cfg.max_dt)

        v_avg = 0.5 * (finite_or(cmd_prev.vx) + finite_or(cmd_curr.vx))
        expected_dist = v_avg * dt

        dx = finite_or(odom_curr.x) - finite_or(odom_prev.x)
        dy = finite_or(odom_curr.y) - finite_or(odom_prev.y)
        measured_dist = math.sqrt(dx * dx + dy * dy)

        measured_yaw = abs(angle_wrap(finite_or(odom_curr.yaw) - finite_or(odom_prev.yaw)))

        return expected_dist, measured_dist, measured_yaw

    # ============================================================
    # Output command selection
    # ============================================================

    def _select_output_cmd(
        self,
        raw_cmd: KinematicCommand,
        computed_safe_cmd: KinematicCommand,
        force_zero: bool,
    ) -> KinematicCommand:
        now = self._now_sec()

        if self.mode == "guard":
            if force_zero:
                return KinematicCommand(0.0, 0.0, stamp=now)
            return computed_safe_cmd

        # observe / passthrough: never alter command stream.
        return KinematicCommand(
            vx=finite_or(raw_cmd.vx),
            wz=finite_or(raw_cmd.wz),
            stamp=now,
            seq=getattr(raw_cmd, "seq", None),
        )

    # ============================================================
    # Brake & Resync
    # ============================================================

    def _enter_brake_and_resync(self, now: float, result, dominant_cause: str) -> None:
        self.resync_gate_active = True
        self.resync_started_time = now
        self.resync_good_count = 0
        self.stats["resync_count"] += 1

        self.last_r_nar = result.r_nar
        self.last_status = GuardStatus.RESYNCING.value
        self.last_action = GuardAction.BRAKE_AND_RESYNC.value
        self.last_safe_cmd = result.safe_cmd

        self._update_status_payload_from_result(
            now=now,
            result=result,
            output_cmd=KinematicCommand(0.0, 0.0, stamp=now),
            dominant_cause=dominant_cause,
            override_status=GuardStatus.RESYNCING.value,
            override_action=GuardAction.BRAKE_AND_RESYNC.value,
            extra={
                "brakeAndResync": True,
                "resyncStartedTime": now,
            },
        )

        if self.flush_buffers_on_red:
            self.cmd_history.clear()
            self.odom_history.clear()
            self.core.reset()
            self.stats["buffer_flush_count"] += 1

        self.get_logger().warn(
            "[KINEMATIC_GUARD] RED_BRAKE -> RESYNCING | "
            f"mode={self.mode} | "
            f"R={result.r_nar:.3f} | "
            f"cause={dominant_cause} | "
            f"reasons={getattr(result, 'reasons', [])}"
        )

    def _flush_buffers_keep_latest(self) -> None:
        latest_cmd = self.cmd_history[-1] if len(self.cmd_history) > 0 else None
        latest_odom = self.odom_history[-1] if len(self.odom_history) > 0 else None

        self.cmd_history.clear()
        self.odom_history.clear()

        if latest_cmd is not None:
            self.cmd_history.append(latest_cmd)

        if latest_odom is not None:
            self.odom_history.append(latest_odom)

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

    def _causal_alignment_from_status(self, status: str) -> str:
        if status in {"GREEN", "RECOVERED"}:
            return "ALIGNED"
        if status in {"YELLOW_SLOWDOWN"}:
            return "DEGRADED"
        if status in {"RED_BRAKE", "RESYNCING"}:
            return "BROKEN"
        return "UNKNOWN"

    def _guard_action_for_mode(self, computed_action: str, force_observe: bool = False) -> str:
        if self.mode == "guard":
            return computed_action
        if force_observe:
            return "OBSERVE_ONLY"
        return "PASS_THROUGH"

    def _set_status_waiting(self, now: float) -> None:
        self.last_status = "WAITING_FOR_DATA"
        self.last_action = "NONE"
        self.last_r_nar = 0.0

        self.last_status_payload = self._build_status_payload(
            now=now,
            status=self.last_status,
            residual=0.0,
            causal_alignment="UNKNOWN",
            dominant_cause="WAITING_FOR_DATA",
            guard_action="NONE",
            output_cmd=self.last_raw_cmd,
            reason="Need at least two cmd samples and two odom samples.",
            components={},
            debug={},
            extra={},
        )

    def _set_status_resyncing(
        self,
        now: float,
        reason: str,
        r_nar: Optional[float] = None,
        components: Optional[Dict] = None,
        debug: Optional[Dict] = None,
        dominant_cause: str = "RESYNC_REQUIRED",
    ) -> None:
        self.last_status = GuardStatus.RESYNCING.value
        self.last_action = GuardAction.BRAKE_AND_RESYNC.value
        self.last_r_nar = finite_or(r_nar, self.last_r_nar)

        output_cmd = self._select_output_cmd(
            raw_cmd=self.last_raw_cmd,
            computed_safe_cmd=KinematicCommand(0.0, 0.0, stamp=now),
            force_zero=True,
        )

        self.last_status_payload = self._build_status_payload(
            now=now,
            status=self.last_status,
            residual=self.last_r_nar,
            causal_alignment="BROKEN",
            dominant_cause=dominant_cause,
            guard_action=self._guard_action_for_mode("BRAKE_AND_RESYNC"),
            output_cmd=output_cmd,
            reason=reason,
            components=components or {},
            debug=debug or {},
            extra={},
        )

        self._publish_residual(self.last_r_nar)

    def _set_status_recovered(self, now: float) -> None:
        self.last_status = GuardStatus.RECOVERED.value
        self.last_action = GuardAction.PASS.value
        self.last_r_nar = 0.0

        self.last_status_payload = self._build_status_payload(
            now=now,
            status=self.last_status,
            residual=0.0,
            causal_alignment="ALIGNED",
            dominant_cause="NONE",
            guard_action="RECOVERED",
            output_cmd=KinematicCommand(0.0, 0.0, stamp=now),
            reason="Fresh command/odom window restored.",
            components={},
            debug={},
            extra={},
        )

    def _update_status_payload_from_result(
        self,
        now: float,
        result,
        output_cmd: KinematicCommand,
        dominant_cause: str,
        override_status: Optional[str] = None,
        override_action: Optional[str] = None,
        extra: Optional[Dict] = None,
    ) -> None:
        status = override_status or result.status.value
        computed_action = override_action or result.action.value

        causal_alignment = self._causal_alignment_from_status(status)
        guard_action = self._guard_action_for_mode(
            computed_action,
            force_observe=(self.mode == "observe"),
        )

        self.last_status_payload = self._build_status_payload(
            now=now,
            status=status,
            residual=float(result.r_nar),
            causal_alignment=causal_alignment,
            dominant_cause=dominant_cause,
            guard_action=guard_action,
            output_cmd=output_cmd,
            reason=";".join([str(r) for r in getattr(result, "reasons", [])]),
            components=getattr(result, "components", {}) or {},
            debug=getattr(result, "debug", {}) or {},
            extra={
                "velocityScale": float(getattr(result, "velocity_scale", 1.0)),
                "recommendedGuardAction": computed_action,
                **(extra or {}),
            },
        )

    def _build_status_payload(
        self,
        now: float,
        status: str,
        residual: float,
        causal_alignment: str,
        dominant_cause: str,
        guard_action: str,
        output_cmd: KinematicCommand,
        reason: str,
        components: Dict,
        debug: Dict,
        extra: Dict,
    ) -> Dict[str, Any]:
        payload = {
            "timestamp": now,
            "status": status,
            "residual": finite_or(residual),
            "r_nar": finite_or(residual),  # backward-compatible alias
            "causalAlignment": causal_alignment,
            "dominantCause": dominant_cause,
            "guardAction": guard_action,
            "action": guard_action,  # backward-compatible alias
            "mode": self.mode,
            "controlInterceptionEnabled": self.mode == "guard",
            "safeCmd": {
                "linear_vx": finite_or(output_cmd.vx),
                "angular_wz": finite_or(output_cmd.wz),
            },
            "safe_cmd": {  # backward-compatible alias
                "vx": finite_or(output_cmd.vx),
                "wz": finite_or(output_cmd.wz),
            },
            "cleanWindowCount": self.resync_good_count,
            "requiredCleanWindowCount": self.node_resync_good_frames,
            "lookbackWindowMs": self.lookback_window_ms,
            "reason": reason,
            "components": {
                str(k): finite_or(v) for k, v in (components or {}).items()
            },
            "debug": {
                str(k): json_safe(v) for k, v in (debug or {}).items()
            },
            "buffers": self._buffer_status(),
            "stats": self.stats,
        }

        if extra:
            payload.update(json_safe(extra))

        return payload

    def _buffer_status(self) -> Dict[str, Any]:
        return {
            "cmdBufferSize": len(self.cmd_history),
            "odomBufferSize": len(self.odom_history),
            "resyncGateActive": self.resync_gate_active,
            "resyncStartedTime": self.resync_started_time,
            "resyncGoodCount": self.resync_good_count,
        }

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(json_safe(self.last_status_payload), separators=(",", ":"))
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
