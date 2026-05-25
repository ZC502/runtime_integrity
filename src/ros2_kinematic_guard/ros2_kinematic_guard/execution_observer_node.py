#!/usr/bin/env python3
"""
execution_observer_node.py

runtime_integrity v0.3-alpha

Observe-only ROS 2 execution-integrity observer.

Core idea
---------
Subscribe to a command stream and odometry stream:

    /cmd_vel + /odom

Evaluate whether the robot's physical motion remains consistent with
the recent command stream over a short sliding window.

Publish the result as standard ROS diagnostics:

    /diagnostics
        runtime_integrity/execution_integrity

This node does NOT:
- publish /safe_cmd_vel
- intercept commands
- modify controllers
- modify Nav2 BTs
- modify base drivers

It only observes and reports.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, Optional, Tuple
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

try:
    from .narh_lite_core import (
        NarhLiteCore,
        NarhLiteConfig,
        KinematicCommand,
        KinematicFeedback,
        GuardStatus,
        GuardAction,
    )
except ImportError:
    from narh_lite_core import (
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
        value = float(x)
        return value if math.isfinite(value) else fallback
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


def enum_value(x: Any) -> str:
    if hasattr(x, "value"):
        return str(x.value)
    return str(x)


def key_value(key: str, value: Any) -> KeyValue:
    return KeyValue(key=str(key), value=str(value))


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

class ExecutionObserverNode(Node):
    def __init__(self) -> None:
        super().__init__("execution_observer_node")

        # --------------------------------------------------------
        # Topics
        # --------------------------------------------------------
        self.declare_parameter("cmd_input_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("diagnostics_topic", "/diagnostics")

        # twist | twist_stamped
        self.declare_parameter("cmd_input_type", "twist")

        # observe-only by design
        self.declare_parameter("mode", "observe")

        self.declare_parameter("data_stale_warn_sec", 1.0)
        self.declare_parameter("data_stale_error_sec", 3.0)
        
        # --------------------------------------------------------
        # Diagnostics identity
        # --------------------------------------------------------
        self.declare_parameter(
            "diagnostic_name",
            "runtime_integrity/execution_integrity",
        )
        self.declare_parameter(
            "hardware_id",
            "physics_boundary_observer",
        )

        # --------------------------------------------------------
        # Window and rates
        # --------------------------------------------------------
        self.declare_parameter("lookback_window_ms", 200.0)
        self.declare_parameter("evaluation_rate_hz", 20.0)
        self.declare_parameter("diagnostics_rate_hz", 10.0)

        # Optional expected publish periods.
        # If <= 0, arrival jitter is estimated from interval-to-interval variation.
        self.declare_parameter("expected_cmd_period_ms", 0.0)
        self.declare_parameter("expected_odom_period_ms", 0.0)

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
        # Cause-labeling heuristics
        # --------------------------------------------------------
        self.declare_parameter("wheel_slip_expected_min_m", 0.03)
        self.declare_parameter("wheel_slip_measured_ratio", 0.25)

        self.declare_parameter("localization_jump_min_m", 0.50)
        self.declare_parameter("localization_jump_ratio", 5.0)

        # --------------------------------------------------------
        # Timestamp behavior
        # --------------------------------------------------------
        self.declare_parameter("replace_zero_header_stamp", True)
        self.declare_parameter("use_receive_time_for_twist", True)

        # --------------------------------------------------------
        # Read parameters
        # --------------------------------------------------------
        self.cmd_input_topic = str(self.get_parameter("cmd_input_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)

        self.cmd_input_type = str(self.get_parameter("cmd_input_type").value).lower().strip()
        self.mode = str(self.get_parameter("mode").value).lower().strip()

        if self.mode != "observe":
            self.get_logger().warn(
                f"execution_observer_node is observe-only. "
                f"Ignoring requested mode='{self.mode}' and using mode='observe'."
            )
            self.mode = "observe"

        self.diagnostic_name = str(self.get_parameter("diagnostic_name").value)
        self.hardware_id = str(self.get_parameter("hardware_id").value)

        self.lookback_window_ms = float(self.get_parameter("lookback_window_ms").value)
        self.evaluation_rate_hz = float(self.get_parameter("evaluation_rate_hz").value)
        self.diagnostics_rate_hz = float(self.get_parameter("diagnostics_rate_hz").value)

        self.expected_cmd_period_ms = float(self.get_parameter("expected_cmd_period_ms").value)
        self.expected_odom_period_ms = float(self.get_parameter("expected_odom_period_ms").value)

        self.wheel_slip_expected_min_m = float(
            self.get_parameter("wheel_slip_expected_min_m").value
        )
        self.wheel_slip_measured_ratio = float(
            self.get_parameter("wheel_slip_measured_ratio").value
        )
        self.localization_jump_min_m = float(
            self.get_parameter("localization_jump_min_m").value
        )
        self.localization_jump_ratio = float(
            self.get_parameter("localization_jump_ratio").value
        )

        self.replace_zero_header_stamp = bool(
            self.get_parameter("replace_zero_header_stamp").value
        )
        self.use_receive_time_for_twist = bool(
            self.get_parameter("use_receive_time_for_twist").value
        )

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
            resync_required_good_frames=int(
                self.get_parameter("resync_required_good_frames").value
            ),
        )

        self.core = NarhLiteCore(cfg)

        self.data_stale_warn_sec = float(self.get_parameter("data_stale_warn_sec").value)
        self.data_stale_error_sec = float(self.get_parameter("data_stale_error_sec").value)
        
        # --------------------------------------------------------
        # Runtime buffers
        # --------------------------------------------------------
        self.cmd_history: Deque[BufferedCommand] = deque(maxlen=512)
        self.odom_history: Deque[BufferedOdom] = deque(maxlen=1024)

        self.cmd_seq = 0

        # Arrival jitter trackers
        self.last_cmd_receive_time: Optional[float] = None
        self.last_odom_receive_time: Optional[float] = None
        self.last_cmd_interval_ms: Optional[float] = None
        self.last_odom_interval_ms: Optional[float] = None
        self.cmd_arrival_jitter_ms = 0.0
        self.odom_arrival_jitter_ms = 0.0

        # Last diagnostic payload
        self.last_payload: Dict[str, Any] = self._waiting_payload()

        self.stats = {
            "cmd_received": 0,
            "odom_received": 0,
            "evaluations": 0,
            "waiting_count": 0,
            "diagnostics_published": 0,
        }

        # --------------------------------------------------------
        # ROS interfaces
        # --------------------------------------------------------
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            self.diagnostics_topic,
            10,
        )

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

        self.evaluation_timer = self.create_timer(
            1.0 / max(self.evaluation_rate_hz, 1.0),
            self._evaluation_tick,
        )

        self.diagnostics_timer = self.create_timer(
            1.0 / max(self.diagnostics_rate_hz, 0.5),
            self._publish_diagnostics,
        )

        self.get_logger().info(
            "runtime_integrity execution observer started | "
            f"cmd={self.cmd_input_topic} ({self.cmd_input_type}) | "
            f"odom={self.odom_topic} | "
            f"diagnostics={self.diagnostics_topic} | "
            f"lookback_window_ms={self.lookback_window_ms:.1f}"
        )

    # ============================================================
    # Time
    # ============================================================

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ============================================================
    # Subscriptions
    # ============================================================

    def _update_cmd_jitter(self, now: float) -> None:
        if self.last_cmd_receive_time is None:
            self.last_cmd_receive_time = now
            return

        interval_ms = (now - self.last_cmd_receive_time) * 1000.0
        self.last_cmd_receive_time = now

        if self.expected_cmd_period_ms > 0.0:
            self.cmd_arrival_jitter_ms = abs(interval_ms - self.expected_cmd_period_ms)
        elif self.last_cmd_interval_ms is not None:
            self.cmd_arrival_jitter_ms = abs(interval_ms - self.last_cmd_interval_ms)

        self.last_cmd_interval_ms = interval_ms

    def _update_odom_jitter(self, now: float) -> None:
        if self.last_odom_receive_time is None:
            self.last_odom_receive_time = now
            return

        interval_ms = (now - self.last_odom_receive_time) * 1000.0
        self.last_odom_receive_time = now

        if self.expected_odom_period_ms > 0.0:
            self.odom_arrival_jitter_ms = abs(interval_ms - self.expected_odom_period_ms)
        elif self.last_odom_interval_ms is not None:
            self.odom_arrival_jitter_ms = abs(interval_ms - self.last_odom_interval_ms)

        self.last_odom_interval_ms = interval_ms

    def _cmd_callback(self, msg: Twist) -> None:
        now = self._now_sec()
        self._update_cmd_jitter(now)

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
        self._update_cmd_jitter(now)

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
        self._update_odom_jitter(now)

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

    @staticmethod
    def _get_window_pair(history: Deque) -> Optional[Tuple[Any, Any]]:
        if len(history) < 2:
            return None
        return history[0], history[-1]

    def _stale_data_payload_if_needed(self, now: float) -> Optional[Dict[str, Any]]:
        cmd_age = None
        odom_age = None

        if self.last_cmd_receive_time is not None:
            cmd_age = now - self.last_cmd_receive_time

        if self.last_odom_receive_time is not None:
            odom_age = now - self.last_odom_receive_time

        missing = []
        stale = []

        if self.last_cmd_receive_time is None:
            missing.append("cmd")
        elif cmd_age is not None and cmd_age >= self.data_stale_warn_sec:
            stale.append("cmd")

        if self.last_odom_receive_time is None:
            missing.append("odom")
        elif odom_age is not None and odom_age >= self.data_stale_warn_sec:
            stale.append("odom")

    if not missing and not stale:
        return None

    max_age = max(
        cmd_age if cmd_age is not None else 0.0,
        odom_age if odom_age is not None else 0.0,
    )

    if missing:
        status = "WAITING_FOR_DATA"
        cause = "MISSING_STREAM"
        level_error = False
    elif max_age >= self.data_stale_error_sec:
        status = "STALE_DATA_TIMEOUT"
        cause = "STALE_DATA"
        level_error = True
    else:
        status = "STALE_DATA"
        cause = "STALE_DATA"
        level_error = False

    return {
        "timestamp": now,
        "status": status,
        "engineStatusRaw": status,
        "dominantCause": cause,
        "totalResidual": 0.0,
        "r_nar": 0.0,
        "causalAlignment": "UNKNOWN",
        "mode": "observe",
        "operatorAttentionRequired": level_error,

        "wheelSlipIndex": 0.0,
        "localizationJumpMetric": 0.0,
        "cmdOdomResidual": 0.0,
        "timeflowResidual": 0.0,
        "phaseResidual": 0.0,
        "staleCommandScore": 0.0,
        "cmdAccelResidual": 0.0,
        "cmdJerkResidual": 0.0,
        "cmdArrivalJitterMs": self.cmd_arrival_jitter_ms,
        "odomArrivalJitterMs": self.odom_arrival_jitter_ms,

        "cmdAgeSec": -1.0 if cmd_age is None else cmd_age,
        "odomAgeSec": -1.0 if odom_age is None else odom_age,

        "cmdTopic": self.cmd_input_topic,
        "odomTopic": self.odom_topic,
        "lookbackWindowMs": self.lookback_window_ms,

        "cmdBufferSize": len(self.cmd_history),
        "odomBufferSize": len(self.odom_history),

        "statsCmdReceived": int(self.stats["cmd_received"]),
        "statsOdomReceived": int(self.stats["odom_received"]),
        "statsEvaluations": int(self.stats["evaluations"]),
    }
        
    # ============================================================
    # Evaluation
    # ============================================================

    def _evaluation_tick(self) -> None:
        now = self._now_sec()

        stale_payload = self._stale_data_payload_if_needed(now)
        if stale_payload is not None:
            self.last_payload = stale_payload
            return

        self._prune_history(now)

        cmd_pair = self._get_window_pair(self.cmd_history)
        odom_pair = self._get_window_pair(self.odom_history)

        if cmd_pair is None or odom_pair is None:
            self.stats["waiting_count"] += 1
            self.last_payload = self._waiting_payload()
            return

        cmd_prev_buf, cmd_curr_buf = cmd_pair
        odom_prev_buf, odom_curr_buf = odom_pair

        try:
            result = self.core.evaluate(
                cmd_prev=cmd_prev_buf.cmd,
                cmd_curr=cmd_curr_buf.cmd,
                odom_prev=odom_prev_buf.odom,
                odom_curr=odom_curr_buf.odom,
                now=now,
            )
        except Exception as exc:
            self.get_logger().error(
                f"NARH evaluation failed, publishing EVALUATION_ERROR diagnostic: {exc}"
            )
            self.last_payload = self._evaluation_error_payload(now, exc)
            return

        self.stats["evaluations"] += 1

        expected_dist, measured_dist, measured_yaw = self._motion_gap(
            cmd_prev=cmd_prev_buf.cmd,
            cmd_curr=cmd_curr_buf.cmd,
            odom_prev=odom_prev_buf.odom,
            odom_curr=odom_curr_buf.odom,
        )

        dominant_cause = self._classify_dominant_cause(
            result=result,
            expected_dist=expected_dist,
            measured_dist=measured_dist,
        )

        self.last_payload = self._build_payload(
            now=now,
            result=result,
            dominant_cause=dominant_cause,
            expected_dist=expected_dist,
            measured_dist=measured_dist,
            measured_yaw=measured_yaw,
            engine_status_raw = enum_value(result.status)
            action_hint = enum_value(result.action)
            status = self._observer_status(result.status)
        )

    def _evaluation_error_payload(self, now: float, exc: Exception) -> Dict[str, Any]:
        return {
            "timestamp": now,
            "status": "EVALUATION_ERROR",
            "engineStatusRaw": "EVALUATION_ERROR",
            "dominantCause": "EVALUATION_EXCEPTION",
            "totalResidual": 0.0,
            "r_nar": 0.0,
            "causalAlignment": "UNKNOWN",
            "mode": "observe",
            "operatorAttentionRequired": True,

            "wheelSlipIndex": 0.0,
            "localizationJumpMetric": 0.0,
            "cmdOdomResidual": 0.0,
            "timeflowResidual": 0.0,
            "phaseResidual": 0.0,
            "staleCommandScore": 0.0,
            "cmdAccelResidual": 0.0,
            "cmdJerkResidual": 0.0,
            "cmdArrivalJitterMs": self.cmd_arrival_jitter_ms,
            "odomArrivalJitterMs": self.odom_arrival_jitter_ms,

            "cmdTopic": self.cmd_input_topic,
            "odomTopic": self.odom_topic,
            "lookbackWindowMs": self.lookback_window_ms,

            "cmdBufferSize": len(self.cmd_history),
            "odomBufferSize": len(self.odom_history),

            "exceptionType": type(exc).__name__,
            "exceptionMessage": str(exc),

            "statsCmdReceived": int(self.stats["cmd_received"]),
            "statsOdomReceived": int(self.stats["odom_received"]),
            "statsEvaluations": int(self.stats["evaluations"]),
        }

    # ============================================================
    # Cause classification and derived metrics
    # ============================================================

    def _motion_gap(
        self,
        cmd_prev: KinematicCommand,
        cmd_curr: KinematicCommand,
        odom_prev: KinematicFeedback,
        odom_curr: KinematicFeedback,
    ) -> Tuple[float, float, float]:
        dt = finite_or(
            (odom_curr.stamp or 0.0) - (odom_prev.stamp or 0.0),
            self.core.cfg.default_dt,
        )
        dt = clamp(dt, self.core.cfg.min_dt, self.core.cfg.max_dt)

        v_avg = 0.5 * (finite_or(cmd_prev.vx) + finite_or(cmd_curr.vx))
        expected_dist = v_avg * dt

        dx = finite_or(odom_curr.x) - finite_or(odom_prev.x)
        dy = finite_or(odom_curr.y) - finite_or(odom_prev.y)
        measured_dist = math.sqrt(dx * dx + dy * dy)

        measured_yaw = abs(angle_wrap(finite_or(odom_curr.yaw) - finite_or(odom_prev.yaw)))

        return expected_dist, measured_dist, measured_yaw

    def _classify_dominant_cause(
        self,
        result,
        expected_dist: float,
        measured_dist: float,
    ) -> str:
        expected_abs = abs(expected_dist)

        jump_threshold = max(
            self.localization_jump_min_m,
            expected_abs * self.localization_jump_ratio + 0.20,
        )
        if measured_dist > jump_threshold:
            return "LOCALIZATION_JUMP"

        if (
            expected_abs >= self.wheel_slip_expected_min_m
            and measured_dist <= expected_abs * self.wheel_slip_measured_ratio
        ):
            return "WHEEL_SLIP"

        reasons = [str(r).upper() for r in getattr(result, "reasons", [])]
        joined = " ".join(reasons)

        if any(k in joined for k in ["DT", "TTL", "STALE", "TIME"]):
            return "TIMING"

        if any(k in joined for k in ["PHASE"]):
            return "PHASE"

        if any(k in joined for k in ["ODOM", "CMD"]):
            return "CMD_ODOM_MISMATCH"

        components = getattr(result, "components", {}) or {}
        if components:
            try:
                key, _ = max(
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
            except Exception:
                pass

        status = enum_value(result.status)
        if status in {"YELLOW_SLOWDOWN"}:
            return "SOFT_DEGRADATION"

        if status in {"RED_BRAKE", "RESYNCING"}:
            return "EXECUTION_COLLAPSE"

        return "NONE"

    def _wheel_slip_index(self, expected_dist: float, measured_dist: float, result) -> float:
        expected_abs = abs(expected_dist)
        if expected_abs < self.wheel_slip_expected_min_m:
            return 0.0

        slip_ratio = max(0.0, 1.0 - measured_dist / max(expected_abs, 1e-6))
        cmd_odom_score = finite_or(
            getattr(result, "components", {}).get("cmd_odom", 0.0),
            0.0,
        )
        return slip_ratio * max(1.0, cmd_odom_score)

    def _localization_jump_metric(
        self,
        expected_dist: float,
        measured_dist: float,
        result,
    ) -> float:
        expected_abs = abs(expected_dist)
        jump_excess = max(0.0, measured_dist - (expected_abs + self.localization_jump_min_m))
        phase_score = finite_or(
            getattr(result, "components", {}).get("phase", 0.0),
            0.0,
        )
        return jump_excess + phase_score

    # ============================================================
    # Payload and diagnostics
    # ============================================================

    def _causal_alignment(self, status: str) -> str:
        if status in {"GREEN", "RECOVERED"}:
            return "ALIGNED"
        if status in {"YELLOW_SLOWDOWN", "DEGRADED"}:
            return "DEGRADED"
        if status in {"RED_BRAKE", "RESYNCING"}:
            return "BROKEN"
        return "UNKNOWN"

    def _observer_status(self, result_status: Any) -> str:
        raw = enum_value(result_status)

        # In observe-only mode, the physical state is broken, but the node does
        # not brake. Report the execution-integrity state, not a control action.
        if raw == "RED_BRAKE":
            return "RESYNCING"

        return raw

    def _diagnostic_level(self, status: str) -> int:
        if status in {"GREEN", "RECOVERED"}:
            return DiagnosticStatus.OK

        if status in {"YELLOW_SLOWDOWN", "DEGRADED"}:
            return DiagnosticStatus.WARN

        if status in {"WAITING_FOR_DATA", "UNKNOWN"}:
            return DiagnosticStatus.WARN

        return DiagnosticStatus.ERROR

    def _waiting_payload(self) -> Dict[str, Any]:
        return {
            "status": "WAITING_FOR_DATA",
            "dominantCause": "WAITING_FOR_DATA",
            "totalResidual": 0.0,
            "causalAlignment": "UNKNOWN",
            "mode": "observe",
            "operatorAttentionRequired": False,
            "wheelSlipIndex": 0.0,
            "localizationJumpMetric": 0.0,
            "cmdOdomResidual": 0.0,
            "timeflowResidual": 0.0,
            "phaseResidual": 0.0,
            "staleCommandScore": 0.0,
            "cmdAccelResidual": 0.0,
            "cmdJerkResidual": 0.0,
            "cmdArrivalJitterMs": self.cmd_arrival_jitter_ms,
            "odomArrivalJitterMs": self.odom_arrival_jitter_ms,
            "cmdTopic": self.cmd_input_topic,
            "odomTopic": self.odom_topic,
            "lookbackWindowMs": self.lookback_window_ms,
            "cmdBufferSize": len(self.cmd_history),
            "odomBufferSize": len(self.odom_history),
        }

    def _build_payload(
        self,
        now: float,
        result,
        dominant_cause: str,
        expected_dist: float,
        measured_dist: float,
        measured_yaw: float,
        "engineStatusRaw": engine_status_raw,
        "actionHint": action_hint,
    ) -> Dict[str, Any]:
        status = self._observer_status(result.status)
        causal_alignment = self._causal_alignment(status)

        components = getattr(result, "components", {}) or {}
        debug = getattr(result, "debug", {}) or {}

        total_residual = finite_or(getattr(result, "r_nar", 0.0))
        cmd_odom_residual = finite_or(components.get("cmd_odom", 0.0))
        timeflow_residual = finite_or(components.get("timeflow", 0.0))
        phase_residual = finite_or(components.get("phase", 0.0))
        stale_score = finite_or(components.get("stale", 0.0))
        cmd_accel = finite_or(components.get("cmd_accel", 0.0))
        cmd_jerk = finite_or(components.get("cmd_jerk", 0.0))

        wheel_slip_index = self._wheel_slip_index(
            expected_dist=expected_dist,
            measured_dist=measured_dist,
            result=result,
        )

        localization_jump_metric = self._localization_jump_metric(
            expected_dist=expected_dist,
            measured_dist=measured_dist,
            result=result,
        )

        diagnostic_level = self._diagnostic_level(status)

        return {
            "timestamp": now,
            "status": status,
            "dominantCause": dominant_cause,
            "totalResidual": total_residual,
            "r_nar": total_residual,
            "causalAlignment": causal_alignment,
            "mode": "observe",
            "operatorAttentionRequired": diagnostic_level == DiagnosticStatus.ERROR,

            "wheelSlipIndex": wheel_slip_index,
            "localizationJumpMetric": localization_jump_metric,
            "cmdOdomResidual": cmd_odom_residual,
            "timeflowResidual": timeflow_residual,
            "phaseResidual": phase_residual,
            "staleCommandScore": stale_score,
            "cmdAccelResidual": cmd_accel,
            "cmdJerkResidual": cmd_jerk,
            "cmdArrivalJitterMs": self.cmd_arrival_jitter_ms,
            "odomArrivalJitterMs": self.odom_arrival_jitter_ms,

            "expectedDistanceM": finite_or(expected_dist),
            "measuredDistanceM": finite_or(measured_dist),
            "measuredYawRad": finite_or(measured_yaw),

            "cmdTopic": self.cmd_input_topic,
            "odomTopic": self.odom_topic,
            "lookbackWindowMs": self.lookback_window_ms,

            "cmdBufferSize": len(self.cmd_history),
            "odomBufferSize": len(self.odom_history),

            "reasons": list(getattr(result, "reasons", []) or []),
            "debugExpectedDx": finite_or(debug.get("expected_dx", 0.0)),
            "debugMeasuredDxLocal": finite_or(debug.get("measured_dx_local", 0.0)),
            "debugMeasuredDyLocal": finite_or(debug.get("measured_dy_local", 0.0)),
            "debugLinearGap": finite_or(debug.get("linear_gap", 0.0)),
            "debugAngularGap": finite_or(debug.get("angular_gap", 0.0)),
            "debugLateralGap": finite_or(debug.get("lateral_gap", 0.0)),

            "statsCmdReceived": int(self.stats["cmd_received"]),
            "statsOdomReceived": int(self.stats["odom_received"]),
            "statsEvaluations": int(self.stats["evaluations"]),
        }

    def _publish_diagnostics(self) -> None:
        payload = self.last_payload

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        st = DiagnosticStatus()
        st.name = self.diagnostic_name
        st.hardware_id = self.hardware_id

        status = str(payload.get("status", "UNKNOWN"))
        cause = str(payload.get("dominantCause", "NONE"))
        level = self._diagnostic_level(status)

        st.level = level

        if cause in {"NONE", ""}:
            st.message = f"{status}: SYSTEM_ALIGNED"
        else:
            st.message = f"{status}: {cause}"

        st.values = [
            key_value("status", payload.get("status", "")),
            key_value("dominantCause", payload.get("dominantCause", "")),
            key_value("totalResidual", f"{finite_or(payload.get('totalResidual', 0.0)):.6f}"),
            key_value("causalAlignment", payload.get("causalAlignment", "")),
            key_value("mode", payload.get("mode", "observe")),
            key_value(
                "operatorAttentionRequired",
                str(bool(payload.get("operatorAttentionRequired", False))).lower(),
            ),

            key_value("wheelSlipIndex", f"{finite_or(payload.get('wheelSlipIndex', 0.0)):.6f}"),
            key_value(
                "localizationJumpMetric",
                f"{finite_or(payload.get('localizationJumpMetric', 0.0)):.6f}",
            ),
            key_value("cmdOdomResidual", f"{finite_or(payload.get('cmdOdomResidual', 0.0)):.6f}"),
            key_value("timeflowResidual", f"{finite_or(payload.get('timeflowResidual', 0.0)):.6f}"),
            key_value("phaseResidual", f"{finite_or(payload.get('phaseResidual', 0.0)):.6f}"),
            key_value("staleCommandScore", f"{finite_or(payload.get('staleCommandScore', 0.0)):.6f}"),
            key_value("cmdAccelResidual", f"{finite_or(payload.get('cmdAccelResidual', 0.0)):.6f}"),
            key_value("cmdJerkResidual", f"{finite_or(payload.get('cmdJerkResidual', 0.0)):.6f}"),
            key_value("cmdArrivalJitterMs", f"{finite_or(payload.get('cmdArrivalJitterMs', 0.0)):.2f}"),
            key_value("odomArrivalJitterMs", f"{finite_or(payload.get('odomArrivalJitterMs', 0.0)):.2f}"),

            key_value("expectedDistanceM", f"{finite_or(payload.get('expectedDistanceM', 0.0)):.6f}"),
            key_value("measuredDistanceM", f"{finite_or(payload.get('measuredDistanceM', 0.0)):.6f}"),
            key_value("measuredYawRad", f"{finite_or(payload.get('measuredYawRad', 0.0)):.6f}"),

            key_value("cmdTopic", payload.get("cmdTopic", self.cmd_input_topic)),
            key_value("odomTopic", payload.get("odomTopic", self.odom_topic)),
            key_value("lookbackWindowMs", payload.get("lookbackWindowMs", self.lookback_window_ms)),

            key_value("cmdBufferSize", payload.get("cmdBufferSize", 0)),
            key_value("odomBufferSize", payload.get("odomBufferSize", 0)),

            key_value("statsCmdReceived", payload.get("statsCmdReceived", 0)),
            key_value("statsOdomReceived", payload.get("statsOdomReceived", 0)),
            key_value("statsEvaluations", payload.get("statsEvaluations", 0)),

            key_value("engineStatusRaw", payload.get("engineStatusRaw", payload.get("status", ""))),
            key_value("actionHint", payload.get("actionHint", "")),
            key_value("cmdAgeSec", f"{finite_or(payload.get('cmdAgeSec', 0.0)):.3f}"),
            key_value("odomAgeSec", f"{finite_or(payload.get('odomAgeSec', 0.0)):.3f}"),
            key_value("exceptionType", payload.get("exceptionType", "")),
            key_value("exceptionMessage", payload.get("exceptionMessage", "")),
        ]

        reasons = payload.get("reasons", [])
        if reasons:
            st.values.append(key_value("reasons", ",".join(str(x) for x in reasons)))

        msg.status.append(st)
        self.diagnostics_pub.publish(msg)
        self.stats["diagnostics_published"] += 1


# ============================================================
# Main
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)

    node = ExecutionObserverNode()

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
