#!/usr/bin/env python3
"""
reporter_node.py

Command Integrity Reporter for ros2_kinematic_guard.

Purpose
-------
Bridge NARH Guard internal status into:

1. ROS-native diagnostics:
   /diagnostics
   diagnostic_msgs/DiagnosticArray

2. VDA5050-style fleet telemetry:
   /command_integrity/vda5050_state
   std_msgs/String JSON

This node does not control the robot.
It only translates NARH Guard's command-integrity signal into
standard diagnostic and fleet-consumable reporting formats.

Pipeline
--------
/kinematic_guard/status
        ↓
command_integrity_reporter_node
        ↓
/diagnostics
/command_integrity/vda5050_state
"""

from __future__ import annotations

import json
import math
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# ============================================================
# Helpers
# ============================================================

def finite_or(x: Any, fallback: float = 0.0) -> float:
    try:
        x = float(x)
        return x if math.isfinite(x) else fallback
    except Exception:
        return fallback


def safe_get(data: Dict[str, Any], key: str, default: Any = None) -> Any:
    if not isinstance(data, dict):
        return default
    return data.get(key, default)


def flatten_float_dict(data: Dict[str, Any]) -> Dict[str, float]:
    out = {}
    if not isinstance(data, dict):
        return out

    for k, v in data.items():
        try:
            fv = float(v)
            if math.isfinite(fv):
                out[str(k)] = fv
        except Exception:
            continue

    return out


def make_kv(key: str, value: Any) -> KeyValue:
    kv = KeyValue()
    kv.key = str(key)
    kv.value = str(value)
    return kv


def diag_level_int(level) -> int:
    """
    Convert diagnostic_msgs DiagnosticStatus constants to plain int.

    Some ROS 2 Python generated constants may behave like bytes in certain
    environments. JSON serialization requires plain int/string values.
    """
    if isinstance(level, bytes):
        return int.from_bytes(level, byteorder="little", signed=False)

    try:
        return int(level)
    except Exception:
        return 3  # STALE fallback


def json_safe(obj):
    """
    Recursively convert payload into JSON-serializable primitives.
    """
    if isinstance(obj, bytes):
        if len(obj) == 1:
            return int.from_bytes(obj, byteorder="little", signed=False)
        try:
            return obj.decode("utf-8")
        except Exception:
            return list(obj)

    if obj is None or isinstance(obj, (str, int, float, bool)):
        return obj

    if isinstance(obj, dict):
        return {str(k): json_safe(v) for k, v in obj.items()}

    if isinstance(obj, (list, tuple)):
        return [json_safe(v) for v in obj]

    return str(obj)

# ============================================================
# Reporter Node
# ============================================================

class CommandIntegrityReporterNode(Node):
    def __init__(self):
        super().__init__("command_integrity_reporter_node")

        # --------------------------------------------------------
        # Topics
        # --------------------------------------------------------
        self.declare_parameter("guard_status_topic", "/kinematic_guard/status")
        self.declare_parameter("guard_residual_topic", "/kinematic_guard/residual")

        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("vda_state_topic", "/command_integrity/vda5050_state")
        self.declare_parameter("summary_topic", "/command_integrity/summary")

        # --------------------------------------------------------
        # Identity / metadata
        # --------------------------------------------------------
        self.declare_parameter("vehicle_id", "ros2_kinematic_guard_demo")
        self.declare_parameter("manufacturer", "unknown")
        self.declare_parameter("serial_number", "unknown")

        # --------------------------------------------------------
        # Thresholds
        # Should match kinematic_guard_node launch params.
        # --------------------------------------------------------
        self.declare_parameter("yellow_threshold", 2.5)
        self.declare_parameter("red_threshold", 5.0)

        # Optional TTL for interpreting stale score into estimated age.
        self.declare_parameter("cmd_ttl", 0.25)
        self.declare_parameter("nominal_dt", 0.05)

        # If no guard status arrives within this time, diagnostics become STALE.
        self.declare_parameter("status_timeout", 1.0)

        # Publish rate
        self.declare_parameter("publish_rate_hz", 5.0)

        # --------------------------------------------------------
        # Read parameters
        # --------------------------------------------------------
        self.guard_status_topic = str(self.get_parameter("guard_status_topic").value)
        self.guard_residual_topic = str(self.get_parameter("guard_residual_topic").value)

        self.diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.vda_state_topic = str(self.get_parameter("vda_state_topic").value)
        self.summary_topic = str(self.get_parameter("summary_topic").value)

        self.vehicle_id = str(self.get_parameter("vehicle_id").value)
        self.manufacturer = str(self.get_parameter("manufacturer").value)
        self.serial_number = str(self.get_parameter("serial_number").value)

        self.yellow_threshold = float(self.get_parameter("yellow_threshold").value)
        self.red_threshold = float(self.get_parameter("red_threshold").value)

        self.cmd_ttl = float(self.get_parameter("cmd_ttl").value)
        self.nominal_dt = float(self.get_parameter("nominal_dt").value)
        self.status_timeout = float(self.get_parameter("status_timeout").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        # --------------------------------------------------------
        # Runtime state
        # --------------------------------------------------------
        self.latest_status_payload: Dict[str, Any] = {}
        self.latest_status_receive_time: Optional[float] = None
        self.latest_residual: Optional[float] = None

        # --------------------------------------------------------
        # ROS interfaces
        # --------------------------------------------------------
        self.status_sub = self.create_subscription(
            String,
            self.guard_status_topic,
            self._status_callback,
            10,
        )

        self.residual_sub = self.create_subscription(
            Float64,
            self.guard_residual_topic,
            self._residual_callback,
            10,
        )

        self.diag_pub = self.create_publisher(
            DiagnosticArray,
            self.diagnostics_topic,
            10,
        )

        self.vda_pub = self.create_publisher(
            String,
            self.vda_state_topic,
            10,
        )

        self.summary_pub = self.create_publisher(
            String,
            self.summary_topic,
            10,
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 0.5),
            self._publish_reports,
        )

        self.get_logger().info(
            "Command Integrity Reporter started | "
            f"status={self.guard_status_topic} -> diagnostics={self.diagnostics_topic}, "
            f"vda_state={self.vda_state_topic}"
        )

    # ============================================================
    # Time
    # ============================================================

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ============================================================
    # Callbacks
    # ============================================================

    def _status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("status JSON is not an object")
            self.latest_status_payload = payload
            self.latest_status_receive_time = self._now_sec()
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse guard status JSON: {exc}")

    def _residual_callback(self, msg: Float64) -> None:
        self.latest_residual = finite_or(msg.data, None)

    # ============================================================
    # Classification
    # ============================================================

    def _status_is_stale(self) -> bool:
        if self.latest_status_receive_time is None:
            return True
        return (self._now_sec() - self.latest_status_receive_time) > self.status_timeout

    def _get_r_nar(self, payload: Dict[str, Any]) -> float:
        # Prefer status payload, fallback to residual topic.
        r = safe_get(payload, "r_nar", None)
        if r is not None:
            return finite_or(r, 0.0)

        if self.latest_residual is not None:
            return finite_or(self.latest_residual, 0.0)

        return 0.0

    def _latency_class(self, payload: Dict[str, Any], r_nar: float) -> str:
        status = str(safe_get(payload, "status", "UNKNOWN"))

        if self._status_is_stale():
            return "CRITICAL"

        if status in {"RED_BRAKE", "RESYNCING"}:
            return "CRITICAL"

        if r_nar >= self.red_threshold:
            return "CRITICAL"

        if status in {"YELLOW_SLOWDOWN"}:
            return "DEGRADED"

        if r_nar >= self.yellow_threshold:
            return "DEGRADED"

        return "NORMAL"

    def _diagnostic_level(self, latency_class: str, execution_state: str) -> int:
        if self._status_is_stale():
            return diag_level_int(DiagnosticStatus.STALE)

        if latency_class == "CRITICAL" or execution_state in {"RED_BRAKE", "RESYNCING"}:
            return diag_level_int(DiagnosticStatus.ERROR)

        if latency_class == "DEGRADED" or execution_state == "YELLOW_SLOWDOWN":
            return diag_level_int(DiagnosticStatus.WARN)

            return diag_level_int(DiagnosticStatus.OK)

    def _recommended_vehicle_response(self, execution_state: str, latency_class: str) -> str:
        if self._status_is_stale():
            return "RESYNC_REQUIRED"

        if execution_state == "RESYNCING":
            return "RESYNC_REQUIRED"

        if execution_state == "RED_BRAKE":
            return "HOLD_POSITION"

        if execution_state == "YELLOW_SLOWDOWN" or latency_class == "DEGRADED":
            return "CONTROLLED_DECELERATION"

        if latency_class == "CRITICAL":
            return "HOLD_POSITION"

        return "NONE"

    def _suggested_fleet_action(self, execution_state: str, latency_class: str) -> str:
        if self._status_is_stale():
            return "HOLD_NEW_ORDERS"

        if execution_state == "RESYNCING":
            return "HOLD_NEW_ORDERS"

        if execution_state == "RED_BRAKE":
            return "AVOID_INTERSECTIONS"

        if latency_class == "CRITICAL":
            return "AVOID_INTERSECTIONS"

        if latency_class == "DEGRADED":
            return "REDUCE_ZONE_SPEED"

        return "NONE"

    def _estimate_command_age_ms(self, components: Dict[str, float]) -> float:
        """
        Estimate command age from stale component when available.

        NarhLiteCore stale score is approximately:
            stale_score = (age - cmd_ttl) / cmd_ttl
        for age > cmd_ttl.

        If stale score is zero, return 0.
        """
        stale_score = finite_or(components.get("stale", 0.0), 0.0)
        if stale_score <= 0.0:
            return 0.0
        return 1000.0 * self.cmd_ttl * (1.0 + stale_score)

    def _estimate_jitter_ms(self, debug: Dict[str, float]) -> float:
        dt_cmd = finite_or(debug.get("dt_cmd", self.nominal_dt), self.nominal_dt)
        dt_odom = finite_or(debug.get("dt_odom", self.nominal_dt), self.nominal_dt)

        j1 = abs(dt_cmd - self.nominal_dt)
        j2 = abs(dt_odom - self.nominal_dt)

        return 1000.0 * max(j1, j2)

    # ============================================================
    # Report construction
    # ============================================================

    def _build_command_integrity_payload(self) -> Dict[str, Any]:
        payload = self.latest_status_payload or {}

        r_nar = self._get_r_nar(payload)

        execution_state = str(safe_get(payload, "status", "UNKNOWN"))
        action = str(safe_get(payload, "action", "UNKNOWN"))

        components = flatten_float_dict(safe_get(payload, "components", {}))
        debug = flatten_float_dict(safe_get(payload, "debug", {}))

        latency_class = self._latency_class(payload, r_nar)

        command_age_ms = self._estimate_command_age_ms(components)
        jitter_ms = self._estimate_jitter_ms(debug)

        stale_detected = components.get("stale", 0.0) > 0.0
        burst_detected = (
            components.get("cmd_accel", 0.0) > 0.0
            or components.get("cmd_jerk", 0.0) > 0.0
            or "IMPOSSIBLE_COMMAND_JERK" in safe_get(payload, "reasons", [])
        )
        replay_window_detected = stale_detected and components.get("timeflow", 0.0) > 0.0

        recommended_vehicle_response = self._recommended_vehicle_response(
            execution_state,
            latency_class,
        )

        suggested_fleet_action = self._suggested_fleet_action(
            execution_state,
            latency_class,
        )

        clean_window_count = int(safe_get(payload, "resync_good_count", 0) or 0)
        buffers = safe_get(payload, "buffers", {})
        if isinstance(buffers, dict):
            clean_window_count = int(buffers.get("resync_good_count", clean_window_count) or 0)

        return {
            "timestamp": self._now_sec(),
            "vehicleId": self.vehicle_id,
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number,

            "commandExecutionIntegrity": {
                "residual": r_nar,
                "residualType": "kinematic_consistency",

                "latencyClass": latency_class,
                "commandAgeMs": command_age_ms,
                "jitterMs": jitter_ms,

                "burstDetected": bool(burst_detected),
                "staleCommandDetected": bool(stale_detected),
                "replayWindowDetected": bool(replay_window_detected),

                "executionState": execution_state,
                "guardAction": action,

                "recommendedVehicleResponse": recommended_vehicle_response,
                "suggestedFleetAction": suggested_fleet_action,

                "cleanWindowCount": clean_window_count,

                "rosDiagnosticLevel": diag_level_int(self._diagnostic_level(latency_class, execution_state)),
                "source": "ros2_kinematic_guard"
            },

            "rawGuardStatus": {
                "reasons": safe_get(payload, "reasons", []),
                "components": components,
                "debug": debug,
            }
        }

    def _build_diagnostic_array(self, integrity: Dict[str, Any]) -> DiagnosticArray:
        now_msg = self.get_clock().now().to_msg()

        command_integrity = integrity["commandExecutionIntegrity"]

        latency_class = command_integrity["latencyClass"]
        execution_state = command_integrity["executionState"]
        level = diag_level_int(self._diagnostic_level(latency_class, execution_state))

        status = DiagnosticStatus()
        status.name = "ros2_kinematic_guard/command_execution_integrity"
        status.hardware_id = self.vehicle_id
        status.level = level

        OK = diag_level_int(DiagnosticStatus.OK)
        WARN = diag_level_int(DiagnosticStatus.WARN)
        ERROR = diag_level_int(DiagnosticStatus.ERROR)

        if level == OK:
            status.message = "Command execution integrity normal"
        elif level == WARN:
            status.message = "Command execution integrity degraded"
        elif level == ERROR:
            status.message = "Command execution integrity critical"
        else:
            status.message = "Command execution integrity status stale"
        
        # Main fields
        status.values.append(make_kv("vehicle_id", self.vehicle_id))
        status.values.append(make_kv("residual", command_integrity["residual"]))
        status.values.append(make_kv("residual_type", command_integrity["residualType"]))
        status.values.append(make_kv("latency_class", latency_class))
        status.values.append(make_kv("execution_state", execution_state))
        status.values.append(make_kv("guard_action", command_integrity["guardAction"]))
        status.values.append(make_kv("recommended_vehicle_response", command_integrity["recommendedVehicleResponse"]))
        status.values.append(make_kv("suggested_fleet_action", command_integrity["suggestedFleetAction"]))

        # Timing
        status.values.append(make_kv("command_age_ms", command_integrity["commandAgeMs"]))
        status.values.append(make_kv("jitter_ms", command_integrity["jitterMs"]))

        # Flags
        status.values.append(make_kv("burst_detected", command_integrity["burstDetected"]))
        status.values.append(make_kv("stale_command_detected", command_integrity["staleCommandDetected"]))
        status.values.append(make_kv("replay_window_detected", command_integrity["replayWindowDetected"]))
        status.values.append(make_kv("clean_window_count", command_integrity["cleanWindowCount"]))

        # Residual components
        raw = integrity.get("rawGuardStatus", {})
        components = raw.get("components", {})
        if isinstance(components, dict):
            for k, v in components.items():
                status.values.append(make_kv(f"component.{k}", v))

        diag = DiagnosticArray()
        diag.header.stamp = now_msg
        diag.status.append(status)
        return diag

    # ============================================================
    # Publish
    # ============================================================

    def _publish_reports(self) -> None:
        integrity = self._build_command_integrity_payload()

        diag = self._build_diagnostic_array(integrity)
        self.diag_pub.publish(diag)

        vda_msg = String()
        vda_msg.data = json.dumps(integrity, indent=2)
        self.vda_pub.publish(vda_msg)

        summary = String()
        cei = integrity["commandExecutionIntegrity"]
        summary.data = (
            f"state={cei['executionState']} "
            f"latency={cei['latencyClass']} "
            f"R_NAR={cei['residual']:.3f} "
            f"vehicle_response={cei['recommendedVehicleResponse']} "
            f"fleet_action={cei['suggestedFleetAction']}"
        )
        self.summary_pub.publish(summary)


# ============================================================
# Main
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)

    node = CommandIntegrityReporterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
