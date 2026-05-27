#!/usr/bin/env python3
"""
diagnostics_to_csv_labeler.py

runtime_integrity failure-mining utility.

Listens to /diagnostics and writes runtime_integrity execution-integrity
events into a CSV file for Sim2Real failure mining, hard-negative collection,
OOD analysis, and RL reward/cost shaping.

Typical use:

    ros2 run ros2_kinematic_guard diagnostics_to_csv_labeler --ros-args \
      -p output_csv:=runtime_integrity_failures.csv

During rosbag replay:

    # Terminal 1
    ros2 bag play your_failure_run --clock

    # Terminal 2
    ros2 run ros2_kinematic_guard execution_observer_node --ros-args \
      -p cmd_input_topic:=/cmd_vel \
      -p odom_topic:=/odom \
      -p use_sim_time:=true

    # Terminal 3
    ros2 run ros2_kinematic_guard diagnostics_to_csv_labeler --ros-args \
      -p output_csv:=labels.csv \
      -p use_sim_time:=true
"""

from __future__ import annotations

import csv
import os
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray


def level_to_int(level: Any) -> int:
    """
    ROS 2 Humble may represent DiagnosticStatus.level as bytes like b"\\x02".
    Normalize it to an int.
    """
    if isinstance(level, (bytes, bytearray)):
        if len(level) == 0:
            return 0
        return int(level[0])
    try:
        return int(level)
    except Exception:
        return 0


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def kv_to_dict(values) -> Dict[str, str]:
    return {str(kv.key): str(kv.value) for kv in values}


def f(value: Optional[str], default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


class DiagnosticsToCsvLabeler(Node):
    def __init__(self) -> None:
        super().__init__("diagnostics_to_csv_labeler")

        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("target_name", "runtime_integrity/execution_integrity")
        self.declare_parameter("output_csv", "runtime_integrity_failure_labels.csv")

        # Default: only record non-OK events.
        # Set include_ok:=true if you also want GREEN samples as negative examples.
        self.declare_parameter("include_ok", False)

        # Default: record WARN and ERROR.
        # 0 = OK, 1 = WARN, 2 = ERROR.
        self.declare_parameter("min_level", 1)

        # By default, missing/stale streams are useful operational labels too.
        # Set include_stream_health:=false if you only want physical failure labels.
        self.declare_parameter("include_stream_health", True)

        self.diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.target_name = str(self.get_parameter("target_name").value)
        self.output_csv = str(self.get_parameter("output_csv").value)
        self.include_ok = bool(self.get_parameter("include_ok").value)
        self.min_level = int(self.get_parameter("min_level").value)
        self.include_stream_health = bool(self.get_parameter("include_stream_health").value)

        self.rows_written = 0

        self.fieldnames = [
            "ros_time_sec",
            "wall_time_sec",
            "diagnostic_name",
            "diagnostic_level_int",
            "diagnostic_level_name",
            "message",

            "status",
            "engineStatusRaw",
            "actionHint",
            "dominantCause",
            "dominantCauseCandidate",
            "causalAlignment",
            "operatorAttentionRequired",

            "totalResidual",
            "cmdOdomResidual",
            "wheelSlipIndex",
            "localizationJumpMetric",
            "timeflowResidual",
            "phaseResidual",
            "staleCommandScore",
            "cmdAccelResidual",
            "cmdJerkResidual",
            "cmdArrivalJitterMs",
            "odomArrivalJitterMs",

            "expectedDistanceM",
            "measuredDistanceM",
            "measuredYawRad",

            "cmdTopic",
            "odomTopic",
            "lookbackWindowMs",
            "cmdBufferSize",
            "odomBufferSize",
            "staleStreams",
            "missingStreams",

            "statsCmdReceived",
            "statsOdomReceived",
            "statsEvaluations",
        ]

        os.makedirs(os.path.dirname(os.path.abspath(self.output_csv)), exist_ok=True)

        self.csv_file = open(self.output_csv, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.writer.writeheader()
        self.csv_file.flush()

        self.sub = self.create_subscription(
            DiagnosticArray,
            self.diagnostics_topic,
            self.callback,
            10,
        )

        self.get_logger().info(
            "runtime_integrity CSV labeler started | "
            f"diagnostics={self.diagnostics_topic} | "
            f"target={self.target_name} | "
            f"output={self.output_csv} | "
            f"include_ok={self.include_ok} | min_level={self.min_level}"
        )

    def should_record(self, level_int: int, kv: Dict[str, str]) -> bool:
        if self.include_ok:
            return True

        if level_int < self.min_level:
            return False

        if self.include_stream_health:
            return True

        cause = kv.get("dominantCause", "")
        status = kv.get("status", "")

        if cause in {"MISSING_STREAM", "STALE_DATA"}:
            return False

        if status in {"MISSING_STREAM_TIMEOUT", "STALE_DATA", "STALE_DATA_TIMEOUT"}:
            return False

        return True

    def callback(self, msg: DiagnosticArray) -> None:
        ros_time_sec = stamp_to_sec(msg.header.stamp)
        wall_time_sec = self.get_clock().now().nanoseconds * 1e-9

        for status_msg in msg.status:
            if status_msg.name != self.target_name:
                continue

            level_int = level_to_int(status_msg.level)
            kv = kv_to_dict(status_msg.values)

            if not self.should_record(level_int, kv):
                continue

            row = {
                "ros_time_sec": f"{ros_time_sec:.9f}",
                "wall_time_sec": f"{wall_time_sec:.9f}",
                "diagnostic_name": status_msg.name,
                "diagnostic_level_int": level_int,
                "diagnostic_level_name": kv.get("diagnosticLevelName", ""),
                "message": status_msg.message,

                "status": kv.get("status", ""),
                "engineStatusRaw": kv.get("engineStatusRaw", ""),
                "actionHint": kv.get("actionHint", ""),
                "dominantCause": kv.get("dominantCause", ""),
                "dominantCauseCandidate": kv.get("dominantCauseCandidate", ""),
                "causalAlignment": kv.get("causalAlignment", ""),
                "operatorAttentionRequired": kv.get("operatorAttentionRequired", ""),

                "totalResidual": f(kv.get("totalResidual")),
                "cmdOdomResidual": f(kv.get("cmdOdomResidual")),
                "wheelSlipIndex": f(kv.get("wheelSlipIndex")),
                "localizationJumpMetric": f(kv.get("localizationJumpMetric")),
                "timeflowResidual": f(kv.get("timeflowResidual")),
                "phaseResidual": f(kv.get("phaseResidual")),
                "staleCommandScore": f(kv.get("staleCommandScore")),
                "cmdAccelResidual": f(kv.get("cmdAccelResidual")),
                "cmdJerkResidual": f(kv.get("cmdJerkResidual")),
                "cmdArrivalJitterMs": f(kv.get("cmdArrivalJitterMs")),
                "odomArrivalJitterMs": f(kv.get("odomArrivalJitterMs")),

                "expectedDistanceM": f(kv.get("expectedDistanceM")),
                "measuredDistanceM": f(kv.get("measuredDistanceM")),
                "measuredYawRad": f(kv.get("measuredYawRad")),

                "cmdTopic": kv.get("cmdTopic", ""),
                "odomTopic": kv.get("odomTopic", ""),
                "lookbackWindowMs": kv.get("lookbackWindowMs", ""),
                "cmdBufferSize": kv.get("cmdBufferSize", ""),
                "odomBufferSize": kv.get("odomBufferSize", ""),
                "staleStreams": kv.get("staleStreams", ""),
                "missingStreams": kv.get("missingStreams", ""),

                "statsCmdReceived": kv.get("statsCmdReceived", ""),
                "statsOdomReceived": kv.get("statsOdomReceived", ""),
                "statsEvaluations": kv.get("statsEvaluations", ""),
            }

            self.writer.writerow(row)
            self.csv_file.flush()
            self.rows_written += 1

            if self.rows_written % 10 == 0:
                self.get_logger().info(
                    f"CSV labeler wrote {self.rows_written} rows -> {self.output_csv}"
                )

    def destroy_node(self) -> bool:
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiagnosticsToCsvLabeler()

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
