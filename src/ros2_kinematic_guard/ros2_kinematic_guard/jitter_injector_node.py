#!/usr/bin/env python3
"""
jitter_injector_node.py

Bad-WiFi / 5G stress injector for ros2_kinematic_guard.

Project:
    ros2_kinematic_guard

Formal name:
    NARH-based Kinematic Guard for ROS 2

Tagline:
    It does not fix the network.
    It prevents bad network timing from becoming dangerous robot motion.

Purpose
-------
This node intentionally corrupts a clean /cmd_vel stream by injecting:

- random delay
- burst release
- packet drop
- duplicate command
- stale replay
- out-of-order delivery

It publishes the corrupted stream to /cmd_vel_jittered.

This is the "examiner" for NARH Guard.

Typical pipeline
----------------
/cmd_vel_raw
    -> jitter_injector_node.py
    -> /cmd_vel_jittered
    -> kinematic_guard_node.py
    -> /kinematic_guard/safe_cmd_vel

Optional:
    If use_demo_cmd=true, this node generates its own demo /cmd_vel_raw.
"""

from __future__ import annotations

import copy
import heapq
import json
import math
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ============================================================
# Profiles
# ============================================================

PROFILES: Dict[str, Dict[str, float]] = {
    "clean": {
        "delay_probability": 0.0,
        "drop_probability": 0.0,
        "burst_probability": 0.0,
        "duplicate_probability": 0.0,
        "replay_probability": 0.0,
        "min_delay_ms": 0.0,
        "max_delay_ms": 0.0,
        "burst_hold_ms": 0.0,
        "burst_max_size": 1,
        "replay_history_size": 20,
    },
    "mild_wifi": {
        "delay_probability": 0.15,
        "drop_probability": 0.01,
        "burst_probability": 0.05,
        "duplicate_probability": 0.02,
        "replay_probability": 0.005,
        "min_delay_ms": 20.0,
        "max_delay_ms": 150.0,
        "burst_hold_ms": 150.0,
        "burst_max_size": 4,
        "replay_history_size": 50,
    },
    "bad_wifi": {
        "delay_probability": 0.35,
        "drop_probability": 0.05,
        "burst_probability": 0.15,
        "duplicate_probability": 0.04,
        "replay_probability": 0.03,
        "min_delay_ms": 50.0,
        "max_delay_ms": 500.0,
        "burst_hold_ms": 300.0,
        "burst_max_size": 8,
        "replay_history_size": 80,
    },
    "5g_burst": {
        "delay_probability": 0.25,
        "drop_probability": 0.03,
        "burst_probability": 0.25,
        "duplicate_probability": 0.04,
        "replay_probability": 0.04,
        "min_delay_ms": 30.0,
        "max_delay_ms": 800.0,
        "burst_hold_ms": 450.0,
        "burst_max_size": 10,
        "replay_history_size": 100,
    },
    "wifi_collapse": {
        "delay_probability": 0.55,
        "drop_probability": 0.10,
        "burst_probability": 0.30,
        "duplicate_probability": 0.10,
        "replay_probability": 0.08,
        "min_delay_ms": 100.0,
        "max_delay_ms": 1200.0,
        "burst_hold_ms": 750.0,
        "burst_max_size": 16,
        "replay_history_size": 120,
    },
}


# ============================================================
# Internal event container
# ============================================================

@dataclass
class QueuedTwist:
    due_time: float
    seq: int
    msg: Twist
    source: str


# ============================================================
# Node
# ============================================================

class JitterInjectorNode(Node):
    def __init__(self):
        super().__init__("jitter_injector_node")

        # --------------------------------------------------------
        # Profile first
        # --------------------------------------------------------
        self.declare_parameter("profile", "bad_wifi")
        profile_name = str(self.get_parameter("profile").value)
        profile = PROFILES.get(profile_name, PROFILES["bad_wifi"])

        # --------------------------------------------------------
        # Topics
        # --------------------------------------------------------
        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel_jittered")
        self.declare_parameter("status_topic", "/jitter_injector/status")
        self.declare_parameter("demo_raw_topic", "/cmd_vel_raw")

        # --------------------------------------------------------
        # Demo generator
        # --------------------------------------------------------
        self.declare_parameter("use_demo_cmd", False)
        self.declare_parameter("demo_rate_hz", 20.0)
        self.declare_parameter("demo_loop_seconds", 16.0)

        # --------------------------------------------------------
        # Randomness
        # --------------------------------------------------------
        self.declare_parameter("seed", 502)

        # --------------------------------------------------------
        # Jitter probabilities
        # --------------------------------------------------------
        self.declare_parameter("delay_probability", float(profile["delay_probability"]))
        self.declare_parameter("drop_probability", float(profile["drop_probability"]))
        self.declare_parameter("burst_probability", float(profile["burst_probability"]))
        self.declare_parameter("duplicate_probability", float(profile["duplicate_probability"]))
        self.declare_parameter("replay_probability", float(profile["replay_probability"]))

        # --------------------------------------------------------
        # Jitter parameters
        # --------------------------------------------------------
        self.declare_parameter("min_delay_ms", float(profile["min_delay_ms"]))
        self.declare_parameter("max_delay_ms", float(profile["max_delay_ms"]))
        self.declare_parameter("burst_hold_ms", float(profile["burst_hold_ms"]))
        self.declare_parameter("burst_max_size", int(profile["burst_max_size"]))
        self.declare_parameter("replay_history_size", int(profile["replay_history_size"]))

        # --------------------------------------------------------
        # Runtime
        # --------------------------------------------------------
        self.declare_parameter("tick_hz", 100.0)
        self.declare_parameter("status_hz", 2.0)
        self.declare_parameter("log_events", True)

        # --------------------------------------------------------
        # Read params
        # --------------------------------------------------------
        self.profile_name = profile_name

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.demo_raw_topic = str(self.get_parameter("demo_raw_topic").value)

        self.use_demo_cmd = bool(self.get_parameter("use_demo_cmd").value)
        self.demo_rate_hz = float(self.get_parameter("demo_rate_hz").value)
        self.demo_loop_seconds = float(self.get_parameter("demo_loop_seconds").value)

        seed = int(self.get_parameter("seed").value)
        self.rng = random.Random(seed)

        self.delay_probability = float(self.get_parameter("delay_probability").value)
        self.drop_probability = float(self.get_parameter("drop_probability").value)
        self.burst_probability = float(self.get_parameter("burst_probability").value)
        self.duplicate_probability = float(self.get_parameter("duplicate_probability").value)
        self.replay_probability = float(self.get_parameter("replay_probability").value)

        self.min_delay_ms = float(self.get_parameter("min_delay_ms").value)
        self.max_delay_ms = float(self.get_parameter("max_delay_ms").value)
        self.burst_hold_ms = float(self.get_parameter("burst_hold_ms").value)
        self.burst_max_size = int(self.get_parameter("burst_max_size").value)
        self.replay_history_size = int(self.get_parameter("replay_history_size").value)

        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.status_hz = float(self.get_parameter("status_hz").value)
        self.log_events = bool(self.get_parameter("log_events").value)

        # --------------------------------------------------------
        # State
        # --------------------------------------------------------
        self._seq = 0
        self._heap: List[Tuple[float, int, QueuedTwist]] = []
        self._history: List[Twist] = []

        self._burst_buffer: List[Twist] = []
        self._burst_release_time: Optional[float] = None

        self._demo_start_time = self._now_sec()

        self.stats = {
            "in": 0,
            "out": 0,
            "dropped": 0,
            "delayed": 0,
            "burst_buffered": 0,
            "burst_released": 0,
            "duplicated": 0,
            "replayed": 0,
            "clean_forwarded": 0,
        }

        # --------------------------------------------------------
        # ROS interfaces
        # --------------------------------------------------------
        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.sub = self.create_subscription(
            Twist,
            self.input_topic,
            self._cmd_callback,
            10,
        )

        self.demo_raw_pub = self.create_publisher(Twist, self.demo_raw_topic, 10)

        self.tick_timer = self.create_timer(
            1.0 / max(self.tick_hz, 1.0),
            self._tick,
        )

        self.status_timer = self.create_timer(
            1.0 / max(self.status_hz, 0.1),
            self._publish_status,
        )

        if self.use_demo_cmd:
            self.demo_timer = self.create_timer(
                1.0 / max(self.demo_rate_hz, 1.0),
                self._publish_demo_cmd,
            )
        else:
            self.demo_timer = None

        self.get_logger().info(
            f"Jitter Injector started | profile={self.profile_name} | "
            f"input={self.input_topic} -> output={self.output_topic} | "
            f"use_demo_cmd={self.use_demo_cmd}"
        )

    # ============================================================
    # Time
    # ============================================================

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ============================================================
    # Input path
    # ============================================================

    def _cmd_callback(self, msg: Twist) -> None:
        self._ingest_command(msg, source="input")

    def _ingest_command(self, msg: Twist, source: str) -> None:
        self.stats["in"] += 1

        msg = copy.deepcopy(msg)
        self._remember(msg)

        # 1. Drop
        if self._event(self.drop_probability):
            self.stats["dropped"] += 1
            self._log_event("DROP", msg)
            return

        # 2. Replay stale command
        if self._event(self.replay_probability) and len(self._history) > 2:
            stale = copy.deepcopy(self.rng.choice(self._history[:-1]))
            self._schedule(stale, delay_s=0.0, source="replay")
            self.stats["replayed"] += 1
            self._log_event("REPLAY_STALE", stale)

        # 3. Duplicate
        duplicate = self._event(self.duplicate_probability)

        # 4. Burst buffering
        if self._event(self.burst_probability) or self._burst_release_time is not None:
            self._add_to_burst(msg)
            if duplicate:
                self._add_to_burst(copy.deepcopy(msg))
                self.stats["duplicated"] += 1
                self._log_event("DUPLICATE_IN_BURST", msg)
            return

        # 5. Delay / clean forward
        delay_s = 0.0
        if self._event(self.delay_probability):
            delay_s = self._random_delay_s()
            self.stats["delayed"] += 1
            self._log_event(f"DELAY_{delay_s:.3f}s", msg)
        else:
            self.stats["clean_forwarded"] += 1

        self._schedule(msg, delay_s=delay_s, source=source)

        if duplicate:
            duplicate_delay_s = delay_s + self.rng.uniform(0.0, 0.05)
            self._schedule(copy.deepcopy(msg), delay_s=duplicate_delay_s, source="duplicate")
            self.stats["duplicated"] += 1
            self._log_event("DUPLICATE", msg)

    # ============================================================
    # Jitter mechanics
    # ============================================================

    def _event(self, probability: float) -> bool:
        probability = max(0.0, min(1.0, probability))
        return self.rng.random() < probability

    def _random_delay_s(self) -> float:
        lo = max(0.0, self.min_delay_ms) * 1e-3
        hi = max(lo, self.max_delay_ms * 1e-3)
        return self.rng.uniform(lo, hi)

    def _schedule(self, msg: Twist, delay_s: float, source: str) -> None:
        now = self._now_sec()
        due = now + max(0.0, delay_s)

        self._seq += 1
        item = QueuedTwist(
            due_time=due,
            seq=self._seq,
            msg=copy.deepcopy(msg),
            source=source,
        )

        heapq.heappush(self._heap, (item.due_time, item.seq, item))

    def _add_to_burst(self, msg: Twist) -> None:
        now = self._now_sec()

        if self._burst_release_time is None:
            hold_s = max(0.0, self.burst_hold_ms) * 1e-3
            self._burst_release_time = now + hold_s
            self._log_event(f"BURST_START_hold={hold_s:.3f}s", msg)

        if len(self._burst_buffer) < max(1, self.burst_max_size):
            self._burst_buffer.append(copy.deepcopy(msg))
            self.stats["burst_buffered"] += 1
        else:
            # If buffer is full, drop the oldest to simulate queue pressure.
            self._burst_buffer.pop(0)
            self._burst_buffer.append(copy.deepcopy(msg))
            self.stats["dropped"] += 1
            self._log_event("BURST_BUFFER_OVERFLOW_DROP_OLDEST", msg)

    def _release_burst_if_due(self) -> None:
        if self._burst_release_time is None:
            return

        now = self._now_sec()
        if now < self._burst_release_time:
            return

        count = len(self._burst_buffer)
        for msg in self._burst_buffer:
            # Schedule all at the same due time to create a burst.
            self._schedule(msg, delay_s=0.0, source="burst_release")

        self.stats["burst_released"] += count
        self._log_event(f"BURST_RELEASE_count={count}", None)

        self._burst_buffer.clear()
        self._burst_release_time = None

    def _tick(self) -> None:
        self._release_burst_if_due()

        now = self._now_sec()
        while self._heap and self._heap[0][0] <= now:
            _, _, item = heapq.heappop(self._heap)
            self.pub.publish(item.msg)
            self.stats["out"] += 1

    def _remember(self, msg: Twist) -> None:
        self._history.append(copy.deepcopy(msg))
        max_size = max(1, self.replay_history_size)
        if len(self._history) > max_size:
            self._history = self._history[-max_size:]

    # ============================================================
    # Demo command generator
    # ============================================================

    def _publish_demo_cmd(self) -> None:
        msg = self._demo_cmd()
        self.demo_raw_pub.publish(msg)
        self._ingest_command(msg, source="demo")

    def _demo_cmd(self) -> Twist:
        """
        Generates a repeatable mobile-base command profile.

        The point is not to simulate a real robot.
        The point is to create clean commands that become dangerous
        after delay / burst / replay injection.
        """
        t = (self._now_sec() - self._demo_start_time) % max(self.demo_loop_seconds, 1.0)

        msg = Twist()

        # Segment 1: gentle forward
        if 0.0 <= t < 4.0:
            msg.linear.x = 0.25
            msg.angular.z = 0.0

        # Segment 2: stop
        elif 4.0 <= t < 6.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # Segment 3: faster forward
        elif 6.0 <= t < 9.0:
            msg.linear.x = 0.55
            msg.angular.z = 0.0

        # Segment 4: turn
        elif 9.0 <= t < 11.0:
            msg.linear.x = 0.20
            msg.angular.z = 0.70

        # Segment 5: reverse
        elif 11.0 <= t < 13.0:
            msg.linear.x = -0.25
            msg.angular.z = 0.0

        # Segment 6: stop
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        return msg

    # ============================================================
    # Status
    # ============================================================

    def _publish_status(self) -> None:
        status = {
            "profile": self.profile_name,
            "input_topic": self.input_topic,
            "output_topic": self.output_topic,
            "queue_size": len(self._heap),
            "burst_buffer_size": len(self._burst_buffer),
            "burst_active": self._burst_release_time is not None,
            "stats": self.stats,
            "params": {
                "delay_probability": self.delay_probability,
                "drop_probability": self.drop_probability,
                "burst_probability": self.burst_probability,
                "duplicate_probability": self.duplicate_probability,
                "replay_probability": self.replay_probability,
                "min_delay_ms": self.min_delay_ms,
                "max_delay_ms": self.max_delay_ms,
                "burst_hold_ms": self.burst_hold_ms,
                "burst_max_size": self.burst_max_size,
            },
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def _log_event(self, name: str, msg: Optional[Twist]) -> None:
        if not self.log_events:
            return

        if msg is None:
            self.get_logger().info(f"[JITTER] {name}")
            return

        self.get_logger().info(
            f"[JITTER] {name} | vx={msg.linear.x:.3f}, wz={msg.angular.z:.3f}"
        )


# ============================================================
# Main
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)

    node = JitterInjectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
