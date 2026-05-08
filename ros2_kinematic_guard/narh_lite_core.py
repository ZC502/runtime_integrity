"""
narh_lite_core.py

NARH-lite core for ros2_kinematic_guard.

Pure Python, ROS-free, framework-free.

Formal project:
    NARH-based Kinematic Guard for ROS 2

Tagline:
    It does not fix the network.
    It prevents bad network timing from becoming dangerous robot motion.

Purpose
-------
Given two consecutive commands and two consecutive feedback states,
estimate whether the command/feedback window is still executable.

Input:
    cmd_prev, cmd_curr
    odom_prev, odom_curr

Output:
    R_NAR residual score
    status
    action hint
    optional safe command

This file intentionally does not import rclpy or ROS message types.
ROS 2 wrappers can translate geometry_msgs/Twist, TwistStamped,
and nav_msgs/Odometry into these plain dataclasses.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple
import math


# ============================================================
# Basic data structures
# ============================================================

@dataclass
class KinematicCommand:
    """
    2D mobile-base command.

    vx:
        Linear velocity in m/s.

    wz:
        Angular velocity around z axis in rad/s.

    stamp:
        Optional source timestamp in seconds.
        If /cmd_vel has no header, the ROS wrapper can use receive time.

    seq:
        Optional sequence id from wrapper / injector.
    """
    vx: float
    wz: float
    stamp: Optional[float] = None
    seq: Optional[int] = None


@dataclass
class KinematicFeedback:
    """
    2D odometry feedback.

    x, y:
        Position in meters.

    yaw:
        Heading in radians.

    vx, wz:
        Optional measured velocity from odom.
        If not available, the core estimates velocity from pose delta.

    stamp:
        Optional odom timestamp in seconds.
    """
    x: float
    y: float
    yaw: float
    vx: Optional[float] = None
    wz: Optional[float] = None
    stamp: Optional[float] = None


class GuardStatus(str, Enum):
    GREEN = "GREEN"
    YELLOW_SLOWDOWN = "YELLOW_SLOWDOWN"
    RED_BRAKE = "RED_BRAKE"
    RESYNCING = "RESYNCING"
    RECOVERED = "RECOVERED"


class GuardAction(str, Enum):
    PASS = "PASS"
    SLOW_DOWN = "SLOW_DOWN"
    BRAKE_AND_RESYNC = "BRAKE_AND_RESYNC"


@dataclass
class NarhLiteConfig:
    """
    Tuning parameters for NARH-lite.

    These defaults are conservative for small mobile robots.
    They should be tuned per platform.
    """

    # Timing
    default_dt: float = 0.05
    min_dt: float = 1e-3
    max_dt: float = 0.50
    cmd_ttl: float = 0.25
    phase_tolerance: float = 0.08

    # Kinematic limits
    max_linear_accel: float = 0.8       # m/s^2
    max_angular_accel: float = 1.5      # rad/s^2
    max_linear_jerk: float = 3.0        # m/s^3
    max_angular_jerk: float = 6.0       # rad/s^3

    # Residual normalization
    position_tolerance: float = 0.03    # m
    yaw_tolerance: float = 0.08         # rad
    lateral_tolerance: float = 0.03     # m

    # Risk thresholds
    yellow_threshold: float = 2.5
    red_threshold: float = 5.0

    # Slowdown / resync
    slowdown_scale: float = 0.45
    resync_required_good_frames: int = 5

    # Component weights
    w_timeflow: float = 1.2
    w_stale: float = 1.5
    w_phase: float = 1.1
    w_cmd_accel: float = 1.0
    w_cmd_jerk: float = 0.7
    w_cmd_odom: float = 1.4

    # Emergency brake limits
    emergency_linear_brake_accel: float = 1.8    # m/s^2
    emergency_angular_brake_accel: float = 3.5   # rad/s^2
    brake_zero_epsilon_vx: float = 0.01          # m/s
    brake_zero_epsilon_wz: float = 0.02          # rad/s

@dataclass
class NarhLiteResult:
    r_nar: float
    status: GuardStatus
    action: GuardAction
    safe_cmd: KinematicCommand
    velocity_scale: float
    reasons: List[str] = field(default_factory=list)
    components: Dict[str, float] = field(default_factory=dict)
    debug: Dict[str, float] = field(default_factory=dict)


# ============================================================
# Utility math
# ============================================================

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def finite_or(x: float, fallback: float = 0.0) -> float:
    try:
        x = float(x)
        return x if math.isfinite(x) else fallback
    except Exception:
        return fallback


def wrap_angle(a: float) -> float:
    """
    Wrap angle to [-pi, pi].
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def safe_norm2(a: float, b: float) -> float:
    return math.sqrt(a * a + b * b)


# ============================================================
# Core
# ============================================================

class NarhLiteCore:
    """
    Pure Python NARH-lite evaluator.

    Main idea
    ---------
    The core does not try to repair the network.

    It checks whether the current command/feedback window is still
    kinematically executable.

    It combines:
    - time-flow residual
    - stale command residual
    - command acceleration / jerk residual
    - command-vs-odom consistency residual
    - phase mismatch residual

    into a single R_NAR score.

    When R_NAR is high, the core can recommend:
    - SLOW_DOWN
    - BRAKE_AND_RESYNC

    This is intentionally a lightweight engineering kernel,
    not a full octonion implementation.
    """

    def __init__(self, config: Optional[NarhLiteConfig] = None):
        self.cfg = config or NarhLiteConfig()

        self._last_cmd_accel_vx: Optional[float] = None
        self._last_cmd_accel_wz: Optional[float] = None

        self._last_safe_cmd = KinematicCommand(0.0, 0.0, stamp=None, seq=None)
        self._resync_frames_left = 0

    def reset(self) -> None:
        self._last_cmd_accel_vx = None
        self._last_cmd_accel_wz = None
        self._last_safe_cmd = KinematicCommand(0.0, 0.0)
        self._resync_frames_left = 0

    # ------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------

    def evaluate(
        self,
        cmd_prev: KinematicCommand,
        cmd_curr: KinematicCommand,
        odom_prev: KinematicFeedback,
        odom_curr: KinematicFeedback,
        now: Optional[float] = None,
    ) -> NarhLiteResult:
        """
        Evaluate one command/feedback window.

        Parameters
        ----------
        cmd_prev, cmd_curr:
            Two consecutive command samples.

        odom_prev, odom_curr:
            Two consecutive odometry samples.

        now:
            Optional receiver time in seconds.
            Used for stale-command detection.

        Returns
        -------
        NarhLiteResult
        """

        cmd_prev = self._sanitize_cmd(cmd_prev)
        cmd_curr = self._sanitize_cmd(cmd_curr)
        odom_prev = self._sanitize_odom(odom_prev)
        odom_curr = self._sanitize_odom(odom_curr)

        dt_cmd, cmd_time_score, cmd_time_reason = self._compute_dt(
            cmd_prev.stamp,
            cmd_curr.stamp,
            label="cmd"
        )

        dt_odom, odom_time_score, odom_time_reason = self._compute_dt(
            odom_prev.stamp,
            odom_curr.stamp,
            label="odom"
        )

        dt_eval = dt_odom if dt_odom > self.cfg.min_dt else dt_cmd
        dt_eval = max(dt_eval, self.cfg.min_dt)

        stale_score, stale_reason = self._stale_score(cmd_curr, now)
        phase_score, phase_reason = self._phase_score(cmd_curr, odom_curr)

        cmd_accel_score, jerk_score, accel_debug, accel_reasons = self._command_dynamics_score(
            cmd_prev,
            cmd_curr,
            dt_cmd
        )

        cmd_odom_score, odom_debug, odom_reasons = self._cmd_odom_consistency_score(
            cmd_prev,
            cmd_curr,
            odom_prev,
            odom_curr,
            dt_eval
        )

        timeflow_score = max(cmd_time_score, odom_time_score)

        components = {
            "timeflow": timeflow_score,
            "stale": stale_score,
            "phase": phase_score,
            "cmd_accel": cmd_accel_score,
            "cmd_jerk": jerk_score,
            "cmd_odom": cmd_odom_score,
        }

        r_nar = self._combine_components(components)

        reasons: List[str] = []
        for item in [
            cmd_time_reason,
            odom_time_reason,
            stale_reason,
            phase_reason,
        ]:
            if item:
                reasons.append(item)

        reasons.extend(accel_reasons)
        reasons.extend(odom_reasons)

        hard_fault = (
            cmd_time_score >= self.cfg.red_threshold
            or odom_time_score >= self.cfg.red_threshold
            or stale_score >= self.cfg.red_threshold
        )

        status, action = self._decide_status_action(r_nar, hard_fault, reasons)

        safe_cmd, velocity_scale = self._compute_safe_command(
            cmd_curr=cmd_curr,
            dt=dt_eval,
            status=status,
            action=action
        )

        debug = {}
        debug.update(accel_debug)
        debug.update(odom_debug)
        debug.update({
            "dt_cmd": dt_cmd,
            "dt_odom": dt_odom,
            "r_nar": r_nar,
            "resync_frames_left": float(self._resync_frames_left),
        })

        return NarhLiteResult(
            r_nar=r_nar,
            status=status,
            action=action,
            safe_cmd=safe_cmd,
            velocity_scale=velocity_scale,
            reasons=reasons,
            components=components,
            debug=debug,
        )

    # ------------------------------------------------------------
    # Sanitization
    # ------------------------------------------------------------

    def _sanitize_cmd(self, cmd: KinematicCommand) -> KinematicCommand:
        return KinematicCommand(
            vx=finite_or(cmd.vx),
            wz=finite_or(cmd.wz),
            stamp=finite_or(cmd.stamp, None) if cmd.stamp is not None else None,
            seq=cmd.seq,
        )

    def _sanitize_odom(self, odom: KinematicFeedback) -> KinematicFeedback:
        return KinematicFeedback(
            x=finite_or(odom.x),
            y=finite_or(odom.y),
            yaw=wrap_angle(finite_or(odom.yaw)),
            vx=finite_or(odom.vx, None) if odom.vx is not None else None,
            wz=finite_or(odom.wz, None) if odom.wz is not None else None,
            stamp=finite_or(odom.stamp, None) if odom.stamp is not None else None,
        )

    # ------------------------------------------------------------
    # Time-flow scores
    # ------------------------------------------------------------

    def _compute_dt(
        self,
        t_prev: Optional[float],
        t_curr: Optional[float],
        label: str,
    ) -> Tuple[float, float, Optional[str]]:
        """
        Return dt, score, reason.

        Missing timestamps fall back to default_dt with no penalty.
        Time rollback or near-zero dt gets a high score.
        Excessively large dt gets a moderate/high score.
        """
        if t_prev is None or t_curr is None:
            return self.cfg.default_dt, 0.0, None

        dt = finite_or(t_curr - t_prev, self.cfg.default_dt)

        if dt <= 0.0:
            return self.cfg.min_dt, self.cfg.red_threshold + 2.0, f"{label.upper()}_TIME_ROLLBACK"

        if dt < self.cfg.min_dt:
            score = self.cfg.red_threshold
            return self.cfg.min_dt, score, f"{label.upper()}_DT_TOO_SMALL"

        if dt > self.cfg.max_dt:
            score = min(self.cfg.red_threshold, (dt / self.cfg.max_dt - 1.0) * 4.0)
            return dt, score, f"{label.upper()}_DT_TOO_LARGE"

        return dt, 0.0, None

    def _stale_score(
        self,
        cmd_curr: KinematicCommand,
        now: Optional[float],
    ) -> Tuple[float, Optional[str]]:
        if now is None or cmd_curr.stamp is None:
            return 0.0, None

        age = finite_or(now - cmd_curr.stamp, 0.0)

        if age <= self.cfg.cmd_ttl:
            return 0.0, None

        score = (age - self.cfg.cmd_ttl) / max(self.cfg.cmd_ttl, 1e-6)
        score = min(self.cfg.red_threshold + 2.0, score)

        reason = "STALE_COMMAND"
        return score, reason

    def _phase_score(
        self,
        cmd_curr: KinematicCommand,
        odom_curr: KinematicFeedback,
    ) -> Tuple[float, Optional[str]]:
        if cmd_curr.stamp is None or odom_curr.stamp is None:
            return 0.0, None

        phase_error = abs(finite_or(cmd_curr.stamp - odom_curr.stamp, 0.0))

        if phase_error <= self.cfg.phase_tolerance:
            return 0.0, None

        score = phase_error / max(self.cfg.phase_tolerance, 1e-6) - 1.0
        score = min(self.cfg.red_threshold, score)

        return score, "CMD_ODOM_PHASE_MISMATCH"

    # ------------------------------------------------------------
    # Command dynamics
    # ------------------------------------------------------------

    def _command_dynamics_score(
        self,
        cmd_prev: KinematicCommand,
        cmd_curr: KinematicCommand,
        dt_cmd: float,
    ) -> Tuple[float, float, Dict[str, float], List[str]]:

        dt = max(dt_cmd, self.cfg.min_dt)

        acc_vx = (cmd_curr.vx - cmd_prev.vx) / dt
        acc_wz = (cmd_curr.wz - cmd_prev.wz) / dt

        lin_acc_score = max(0.0, abs(acc_vx) / max(self.cfg.max_linear_accel, 1e-6) - 1.0)
        ang_acc_score = max(0.0, abs(acc_wz) / max(self.cfg.max_angular_accel, 1e-6) - 1.0)

        accel_score = safe_norm2(lin_acc_score, ang_acc_score)

        jerk_score = 0.0
        jerk_vx = 0.0
        jerk_wz = 0.0

        if self._last_cmd_accel_vx is not None and self._last_cmd_accel_wz is not None:
            jerk_vx = (acc_vx - self._last_cmd_accel_vx) / dt
            jerk_wz = (acc_wz - self._last_cmd_accel_wz) / dt

            lin_jerk_score = max(0.0, abs(jerk_vx) / max(self.cfg.max_linear_jerk, 1e-6) - 1.0)
            ang_jerk_score = max(0.0, abs(jerk_wz) / max(self.cfg.max_angular_jerk, 1e-6) - 1.0)
            jerk_score = safe_norm2(lin_jerk_score, ang_jerk_score)

        self._last_cmd_accel_vx = acc_vx
        self._last_cmd_accel_wz = acc_wz

        reasons: List[str] = []
        if accel_score > 0.0:
            reasons.append("IMPOSSIBLE_COMMAND_ACCELERATION")
        if jerk_score > 0.0:
            reasons.append("IMPOSSIBLE_COMMAND_JERK")

        debug = {
            "cmd_acc_vx": acc_vx,
            "cmd_acc_wz": acc_wz,
            "cmd_jerk_vx": jerk_vx,
            "cmd_jerk_wz": jerk_wz,
            "cmd_accel_score": accel_score,
            "cmd_jerk_score": jerk_score,
        }

        return accel_score, jerk_score, debug, reasons

    # ------------------------------------------------------------
    # Command vs odometry consistency
    # ------------------------------------------------------------

    def _cmd_odom_consistency_score(
        self,
        cmd_prev: KinematicCommand,
        cmd_curr: KinematicCommand,
        odom_prev: KinematicFeedback,
        odom_curr: KinematicFeedback,
        dt: float,
    ) -> Tuple[float, Dict[str, float], List[str]]:

        dt = max(dt, self.cfg.min_dt)

        # Expected local delta from command using trapezoidal velocity.
        v_avg = 0.5 * (cmd_prev.vx + cmd_curr.vx)
        w_avg = 0.5 * (cmd_prev.wz + cmd_curr.wz)

        expected_dx = v_avg * dt
        expected_dyaw = w_avg * dt

        # Measured local delta from odometry.
        dx_world = odom_curr.x - odom_prev.x
        dy_world = odom_curr.y - odom_prev.y

        c = math.cos(odom_prev.yaw)
        s = math.sin(odom_prev.yaw)

        measured_dx_local = c * dx_world + s * dy_world
        measured_dy_local = -s * dx_world + c * dy_world
        measured_dyaw = wrap_angle(odom_curr.yaw - odom_prev.yaw)

        linear_gap = measured_dx_local - expected_dx
        angular_gap = wrap_angle(measured_dyaw - expected_dyaw)
        lateral_gap = measured_dy_local

        linear_score = abs(linear_gap) / (
            self.cfg.position_tolerance + 0.25 * abs(expected_dx) + 1e-6
        )

        angular_score = abs(angular_gap) / (
            self.cfg.yaw_tolerance + 0.25 * abs(expected_dyaw) + 1e-6
        )

        lateral_score = abs(lateral_gap) / (
            self.cfg.lateral_tolerance + 0.20 * abs(expected_dx) + 1e-6
        )

        score = math.sqrt(
            linear_score * linear_score
            + angular_score * angular_score
            + 0.5 * lateral_score * lateral_score
        )

        reasons: List[str] = []
        if score >= 1.0:
            reasons.append("CMD_ODOM_INCONSISTENCY")

        # Direction contradiction is a strong signal.
        if abs(cmd_curr.vx) > 0.05:
            measured_vx = measured_dx_local / dt
            if cmd_curr.vx * measured_vx < -0.02:
                score += 2.0
                reasons.append("CMD_ODOM_DIRECTION_CONTRADICTION")

        debug = {
            "expected_dx": expected_dx,
            "expected_dyaw": expected_dyaw,
            "measured_dx_local": measured_dx_local,
            "measured_dy_local": measured_dy_local,
            "measured_dyaw": measured_dyaw,
            "linear_gap": linear_gap,
            "angular_gap": angular_gap,
            "lateral_gap": lateral_gap,
            "cmd_odom_score": score,
        }

        return score, debug, reasons

    # ------------------------------------------------------------
    # Residual fusion
    # ------------------------------------------------------------

    def _combine_components(self, components: Dict[str, float]) -> float:
        c = self.cfg

        weighted_sum = (
            c.w_timeflow * components["timeflow"] ** 2
            + c.w_stale * components["stale"] ** 2
            + c.w_phase * components["phase"] ** 2
            + c.w_cmd_accel * components["cmd_accel"] ** 2
            + c.w_cmd_jerk * components["cmd_jerk"] ** 2
            + c.w_cmd_odom * components["cmd_odom"] ** 2
        )

        return math.sqrt(max(0.0, weighted_sum))

    # ------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------

    def _decide_status_action(
        self,
        r_nar: float,
        hard_fault: bool,
        reasons: List[str],
    ) -> Tuple[GuardStatus, GuardAction]:

        red = hard_fault or r_nar >= self.cfg.red_threshold
        yellow = r_nar >= self.cfg.yellow_threshold

        if red:
            self._resync_frames_left = self.cfg.resync_required_good_frames
            return GuardStatus.RED_BRAKE, GuardAction.BRAKE_AND_RESYNC

        if self._resync_frames_left > 0:
            if yellow:
                self._resync_frames_left = self.cfg.resync_required_good_frames
                return GuardStatus.RESYNCING, GuardAction.BRAKE_AND_RESYNC

            self._resync_frames_left -= 1
            if self._resync_frames_left > 0:
                return GuardStatus.RESYNCING, GuardAction.BRAKE_AND_RESYNC

            return GuardStatus.RECOVERED, GuardAction.PASS

        if yellow:
            return GuardStatus.YELLOW_SLOWDOWN, GuardAction.SLOW_DOWN

        return GuardStatus.GREEN, GuardAction.PASS

    # ------------------------------------------------------------
    # Safe command generation
    # ------------------------------------------------------------

    def _compute_safe_command(
        self,
        cmd_curr: KinematicCommand,
        dt: float,
        status: GuardStatus,
        action: GuardAction,
    ) -> Tuple[KinematicCommand, float]:

        dt = max(dt, self.cfg.min_dt)

        if action == GuardAction.BRAKE_AND_RESYNC:
            safe = self._ramp_to_zero(dt)
            self._last_safe_cmd = safe
            return safe, 0.0

        if action == GuardAction.SLOW_DOWN:
            scale = self.cfg.slowdown_scale
        else:
            scale = 1.0

        target = KinematicCommand(
            vx=cmd_curr.vx * scale,
            wz=cmd_curr.wz * scale,
            stamp=cmd_curr.stamp,
            seq=cmd_curr.seq,
        )

        safe = self._accel_limited_command(target, dt)
        self._last_safe_cmd = safe
        return safe, scale

    def _ramp_to_zero(self, dt: float) -> KinematicCommand:
        target = KinematicCommand(0.0, 0.0)
        return self._limited_command(
            target=target,
            dt=dt,
            linear_accel_limit=max(
                self.cfg.max_linear_accel,
                self.cfg.emergency_linear_brake_accel
            ),
            angular_accel_limit=max(
                self.cfg.max_angular_accel,
                self.cfg.emergency_angular_brake_accel
            ),
        )


    def _accel_limited_command(
        self,
        target: KinematicCommand,
        dt: float,
    ) -> KinematicCommand:
        return self._limited_command(
            target=target,
            dt=dt,
            linear_accel_limit=self.cfg.max_linear_accel,
            angular_accel_limit=self.cfg.max_angular_accel,
        )


    def _limited_command(
        self,
        target: KinematicCommand,
        dt: float,
        linear_accel_limit: float,
        angular_accel_limit: float,
    ) -> KinematicCommand:

        dt = max(dt, self.cfg.min_dt)
        last = self._last_safe_cmd

        max_dv = abs(linear_accel_limit) * dt
        max_dw = abs(angular_accel_limit) * dt

        vx = last.vx + clamp(target.vx - last.vx, -max_dv, max_dv)
        wz = last.wz + clamp(target.wz - last.wz, -max_dw, max_dw)

        if abs(vx) < self.cfg.brake_zero_epsilon_vx and target.vx == 0.0:
            vx = 0.0

        if abs(wz) < self.cfg.brake_zero_epsilon_wz and target.wz == 0.0:
           wz = 0.0

        return KinematicCommand(
            vx=vx,
            wz=wz,
            stamp=target.stamp,
            seq=target.seq,
        )


# ============================================================
# Minimal smoke test
# ============================================================

if __name__ == "__main__":
    core = NarhLiteCore()

    print("=" * 70)
    print("NARH-lite core smoke test")
    print("=" * 70)

    # Normal window
    r1 = core.evaluate(
        cmd_prev=KinematicCommand(0.20, 0.0, stamp=0.00),
        cmd_curr=KinematicCommand(0.22, 0.0, stamp=0.05),
        odom_prev=KinematicFeedback(0.000, 0.0, 0.0, stamp=0.00),
        odom_curr=KinematicFeedback(0.011, 0.0, 0.0, stamp=0.05),
        now=0.05,
    )

    print("\nNormal window")
    print("R_NAR:", round(r1.r_nar, 4))
    print("Status:", r1.status)
    print("Action:", r1.action)
    print("Safe cmd:", r1.safe_cmd)
    print("Reasons:", r1.reasons)

    # Bad Wi-Fi / stale burst window
    r2 = core.evaluate(
        cmd_prev=KinematicCommand(0.20, 0.0, stamp=0.05),
        cmd_curr=KinematicCommand(1.80, 0.0, stamp=0.06),
        odom_prev=KinematicFeedback(0.011, 0.0, 0.0, stamp=0.05),
        odom_curr=KinematicFeedback(0.012, 0.0, 0.0, stamp=0.10),
        now=0.40,
    )

    print("\nBad timing / burst window")
    print("R_NAR:", round(r2.r_nar, 4))
    print("Status:", r2.status)
    print("Action:", r2.action)
    print("Safe cmd:", r2.safe_cmd)
    print("Reasons:", r2.reasons)
    print("Components:", {k: round(v, 3) for k, v in r2.components.items()})
