This project is now part of the broader CLIM(Causal Link Integrity Middleware) concept:

Causal Link Integrity Middleware, powered by NARH. https://github.com/ZC502/CLIM-Causal-Link-Integrity-Middleware.git

# ROS 2 Kinematic Guard (NARH)

## Command-Execution-Integrity Telemetry for ROS 2 and VDA5050-style Mixed Fleets

> ROS 2 moves the robot.  
> VDA 5050 coordinates the fleet.  
> NARH Guard reports whether the recent command window is still executable.

`ros2_kinematic_guard` is a lightweight ROS 2 middleware project for detecting degraded command execution under bad Wi-Fi / 5G timing.

It has two roles:

1. **Local protection**  
   It can stop forwarding poisoned `/cmd_vel` windows and enter `BRAKE_AND_RESYNC`.

2. **Fleet observability**  
   It translates the same NARH residual into:
   - ROS-native `/diagnostics`
   - VDA5050-style `commandExecutionIntegrity` telemetry
   - compact `/command_integrity/summary`

This project is **not** a VDA 5050 implementation.  
It is **not** a certified safety controller.  
It does **not** replace `ros2_control`, hardware safety, safety PLCs, emergency stops, or vehicle-level functional safety requirements.

It provides a missing signal:

> **Is the recent command/feedback window still trustworthy?**

---

## Why this exists

In real mobile-robot deployments, especially over Wi-Fi / private LTE / 5G, the problem is not only packet loss.

A command may still arrive.

It may still be syntactically valid.

But it may no longer be executable relative to the robot’s current physical state.

Typical failure modes include:

- stale commands arriving late
- burst delivery after buffering
- replay-like behavior after reconnection
- near-zero `dt` between command windows
- command/odometry phase mismatch
- command acceleration or jerk exceeding feasible limits

Traditional heartbeat or timeout checks answer:

```text
Did a message arrive recently?
```

NARH Guard asks a stricter question:

```text
Is this command still kinematically executable under the current odometry stream?
```

---

## Architecture

```text
Bad Wi-Fi / 5G timing collapse
        │
        ▼
/cmd_vel_jittered
        │
        ▼
┌──────────────────────────────┐
│  kinematic_guard_node.py     │
│  NARH-lite residual R_NAR    │
│  BRAKE_AND_RESYNC            │
└───────────────┬──────────────┘
                │
                ├── /kinematic_guard/safe_cmd_vel
                │       local protected command output
                │
                ├── /kinematic_guard/status
                │       guard state: GREEN / RED_BRAKE / RESYNCING
                │
                ▼
┌──────────────────────────────┐
│  reporter_node.py            │
│  Command Integrity Reporter  │
└───────────────┬──────────────┘
                │
                ├── /diagnostics
                │       diagnostic_msgs/DiagnosticArray
                │
                ├── /command_integrity/vda5050_state
                │       VDA5050-style JSON telemetry
                │
                └── /command_integrity/summary
                        compact fleet-readable status
```

---

## Quick Demo

### 1. Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Run the Bad-Wi-Fi pressure test

```bash
ros2 launch ros2_kinematic_guard start_pressure_test.launch.py profile:=wifi_collapse
```

This starts:

```text
jitter_injector_node.py
kinematic_guard_node.py
synthetic_odom_provider.py
reporter_node.py
static_transform_publisher
```

You do not need:

```text
real robot
Gazebo
Isaac Sim
Nav2
hardware motor driver
```

The repo includes a closed-loop synthetic test:

```text
jitter_injector_node.py
    creates delayed / duplicated / bursty / replayed /cmd_vel

kinematic_guard_node.py
    computes R_NAR and outputs protected safe_cmd_vel

synthetic_odom_provider.py
    acts as a virtual robot body and publishes /odom

reporter_node.py
    translates NARH Guard state into ROS diagnostics and VDA5050-style telemetry
```

---

## Watch the telemetry

### Compact fleet-readable summary

```bash
ros2 topic echo /command_integrity/summary
```

Example output:

```text
data: state=RESYNCING latency=CRITICAL R_NAR=5.478 vehicle_response=RESYNC_REQUIRED fleet_action=HOLD_NEW_ORDERS
---
data: state=RESYNCING latency=CRITICAL R_NAR=1412.078 vehicle_response=RESYNC_REQUIRED fleet_action=HOLD_NEW_ORDERS
---
data: state=RECOVERED latency=NORMAL R_NAR=0.000 vehicle_response=NONE fleet_action=NONE
---
```

This means:

```text
The local guard is still in RESYNCING.
The command stream is not yet trusted again.
The vehicle response should remain in RESYNC_REQUIRED.
The fleet layer should hold new orders.
```

---

### ROS-native diagnostics

```bash
ros2 topic echo /diagnostics
```

Expected fields include:

```text
name: ros2_kinematic_guard/command_execution_integrity
level: WARN / ERROR / STALE
message: Command execution integrity critical
values:
  residual
  latency_class
  execution_state
  recommended_vehicle_response
  suggested_fleet_action
```

This makes the command-integrity signal visible to standard ROS diagnostic tooling.

---

### VDA5050-style telemetry

```bash
ros2 topic echo /command_integrity/vda5050_state
```

Example payload:

```json
{
  "vehicleId": "demo_amr_001",
  "manufacturer": "ros2_kinematic_guard_demo",
  "serialNumber": "demo",
  "commandExecutionIntegrity": {
    "residual": 5.478,
    "residualType": "kinematic_consistency",
    "latencyClass": "CRITICAL",
    "commandAgeMs": 0.0,
    "jitterMs": 0.0,
    "burstDetected": true,
    "staleCommandDetected": false,
    "replayWindowDetected": false,
    "executionState": "RESYNCING",
    "guardAction": "BRAKE_AND_RESYNC",
    "recommendedVehicleResponse": "RESYNC_REQUIRED",
    "suggestedFleetAction": "HOLD_NEW_ORDERS",
    "cleanWindowCount": 0,
    "source": "ros2_kinematic_guard"
  }
}
```

This is **not an official VDA 5050 field**.

It is a proof-of-concept for a possible vendor-neutral command-execution-integrity telemetry extension.

The goal is to make wireless command degradation visible at the fleet level instead of hiding it inside local controller or driver behavior.

---

## Demo: From Bad Wi-Fi collapse to fleet-level observability

Recent mixed-fleet discussions around VDA 5050 highlight the commercial value of deterministic fleet coordination.

This demo does **not** claim to measure fleet throughput directly.

Instead, it shows the missing observability layer:

```text
When wireless timing collapses,
the vehicle can expose a quantitative command-execution-integrity signal
before the fleet manager assigns new orders,
intersection crossings,
or zone-speed policies.
```

A fleet manager should be able to know:

```text
This vehicle is still command-consistent.
This vehicle is degraded.
This vehicle is in RESYNCING.
This vehicle should not receive new orders yet.
```

---

## Node overview

### `jitter_injector_node.py`

Creates toxic command timing patterns:

```text
DUPLICATE_IN_BURST
BURST_BUFFER_OVERFLOW_DROP_OLDEST
BURST_RELEASE_count=16
DELAY_0.653s
DROP
REPLAY_STALE
```

It publishes:

```text
/cmd_vel_jittered
/jitter_injector/status
```

---

### `kinematic_guard_node.py`

Consumes:

```text
/cmd_vel_jittered
/odom
```

Publishes:

```text
/kinematic_guard/safe_cmd_vel
/kinematic_guard/status
/kinematic_guard/residual
```

It computes the NARH-lite residual:

```text
R_NAR
```

and drives the local finite-state machine:

```text
GREEN
YELLOW_SLOWDOWN
RED_BRAKE
RESYNCING
RECOVERED
```

---

### `synthetic_odom_provider.py`

Acts as a virtual robot body.

Consumes:

```text
/kinematic_guard/safe_cmd_vel
```

Publishes:

```text
/odom
/synthetic_odom/status
```

It can also inject physical disturbance events such as:

```text
SLIP_START
SLIP_END
```

---

### `reporter_node.py`

Bridges local guard state to ROS and fleet-level telemetry.

Consumes:

```text
/kinematic_guard/status
/kinematic_guard/residual
```

Publishes:

```text
/diagnostics
/command_integrity/vda5050_state
/command_integrity/summary
```

This is the bridge between:

```text
ROS 2 local command execution
        ↓
ROS-native diagnostics
        ↓
VDA5050-style fleet observability
```

---

## Why this is not just a timeout

| Failure Mode | Heartbeat / Timeout | NARH Kinematic Guard |
|---|---|---|
| Packet loss | Can detect silence | Can detect silence and brake |
| Stale command | Often missed | Detected through timing + kinematic inconsistency |
| Burst command | Often missed | Detected through residual spike |
| Replay-like command | Often missed | Detected through stale / command-odom conflict |
| Command/odom contradiction | Usually not checked | Directly measured |
| Recovery | Timeout-based | Fresh-window resync gate |
| Fleet reporting | Usually absent | Exposed as ROS diagnostics + VDA5050-style telemetry |

Heartbeat and timeout mechanisms answer:

```text
Did a message arrive recently?
```

NARH Guard asks:

```text
Is the recent command/feedback window still trustworthy?
```

---

## Relationship to `ros2_control`

This project is complementary to `ros2_control`.

`ros2_control` provides important controller-side mechanisms such as:

```text
stamped references
timeouts
speed limiting
acceleration limiting
jerk limiting
smooth stop behavior in supported controllers
```

`ros2_kinematic_guard` is not intended to replace those mechanisms.

It provides a controller-agnostic command-integrity layer that can be useful when:

```text
the robot does not use ros2_control
the base driver is proprietary
the driver only exposes a raw command subscriber
fleet-level diagnostics are needed
bag / MCAP post-analysis is needed
VDA5050-style state reporting is desired
```

In short:

```text
ros2_control:
  executes safely inside well-structured controllers

ros2_kinematic_guard:
  reports whether the recent command/feedback window is still trustworthy

reporter_node.py:
  translates that signal into ROS diagnostics and VDA5050-style fleet telemetry
```

---

## Scope

`ros2_kinematic_guard` is **not** a certified safety controller.

It does not replace:

```text
hardware emergency stop
certified safety PLCs
safety-rated stop functions
vehicle-level safety design
ISO 3691-4 safety validation
ros2_control controller-side protections
manufacturer braking curves
```

It provides:

```text
a local command-integrity guard
a NARH-lite residual signal
a deterministic resync gate
ROS-native diagnostics
VDA5050-style telemetry proof-of-concept
```

The `VDA5050-style` payload is not official VDA 5050 schema.

It is an experimental structure for discussing how command-execution integrity could be exposed in heterogeneous fleets.

---

## Key concepts

### Command semantic collapse

```text
The message arrives.
But it is no longer executable.
```

This can happen when wireless timing causes stale, bursty, duplicated, delayed, or replayed command windows.

---

### NARH-lite residual

NARH-lite compares:

```text
previous command
current command
previous odometry
current odometry
```

It combines timing, smoothness, and command-vs-odom consistency into one residual:

```text
R_NAR
```

If `R_NAR` crosses thresholds, the guard can slow down, brake, or enter resync.

---

### BRAKE_AND_RESYNC

When command timing collapses, the guard does not simply keep forwarding the latest message.

It can:

```text
1. stop forwarding poisoned command windows
2. output safe zero or limited command
3. flush local command/odom buffers
4. wait for fresh command + fresh odometry
5. require N clean windows
6. release control through RECOVERED
```

---

### Fleet observability

The same residual that triggers local protection is translated into:

```text
latencyClass
executionState
recommendedVehicleResponse
suggestedFleetAction
```

Example:

```text
state=RESYNCING
latency=CRITICAL
vehicle_response=RESYNC_REQUIRED
fleet_action=HOLD_NEW_ORDERS
```

This allows the fleet/orchestration layer to reason about degraded vehicles before assigning new orders or intersection-heavy tasks.

---

## Technical design

### Local window

For each evaluation step:

```text
C_prev = previous command
C_curr = current command

O_prev = previous odometry
O_curr = current odometry
```

The guard estimates:

```text
expected_delta = integrate(C_prev, C_curr, dt)
measured_delta = O_curr - O_prev
```

---

### Residual components

The NARH-lite residual combines:

```text
timeflow residual
stale command residual
phase mismatch residual
command acceleration violation
command jerk violation
command-odom inconsistency
```

Conceptually:

```text
R_NAR =
  w1 * R_timeflow
+ w2 * R_stale
+ w3 * R_phase
+ w4 * R_accel
+ w5 * R_jerk
+ w6 * R_cmd_odom
```

The implementation separates components so that diagnostics can explain why the guard intervened.

---

### State transitions

Simplified:

```text
if R_NAR < yellow_threshold:
    GREEN

if R_NAR >= yellow_threshold:
    YELLOW_SLOWDOWN

if R_NAR >= red_threshold:
    RED_BRAKE -> RESYNCING

if fresh command/odom windows remain consistent for N frames:
    RECOVERED -> GREEN
```

A low instantaneous `R_NAR` does not always mean immediate recovery.

If the state is still `RESYNCING`, the reporter may still output:

```text
latency=CRITICAL
vehicle_response=RESYNC_REQUIRED
fleet_action=HOLD_NEW_ORDERS
```

until enough clean windows have been observed.

---

## Performance

The NARH-lite core is intentionally lightweight.

It does not run:

```text
global optimization
factor graph solving
full rigid-body simulation
full dynamics control
```

It uses:

```text
constant-size buffers
simple SE(2)-style integration
scalar residual components
threshold-based state transitions
```

Per evaluation complexity is effectively:

```text
O(1)
```

The default launch file runs the guard loop at:

```text
20 Hz
```

which gives an intervention opportunity every control tick, approximately:

```text
50 ms
```

The current implementation is Python-based for transparency and easy testing.

For production deployment, the same NARH-lite core could be ported to C++ or embedded closer to a lower-level controller.

---

## Supported interfaces

The first version targets mobile robot command/feedback streams using:

```text
geometry_msgs/Twist
geometry_msgs/TwistStamped
nav_msgs/Odometry
std_msgs/String
diagnostic_msgs/DiagnosticArray
```

It is naturally suitable for:

```text
differential-drive robots
skid-steer robots
omni-directional mobile bases
simulation replay
rosbag / MCAP analysis
wireless teleoperation pipelines
```

For Ackermann steering or more complex platforms, the same architecture can be used, but the expected-motion model should be adapted.

Examples:

```text
Differential drive:
  command = (vx, wz)

Ackermann:
  command = (speed, steering_angle)

Omni base:
  command = (vx, vy, wz)

Legged base:
  command = body velocity + gait / foot-contact consistency
```

The guard is not tied to one robot model.

It is tied to one principle:

```text
a command should remain executable under the observed feedback stream
```

---

## Parameters

Important parameters include:

```text
yellow_threshold
red_threshold
slowdown_scale
cmd_ttl
min_dt
max_dt
phase_tolerance
max_linear_accel
max_angular_accel
max_linear_jerk
max_angular_jerk
position_tolerance
yaw_tolerance
lateral_tolerance
node_resync_good_frames
resync_required_good_frames
publish_zero_until_ready
flush_buffers_on_red
```

Reporter parameters include:

```text
guard_status_topic
guard_residual_topic
diagnostics_topic
vda_state_topic
summary_topic
vehicle_id
manufacturer
serial_number
status_timeout
publish_rate_hz
```

These can be configured through:

```text
ROS 2 launch arguments
YAML parameter files
ROS 2 node parameters
```

---

## Package structure

```text
ros2_kinematic_guard/
├── launch/
│   └── start_pressure_test.launch.py
├── ros2_kinematic_guard/
│   ├── __init__.py
│   ├── narh_lite_core.py
│   ├── jitter_injector_node.py
│   ├── kinematic_guard_node.py
│   ├── synthetic_odom_provider.py
│   └── reporter_node.py
├── package.xml
├── setup.py
├── setup.cfg
└── resource/
    └── ros2_kinematic_guard
```

---

## Mathematical appendix: NARH-lite for ROS 2 command flow

### Background

The original NARH formulation was developed for discrete rigid-body simulation pipelines.

In that setting, a system state is advanced by a sequence of sub-operators:

```text
s[t+1] = Ψσ(k) ∘ ... ∘ Ψσ(1)(s[t])
```

where the execution order may depend on solver internals such as constraint partitioning, thread scheduling, batching, or projection steps.

The original discrete associator is written as:

```text
A(a,b,c;s) =
    ((Ψa ∘ Ψb) ∘ Ψc)(s)
  - (Ψa ∘ (Ψb ∘ Ψc))(s)
```

and the residual is:

```text
R[t] = || A(a,b,c;s[t]) ||
```

The important point is that NARH does **not** claim that the physical state space itself is mathematically invalid.

It measures order-dependent deviations introduced by discrete numerical or computational pipelines.

`ros2_kinematic_guard` applies the same idea to ROS 2 command-flow consistency.

For the full high-dimensional NARH research background, see:

```text
SIPA — Simulation Integrity & Physics Auditor
https://github.com/ZC502/SIPA
```

---

### Command-flow operators

In ROS 2 mobile robot control, the relevant pipeline is not a simulator constraint solver.

It is a distributed command/feedback stream:

```text
command message
network delivery
controller execution
odometry feedback
resync / recovery
```

NARH-lite models this using a local kinematic window:

```text
C_prev = previous command
C_curr = current command

O_prev = previous odometry
O_curr = current odometry
```

The guard compares two interpretations of the same short motion window.

---

### Expected motion

The command stream predicts a local motion delta:

```text
Δ_expected = Integrate(C_prev, C_curr, dt)
```

For the default differential-drive / planar base model:

```text
v_avg = 0.5 * (v_prev + v_curr)
w_avg = 0.5 * (w_prev + w_curr)

Δx_expected   = v_avg * dt
Δyaw_expected = w_avg * dt
```

---

### Measured motion

The odometry stream gives the measured local delta:

```text
Δ_measured = LocalFrame(O_curr - O_prev)
```

For planar motion:

```text
Δx_measured
Δy_measured
Δyaw_measured
```

---

### Kinematic residual

The command/feedback residual is:

```text
R_cmd_odom =
    || Δ_expected - Δ_measured ||
```

In practice, the implementation separates the residual into interpretable components:

```text
linear_gap
lateral_gap
angular_gap
```

This allows diagnostics to explain why the guard intervened.

---

### Time-flow residual

Bad wireless timing often appears as:

```text
stale commands
near-zero dt
large dt jumps
phase mismatch between cmd and odom
burst delivery
replay windows
```

NARH-lite therefore adds timing residuals:

```text
R_timeflow
R_stale
R_phase
```

---

### Command smoothness residual

A command may be syntactically valid but physically unreasonable.

For example, a burst window can imply impossible acceleration or jerk:

```text
R_accel = violation(command_acceleration_limit)
R_jerk  = violation(command_jerk_limit)
```

---

### Final NARH-lite residual

The final residual is a weighted composition:

```text
R_NAR =
sqrt(
    w_timeflow * R_timeflow^2
  + w_stale    * R_stale^2
  + w_phase    * R_phase^2
  + w_accel    * R_accel^2
  + w_jerk     * R_jerk^2
  + w_odom     * R_cmd_odom^2
)
```

This is not a full octonion solver.

It is a lightweight engineering projection of NARH onto ROS 2 command-flow integrity.

---

## Roadmap

Planned improvements:

```text
C++ implementation of NARH-lite core
Dedicated CommandIntegrity message experiment
DiagnosticStatus convention documentation
rosbag / MCAP offline analyzer
VDA5050 bridge prototype
Ackermann model adapter
Omni-base model adapter
Foxglove / RViz visualization
Benchmark script for per-evaluation latency
```

---

## Suggested use cases

```text
mobile robots over Wi-Fi / 5G
recovery teleoperation
warehouse AMR diagnostics
research robot command-stream auditing
bag / MCAP post-incident analysis
fleet-level degraded-mode observability
VDA5050-style mixed-fleet telemetry experiments
```

---

## License

Apache-2.0

---

## Status

Experimental research prototype.

Use it as:

```text
a diagnostic layer
a pressure-test tool
a command-integrity telemetry bridge
a starting point for ROS 2 / fleet-level degraded-mode discussions
```

Do not use it as a certified safety function.
