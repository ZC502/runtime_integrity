# runtime_integrity
 **A ROS 2 Physical Execution Auditor and Lightweight Failure Labeler for Learned Autonomy.**

> Turn command-to-physical execution consistency into standard ROS diagnostics and machine-readable Sim2Real failure labels.

## The Sim2Real Failure-Mining Bottleneck

Learned local planners, reinforcement-learning navigation policies, and VLA-style robot policies can perform extremely well in simulation.

But when deployed on physical robots, they often fail for physical reasons that are hard to mine from raw logs:

```text
wheel slip
terrain friction changes
command/feedback timing jitter
localization jumps
stale command streams
odom discontinuities
```

For ML and robot-learning teams, the bottleneck is not only that these failures happen.

The bottleneck is that they are rarely converted into clean, reusable failure labels.

Typical field-data problem:

| Bottleneck            | What happens in practice                                                                                           |
| --------------------- | ------------------------------------------------------------------------------------------------------------------ |
| Data scale            | A long field run can produce tens of GB of rosbag data, while the useful failure event may last only a few seconds |
| Labeling cost         | Physical failures such as slip are often assumed to require torque, tactile, wheel-deformation, or terrain sensors |
| Extraction difficulty | Localization jumps and timing faults can require cross-checking odom, tf, lidar, IMU, and command streams manually |

When a learned planner commands:

```text
go forward
```

there is often no explicit runtime label that says:

```text
the robot physically executed the command
```

or:

```text
the robot slipped, jumped, drifted, desynchronized, or stopped responding physically
```

`runtime_integrity` is built to expose that missing signal.

## Automated Failure Mining from Command/Odom Consistency

`runtime_integrity` observes the causal alignment between a robot’s command stream and physical feedback:

```text
/cmd_vel + /odom
```

It then publishes execution-integrity state through standard ROS diagnostics:

```text
/diagnostics
  runtime_integrity/execution_integrity
```

The included CSV labeler converts those diagnostic events into machine-readable failure labels:

```bash
ros2 run ros2_kinematic_guard diagnostics_to_csv_labeler --ros-args \
  -p output_csv:=runtime_integrity_failure_labels.csv
```

This means you can replay a failed robot run or rosbag, run the observer, and export timestamps and residual metrics for events such as:

```text
WHEEL_SLIP
LOCALIZATION_JUMP
CMD_ODOM_MISMATCH
TIMING
STALE_DATA
MISSING_STREAM
```

Example CSV output:

```csv
ros_time_sec,diagnostic_level_name,status,dominantCause,totalResidual,cmdOdomResidual,wheelSlipIndex,localizationJumpMetric
1779799999.12,ERROR,RESYNCING,WHEEL_SLIP,1.730000,1.462117,1.23,0.0
1779800001.54,ERROR,RESYNCING,LOCALIZATION_JUMP,50.757462,42.897885,0.0,1.349308
```

These labels can be used for:

```text
Sim2Real debugging
hard-negative mining
OOD event detection
RL reward / cost shaping
failure dataset construction
post-deployment model analysis
```
💡 **Data Architecture Tip**: You don't just have to run this manually. For large-scale fleet operations or continuous Sim2Real training, you can integrate this into your overnight data ingestion pipeline. Replay your fleet's raw rosbags with `use_sim_time:=true` in headless CI/CD nodes to automatically index, label, and harvest physical failures from hundreds of hours of operational data without human intervention.

## Observe-Only by Design

`runtime_integrity` v0.3-alpha does not modify robot control behavior.

```text
No command interception.
No controller modification.
No Nav2 BT modification.
No base-driver modification.
```

It only observes command-to-physical execution consistency and exports the result through ROS-native diagnostics and CSV labels.


`runtime_integrity` currently focuses on three execution-integrity failure classes:

### Wheel Slip
The command stream expects motion, but measured odometry shows insufficient physical progress.
Typical diagnostic fields:
```text
dominantCause: WHEEL_SLIP
wheelSlipIndex: high
cmdOdomResidual: high
```

### Localization Jump
Odometry or pose feedback changes in a way that is inconsistent with the recent command window.
Typical diagnostic fields:
```
dominantCause: LOCALIZATION_JUMP
localizationJumpMetric: high
phaseResidual: high
```
### Timing / Command-Stream Disturbance
Command arrival timing becomes inconsistent with the expected control rhythm.
Typical diagnostic fields:
```
dominantCause: TIMING
cmdArrivalJitterMs: high
timeflowResidual: high
```
**What `runtime_integrity` is not**

`runtime_integrity` is not:
- a certified safety controller
- a safety-rated PLC
- a replacement for hardware E-stops
- a replacement for safety laser scanners
- a path planner
- a motion controller
- a sensor-fusion stack

**It is a passive runtime observer that turns command-to-physical execution consistency into a standard ROS diagnostic signal.**

One interesting edge case is localization jump detection. In real robots, EKF/SLAM pipelines may smooth sudden pose corrections over several frames, so the same physical issue may appear either as a hard **LOCALIZATION_JUMP** or as a sustained command/odom inconsistency. The observer exposes both the public diagnosis and the underlying residual metrics, so developers can see whether the failure was a hard pose discontinuity or a smoothed tracking inconsistency.

The current v0.3-alpha focus is strictly observational:
```
No command interception.
No controller modification.
No BT modification.
No base-driver modification.
```
Only observe and expose execution integrity through standard ROS diagnostics.

---

### Core Concept

Traditional robot health diagnostics usually answer infrastructure questions: Did the node crash? Did a message arrive? Did a controller timeout? Is a sensor publishing?

`runtime_integrity` asks a physics-boundary question:

>The autonomy stack commanded velocity X.
**Did the physical robot execute motion consistent with X?**

The command may come from:

- Nav2
- teleoperation
- a classical controller
- a learned local planner
- an RL policy
- a custom fleet stack

Once the command becomes robot motion intent, the runtime question is the same:

**Did the robot physically execute something consistent with the command?**

When command and physical feedback diverge due to contact dynamics, localization discontinuity, timing disturbance, or telemetry degradation, `runtime_integrity` publishes structured evidence directly to the standard ROS /diagnostics framework.

---

### ROS Diagnostic Signal

`runtime_integrity` publishes:
```
/diagnostics
```
with the diagnostic name:
```
runtime_integrity/execution_integrity
```

### Diagnostic Levels

| Runtime condition | Diagnostic level | Meaning |
|---|---:|---|
| Command and odometry are aligned | `OK` / Level 0 | Physical execution matches command intent |
| Mild residual or timing disturbance | `WARN` / Level 1 | Execution integrity is degraded |
| Severe execution collapse | `ERROR` / Level 2 | Command and physical feedback are no longer consistent |

---

### Target High-Density Diagnostic Output
```YAML
name: "runtime_integrity/execution_integrity"
level: 2
message: "BROKEN: WHEEL_SLIP"
hardware_id: "physics_boundary_observer"
values:
  - key: "status"
    value: "RESYNCING"
  - key: "dominantCause"
    value: "WHEEL_SLIP"
  - key: "totalResidual"
    value: "5.391204"
  - key: "causalAlignment"
    value: "BROKEN"
  - key: "mode"
    value: "observe"
  - key: "operatorAttentionRequired"
    value: "true"
  - key: "wheelSlipIndex"
    value: "4.821094"
  - key: "localizationJumpMetric"
    value: "0.000120"
  - key: "cmdOdomResidual"
    value: "5.180921"
  - key: "timeflowResidual"
    value: "0.041200"
  - key: "phaseResidual"
    value: "0.169083"
  - key: "cmdArrivalJitterMs"
    value: "4.21"
  - key: "odomArrivalJitterMs"
    value: "1.83"
  - key: "cmdTopic"
    value: "/cmd_vel"
  - key: "odomTopic"
    value: "/odom"
  - key: "lookbackWindowMs"
    value: "200"
```
Note: for unstamped `/cmd_vel`, timing metrics are based on observed message arrival intervals. For stronger stale-command and transport-latency detection, stamped command interfaces such as `TwistStamped` are recommended.

---

## Quick Start: Observe Your Own Robot

> Current ROS package name: `ros2_kinematic_guard`  
> Runtime concept name: `runtime_integrity`

### 1. Build the package

From the ROS 2 workspace root, the directory that contains `src/`:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ros2_kinematic_guard
source install/setup.bash
```
For every new terminal:
```Bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```
### 2. Launch the observer
```Bash
ros2 run ros2_kinematic_guard execution_observer_node --ros-args \
  -p cmd_input_topic:=/cmd_vel \
  -p odom_topic:=/odom \
  -p mode:=observe
```
For fused odometry:
```Bash
ros2 run ros2_kinematic_guard execution_observer_node --ros-args \
  -p cmd_input_topic:=/cmd_vel \
  -p odom_topic:=/odometry/filtered \
  -p mode:=observe
```
Or:
```Bash
ros2 run ros2_kinematic_guard execution_observer_node --ros-args \
  -p cmd_input_topic:=/cmd_vel \
  -p odom_topic:=/fusion/odom \
  -p mode:=observe
```
### 3. Inspect standard ROS diagnostics

Note: `DiagnosticStatus.level` is a byte field. Some terminals may display WARN as `"\x01"` and ERROR as `"\x02"`. For readability, `runtime_integrity` also publishes `diagnosticLevelInt` and `diagnosticLevelName` in the diagnostic key-value list.
```Bash
ros2 topic echo /diagnostics
```
Look for:
```
runtime_integrity/execution_integrity
```
Recommended visualization tools:
- Foxglove Studio
- `rqt_robot_monitor`
- `diagnostic_aggregator`
- any tool that consumes `/diagnostics`

---

### Real-World Transparency Test

To evaluate physical execution visibility, run `runtime_integrity` alongside your own robot or simulation and observe `/diagnostics` while the platform approaches known execution-boundary conditions.

Safe test options include:

- low-speed traction tests on a controlled test surface
- roller-bench or test-stand experiments
- recorded rosbag / MCAP replay
- Isaac Sim / Gazebo / Webots fault injection
- controlled localization jump simulation
- network shaping with tools such as tc/netem

Avoid unsafe field tests. Do not intentionally induce slip, localization failure, or network disruption on heavy robots in occupied or production environments.

The goal is simple:

**When the physical command-execution chain collapses, `runtime_integrity` should make that collapse visible through standard ROS diagnostics.**

Expected transition:

- OK    → command and physical motion are aligned
- WARN  → timing or residual degradation appears
- ERROR → physical execution consistency is broken

Example:
```
runtime_integrity/execution_integrity
level: ERROR
message: BROKEN: WHEEL_SLIP
```

---

### Synthetic Testing

If you do not have immediate access to real hardware or simulation, see:
```
docs/synthetic_testing.md
```
Synthetic tests are provided only as reproducible development tools.

The primary purpose of `runtime_integrity` is to observe real command-to-physical execution consistency in live robots, real simulations, or recorded operational data.

---

## Notes on Localization Jump Detection

`runtime_integrity` detects localization jumps by checking whether the observed pose change is physically plausible under the recent command window.

A hard pose discontinuity can produce a clear diagnostic event:

```yaml
message: "ERROR | RESYNCING: LOCALIZATION_JUMP"
dominantCause: "LOCALIZATION_JUMP"
totalResidual: "50.757462"
localizationJumpMetric: "1.349308"
cmdOdomResidual: "42.897885"
staleStreams: ""
```
However, real localization pipelines often do not expose pose correction as a perfect step function.

If the upstream EKF, SLAM backend, or sensor-fusion stack smooths a 2-meter correction over multiple frames, the same physical issue may appear as:
```
CMD_ODOM_MISMATCH
TIMING
sustained residual growth
```
rather than a single-frame `LOCALIZATION_JUMP`.

For reliable localization-jump testing, use one of the following:
- raw or minimally filtered pose output
- bag replay with known pose discontinuities
- simulator fault injection
- a controlled synthetic jump publisher
- a longer `lookback_window_ms` when observing heavily smoothed localization outputs

For unstamped `/cmd_vel`, timing alignment is based on message arrival time. For stricter command/pose causality analysis, a stamped command interface such as TwistStamped is recommended.

---

### Internal Residual Engine

`runtime_integrity` uses an internal residual engine to compare recent command streams with measured robot motion over short time windows.

The implementation details are intentionally abstracted at the middleware layer.

Integrators only need access to the resulting ROS diagnostic signal and structured execution-integrity evidence.

