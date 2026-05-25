# runtime_integrity v0.3-alpha

### Turning command-to-physical execution consistency into a first-class ROS diagnostic signal.

`runtime_integrity` is a non-invasive, diagnostics-native execution observer for ROS 2 mobile robots. It monitors whether the robot’s physical motion remains consistent with the recent command stream over short runtime windows.

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

Diagnostic Levels
Runtime condition	Diagnostic level	Meaning
Command and odometry are aligned	OK / Level 0	Physical execution matches command intent
Mild residual or timing disturbance	WARN / Level 1	Execution integrity is degraded
Severe execution collapse	ERROR / Level 2	Command and physical feedback are no longer consistent

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

From the workspace root:

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

### Internal Residual Engine

`runtime_integrity` uses an internal residual engine to compare recent command streams with measured robot motion over short time windows.

The implementation details are intentionally abstracted at the middleware layer.

Integrators only need access to the resulting ROS diagnostic signal and structured execution-integrity evidence.

