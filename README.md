# runtime_integrity

### Runtime Accountability & Execution Integrity Middleware for Autonomous Mobile Robots

In regulated autonomous systems, **“Why did the robot do that?”** is no longer only a debugging question. It is becoming an operational, safety, and compliance requirement.

`runtime_integrity` is a middleware layer that monitors the relationship between autonomous commands and physical execution.

It helps answer:

```text
What did the autonomy stack command?
What did the robot actually do?
Was the command still physically consistent?
Was any intervention or downgrade triggered?
Can this event be explained later to an operator, safety officer, or auditor?
```

The project was formerly known as `ros2_kinematic_guard`.

---

### What runtime_integrity Does

`runtime_integrity` observes command streams and physical feedback over short runtime windows.

When the robot’s measured motion no longer matches the command stream, it emits structured execution-integrity evidence.

Typical outputs include:
- execution state
- residual score
- dominant cause
- intervention recommendation
- safe command output
- audit-ready event data

Example:
```JSON
{
  "status": "RESYNCING",
  "residual": 5.391,
  "causalAlignment": "BROKEN",
  "dominantCause": "WHEEL_SLIP",
  "guardAction": "BRAKE_AND_RESYNC",
  "safeCmd": {
    "linear_vx": 0.0,
    "angular_wz": 0.0
  }
}
```

---

### Compliance Readiness

`runtime_integrity` is designed to support manufacturers and integrators preparing for runtime accountability, technical logging, and human oversight requirements.

It can support evidence workflows related to:
- runtime audit logging
- intervention evidence
- human oversight dashboards
- post-incident reconstruction
- fleet-level operational monitoring
- structured event export for external audit systems

**Compliance-Oriented Capabilities**

| Capability            | Purpose                                                                                    |
| --------------------- | ------------------------------------------------------------------------------------------ |
| Runtime Audit         | Tracks whether command streams remain physically consistent with robot motion.             |
| Intervention Evidence | Records when a command is downgraded, clamped, or flagged for resync.                      |
| Human Oversight Hooks | Provides structured runtime signals for HMI, FMS, or operator dashboards.                  |
| Audit Event Output    | Emits machine-readable JSON events for logging, SIEM, audit databases, or fleet analytics. |
| Cause Classification  | Labels events such as `WHEEL_SLIP`, `LOCALIZATION_JUMP`, `TIMING`, or `CMD_ODOM_MISMATCH`. |

`runtime_integrity` is **not a compliance product by itself**. It provides runtime evidence that can be integrated into a broader compliance, safety, and quality-management system.

---

### Human Oversight Workflow

A typical deployment can look like this:
```
[ Autonomy / Navigation Stack ]
        │
        ▼
Generates command stream
        │
        ▼
[ runtime_integrity ]
Monitors command-to-feedback consistency
        │
        ▼
Execution anomaly detected
        │
        ▼
Structured causal event emitted
        │
        ▼
[ HMI / FMS / Audit Gateway ]
Maps event to operator-facing alert
        │
        ▼
[ Human Decision Loop ]
Operator can acknowledge, override, pause, resume, inspect, or escalate
```
`runtime_integrity` does not prescribe a universal HMI. Industrial HMI and Fleet Management System architectures are fragmented across vendors.

Instead, the project exposes structured status and event streams that can be mapped into custom HMI, FMS, PLC, SIEM, or audit workflows.

---

### Operational Context & Scenarios

`runtime_integrity` is intended for mobile robots operating in physically uncertain environments where command execution can diverge from system intent.

Example scenarios:
- **AMR traction loss**
Micro-slip or severe slip on polished, wet, dusty, or oily floors.
- **Localization inconsistency**
Sudden pose or odometry jumps after SLAM relocalization, lidar mismatch, Wi-Fi roaming recovery, or sensor dropouts.
- **GNSS / outdoor logistics drift**
Mismatch between commanded paths and fused GNSS / IMU / encoder odometry in outdoor logistics corridors.
- **Autonomous planner divergence**
High-level autonomy continues issuing commands while measured robot motion no longer matches expected physical behavior.
- **Mixed-fleet accountability**
Structured execution-integrity evidence can be aggregated across robot types, vendors, and fleet-management systems.

---

### What runtime_integrity Is NOT

`runtime_integrity` is not:

- a safety-rated PLC
- a certified collision-avoidance system
- a replacement for hardware E-stops
- a replacement for safety laser scanners
- a full sensor-fusion stack
- a path planner
- a motion controller
- a legal compliance guarantee

It is a **runtime execution-integrity and accountability layer** designed to observe, explain, and record command-to-physical-execution divergence.

Certified functional safety systems remain mandatory where required.

---

### Deployment Model

`runtime_integrity` can operate as an inline middleware component.
```
Navigation / Teleop / Planner
        ↓
      /cmd_vel
        ↓
runtime_integrity
        ↓
  /safe_cmd_vel
        ↓
Base Driver
```
This provides a low-friction deployment path:
- no modification to Nav2
- no modification to behavior trees
- no modification to existing planners
- no modification to proprietary base drivers
- can start in passive observe mode
- can later be connected to custom HMI / fleet dashboards

---

### Core Question

Traditional timeout checks ask:
```
Did a command arrive recently?
```
`runtime_integrity` asks:

**Is the robot still moving according to the command it was just given?**

This distinction matters because a robot can continue receiving commands while physical execution has already become inconsistent.

Examples:
- wheels are spinning but the robot is not moving as expected
- localization jumps while commands continue
- stale or bursty commands arrive after network recovery
- odometry diverges from commanded motion
- the system begins over-correcting before hardware safety layers intervene

---

### Planned Enterprise Audit Event Schema

The current runtime status stream already exposes execution-integrity evidence.

A future enterprise audit event schema may include:
```JSON
{
  "timestamp": "2026-05-22T11:04:23.194Z",
  "audit_event_id": "evt-2026-00014",
  "robot_id": "amr-fleet-04",
  "source_id": "runtime_integrity_node_01",

  "status": "RESYNCING",
  "dominantCause": "WHEEL_SLIP",
  "residual": 5.39,
  "guardAction": "BRAKE_AND_RESYNC",

  "interventionRequired": true,

  "inputCommand": {
    "topic": "/cmd_vel",
    "linear_vx": 0.8,
    "angular_wz": 0.0
  },

  "outputCommand": {
    "topic": "/safe_cmd_vel",
    "linear_vx": 0.0,
    "angular_wz": 0.0
  },

  "stateSource": {
    "topic": "/fusion/odom",
    "type": "nav_msgs/Odometry"
  },

  "complianceTags": [
    "human_oversight",
    "runtime_intervention",
    "execution_integrity_audit"
  ]
}
```
These fields are intended for integration with external audit sinks, fleet-management systems, signed evidence chains, SIEM systems, or compliance reporting pipelines.

---

### Use the Best Available Odometry

`runtime_integrity` does not require raw wheel odometry.

For real AMR/AGV deployments, it is usually better to feed the system with the most trusted odometry source available:
- `/odom`
- `/odometry/filtered`
- `/fusion/odom`
- visual-inertial odometry
- GNSS / IMU / encoder fused odometry

`runtime_integrity` is not a sensor-fusion stack. It does not decide what the robot’s state estimate should be.

Instead, it asks:

**Given the best available odometry, does the robot’s measured motion still match the command stream?**

Example:
```BASH
ros2 run runtime_integrity kinematic_guard_node --ros-args \
  -p cmd_input_topic:=/cmd_vel \
  -p odom_topic:=/fusion/odom \
  -p cmd_output_topic:=/safe_cmd_vel \
  -p mode:=observe
```
If you already use `robot_localization`, FusionCore, or another state-estimation pipeline, `runtime_integrity` should consume that fused odometry output rather than raw encoder-only odometry whenever possible.

A cleaner odometry signal can reduce false positives and improve the quality of execution-integrity decisions.

---

**Current ROS 2 Demo**

The current open-source demo is ROS 2 based.

It demonstrates the core concept using a lightweight virtual AMR/AGV, without Gazebo, Isaac Sim, or real hardware.

---

### Repository Layout

This repository is organized as a ROS 2 workspace:
```
repo-root/
├── src/
│   └── runtime_integrity/
│       ├── package.xml
│       ├── setup.py
│       ├── launch/
│       └── runtime_integrity/
│           ├── kinematic_guard_node.py
│           ├── mock_robot_simulator.py
│           └── narh_lite_core.py
```
Run `colcon build` from the repository root, not from inside `src/runtime_integrity`.

---

## Quick Start

Run the following commands from the repository root, the directory that contains `src/`.

```bash
# Check that you are at the repository root
ls src

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
If you open this repository in GitHub Codespaces, the terminal usually starts at the repository root.

If you cloned it locally:
```bash
git clone https://github.com/ZC502/runtime_integrity.git
cd runtime_integrity

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
Every new terminal must source the overlay again:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```
---

## 5-minute Demo: Wheel Slip Before Hard E-stop

This demo creates the following closed loop:

```text
/cmd_vel
   ↓
runtime integrity
   ↓
/safe_cmd_vel
   ↓
Mock Robot
   ↓
/odom
   ↑
runtime integrity
```

The mock robot injects a wheel-slip fault after it receives the first non-zero `/safe_cmd_vel`.

There are two recommended demo modes:

- **Demo A: Lifecycle Demo** — shows `GREEN → RESYNCING` and then returns to a healthy window after the slip ends.
- **Demo B: Persistent Fault Debug Demo** — keeps wheel slip active for a long time so you do not miss the fault window.

For the first community-facing demo, use **Demo A**.

---

### Deployment Modes

- `mode:=observe` — passive monitoring only. No control intervention.
- `mode:=passthrough` — inline wiring test. `/safe_cmd_vel` equals `/cmd_vel`.
- `mode:=guard` — active mode. Can clamp velocity or enter `BRAKE_AND_RESYNC`.

---

### Terminal 0: Clean old demo processes

Before switching between `observe` and `guard`, stop old nodes:

```bash
pkill -f kinematic_guard_node || true
pkill -f mock_robot_simulator || true
pkill -f "ros2 topic pub" || true
ros2 daemon stop
ros2 daemon start
```

You can also check that there is only one `/odom` publisher:

```bash
ros2 topic info /odom -v
```

There should be only one `/odom` publisher from `mock_robot`.

---

## Demo A: Lifecycle Demo

This is the recommended demo for first-time users.

It gives you 10 seconds to open the monitoring terminals before wheel slip begins, then injects wheel slip for 12 seconds.

Expected story:

```text
healthy motion
    ↓
GREEN
    ↓
wheel slip starts
    ↓
RESYNCING
    ↓
wheel slip ends
    ↓
healthy window again
```

---

### Terminal 1A: Observe Mode — passive, no intervention

Use this first. It lets you see the Guard detect the failure without changing the command stream.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch runtime_integrity start_pre_estop_demo.launch.py \
  profile:=wheel_slip \
  mode:=observe \
  slip_start_sec:=10.0 \
  slip_duration_sec:=12.0
```

---

### Terminal 1B: Guard Mode — active clamp / brake / resync

Use this instead of Terminal 1A when you want to see `/safe_cmd_vel` being clamped or set to zero.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch runtime_integrity start_pre_estop_demo.launch.py \
  profile:=wheel_slip \
  mode:=guard \
  slip_start_sec:=10.0 \
  slip_duration_sec:=12.0
```

---

### Terminal 2: Prepare runtime integrity status monitor

Start this before publishing `/cmd_vel`.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

watch -n 0.2 'ros2 topic echo /kinematic_guard/status --field data --once --full-length | awk "/^---$/{exit} {print}" | python3 -m json.tool'
```

---

### Terminal 3: Prepare mock robot status monitor

This confirms whether the virtual robot is currently slipping.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

watch -n 0.2 'ros2 topic echo /mock_robot/status --field data --once --full-length | awk "/^---$/{exit} {print}" | python3 -m json.tool'
```

Wait for:

```json
{
  "profile": "wheel_slip",
  "faultState": "WHEEL_SLIP"
}
```

If you see:

```json
{
  "faultState": "NONE"
}
```

then the robot is currently in a healthy window, and `/kinematic_guard/status` may correctly remain `GREEN`.

---

### Terminal 4: Publish a smooth velocity command

Start this after the monitors are ready.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8}, angular: {z: 0.0}}"
```

---

### Terminal 5: Watch the actual command sent to the base

This is especially useful in `mode:=guard`.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /safe_cmd_vel
```

---

## Demo B: Persistent Fault Debug Demo

Use this when you want a long fault window and do not want to miss the slip event.

Observe mode:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch runtime_integrity start_pre_estop_demo.launch.py \
  profile:=wheel_slip \
  mode:=observe \
  slip_start_sec:=3.0 \
  slip_duration_sec:=9999.0
```

Guard mode:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch runtime_integrity start_pre_estop_demo.launch.py \
  profile:=wheel_slip \
  mode:=guard \
  slip_start_sec:=3.0 \
  slip_duration_sec:=9999.0
```

This is useful for debugging, screenshots, and stress testing.

Because the fault persists for a long time, the system may stay in `RESYNCING` until the demo is restarted or the fault window ends.

---

## Expected Behavior

### Healthy window

When `/cmd_vel` and `/odom` agree, the Guard should stay quiet:

```json
{
  "status": "GREEN",
  "residual": 0.0009,
  "causalAlignment": "ALIGNED",
  "dominantCause": "NONE",
  "guardAction": "OBSERVE_ONLY",
  "safeCmd": {
    "linear_vx": 0.8,
    "angular_wz": 0.0
  }
}
```

This shows that `runtime_integrity` does not create false positives when robot motion matches the command stream.

---

### Wheel-slip window in observe mode

When `/mock_robot/status` shows `faultState=WHEEL_SLIP`, `runtime_integrity` should report that command-feedback integrity is broken.

In `mode:=observe`, the Guard reports the failure but does not modify the command stream:

```json
{
  "status": "RESYNCING",
  "causalAlignment": "BROKEN",
  "dominantCause": "WHEEL_SLIP",
  "guardAction": "OBSERVE_ONLY",
  "mode": "observe",
  "controlInterceptionEnabled": false,
  "safeCmd": {
    "linear_vx": 0.8,
    "angular_wz": 0.0
  }
}
```

---

### Wheel-slip window in guard mode

In `mode:=guard`, the system can clamp or brake the command stream:

```json
{
  "status": "RESYNCING",
  "causalAlignment": "BROKEN",
  "dominantCause": "WHEEL_SLIP",
  "guardAction": "BRAKE_AND_RESYNC",
  "mode": "guard",
  "controlInterceptionEnabled": true,
  "safeCmd": {
    "linear_vx": 0.0,
    "angular_wz": 0.0
  }
}
```

At the same time, `/safe_cmd_vel` should show the clamped or zero command.

---

## Pretty-print KinematicStatus JSON

`/kinematic_guard/status` is published as `std_msgs/String`, so a raw ROS 2 echo looks like this:

```text
data: '{"timestamp": ... }'
---
```

For a clean, human-readable JSON view, use:

```bash
ros2 topic echo /kinematic_guard/status --field data --once --full-length \
| awk '/^---$/{exit} {print}' \
| python3 -m json.tool
```

For continuous monitoring:

```bash
watch -n 0.2 'ros2 topic echo /kinematic_guard/status --field data --once --full-length | awk "/^---$/{exit} {print}" | python3 -m json.tool'
```

To save one status sample:

```bash
ros2 topic echo /kinematic_guard/status --field data --once --full-length \
| awk '/^---$/{exit} {print}' \
| python3 -m json.tool \
> runtime_integrity_status_example.json
```

---

### Optional Demo: Localization Jump
```Bash
ros2 launch runtime_integrity start_pre_estop_demo.launch.py profile:=localization_jump mode:=guard
```
Then publish a smooth command:
```Bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```
Expected `dominantCause`:
```
LOCALIZATION_JUMP
```

---

## Troubleshooting

### I only see `GREEN`

First check the mock robot:

```bash
ros2 topic echo /mock_robot/status --field data --once --full-length \
| awk '/^---$/{exit} {print}' \
| python3 -m json.tool
```

If `faultState` is `NONE`, then the robot is not currently slipping. This is a healthy window.

If `faultState` is `WHEEL_SLIP` but `runtime_integrity` still stays `GREEN`, check for duplicate `/odom` publishers:

```bash
ros2 topic info /odom -v
```

There should be only one `/odom` publisher from `mock_robot`.

---

### I see `LOCALIZATION_JUMP` during the wheel-slip demo

This usually means there are multiple `/odom` publishers or old demo nodes still running.

Clean old processes:

```bash
pkill -f kinematic_guard_node || true
pkill -f mock_robot_simulator || true
pkill -f "ros2 topic pub" || true
ros2 daemon stop
ros2 daemon start
```

---

### In guard mode, the system stays in `RESYNCING`

This can happen if the command publisher keeps sending a forward command while the Guard is braking.

Stop the `/cmd_vel` publisher, or publish zero velocity:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Then watch for clean windows and recovery.

---

### Internal Residual Engine

`runtime_integrity` uses an internal residual engine to compare recent command streams with measured robot motion over short time windows.

The implementation details are intentionally abstracted at the middleware layer. Integrators only need access to the resulting execution-integrity signals and structured audit events.
