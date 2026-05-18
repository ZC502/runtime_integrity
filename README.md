# ros2_kinematic_guard

## A pre-E-stop guard for ROS 2 AMR/AGV systems

`ros2_kinematic_guard` monitors `/cmd_vel` and `/odom` to detect when a mobile robot’s physical response no longer matches the command stream.

Before the robot escalates into shaking, spinning, collision risk, or hard E-stop, Kinematic Guard can slow down, brake, and resync locally.

It is designed for common AMR/AGV failure modes:

- wheel slip on wet or oily floors
- wheel-speed / odometry mismatch
- localization jumps from lidar / SLAM glitches
- bad Wi-Fi / 5G command bursts
- stale or replayed command windows
- robot shaking, spinning, or over-correcting before safety lidar cuts power

## Why Pre-E-stop Detection Matters

Safety-rated E-stop systems are the final protection layer.

Kinematic Guard does not replace them.

It tries to detect execution collapse earlier, before the certified safety layer is forced to intervene.

Frequent hard stops may contribute to:

- manual recovery time
- production interruption
- payload instability
- mechanical stress on wheels, reducers, and brakes
- unclear root cause during post-incident debugging

## Why Traditional Methods Fail (Why Not Just A Timeout?)

Most ROS 2 mobile robots rely on a basic `cmd_vel_timeout` node. If no command arrives within 0.5s, it publishes 0 to the base driver. 

Timeouts are necessary, but they answer only one question:
```text
Did a command arrive recently?
```
They do not answer:

**Does the robot’s measured motion still match the command stream?**

- **The "Ostrich Strategy" & Hard E-Stop Escalation:**
   When a robot spins out due to wheel slip or localization jumps, the controller continues to over-correct, sending aggressive commands. Currently, the industry relies on a crude fallback: let the robot shake, spin, or crash until the physical 
**Safety LIDAR / Hardware E-stop**
 triggers a hard cut.
- **The "Ghost Commands" Burst (Wi-Fi Jitter):**
   A standard timeout only protects against 
*silence*. It cannot handle network jitter. When a robot passes a Wi-Fi blind spot or roaming AP, the network layers buffer the commands. The moment connectivity recovers, 20 frames of queued `/cmd_vel`
 are injected into the base driver within 5 milliseconds like a machine gun. The robot experiences a violent acceleration burst ("Ghost Commands") before the timeout can even react.

### The Hidden Cost of Frequent Hard E-Stops:

Depending on the platform and payload, frequent hard stops may contribute to:
- **Mechanical Trauma:**
 A heavy AMR stopping instantly from full speed experiences massive inertial shock, causing gear striping (减速机打齿), shaft deformation, and wheel wear.
- **Electrical Back-EMF:**
 Sudden hard braking generates massive regenerative voltage spikes, risking damage to servo drive buses or BMS protection boards.
- **Production Downtime:**
 A locked E-stop requires a field engineer to walk onto the manufacturing line, manually reset the chassis with a joystick, and clear faults. It halts production and costs real money.

`ros2_kinematic_guard` bridges this gap. It acts as a **Pre-E-stop sanity layer**, letting the robot locally slow down, brake, and resync *before*
 the hardware safety layer is forced to intervene.

## Zero-code modification

Kinematic Guard works as an inline ROS 2 topic filter.

You do not need to modify Nav2, behavior trees, planners, controllers, or proprietary base drivers.

```text
Nav2 / teleop / planner
        ↓
      /cmd_vel
        ↓
Kinematic Guard
        ↓
  /safe_cmd_vel
        ↓
base driver
```

## Core question
Traditional timeout checks ask:
**Did a command arrive recently?**
Kinematic Guard asks:
**Is the robot still moving according to the command it was just given?**

## Main KinematicStatus Output
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

## Quick Start

```bash
cd ros2_kinematic_guard
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5-minute Demo: Wheel Slip Before Hard E-stop

**Deployment Modes**
- `mode:=observe` — passive monitoring only. No control intervention.
- `mode:=passthrough` — inline wiring test. `/safe_cmd_vel` equals `/cmd_vel`.
- `mode:=guard` — active mode. Can clamp velocity or enter `BRAKE_AND_RESYNC`.

Runtime parameter tuning is planned. For v0.2, thresholds are configured through launch arguments or YAML parameters.

**Terminal 1:**

Option A: Observe Mode — passive, no intervention
```bash
ros2 launch ros2_kinematic_guard start_pre_estop_demo.launch.py profile:=wheel_slip mode:=observe
```
Option B: Guard Mode — active clamp / brake / resync
```bash
ros2 launch ros2_kinematic_guard start_pre_estop_demo.launch.py profile:=wheel_slip mode:=guard
```

Terminal 2:
```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8}, angular: {z: 0.0}}"
```
Terminal 3:
```bash
ros2 topic echo /kinematic_guard/status
```
The robot was still receiving valid velocity commands, but its odometry no longer matched the commanded motion.
Kinematic Guard detected the execution collapse before a hard E-stop would be required.

### Pretty-print KinematicStatus JSON

`/kinematic_guard/status` is published as `std_msgs/String`, so a raw ROS 2 echo looks like this:

```text
data: '{"timestamp": ... }'
---
```
For a clean, human-readable JSON view, use:
```
ros2 topic echo /kinematic_guard/status --field data --once --full-length \
| awk '/^---$/{exit} {print}' \
| python3 -m json.tool
```
For continuous monitoring:
```Bash
watch -n 0.5 'ros2 topic echo /kinematic_guard/status --field data --once --full-length | awk "/^---$/{exit} {print}" | python3 -m json.tool'
```
To save one status sample:
```Bash
ros2 topic echo /kinematic_guard/status --field data --once --full-length \
| awk '/^---$/{exit} {print}' \
| python3 -m json.tool \
> kinematic_status_example.json
```
Verification Command:
```Bash
ros2 topic echo /kinematic_guard/status --field data --once --full-length \
| awk '/^---$/{exit} {print}' \
| python3 -m json.tool
```
```Bash
ros2 topic echo /safe_cmd_vel
```

## Expected Behavior

During the wheel-slip window, you should see:

```json
{
  "status": "RESYNCING",
  "causalAlignment": "BROKEN",
  "dominantCause": "WHEEL_SLIP",
  "guardAction": "BRAKE_AND_RESYNC",
  "safeCmd": {
    "linear_vx": 0.0,
    "angular_wz": 0.0
  }
}
```

In `mode:=observe`, the status turns red but the command stream is not modified.

In `mode:=guard`, /`safe_cmd`_vel is clamped or set to zero during `BRAKE_AND_RESYNC`.

## Optional Demo: Localization Jump

```bash
ros2 launch ros2_kinematic_guard start_pre_estop_demo.launch.py profile:=localization_jump mode:=guard
```

hen publish a smooth command:
```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```
Expected `dominantCause`:
```
LOCALIZATION_JUMP
```

## What is NARH-lite?

NARH-lite is the lightweight residual engine used inside Kinematic Guard.

It compares the recent command stream with odometry feedback over a sliding time window and asks:

```text
Did the robot move in a way that is still consistent with the command it just received?
```
In this package, NARH-lite is used only as an engineering metric for runtime command-feedback consistency.

It does not replace safety-rated E-stop systems, certified safety controllers, or hardware safety layers.
