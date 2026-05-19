# ros2_kinematic_guard

## A pre-E-stop guard for ROS 2 AMR/AGV systems

`ros2_kinematic_guard` monitors `/cmd_vel` and `/odom` to detect when a mobile robot’s physical response no longer matches the command stream.

Before the robot escalates into shaking, spinning, collision risk, or hard E-stop, Kinematic Guard can slow down, brake, and resync locally. It is designed for common AMR/AGV failure modes:

- wheel slip on wet or oily floors
- wheel-speed / odometry mismatch
- localization jumps from lidar / SLAM glitches
- bad Wi-Fi / 5G command bursts
- stale or replayed command windows
- robot shaking, spinning, or over-correcting before safety lidar cuts power

## Why Pre-E-stop Detection Matters

Safety-rated E-stop systems are the final protection layer. Kinematic Guard does not replace them.

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

## Repository Layout

This repository is organized as a ROS 2 workspace:

```text
repo-root/
├── src/
│   └── ros2_kinematic_guard/
│       ├── package.xml
│       ├── setup.py
│       ├── launch/
│       └── ros2_kinematic_guard/
│           ├── kinematic_guard_node.py
│           ├── mock_robot_simulator.py
│           └── narh_lite_core.py
```
Run `colcon build` from the repository root, not from inside `src/ros2_kinematic_guard`.

## Quick Start

This repository is a ROS 2 workspace. Run the following commands from the repository root, the directory that contains `src/`.

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
git clone https://github.com/ZC502/ros2_kinematic_guard.git
cd ros2_kinematic_guard

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
Every new terminal must source the overlay again:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 5-minute Demo: Wheel Slip Before Hard E-stop

This demo runs a lightweight virtual AMR/AGV without Gazebo or Isaac Sim.

It creates this closed loop:

~~~text
/cmd_vel
   ↓
Kinematic Guard
   ↓
/safe_cmd_vel
   ↓
Mock Robot
   ↓
/odom
   ↑
Kinematic Guard
~~~

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

~~~bash
pkill -f kinematic_guard_node || true
pkill -f mock_robot_simulator || true
pkill -f "ros2 topic pub" || true
ros2 daemon stop
ros2 daemon start
~~~

You can also check that there is only one `/odom` publisher:

~~~bash
ros2 topic info /odom -v
~~~

There should be only one `/odom` publisher from `mock_robot`.

---

## Demo A: Lifecycle Demo

This is the recommended demo for first-time users.

It gives you 10 seconds to open the monitoring terminals before wheel slip begins, then injects wheel slip for 12 seconds.

Expected story:

~~~text
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
~~~

---

### Terminal 1A: Observe Mode — passive, no intervention

Use this first. It lets you see the Guard detect the failure without changing the command stream.

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ros2_kinematic_guard start_pre_estop_demo.launch.py \
  profile:=wheel_slip \
  mode:=observe \
  slip_start_sec:=10.0 \
  slip_duration_sec:=12.0
~~~

In this mode, the Guard reports the failure but does not modify the command stream.

---

### Terminal 1B: Guard Mode — active clamp / brake / resync

Use this instead of Terminal 1A when you want to see `/safe_cmd_vel` being clamped or set to zero.

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ros2_kinematic_guard start_pre_estop_demo.launch.py \
  profile:=wheel_slip \
  mode:=guard \
  slip_start_sec:=10.0 \
  slip_duration_sec:=12.0
~~~

---

### Terminal 2: Prepare Kinematic Guard status monitor

Start this before publishing `/cmd_vel`.

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash

watch -n 0.2 'ros2 topic echo /kinematic_guard/status --field data --once --full-length | awk "/^---$/{exit} {print}" | python3 -m json.tool'
~~~

---

### Terminal 3: Prepare mock robot status monitor

This confirms whether the virtual robot is currently slipping.

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash

watch -n 0.2 'ros2 topic echo /mock_robot/status --field data --once --full-length | awk "/^---$/{exit} {print}" | python3 -m json.tool'
~~~

Wait for:

~~~json
{
  "profile": "wheel_slip",
  "faultState": "WHEEL_SLIP"
}
~~~

If you see:

~~~json
{
  "faultState": "NONE"
}
~~~

then the robot is currently in a healthy window, and `/kinematic_guard/status` may correctly remain `GREEN`.

---

### Terminal 4: Publish a smooth velocity command

Start this after the monitors are ready.

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8}, angular: {z: 0.0}}"
~~~

---

### Terminal 5: Watch the actual command sent to the base

This is especially useful in `mode:=guard`.

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /safe_cmd_vel
~~~

---

## Demo B: Persistent Fault Debug Demo

Use this when you want a long fault window and do not want to miss the slip event.

This mode keeps wheel slip active for a long time:

~~~text
slip_duration_sec:=9999.0
~~~

Observe mode:

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ros2_kinematic_guard start_pre_estop_demo.launch.py \
  profile:=wheel_slip \
  mode:=observe \
  slip_start_sec:=3.0 \
  slip_duration_sec:=9999.0
~~~

Guard mode:

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ros2_kinematic_guard start_pre_estop_demo.launch.py \
  profile:=wheel_slip \
  mode:=guard \
  slip_start_sec:=3.0 \
  slip_duration_sec:=9999.0
~~~

This is useful for debugging, screenshots, and stress testing.

Because the fault persists for a long time, the system may stay in `RESYNCING` until the demo is restarted or the fault window ends.

---

## Expected Behavior

### Healthy window

When `/cmd_vel` and `/odom` agree, the Guard should stay quiet:

~~~json
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
~~~

This is normal and desirable. It shows that Kinematic Guard does not create false positives when the robot motion matches the command stream.

---

### Wheel-slip window in observe mode

When `/mock_robot/status` shows `faultState=WHEEL_SLIP`, Kinematic Guard should report that command-feedback integrity is broken.

In `mode:=observe`, the Guard reports the failure but does not modify the command stream:

~~~json
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
~~~

---

### Wheel-slip window in guard mode

In `mode:=guard`, the Guard can clamp or brake the command stream:

~~~json
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
~~~

At the same time, `/safe_cmd_vel` should show the clamped or zero command.

---

## Pretty-print KinematicStatus JSON

`/kinematic_guard/status` is published as `std_msgs/String`, so a raw ROS 2 echo looks like this:

~~~text
data: '{"timestamp": ... }'
---
~~~

For a clean, human-readable JSON view, use:

~~~bash
ros2 topic echo /kinematic_guard/status --field data --once --full-length \
| awk '/^---$/{exit} {print}' \
| python3 -m json.tool
~~~

For continuous monitoring:

~~~bash
watch -n 0.2 'ros2 topic echo /kinematic_guard/status --field data --once --full-length | awk "/^---$/{exit} {print}" | python3 -m json.tool'
~~~

To save one status sample:

~~~bash
ros2 topic echo /kinematic_guard/status --field data --once --full-length \
| awk '/^---$/{exit} {print}' \
| python3 -m json.tool \
> kinematic_status_example.json
~~~

---

## Troubleshooting

### I only see `GREEN`

First check the mock robot:

~~~bash
ros2 topic echo /mock_robot/status --field data --once --full-length \
| awk '/^---$/{exit} {print}' \
| python3 -m json.tool
~~~

If `faultState` is `NONE`, then the robot is not currently slipping. This is a healthy window.

If `faultState` is `WHEEL_SLIP` but Kinematic Guard still stays `GREEN`, check for duplicate `/odom` publishers:

~~~bash
ros2 topic info /odom -v
~~~

There should be only one `/odom` publisher from `mock_robot`.

---

### I see `LOCALIZATION_JUMP` during the wheel-slip demo

This usually means there are multiple `/odom` publishers or old demo nodes still running.

Clean old processes:

~~~bash
pkill -f kinematic_guard_node || true
pkill -f mock_robot_simulator || true
pkill -f "ros2 topic pub" || true
ros2 daemon stop
ros2 daemon start
~~~

---

### In guard mode, the system stays in `RESYNCING`

This can happen if the command publisher keeps sending a forward command while the Guard is braking.

Stop the `/cmd_vel` publisher, or publish zero velocity:

~~~bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
~~~

Then watch for clean windows and recovery.

## Optional Demo: Localization Jump

```bash
ros2 launch ros2_kinematic_guard start_pre_estop_demo.launch.py profile:=localization_jump mode:=guard
```

Then publish a smooth command:
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
