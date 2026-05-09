# ROS 2 Kinematic Guard
**NARH-based（(Non-Associative Residual Hypothesis) Kinematic Guard for ROS 2**
>ROS 2 transmits messages. NARH Guard ensures those messages are still executable.
It does not fix the network. It prevents bad network timing from becoming dangerous robot motion.

`ros2_kinematic_guard` is a lightweight safety middleware for ROS 2 mobile robots.
It detects stale, bursty, delayed, replayed, or physically inconsistent velocity commands, then outputs a protected `safe_cmd_vel`.

The killer feature is:
```
BRAKE_AND_RESYNC
```
When command timing collapses, the guard does not merely warn. It can actively cut motion, flush poisoned command windows, wait for a fresh command/odom window, and then recover.

---

## 1. ⚠️ The Pain: Control Semantic Collapse

In real Wi-Fi / 5G / congested robot networks, the problem is not only packet loss.

The deeper issue is **control semantic collapse**:
```
The message arrives.
But it is no longer physically executable.
```

Typical failure modes:

### 1.1 Stale command replay

A robot receives a command that was valid hundreds of milliseconds ago, but dangerous now.

Example:
```
old forward command arrives after a stop command
```
The base may execute a “ghost command” from the past.

### 1.2 Burst release

Under bad wireless timing, commands may accumulate in buffers and then arrive in a burst.

Your controller sees:
```
cmd_1, cmd_2, cmd_3, cmd_4...
```
all within a tiny time window.

This can create abnormal acceleration or jerk demand.

### 1.3 Time-flow disorder

`/cmd_vel` is commonly published as `geometry_msgs/Twist`, which has no header timestamp.
Many lower-level controllers therefore execute whatever command arrives, without knowing whether that command is fresh, stale, duplicated, or replayed.

### Demo Proof: The Chaos
The built-in Bad-WiFi injector produces exactly these toxic timing patterns:
```
DUPLICATE_IN_BURST
BURST_BUFFER_OVERFLOW_DROP_OLDEST
BURST_RELEASE_count=8
DELAY_0.453s
DROP
REPLAY_STALE
```
```
TE_IN_BURST | vx=0.250, wz=0.000
[jitter_injector_node-1] [INFO] [1778242894.834988181] [jitter_injector]: [JITTER] DUPLICATE_IN_BURST | vx=0.250, wz=0.000
[jitter_injector_node-1] [INFO] [1778242894.884439806] [jitter_injector]: [JITTER] BURST_BUFFER_OVERFLOW_DROP_OLDEST | vx=0.250, wz=0.000
[jitter_injector_node-1] [INFO] [1778242894.885795719] [jitter_injector]: [JITTER] BURST_BUFFER_OVERFLOW_DROP_OLDEST | vx=0.250, wz=0.000
[jitter_injector_node-1] [INFO] [1778242894.941915902] [jitter_injector]: [JITTER] BURST_BUFFER_OVERFLOW_DROP_OLDEST | vx=0.250, wz=0.000
[jitter_injector_node-1] [INFO] [1778242894.971344953] [jitter_injector]: [JITTER] BURST_BUFFER_OVERFLOW_DROP_OLDEST | vx=0.250, wz=0.000
[jitter_injector_node-1] [INFO] [1778242894.990200348] [jitter_injector]: [JITTER] BURST_BUFFER_OVERFLOW_DROP_OLDEST | vx=0.250, wz=0.000
[jitter_injector_node-1] [INFO] [1778242895.000384268] [jitter_injector]: [JITTER]
```
The test log also shows synthetic physical disturbance events such as SLIP_START / SLIP_END, meaning the demo includes both network timing corruption and feedback disturbance. 

## 2. ✅ The Cure: NARH-lite Guard & Quick Start
NARH Guard monitors the relationship between:
```
previous command
current command
previous odometry
current odometry
```
It computes a lightweight residual:
```
R_NAR
```
If the command/feedback window is still executable, the guard passes or gently smooths the command.
If the residual crosses a danger threshold, the guard enters:
```
RED_BRAKE
BRAKE_AND_RESYNC
RESYNCING
```
```
data: '{"timestamp": 1778242938.816956, "status": "RESYNCING", "action": "BRAKE_AND_RESYNC", "r_nar": 0.0035744605798055853, "safe_cmd"...'
---
data: '{"timestamp": 1778242939.020653, "status": "RESYNCING", "action": "BRAKE_AND_RESYNC", "r_nar": 0.0019947126815960275, "safe_cmd"...'
---
data: '{"timestamp": 1778242939.2178075, "status": "RESYNCING", "action": "BRAKE_AND_RESYNC", "r_nar": 5.477226981939539, "safe_cmd": {...'
---
data: '{"timestamp": 1778242939.4169483, "status": "RESYNCING", "action": "BRAKE_AND_RESYNC", "r_nar": 5.749790876935357, "safe_cmd": {...'
---
data: '{"timestamp": 1778242939.6169808, "status": "RESYNCING", "action": "BRAKE_AND_RESYNC", "r_nar": 6094.8047968893925, "safe_cmd": ...'
---
data: '{"timestamp": 1778242939.8169532, "status": "RESYNCING", "action": "BRAKE_AND_RESYNC", "r_nar": 0.08677715954568423, "safe_cmd":...'
```
In the pressure test, `/kinematic_guard/status` shows `RESYNCING` and `BRAKE_AND_RESYNC` repeatedly during toxic command windows, with `R_NAR` spikes such as `5.47`, `5.74`, `13.83`, and even `6094.80`in an extreme timing-collapse window. 

Example status:
```
{
  "status": "RESYNCING",
  "action": "BRAKE_AND_RESYNC",
  "r_nar": 5.749,
  "safe_cmd": {
    "vx": 0.0,
    "wz": 0.0
  }
}
```
Meaning:
```
The command stream is no longer trusted.
Motion is cut.
The guard waits for a fresh command/odom window.
```

---

# Quick Start: 30 Seconds to Chaos
**Build**
```
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
**Run the pressure test**
```
ros2 launch ros2_kinematic_guard start_pressure_test.launch.py
```
**More aggressive test**
```
ros2 launch ros2_kinematic_guard start_pressure_test.launch.py \
  profile:=wifi_collapse \
  slip_probability:=0.02 \
  red_threshold:=4.0
```
**Watch the guard status**
```
ros2 topic echo /kinematic_guard/status
```
**Watch the residual**
```
ros2 topic echo /kinematic_guard/residual
```
**Watch the protected command**
```
ros2 topic echo /kinematic_guard/safe_cmd_vel
```

---

**No Robot Required**

You do not need:
```
real robot
Gazebo
Isaac Sim
Nav2
hardware motor driver
```
This repo includes a complete closed-loop pressure test:
```
jitter_injector_node.py
    creates toxic /cmd_vel_jittered

kinematic_guard_node.py
    computes R_NAR and outputs /kinematic_guard/safe_cmd_vel

synthetic_odom_provider.py
    acts as a virtual robot body and publishes /odom
```
The full loop:
```
/cmd_vel_raw
  ↓
jitter_injector_node.py
  ↓
/cmd_vel_jittered
  ↓
kinematic_guard_node.py
  ↓
/kinematic_guard/safe_cmd_vel
  ↓
synthetic_odom_provider.py
  ↓
/odom
  ↺
kinematic_guard_node.py
```

---

**Why this is not just a velocity smoother**

| Failure Mode               | Heartbeat / Timeout             | NARH Kinematic Guard                          |
| -------------------------- | ------------------------------- | --------------------------------------------- |
| Packet loss                | Can stop motion                 | Can stop motion                               |
| Stale command              | Often missed                    | Detected by timing + kinematic inconsistency  |
| Replay command             | Often missed                    | Can trigger `BRAKE_AND_RESYNC`                |
| Burst command              | Often dangerous                 | Detected as time-flow / acceleration residual |
| Command-odom contradiction | Usually not checked             | Directly measured                             |
| Recovery                   | Usually manual or timeout-based | Fresh-window resync gate                      |

---

## 3. Technical Design & FAQ

NARH Guard does not attempt to repair the network.

It asks a different question:
```
Given the last two commands and the last two odometry states,
is this motion window still executable?
```
### 3.1 Kinematic Windowing

For each window:
```
C_prev = previous command
C_curr = current command

O_prev = previous odometry
O_curr = current odometry
```
The guard estimates:
```
expected_delta = integrate(C_prev, C_curr, dt)
measured_delta = O_curr - O_prev
```

### 3.2 Residual Construction

The NARH-lite residual combines:
```
timeflow residual
stale command residual
phase mismatch residual
command acceleration violation
command jerk violation
command-odom inconsistency
```
Conceptually:
```
R_NAR =
  w1 * R_timeflow
+ w2 * R_stale
+ w3 * R_phase
+ w4 * R_accel
+ w5 * R_jerk
+ w6 * R_cmd_odom
```
When:
```
R_NAR > yellow_threshold
```
the guard enters:
```
YELLOW_SLOWDOWN
```
When:
```
R_NAR > red_threshold
```
the guard enters:
```
RED_BRAKE → RESYNCING
```

### 3.3 Resync Gate

After a dangerous timing-collapse window, old command/odom buffers are flushed.

The guard only releases motion after receiving several clean, fresh, physically consistent windows.
```
bad timing
  ↓
RED_BRAKE
  ↓
flush poisoned window
  ↓
RESYNCING
  ↓
fresh cmd + fresh odom
  ↓
RECOVERED
```
### 3.4 Performance Overhead
`ros2_kinematic_guard` is designed as a lightweight middleware layer.
| Failure Mode               | Heartbeat / Timeout | NARH Kinematic Guard                                |
| -------------------------- | ------------------- | --------------------------------------------------- |
| Packet loss                | Can detect silence  | Can detect silence and brake                        |
| Stale command              | Often missed        | Detected through timing and kinematic inconsistency |
| Burst command              | Often missed        | Detected through residual spike                     |
| Replay command             | Often missed        | Detected through stale / command-odom conflict      |
| Command/odom contradiction | Not checked         | Directly measured                                   |
| Recovery                   | Timeout-based       | Fresh-window resync gate                            |


The NARH-lite core does not run a global optimizer, a factor graph, or a full dynamics simulator. Each guard tick only evaluates a small local window:
```
previous command
current command
previous odometry
current odometry
```
The current implementation uses:
```
constant-size buffers
simple SE(2)-style kinematic integration
scalar residual components
threshold-based finite-state logic
```
So the computational complexity per evaluation is effectively:
```
O(1)
```
This makes it suitable for typical ROS 2 mobile robot control rates such as:
```
20 Hz
50 Hz
100 Hz
```
The default launch file runs the guard loop at `20 Hz`, corresponding to an intervention opportunity every control tick, approximately `50 ms`.

>Note: The current Python implementation is intended as a transparent reference implementation. For production deployment, the same NARH-lite core can be ported to C++ or embedded inside a lower-level controller.

### 3.5 Generalization Across Robot Types
The first version targets mobile robot command/feedback streams using standard ROS 2 interfaces:
```
geometry_msgs/Twist
geometry_msgs/TwistStamped
nav_msgs/Odometry
```
It is naturally suitable for:
```
differential-drive robots
skid-steer robots
omni-directional mobile bases
simulation / rosbag / MCAP replay
wireless teleoperation pipelines
```
For Ackermann steering or more complex platforms, the architecture remains the same, but the expected-motion model should be adapted.

In the current implementation, this means modifying the expected kinematic delta inside the NARH-lite core:
```
expected_delta = integrate(command_window, dt)
measured_delta = odom_curr - odom_prev
R_cmd_odom = || expected_delta - measured_delta ||
```
For example:
```
Differential drive:
  command = (vx, wz)

Ackermann:
  command = (speed, steering_angle)

Omni base:
  command = (vx, vy, wz)

Legged base:
  command = body velocity + gait/foot-contact consistency
```
The guard is therefore not tied to one robot model. It is tied to one principle:
```
a command should remain executable under the observed feedback stream
```
### 3.6 FSM Predictability
`ros2_kinematic_guard` uses a deterministic finite-state machine.
The main states are:
```
GREEN
YELLOW_SLOWDOWN
RED_BRAKE
RESYNCING
RECOVERED
```
The transition logic is transparent and parameter-driven.
A simplified view:
```
if R_NAR < yellow_threshold:
    GREEN

if R_NAR >= yellow_threshold:
    YELLOW_SLOWDOWN

if R_NAR >= red_threshold:
    RED_BRAKE -> RESYNCING

if fresh command/odom windows remain consistent for N frames:
    RECOVERED -> GREEN
```
The emergency behavior is not random. It is controlled by explicit parameters:
```
yellow_threshold
red_threshold
slowdown_scale
node_resync_good_frames
resync_required_good_frames
cmd_ttl
max_linear_accel
max_angular_accel
max_linear_jerk
max_angular_jerk
position_tolerance
yaw_tolerance
lateral_tolerance
```
These parameters can be configured through:
```
ROS 2 launch arguments
YAML parameter files
ROS 2 node parameters
```
This gives developers control over:
```
when slowdown happens
when braking happens
how many clean frames are required before recovery
how aggressive the kinematic consistency check should be
```
**Why Not Just Use Heartbeat or Timeout?**

Heartbeat and timeout mechanisms answer a simpler question:
```
Did a message arrive recently?
```
NARH Guard asks a stricter question:
```
Is this command still executable under the current odometry stream?
```
That difference matters under bad Wi-Fi / 5G timing.

A stale command may still arrive before a timeout.

A burst of commands may still be “valid messages.”

A replayed command may still look syntactically correct.

But NARH Guard checks whether the command window and feedback window remain kinematically consistent.

---

## 4. Mathematical Appendix: NARH-lite for ROS 2 Command Flow
*The theoretical origin of this project stems from the original NARH (Non-Associative Residual Hypothesis) formulated for discrete physics engines. For the full mathematical derivation and high-dimensional analysis, please refer to SIPA [(Simulation Integrity & Physics Auditor)](https://github.com/ZC502/SIPA/blob/main/README.md#non-associative-residual-hypothesis-narh)..*
### 4.1 Background
The original NARH formulation was developed for discrete rigid-body simulation pipelines.

In that setting, a system state is advanced by a sequence of sub-operators:
```
s[t+1] = Ψσ(k) ∘ ... ∘ Ψσ(1)(s[t])
```
where the execution order may depend on solver internals such as constraint partitioning, thread scheduling, batching, or projection steps. The original discrete associator is written as:
```
A(a,b,c;s) =
    ((Ψa ∘ Ψb) ∘ Ψc)(s)
  - (Ψa ∘ (Ψb ∘ Ψc))(s)
```
and the residual is:
```
R[t] = || A(a,b,c;s[t]) ||
```
The important point is that NARH does **not** claim that the physical state space itself is mathematically invalid. It measures order-dependent deviations introduced by discrete numerical or computational pipelines.

`ros2_kinematic_guard` applies the same idea to ROS 2 command-flow consistency.

### 4.2 Command-Flow Operators
In ROS 2 mobile robot control, the relevant pipeline is not a simulator constraint solver. It is a distributed command/feedback stream:
```
command message
network delivery
controller execution
odometry feedback
resync / recovery
```
NARH-lite models this using a local kinematic window.

For each evaluation step:
```
C_prev = previous command
C_curr = current command

O_prev = previous odometry
O_curr = current odometry
```
The guard compares two interpretations of the same short motion window.

### 4.3 Command-Flow OperatorsExpected Motion
The command stream predicts a local motion delta:
```
Δ_expected = Integrate(C_prev, C_curr, dt)
```
For the default differential-drive / planar base model:
```
v_avg = 0.5 * (v_prev + v_curr)
w_avg = 0.5 * (w_prev + w_curr)

Δx_expected   = v_avg * dt
Δyaw_expected = w_avg * dt
```
### 4.4 Measured Motion
The odometry stream gives the measured local delta:
```
Δ_measured = LocalFrame(O_curr - O_prev)
```
For planar motion:
```
Δx_measured
Δy_measured
Δyaw_measured
```
### 4.5 Kinematic Residual
The command/feedback residual is:
```
R_cmd_odom =
    || Δ_expected - Δ_measured ||
```
In practice, the implementation separates the residual into interpretable components:
```
linear_gap
lateral_gap
angular_gap
```
This allows the status output to explain why the guard intervened.

### 4.6 Time-Flow Residual
Bad wireless timing often appears as:
```
stale commands
near-zero dt
large dt jumps
phase mismatch between cmd and odom
burst delivery
replay windows
```
NARH-lite therefore adds timing residuals:
```
R_timeflow
R_stale
R_phase
```

### 4.7 Command Smoothness Residual
A command may be syntactically valid but physically unreasonable.
For example, a burst window can imply impossible acceleration or jerk:
```
R_accel = violation(command_acceleration_limit)
R_jerk  = violation(command_jerk_limit)
```

### 4.8 Final NARH-lite Residual
The final residual is a weighted composition:
```
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
This is not a full octonion solver. It is a lightweight engineering projection of NARH onto ROS 2 command-flow safety.

### 4.9 Decision Rule
The finite-state machine uses transparent thresholds:
```
if R_NAR < yellow_threshold:
    GREEN

if R_NAR >= yellow_threshold:
    YELLOW_SLOWDOWN

if R_NAR >= red_threshold:
    RED_BRAKE
    BRAKE_AND_RESYNC
```
During `BRAKE_AND_RESYNC`, the guard:
```
1. outputs safe zero or limited safe command
2. flushes poisoned command/odom windows
3. waits for fresh command + fresh odometry
4. requires N clean windows
5. releases control through RECOVERED
```

### 4.10 Interpretation
NARH-lite does not repair Wi-Fi, 5G, DDS, QoS, or robot drivers.

It provides a physical executability check:
```
This message arrived.
But should the robot still execute it?
```
If the answer is no, the guard intervenes before bad timing becomes dangerous motion.
