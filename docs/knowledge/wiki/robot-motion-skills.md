# Robot Motion Skills

# Robot Motion Skills

The Robot Motion Skills module provides ROS 2 action interfaces and runtime implementations for NAO robot motion primitives. It bridges high-level orchestration requests to low-level NAOqi execution through a layered skill architecture.

## Architecture Overview

```mermaid
graph TB
    subgraph "Interface Layer"
        NA[nao_skills<br/>Action Definitions]
        MS[motions_skills<br/>Generic Motion Interfaces]
    end
    
    subgraph "Implementation Layer"
        RM[nao_replay_motion<br/>ReplayMotionSkillServer]
        HM[HeadMotionSkillServer]
        PB[nao_posture_bridge_node<br/>C++ Bridge]
        LA[nao_look_at<br/>LookAt Implementation]
    end
    
    subgraph "External"
        NAO[NAOqi/ALRobotPosture]
        JA[/joint_angles topic]
        JS[/joint_states topic]
        TF[TF2]
    end
    
    ORC[nao_orchestrator] --> NA
    ORC --> MS
    
    NA --> RM
    NA --> HM
    MS --> LA
    
    RM --> NAO
    RM --> JA
    HM --> JA
    HM --> JS
    PB --> NAO
    LA --> JA
    LA --> TF
```

## Package Structure

| Package | Role | Language |
|---------|------|----------|
| `nao_skills` | Action interface definitions | ROS 2 interfaces only |
| `nao_replay_motion` | Motion execution servers | Python + C++ |
| `motions_skills` | Generic motion interfaces | ROS 2 interfaces only |
| `nao_look_at` | Gaze/look-at implementation | Python (lifecycle node) |

---

## Action Interfaces

### nao_skills/action/ReplayMotion

The canonical entry point for NAO posture-style motion execution.

**Goal:**
| Field | Type | Description |
|-------|------|-------------|
| `motion_name` | `string` | Named motion primitive (stand, sit, kneel, crouch, standinit) |
| `speed` | `float32` | Execution speed fraction [0.0, 1.0] |

**Result:**
| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether motion completed successfully |
| `message` | `string` | Execution result message |
| `duration` | `float32` | Execution duration in seconds |

**Feedback:**
| Field | Type | Description |
|-------|------|-------------|
| `status` | `string` | Phase: "preparing", "executing", "completing" |
| `progress` | `float32` | Progress fraction [0.0, 1.0] |

**Default endpoint:** `/skill/replay_motion`

### nao_skills/action/DoPosture

Compatibility interface layered on top of ReplayMotion during migration.

**Goal:**
| Field | Type | Description |
|-------|------|-------------|
| `posture_name` | `string` | Target posture (Stand, Sit, Crouch, StandInit) |
| `speed` | `float32` | Transition speed [0.0, 1.0] |

**Default endpoint:** `/skill/do_posture`

### nao_skills/action/DoHeadMotion

Head motion control via absolute or relative joint angles.

**Goal:**
| Field | Type | Description |
|-------|------|-------------|
| `yaw` | `float32` | HeadYaw angle in radians |
| `pitch` | `float32` | HeadPitch angle in radians |
| `speed` | `float32` | Joint speed fraction [0.0, 1.0] |
| `relative` | `bool` | If true, yaw/pitch are deltas from current position |

**Default endpoint:** `/skill/do_head_motion`

### motions_skills/action/ExecuteJointTrajectory

Generic joint trajectory execution for broader robot platforms.

**Goal:**
| Field | Type | Description |
|-------|------|-------------|
| `meta` | `std_skills/Meta` | Skill metadata including priority |
| `trajectory` | `trajectory_msgs/JointTrajectory` | Joint trajectory specification |
| `safe_mode` | `bool` | Enable collision avoidance and speed limits |

### motions_skills/action/ExecuteCartesianTrajectory

Cartesian space trajectory execution.

**Goal:**
| Field | Type | Description |
|-------|------|-------------|
| `meta` | `std_skills/Meta` | Skill metadata |
| `trajectory` | `moveit_msgs/CartesianTrajectory` | Cartesian trajectory specification |

---

## Motion Catalog

The `ReplayMotionSkillServer` normalizes motion names and maps them to NAOqi posture primitives:

| Input Alias | Motion Name | NAOqi Posture |
|-------------|-------------|---------------|
| `stand`, `standfull` | stand | Stand |
| `standinit` | standinit | StandInit |
| `standzero` | standzero | StandZero |
| `sit` | sit | Sit |
| `sitrelax` | sitrelax | SitRelax |
| `kneel` | kneel | Crouch |
| `crouch` | crouch | Crouch |
| `lyingback` | lyingback | LyingBack |
| `lyingbelly` | lyingbelly | LyingBelly |

Name resolution is case-insensitive and whitespace-tolerant via `_normalize_name()`.

---

## Key Components

### ReplayMotionSkillServer

**File:** `nao_replay_motion/nao_replay_motion/replay_motion_skill_server.py`

Serves both `/skill/replay_motion` and `/skill/do_posture` actions. Execution flow:

1. **Goal validation** — `replay_goal_callback()` checks motion name resolution and speed bounds
2. **Connection management** — `_ensure_connection()` maintains NAOqi session with reconnect-on-failure support
3. **Execution** — `_execute_motion()` attempts direct NAOqi `ALRobotPosture.goToPosture()` call
4. **Fallback** — If NAOqi unavailable and `fallback_to_posture_topic` is enabled, publishes to `/chatbot/posture_command`

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `nao_ip` | `127.0.0.1` | NAO robot IP address |
| `nao_port` | `9559` | NAOqi port |
| `default_speed` | `0.8` | Default execution speed |
| `reconnect_on_failure` | `true` | Auto-reconnect on NAOqi errors |
| `fallback_to_posture_topic` | `true` | Use topic fallback when disconnected |

### HeadMotionSkillServer

**File:** `nao_replay_motion/nao_replay_motion/head_motion_skill_server.py`

Serves `/skill/do_head_motion` by publishing `JointAnglesWithSpeed` messages and monitoring `/joint_states` for convergence.

**Execution flow:**
1. Validate yaw/pitch against joint limits (unless `relative=true`)
2. Wait for current head state from `/joint_states`
3. Resolve target angles (add current position if relative)
4. Publish command to `/joint_angles`
5. Poll until convergence within `convergence_tolerance_rad` or timeout

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `yaw_min` / `yaw_max` | `±2.0857` rad | HeadYaw joint limits |
| `pitch_min` / `pitch_max` | `-0.6720` / `0.5149` rad | HeadPitch joint limits |
| `convergence_timeout_sec` | `3.0` | Max wait for convergence |
| `convergence_tolerance_rad` | `0.08` | Position tolerance |
| `joint_state_wait_sec` | `1.0` | Timeout for initial joint state |

### NaoPostureBridge (C++)

**File:** `nao_replay_motion/src/nao_posture_bridge_node.cpp`

Topic-based posture bridge subscribing to `/chatbot/posture_command`. Provides legacy compatibility during migration.

**Features:**
- Command deduplication within configurable time window
- Posture name normalization and alias resolution
- Automatic NAOqi session management with reconnect
- Optional `ALAutonomousLife` disable and `ALMotion.wakeUp` on connect

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `connect_on_startup` | `true` | Connect to NAOqi at node startup |
| `disable_autonomous_life_on_connect` | `false` | Disable autonomous life when connected |
| `wake_up_on_connect` | `false` | Call `ALMotion.wakeUp()` on connect |
| `command_dedupe_window_sec` | `1.5` | Duplicate command suppression window |

### NaoLookAtSkill

**File:** `nao_look_at/nao_look_at/skill_impl.py`

Lifecycle node implementing `interaction_skills/action/LookAt` for NAO. Transforms target frames into head joint angles.

**Supported policies:**
| Policy | Behavior |
|--------|----------|
| `RESET` | Return head to neutral pose (yaw=0, pitch=0) |
| `GLANCE` | Look at target, hold, then reset |
| Target frame | Track 3D point in specified reference frame |

**TF resolution order:**
1. Try `look_from_frame` (default: `CameraTop_frame`)
2. Fall back to `fallback_look_from_frame` (default: `base_link`)

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `look_from_frame` | `CameraTop_frame` | Primary TF frame for gaze projection |
| `fallback_look_from_frame` | `base_link` | Secondary TF frame |
| `reset_yaw` / `reset_pitch` | `0.0` | Neutral head pose |
| `glance_hold_sec` | `0.7` | Hold time before GLANCE reset |
| `max_yaw_abs` | `1.5` rad | Yaw angle clamp |
| `min_pitch` / `max_pitch` | `-0.67` / `0.51` rad | Pitch angle clamps |

---

## Launch Configuration

### nao_replay_motion

```bash
ros2 launch nao_replay_motion nao_replay_motion.launch.py \
  nao_ip:=172.26.112.62 \
  posture_bridge_connect_on_startup:=true \
  posture_bridge_disable_autonomous_life_on_connect:=false \
  posture_bridge_wake_up_on_connect:=false
```

### nao_look_at

```bash
ros2 launch nao_look_at nao_look_at.launch.py
```

The look_at node is a lifecycle node that requires activation. The launch file includes a bootstrap script that transitions it to `active` state.

---

## Client Usage

### ReplayMotionSkillClient

```python
from nao_replay_motion.replay_motion_client import ReplayMotionSkillClient

client = ReplayMotionSkillClient(node, action_name="/skill/replay_motion")
client.wait_for_server(timeout_sec=5.0)

client.send_goal(
    motion_name="stand",
    speed=0.8,
    turn_id="turn_42",
    result_callback=lambda result: print(f"Success: {result.success}")
)
```

### HeadMotionSkillClient

```python
from nao_replay_motion.head_motion_skill_client import HeadMotionSkillClient

client = HeadMotionSkillClient(node, default_speed=0.25)
client.wait_for_server(timeout_sec=5.0)

client.send_goal(
    yaw=0.5,      # radians
    pitch=-0.2,   # radians
    relative=False,
    result_callback=lambda result: print(f"Duration: {result.duration}s")
)
```

---

## Integration Points

### Upstream Consumers

- **nao_orchestrator** — Dispatches motion goals via action clients
- **dialogue_manager** — Uses motion skills during dialogue execution

### Downstream Dependencies

- **NAOqi** — Direct `ALRobotPosture` service calls for posture execution
- **naoqi_bridge_msgs** — `JointAnglesWithSpeed` message type for head control
- **TF2** — Frame transforms for look_at target resolution
- **sensor_msgs/JointState** — Head position feedback for convergence detection

### Topic Interfaces

| Topic | Direction | Message Type | Purpose |
|-------|-----------|--------------|---------|
| `/joint_angles` | Publish | `JointAnglesWithSpeed` | Head motion commands |
| `/joint_states` | Subscribe | `JointState` | Head position feedback |
| `/chatbot/posture_command` | Subscribe | `String` | Legacy posture bridge input |

---

## Testing

Unit tests validate core logic without hardware:

```bash
# Run unit tests
colcon test --packages-select nao_replay_motion nao_look_at
```

**Test coverage:**
- Motion name normalization and alias resolution
- Head motion angle validation and clamping
- Relative angle resolution from current state
- Convergence detection within tolerance
- TF vector-to-angle conversion conventions