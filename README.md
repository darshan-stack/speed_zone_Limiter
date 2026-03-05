# speed_zone_limiter — ROS 2 Humble

A ROS 2 node that enforces a maximum linear velocity whenever the robot
enters a user-defined polygonal slow zone on the map.

---

## How It Fits in the Nav2 Pipeline

The `speed_zone_limiter` node sits **between Nav2's output and the robot's
velocity driver**.  
Nav2 publishes velocity commands on `/cmd_vel_raw` (remapped at launch time).
The node subscribes to `/cmd_vel_raw` for velocity commands and to `/amcl_pose`
for the robot's current map-frame position. On every incoming velocity message
it checks whether the robot is inside the configured polygon; if so, `linear.x`
is capped to `max_speed` before the message is republished on `/cmd_vel_safe`.
A lightweight `topic_tools/relay` node then forwards `/cmd_vel_safe` back to
the `/cmd_vel` topic that the TurtleBot3 driver actually consumes, completing
the intercept loop.

```
Nav2 → /cmd_vel_raw → [speed_zone_limiter] → /cmd_vel_safe → [relay] → /cmd_vel → robot
                              ↑
                        /amcl_pose
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 |
| ROS 2 | Humble Hawksbill |
| Python | ≥ 3.10 |

Install system dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo \
  ros-humble-topic-tools \
  python3-shapely
```

---

## Build

```bash
# 1. Create (or reuse) your ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone / copy the package
git clone https://github.com/<your-username>/speed_zone_limiter.git
#   — or unzip and place the speed_zone_limiter/ folder here —

# 3. Build
cd ~/ros2_ws
colcon build --packages-select speed_zone_limiter
source install/setup.bash
```

---

## Zone Configuration

Edit `config/zones.yaml` to define your slow zone:

```yaml
speed_zone_limiter:
  ros__parameters:
    zone:
      name: loading_bay
      max_speed: 0.10          # m/s
      polygon:                 # map-frame vertices [x0,y0,x1,y1,...] flat list
        - 1.2
        - 0.5
        - 3.4
        - 0.5
        - 3.4
        - 2.1
        - 1.2
        - 2.1
```

Coordinates are in **metres in the map frame** and must describe a closed
polygon (any convex or concave shape is supported via Shapely).

---

## Launch

```bash
export TURTLEBOT3_MODEL=burger

# Terminal 1 — Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 — Speed-zone limiter + Nav2
source ~/ros2_ws/install/setup.bash
ros2 launch speed_zone_limiter speed_zone_limiter.launch.py
```

> **Note:** If you already have a Nav2 bringup running separately, launch only
> the limiter node directly:
> ```bash
> ros2 run speed_zone_limiter speed_zone_limiter_node \
>   --ros-args --params-file ~/ros2_ws/src/speed_zone_limiter/config/zones.yaml
> ```

---

## Verifying It Works

### 1. Confirm topics are alive

```bash
ros2 topic list | grep cmd_vel
# Expected:
# /cmd_vel
# /cmd_vel_raw
# /cmd_vel_safe
```

### 2. Send a navigation goal through the slow zone

In RViz2: use the **Nav2 Goal** button to send the robot through the polygon
defined in `zones.yaml` (default: x ∈ [1.2, 3.4], y ∈ [0.5, 2.1]).

### 3. Echo both velocity topics simultaneously

Open two terminals:

```bash
# Terminal A
ros2 topic echo /cmd_vel_raw

# Terminal B
ros2 topic echo /cmd_vel_safe
```

**Expected behaviour:**

| Robot position | `/cmd_vel_raw linear.x` | `/cmd_vel_safe linear.x` |
|---|---|---|
| Outside zone | e.g. `0.26` | `0.26` (unchanged) |
| Inside zone  | e.g. `0.26` | `0.10` (capped) |

### 4. Check node logs

```bash
ros2 topic echo /rosout | grep speed_zone
# You'll see: "In zone 'loading_bay': capped 0.260 → 0.100 m/s"
```

---

## Package Layout

```
speed_zone_limiter/
├── config/
│   └── zones.yaml                  # Slow-zone configuration
├── launch/
│   └── speed_zone_limiter.launch.py
├── speed_zone_limiter/
│   ├── __init__.py
│   └── speed_zone_limiter_node.py  # Main node
├── resource/
│   └── speed_zone_limiter
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `ModuleNotFoundError: shapely` | `sudo apt install python3-shapely` |
| `/cmd_vel_safe` not published | Check that `/amcl_pose` is being received (`ros2 topic hz /amcl_pose`) |
| Robot ignores speed cap | Ensure the `relay` node is running and `/cmd_vel_safe` → `/cmd_vel` is active |
| Zone not loaded | Confirm the params file path is correct in the launch file |

---

## Notes on Approach

- **Shapely** was chosen for the point-in-polygon check because it cleanly handles
  arbitrary polygons and is already a standard Python GIS library, avoiding the
  need to implement ray-casting manually.
- The node handles the **startup race condition** (pose not yet received) by
  passing velocity through unchanged until the first `/amcl_pose` message
  arrives, logging once when that happens.
- Angular velocity is **never capped** — only `linear.x` is limited, so the
  robot can still rotate freely inside the zone.
- Parameters are loaded entirely from the YAML file; nothing is hardcoded in
  the node source.
