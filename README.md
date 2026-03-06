# speed_zone_limiter

A ROS 2 Humble package for TurtleBot3 that caps the robot's linear velocity whenever it enters a defined map-zone — the kind of thing you'd want near a loading dock or a busy corridor.

---

## Why this exists

Nav2's `controller_server` has no built-in concept of "slow here, fast everywhere else." The cleanest way to add that without touching Nav2 internals is to intercept the velocity stream after the controller writes it and before the hardware driver reads it. That's exactly what this node does.

---

## How the nodes connect

```
┌──────────────────┐      /cmd_vel       ┌──────────────────────┐     /cmd_vel_safe
│ controller_server│ ──────────────────▶ │  speed_zone_limiter  │ ──────────────────▶ robot driver
│    (Nav2)        │                     │                      │
└──────────────────┘                     │  checks /amcl_pose   │
                                         │  to know robot pos   │
┌──────────────────┐     /amcl_pose      │                      │
│      AMCL        │ ──────────────────▶ │                      │
└──────────────────┘                     └──────────────────────┘
```

Nav2's `controller_server` publishes velocity commands on `/cmd_vel`. The `speed_zone_limiter` node subscribes to that topic — it also subscribes to `/amcl_pose` from AMCL to get a continuous fix on the robot's map-frame position. Every time a `/cmd_vel` message arrives, the node checks whether the current pose falls inside the configured polygon. If it does, `linear.x` gets capped; if not, the message passes through unchanged. The result always goes out on `/cmd_vel_safe`, which the robot driver actually consumes.

Nothing in Nav2 is modified. The node is a transparent proxy — Nav2 thinks it's talking directly to the robot.

---

## Key parts of the code

### 1. Loading zone config from the parameter server

Zone geometry lives in `config/zones.yaml` and is loaded at startup via `get_parameter()`. Nothing is hardcoded.

```python
self.declare_parameter('zone_name', 'unnamed_zone')
self.declare_parameter('max_speed', 0.1)
self.declare_parameter('polygon', [1.2, 0.5, 3.4, 0.5, 3.4, 2.1, 1.2, 2.1])

self.zone_name = self.get_parameter('zone_name').get_parameter_value().string_value
self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
flat_polygon  = self.get_parameter('polygon').get_parameter_value().double_array_value
```

The polygon is stored as a flat list `[x0, y0, x1, y1, ...]` in the YAML, then zipped into coordinate pairs for Shapely:

```python
coords = list(zip(flat_polygon[0::2], flat_polygon[1::2]))
self.zone_polygon = Polygon(coords)
```

### 2. Startup race condition

AMCL takes a few seconds to publish the first pose. Rather than crashing or blocking, the node tracks whether a pose has arrived yet and simply passes velocity through unchanged until it has:

```python
def _cmd_vel_callback(self, msg):
    if not self._pose_received:
        safe_msg.linear = msg.linear
        self.safe_pub.publish(safe_msg)
        return
```

This means the node is safe to launch before AMCL is fully up.

### 3. Point-in-polygon check and velocity cap

Once a pose is available, every incoming velocity is checked against the polygon. Only `linear.x` is capped — angular velocity always passes through so the robot can still rotate inside the zone:

```python
in_zone = self.zone_polygon.contains(self._robot_pose)

if in_zone:
    capped = min(msg.linear.x, self.max_speed)
    safe_msg.linear.x = capped if msg.linear.x >= 0 else msg.linear.x
    safe_msg.linear.y = msg.linear.y
    safe_msg.linear.z = msg.linear.z
    if msg.linear.x > self.max_speed:
        self.get_logger().info(
            f'In zone "{self.zone_name}": capped {msg.linear.x:.3f} -> {capped:.3f} m/s'
        )
else:
    safe_msg.linear = msg.linear

self.safe_pub.publish(safe_msg)
```

`Polygon.contains()` from Shapely handles all the edge cases for arbitrary polygons — no need to roll a ray-casting implementation.

---

## Prerequisites

| | |
|---|---|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble Hawksbill |
| Python | ≥ 3.10 |

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo \
  python3-shapely
```

---

## Build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/darshan-stack/speed_zone_limiter.git

cd ~/ros2_ws
colcon build --packages-select speed_zone_limiter
source install/setup.bash
```

---

## Zone config

Edit `config/zones.yaml` to change the zone shape or speed limit:

```yaml
speed_zone_limiter:
  ros__parameters:
    zone_name: "loading_bay"
    max_speed: 0.10        # m/s
    polygon: [1.2, 0.5, 3.4, 0.5, 3.4, 2.1, 1.2, 2.1]
```

Coordinates are map-frame metres. Any convex or concave polygon shape works.

---

## Running

Open three terminals:

```bash
# Terminal 1 — simulation
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

```bash
# Terminal 2 — Nav2 + RViz
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```

```bash
# Terminal 3 — speed limiter
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run speed_zone_limiter speed_zone_limiter_node \
  --ros-args --params-file ~/ros2_ws/src/speed_zone_limiter/config/zones.yaml
```

In RViz2: click **2D Pose Estimate** and click the robot's position on the map to initialise AMCL.

---

## Verifying it works

Send the robot into the slow zone:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.5, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

Watch both velocity topics side by side:

```bash
# terminal A
ros2 topic echo /cmd_vel

# terminal B
ros2 topic echo /cmd_vel_safe
```

Inside the zone `linear.x` on `/cmd_vel_safe` should read `0.10` regardless of what Nav2 is commanding. Outside it matches `/cmd_vel` exactly. The node logs every cap event:

```
[speed_zone_limiter]: In zone "loading_bay": capped 0.210 -> 0.100 m/s
```

Send the robot back outside to confirm normal speed resumes:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## Package layout

```
speed_zone_limiter/
├── config/
│   └── zones.yaml
├── launch/
│   └── speed_zone_limiter.launch.py
├── speed_zone_limiter/
│   ├── __init__.py
│   └── speed_zone_limiter_node.py
├── resource/
│   └── speed_zone_limiter
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `ModuleNotFoundError: shapely` | `sudo apt install python3-shapely` |
| `/cmd_vel_safe` not appearing | Check AMCL is running: `ros2 topic hz /amcl_pose` |
| Speed not capped in zone | Confirm robot coordinates fall inside the polygon bounds |
| YAML parse error on launch | No tabs in zones.yaml — spaces only |
| Package not found | `source ~/ros2_ws/install/setup.bash` in every new terminal |
