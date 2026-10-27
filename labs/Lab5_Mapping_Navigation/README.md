# Lab 5 — SLAM Mapping and Autonomous Navigation with Nav2

**Course:** Robotic and Autonomous Systems Design
**Platform:** Yahboom MicroROS-ESP32 robot (lidar, IMU, wheel encoders) · ROS 2 Humble · micro-ROS agent over Wi-Fi
**Duration:** 2 × 90 min
**Work in:** teams of 2–3, one robot per team

Based on the Yahboom MicroROS tutorials
[Gmapping mapping](https://www.yahboom.net/study/MicroROS-ESP32/10506),
[Cartographer mapping](https://www.yahboom.net/study/MicroROS-ESP32/10507) and
[Navigation2 navigation avoid](https://www.yahboom.net/study/MicroROS-ESP32/10508),
adapted to the packages in this repository.

---

## 1. Objectives

By the end of this lab you should be able to:

1. Build a 2D occupancy-grid map of a room with **two SLAM algorithms** (Gmapping and Cartographer) and compare them.
2. Save a map and explain every field of the map `.yaml` file.
3. Localize the robot on a saved map with **AMCL** and send it to goals with **Nav2**, including multi-waypoint missions.
4. Read the Nav2 TF tree and node graph, and explain how `map → odom → base_footprint → base_link → laser_frame` is produced.
5. Change Nav2 parameters (speed, inflation, goal tolerance) and measure their effect on real robot behaviour.

## 2. Equipment and prerequisites

- Robot, charged battery, a closed area of about 3 × 3 m or larger with some static obstacles (boxes, chair legs, walls).
- A laptop with **Ubuntu 22.04 + ROS 2 Humble** (native install or the Yahboom VM), on the **same Wi-Fi network** as the robot.
- This repository cloned to the laptop.
- From earlier labs: the robot board is configured with the laptop's IP, agent port `8090`, and the team's `ROS_DOMAIN_ID`. You can start the agent and drive the robot with the keyboard.

> **Mac users:** Docker Desktop on macOS does not forward Wi-Fi UDP traffic with `--net=host`. Use the Ubuntu VM in bridged network mode, or a native Ubuntu laptop.

Throughout this lab:

```bash
export REPO=~/MicroROS-Robot-TSI          # where you cloned this repository
export ROS_DOMAIN_ID=21                   # use YOUR team's domain ID, as set on the board
```

Put both lines in `~/.bashrc` so every new terminal gets them.

## 3. Background (read before the lab)

### 3.1 Occupancy-grid SLAM

SLAM (Simultaneous Localization And Mapping) estimates the robot pose and the map at the same time. The output here is an **occupancy grid** (`nav_msgs/OccupancyGrid` on `/map`): each cell holds the probability that it is occupied. Cells are shown as free (white), occupied (black) or unknown (grey).

| | **Gmapping** | **Cartographer** |
|---|---|---|
| Method | Rao-Blackwellized particle filter. Each particle carries its own map. | Graph optimization. Scans are matched into submaps, and a pose graph is optimized in the background. |
| Loop closure | None. Large loops can leave the map misaligned. | Yes. Scan matching between submaps corrects drift. |
| Cost | Grows with number of particles × map size | Higher CPU baseline, but scales to large areas |
| Best for | Small indoor rooms | Larger areas and loops |
| Limit here | Fails if one scan has more than 1440 points (`[mapping-4] process has died`) | — |

### 3.2 Nav2

Nav2 is a set of lifecycle-managed servers, coordinated by a behaviour tree:

| Server | Role in this lab |
|---|---|
| `map_server` | Loads your saved map and publishes `/map` |
| `amcl` | Particle-filter localization. Publishes `map → odom`. |
| `planner_server` (NavFn) | Global path from the current pose to the goal on the **global costmap** |
| `controller_server` (DWB) | Follows the path, avoids obstacles using the **local costmap**, publishes `/cmd_vel` |
| `behavior_server` | Recovery behaviours: spin, back up, wait |
| `bt_navigator` | Runs the behaviour tree for `NavigateToPose` and `NavigateThroughPoses` |
| `waypoint_follower` | Visits a list of poses in order |
| `velocity_smoother` | Limits accelerations of the commanded velocity |

Documentation: <https://docs.nav2.org>

### 3.3 Software stack used in this lab

```
robot (ESP32, micro-ROS) ── Wi-Fi UDP 8090 ──► micro-ROS agent (laptop)
        /scan  /imu  /odom_raw  ◄──►  /cmd_vel
                    │
yahboomcar_bringup  ├─ imu_filter_madgwick   /imu → /imu/data
                    ├─ robot_localization EKF  /odom_raw + /imu/data → odom → base_footprint
                    └─ yahboomcar_description  URDF, base_footprint → base_link, base_link → imu_frame
yahboomcar_nav      ├─ map_gmapping_launch.py      slam_gmapping + base_link → laser_frame
                    ├─ map_cartographer_launch.py  cartographer + occupancy grid + base_link → laser_frame
                    ├─ save_map_launch.py          nav2 map_saver_cli
                    ├─ navigation_dwb_launch.py    nav2_bringup (AMCL + DWB) + stop_car
                    └─ display_launch.py           RViz with rviz/view.rviz
```

Relevant files in this repository:

| File | Contents |
|---|---|
| `yahboomcar_ws/src/yahboomcar_nav/launch/*.py` | Launch files used below |
| `additional_ws/gmapping_ws/src/slam_gmapping/params/slam_gmapping.yaml` | Gmapping parameters |
| `yahboomcar_ws/src/yahboomcar_nav/params/lds_2d.lua` | Cartographer configuration |
| `yahboomcar_ws/src/yahboomcar_nav/params/dwb_nav_params.yaml` | Nav2 parameters (AMCL, DWB, costmaps, …) |
| `yahboomcar_ws/src/yahboomcar_nav/maps/` | Saved maps |

---

## 4. Part 0 — Setup (once per laptop, ~20 min)

### 4.1 Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-cartographer-ros ros-humble-robot-localization \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-xacro ros-humble-joy ros-humble-tf2-tools \
  ros-humble-rqt-graph ros-humble-topic-tools \
  python3-pykdl python3-colcon-common-extensions
```

### 4.2 Build the three workspaces

`slam_gmapping` and `imu_filter_madgwick` are built from source from `additional_ws`. Build them first, then `yahboomcar_ws` on top of them:

```bash
source /opt/ros/humble/setup.bash

cd $REPO/additional_ws/imu_ws      && colcon build && source install/setup.bash
cd $REPO/additional_ws/gmapping_ws && colcon build && source install/setup.bash
cd $REPO/yahboomcar_ws             && colcon build && source install/setup.bash
```

Add to `~/.bashrc`, **in this order**:

```bash
source /opt/ros/humble/setup.bash
source $REPO/additional_ws/imu_ws/install/setup.bash
source $REPO/additional_ws/gmapping_ws/install/setup.bash
source $REPO/yahboomcar_ws/install/setup.bash
```

Check:

```bash
ros2 pkg prefix slam_gmapping yahboomcar_nav yahboomcar_bringup imu_filter_madgwick
```

All four should print an install path.

### 4.3 Start the agent and connect the robot

Terminal 1:

```bash
$REPO/helper_scripts/start_agent_computer.sh
# = docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \
#     microros/micro-ros-agent:humble udp4 --port 8090 -v4
```

Switch the robot on. Within a few seconds the agent log should show `session established` and `create_publisher` and `create_subscriber` lines.

Terminal 2:

```bash
ros2 node list          # robot node(s) visible
ros2 topic list         # must contain /scan /imu /odom_raw /cmd_vel
ros2 topic hz /scan     # note the scan rate
```

**✅ Checkpoint 0:** write in the report the robot's topics, the `/scan` rate, and the number of points in one scan:

```bash
ros2 topic echo /scan --once --field ranges | tr ',' '\n' | wc -l
```

Gmapping requires fewer than 1440 points per scan.

---

## 5. Part 1 — Mapping with Gmapping (~30 min)

Open four terminals. Keep the agent from Part 0 running.

```bash
# T2 — low-level data processing (IMU filter, EKF, URDF, TF)
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py

# T3 — RViz (map will be empty until SLAM runs)
ros2 launch yahboomcar_nav display_launch.py

# T4 — Gmapping SLAM
ros2 launch yahboomcar_nav map_gmapping_launch.py

# T5 — teleoperation (choose one)
ros2 run yahboomcar_ctrl yahboom_keyboard
#   or gamepad:
# ros2 run joy joy_node   &   ros2 run yahboomcar_ctrl yahboom_joy
```

### 1.1 Drive and map

Drive slowly, about 0.2 m/s or less, and turn gently. Follow the walls first, then cover the inside of the area. Return to the starting point at the end.

Watch the map in RViz. Gmapping only updates the map after the robot has moved `linearUpdate` (1.0 m) or turned `angularUpdate` (0.5 rad), or every `map_update_interval` (5 s).

### 1.2 Save the map

When the map is complete, **keep SLAM running** and save the map from a new terminal:

```bash
ros2 launch yahboomcar_nav save_map_launch.py \
  map_path:=$REPO/yahboomcar_ws/src/yahboomcar_nav/maps/lab5_gmapping
```

This creates `lab5_gmapping.pgm` and `lab5_gmapping.yaml`. Without `map_path:=` the map is saved as `maps/yahboom_map` and overwrites the existing one.

Open the `.yaml`:

```yaml
image: lab5_gmapping.pgm
mode: trinary
resolution: 0.05
origin: [-10, -10, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

| Field | Meaning |
|---|---|
| `image` | Map bitmap (PGM) |
| `mode` | `trinary` (free, occupied, unknown), `scale` or `raw` |
| `resolution` | metres per pixel |
| `origin` | pose (x, y, yaw) of the **lower-left** pixel in the `map` frame |
| `negate` | swap the meaning of white and black |
| `occupied_thresh` / `free_thresh` | probability thresholds for occupied and free cells |

### 1.3 Inspect the system

```bash
ros2 run rqt_graph rqt_graph      # select "Nodes/Topics (all)", then refresh
ros2 run tf2_tools view_frames    # writes frames_<date>.pdf in the current directory
```

**✅ Checkpoint 1:** screenshot of the finished map in RViz, the saved `.pgm`, the TF tree PDF, and the rqt_graph screenshot. In the report, name the node that publishes `map → odom` and the node that publishes `odom → base_footprint`.

### 1.4 Experiment: particle count

Gmapping parameters are in `additional_ws/gmapping_ws/src/slam_gmapping/params/slam_gmapping.yaml`.

1. Change `particles: 30` to `particles: 5`.
2. Rebuild `gmapping_ws` (`colcon build`), map the **same route** again, and save the map as `lab5_gmapping_p5`.
3. Restore `particles: 30` and rebuild.

Compare both maps (wall straightness, doubled walls, CPU usage in `htop`).

---

## 6. Part 2 — Mapping with Cartographer (~25 min)

Stop Gmapping (Ctrl+C in T4). Keep bringup, RViz and teleop running. In T4:

```bash
ros2 launch yahboomcar_nav map_cartographer_launch.py
```

This starts `cartographer_node`, configured by `params/lds_2d.lua`, and `cartographer_occupancy_grid_node`, which publishes `/map` at 0.05 m resolution once per second.

Drive the **same route** as in Part 1 and **close at least one loop**, coming back past a place you mapped earlier. Watch for the moment when the map "snaps" into place after a loop closure.

Save the map:

```bash
ros2 launch yahboomcar_nav save_map_launch.py \
  map_path:=$REPO/yahboomcar_ws/src/yahboomcar_nav/maps/lab5_cartographer
```

Key settings in `lds_2d.lua`:

| Setting | Value | Meaning |
|---|---|---|
| `tracking_frame` | `base_footprint` | frame whose pose is estimated |
| `published_frame` / `provide_odom_frame` | `odom` / `false` | Cartographer publishes `map → odom` and uses the EKF's `odom` |
| `use_odometry` | `true` | uses `/odom` |
| `TRAJECTORY_BUILDER_2D.use_imu_data` | `false` | IMU not used by Cartographer |
| `min_range` / `max_range` | 0.10 / 3.5 m | lidar range accepted for mapping |
| `POSE_GRAPH.constraint_builder.min_score` | 0.65 | minimum match score to accept a loop closure |

**✅ Checkpoint 2:** screenshot of the Cartographer map and the saved `.pgm`. Put the Gmapping and Cartographer maps of the same area side by side in the report.

---

## 7. Part 3 — Navigation with Nav2 (~45 min)

Stop SLAM (T4) and teleop (T5). Keep the agent, bringup and RViz running. Place the robot at a known spot on the map, for example where mapping started.

### 3.0 Pre-flight check: odometry topic

`dwb_nav_params.yaml` expects odometry on `/odom` (`bt_navigator.odom_topic`, `velocity_smoother.odom_topic`). Check what bringup publishes:

```bash
ros2 topic list | grep -E "odom"
```

If you see `/odometry/filtered` but **no** `/odom`, relay it in a separate terminal before starting Nav2:

```bash
ros2 run topic_tools relay /odometry/filtered /odom
```

### 3.1 Start Nav2 on your map

```bash
# T4
ros2 launch yahboomcar_nav navigation_dwb_launch.py \
  maps:=$REPO/yahboomcar_ws/src/yahboomcar_nav/maps/lab5_cartographer.yaml
```

> ⚠️ Always pass `maps:=`. The default in this repository's `navigation_dwb_launch.py` is the hard-coded path `/home/yahboom/yahboomcar_ws/...`, which does not exist on your laptop.

Wait until the terminal prints `Managed nodes are active` and the map appears in RViz.

### 3.2 Initial pose (AMCL)

In RViz click **2D Pose Estimate**, then click on the robot's real position and drag in its heading direction. The red laser points should line up with the black walls of the map. If they don't, set the pose again.

Equivalent command line:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

### 3.3 Single goal

Click **2D Goal Pose** (Nav2 Goal) and pick a reachable point about 2 m away. The robot plans a path (green line on `/plan`), drives there, and stops within `xy_goal_tolerance` = 0.25 m and `yaw_goal_tolerance` = 0.25 rad.

Same thing from the command line, with live feedback:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}" \
  --feedback
```

### 3.4 Obstacle avoidance

Send a goal, then **place a box on the planned path** while the robot is driving. Observe how the local costmap marks it, how DWB slows down or steers around it, and when the planner replans. Also try fully blocking a corridor and watch the recovery behaviours (spin, back up, wait).

Add these displays in RViz (**Add → By topic**) to see what Nav2 sees:
- `/local_costmap/costmap` (Map, colour scheme *costmap*)
- `/global_costmap/costmap` (Map, colour scheme *costmap*)
- `/local_plan` (Path)
- `/particle_cloud` (ParticleCloud, AMCL particles)

### 3.5 Multi-waypoint mission

In the **Navigation 2** panel (already in `view.rviz`):

1. Click **Waypoint / Nav Through Poses Mode**.
2. Place 3–4 goals with **Nav2 Goal** around the area.
3. Click **Start Waypoint Following**.

The robot visits the points in order. `waypoint_follower` waits `waypoint_pause_duration` = 200 ms at each one.

### 3.6 Stopping safely

Press Ctrl+C in T4. The `stop_car` node publishes a zero `/cmd_vel` on exit, so the robot stops. If the robot ever keeps moving:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

…or switch it off.

**✅ Checkpoint 3:** screen recording or photos of (a) a single-goal run, (b) avoidance of a newly placed obstacle, (c) a waypoint mission. Also the TF tree during navigation and the rqt_graph.

### 3.7 Measurements

For **5 goals**, record in the table:

| # | Goal (x, y, yaw) | Reached? | Time to goal [s] | Final error measured on the floor [cm] | Recoveries triggered |
|---|---|---|---|---|---|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |
| 4 | | | | | |
| 5 | | | | | |

---

## 8. Part 4 — Parameter study (~20 min)

Do not edit the original file. Copy it and pass the copy to the launch file:

```bash
cp $REPO/yahboomcar_ws/src/yahboomcar_nav/params/dwb_nav_params.yaml ~/lab5_nav_params.yaml

ros2 launch yahboomcar_nav navigation_dwb_launch.py \
  maps:=$REPO/yahboomcar_ws/src/yahboomcar_nav/maps/lab5_cartographer.yaml \
  params_file:=$HOME/lab5_nav_params.yaml
```

Change **one** parameter at a time, restart Nav2, and repeat goal #1 from Part 3.7:

| Experiment | Parameter (in `~/lab5_nav_params.yaml`) | Default | Try |
|---|---|---|---|
| A | `controller_server.FollowPath.max_vel_x` and `max_speed_xy`, plus `velocity_smoother.max_velocity[0]` | 0.30 / 0.30 / 0.26 | 0.15 |
| B | `inflation_radius` (local **and** global costmap) | 0.55 | 0.30 and 0.80 |
| C | `general_goal_checker.xy_goal_tolerance` | 0.25 | 0.10 |
| D | `local_costmap.width` / `height` | 3 | 1.5 |

For each experiment note the time to the goal, the final error, the path shape, and whether the robot still passes through narrow gaps.

---

## 9. Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| Agent never shows `session established` | Wrong laptop IP or port on the board; firewall (`sudo ufw allow 8090/udp`); laptop and robot on different Wi-Fi networks |
| Agent connected but `ros2 topic list` has no robot topics | `ROS_DOMAIN_ID` on the laptop differs from the board |
| RViz: "No transform from laser_frame" | The SLAM or Nav2 launch file is not running (they publish `base_link → laser_frame`) or bringup is not running |
| Gmapping: `[mapping-4] process has died` | More than 1440 points per scan (see Checkpoint 0) |
| Map smeared or doubled | Driving or turning too fast; odometry not calibrated (redo the linear and angular calibration with `calibrate_linear` / `calibrate_angular` from `yahboomcar_bringup`) |
| `save_map_launch.py` times out | SLAM was stopped before saving, so no `/map` is being published |
| Nav2 waits for the map forever | Wrong `maps:=` path (use an absolute path to the `.yaml`) |
| Robot doesn't move after a goal | No initial pose set; `/odom` missing (see 3.0); goal inside an inflated obstacle |
| Robot oscillates near the goal | Goal tolerance too tight; lower DWB speeds |
| AMCL pose jumps | Bad initial pose; dynamic objects (people) in front of the lidar |

---

## 10. Report (submit as one PDF per team)

1. **Setup:** robot topics, scan rate and points per scan (Checkpoint 0).
2. **Gmapping:** map screenshot and `.pgm`, TF tree, rqt_graph, the explained `.yaml`, and the particle experiment with your conclusion.
3. **Cartographer:** map screenshot and `.pgm`. Compare with Gmapping: wall quality, loop closure, CPU usage, time to map.
4. **Navigation:** evidence for 3.3–3.5 and the measurement table from 3.7 with mean and standard deviation of the final error.
5. **Parameter study:** results table and a short explanation of each effect.
6. **Questions** (short answers):
   1. Which node publishes `map → odom` during mapping, and which during navigation? Why must there be exactly one?
   2. Why does Gmapping become expensive in large environments, and how does Cartographer avoid this?
   3. What is the difference between the global and the local costmap in Nav2? Why does the local costmap use the `odom` frame?
   4. What does `inflation_radius` represent physically? How would you choose it for this robot (`robot_radius` = 0.22 m)?
   5. AMCL uses `min_particles` = 500 and `max_particles` = 2000. When does it use more particles?
   6. The robot's `/cmd_vel` comes from two possible sources in this lab (teleop and Nav2). What could go wrong if both run at the same time?

### Grading

| Part | Points |
|---|---|
| Setup and Checkpoint 0 | 5 |
| Gmapping map and analysis | 20 |
| Cartographer map and comparison | 20 |
| Navigation (single goal, avoidance, waypoints, measurements) | 30 |
| Parameter study | 15 |
| Questions | 10 |
| **Total** | **100** |

### Files to leave in the repository (team branch)

```
yahboomcar_ws/src/yahboomcar_nav/maps/lab5_gmapping.{pgm,yaml}
yahboomcar_ws/src/yahboomcar_nav/maps/lab5_gmapping_p5.{pgm,yaml}
yahboomcar_ws/src/yahboomcar_nav/maps/lab5_cartographer.{pgm,yaml}
labs/Lab5_Mapping_Navigation/team_<N>/lab5_nav_params.yaml   # your best parameter set
```
