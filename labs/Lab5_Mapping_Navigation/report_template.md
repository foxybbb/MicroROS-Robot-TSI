# Lab 5 Report — SLAM Mapping and Nav2 Navigation

| | |
|---|---|
| Team | |
| Members | |
| Robot ID / `ROS_DOMAIN_ID` | |
| Date | |

## 0. Setup

`./check_lab5.sh robot` output (paste):

```
```

| Item | Value |
|---|---|
| Robot topics | |
| `/scan` rate [Hz] | |
| Points per scan | |
| Odometry topic used by Nav2 (`/odom` or relay) | |

## 1. Gmapping

- Map screenshot (RViz): *insert*
- Saved map `lab5_gmapping.pgm`: *insert*
- TF tree (`view_frames` PDF): *insert*
- rqt_graph: *insert*

`lab5_gmapping.yaml` (paste and explain each field):

```yaml
```

| Field | Value | Meaning |
|---|---|---|
| image | | |
| mode | | |
| resolution | | |
| origin | | |
| negate | | |
| occupied_thresh | | |
| free_thresh | | |

Node that publishes `map → odom`: ______  Node that publishes `odom → base_footprint`: ______

### Particle experiment

| | particles = 30 | particles = 5 |
|---|---|---|
| Map image | | |
| Wall straightness / doubled walls | | |
| CPU usage of `slam_gmapping` [%] | | |

Conclusion:

## 2. Cartographer

- Map screenshot (RViz): *insert*
- Saved map `lab5_cartographer.pgm`: *insert*
- Did you see a loop closure? Describe:

| Criterion | Gmapping | Cartographer |
|---|---|---|
| Wall quality | | |
| Loop closure | | |
| CPU usage [%] | | |
| Time to map the area [min] | | |
| Preferred for this room, and why | | |

## 3. Navigation

Map used: ______  Initial pose method (RViz / `/initialpose`): ______

Evidence (links to videos or photos):
- (a) single goal:
- (b) avoidance of a newly placed obstacle:
- (c) waypoint mission:

| # | Goal (x, y, yaw) | Reached? | Time to goal [s] | Final error on the floor [cm] | Recoveries triggered |
|---|---|---|---|---|---|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |
| 4 | | | | | |
| 5 | | | | | |
| **Mean ± std** | | | | | |

## 4. Parameter study (goal #1 repeated)

| Experiment | Parameter = value | Time [s] | Final error [cm] | Path shape / narrow gaps | Observation |
|---|---|---|---|---|---|
| Baseline | defaults | | | | |
| A | max_vel_x = 0.15 | | | | |
| B1 | inflation_radius = 0.30 | | | | |
| B2 | inflation_radius = 0.80 | | | | |
| C | xy_goal_tolerance = 0.10 | | | | |
| D | local costmap 1.5 × 1.5 m | | | | |

Best parameter set (attach `lab5_nav_params.yaml`) and why:

## 5. Questions

1. Which node publishes `map → odom` during mapping, and which during navigation? Why must there be exactly one?
2. Why does Gmapping become expensive in large environments, and how does Cartographer avoid this?
3. What is the difference between the global and the local costmap? Why does the local costmap use the `odom` frame?
4. What does `inflation_radius` represent physically? How would you choose it for this robot (`robot_radius` = 0.22 m)?
5. AMCL uses `min_particles` = 500 and `max_particles` = 2000. When does it use more particles?
6. `/cmd_vel` can come from teleop and from Nav2. What could go wrong if both run at the same time?

## 6. Problems encountered and how you solved them
