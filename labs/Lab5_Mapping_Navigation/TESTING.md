# Lab 5 — To test on the robot (instructor)

Nothing below has run on a real robot yet. Tick each item on one robot and one lab laptop before giving the lab to students, and fix `README.md` where reality differs.

Already checked without a robot:
- All 15 workspace packages (`yahboomcar_ws` + `additional_ws`, except `rviz_imu_plugin`) build with `colcon build` on ROS 2 Humble.
- Launch-argument names match the launch files: `maps`, `params_file` (`navigation_dwb_launch.py`), `map_path` (`save_map_launch.py`).
- `run_lab5.sh` and `check_lab5.sh` pass `bash -n`; `run_lab5.sh nav <missing map>` exits with an error.
- `check_lab5.sh robot` reports correctly against simulated `/scan` (450 points, 10 Hz), `/imu`, `/odom_raw`, `/cmd_vel`.

## Setup and connection

- [ ] The apt package list in README Part 0 installs cleanly on Ubuntu 22.04 (not checked: the ROS apt server was unreachable from the build machine)
- [ ] All three workspaces build on a lab laptop, including `rviz_imu_plugin`
- [ ] Agent shows `session established` with `ROS_DOMAIN_ID=21`; note how long the robot takes to connect
- [ ] Robot topics are exactly `/scan`, `/imu`, `/odom_raw`, `/cmd_vel`; record any others it publishes
- [ ] Points per scan are below 1440; record the number and the `/scan` rate
- [ ] The `frame_id` of `/scan` is `laser_frame` (`ros2 topic echo /scan --once --field header`)
- [ ] Mac + VM in bridged mode receives the robot (or drop the Mac note)

## Bringup

- [ ] `/imu/data` is published by the IMU filter
- [ ] Which odometry topic the EKF publishes: `/odom` or `/odometry/filtered`. If only the latter, decide: keep the `relay` step, or remap in the launch file
- [ ] `check_lab5.sh bringup` passes all TF checks: `odom → base_footprint → base_link → imu_frame`

## Mapping

- [ ] Keyboard teleop drives the robot; the gamepad path works (`run_lab5.sh teleop joy`)
- [ ] Gmapping builds a usable map of the lab room at 0.2 m/s
- [ ] Cartographer receives odometry (it reads `/odom`; the relay is started before SLAM in Part 1)
- [ ] `run_lab5.sh save <name>` writes both files into `yahboomcar_nav/maps/`
- [ ] `particles: 5` gives a visibly worse map than 30 (otherwise pick a lower value)
- [ ] `check_lab5.sh slam` passes with each SLAM running

## Navigation

- [ ] `run_lab5.sh nav lab5_cartographer` reaches `Managed nodes are active`
- [ ] RViz shows the Navigation 2 panel, 2D Pose Estimate and Nav2 Goal tools
- [ ] AMCL localizes after 2D Pose Estimate; `check_lab5.sh nav` passes
- [ ] Robot reaches a goal 2 m away within 0.25 m; note the time
- [ ] Robot avoids a box placed on the path; recoveries trigger when a corridor is blocked
- [ ] Waypoint following visits 3–4 points in order
- [ ] Ctrl+C on Nav2 stops the robot (`stop_car`)
- [ ] Each Part 4 experiment produces a visible difference; adjust the "Try" values if not

## Timing

- [ ] The whole lab fits in 2 × 90 min for one team; trim Part 4 if it does not
