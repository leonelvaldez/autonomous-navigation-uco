# autonomous-navigation-uco

Autonomous indoor navigation for a TurtleBot2 (Kobuki base) using ROS.
Internship project at Universidad de Córdoba (UCO), supervised by Dr.
Joaquín Olivares.

## Setup

Two machines talking over one ROS master:

- **Raspberry Pi** (ROS Melodic, Ubuntu 18.04) — runs `turtlebot_bringup`
  and the RPLidar driver. This is the robot itself.
- **Laptop VM** (ROS Noetic, Ubuntu 20.04) — runs localization
  (`amcl`) and navigation (`move_base`).

Both machines need `ROS_MASTER_URI` pointed at the Pi and their own
`ROS_IP` set.

```bash
cd ~/internship_ws
catkin_make
source devel/setup.bash
```

## Task 2 — map

`maps/map.pgm` + `maps/map.yaml`, produced with `slam_gmapping` run
directly (no launch file for this yet, was done via `rosrun`).

Two things had to be right for the map to come out clean:
- lidar's `frame_id` is `laser_frame`, not `laser`
- the lidar is mounted backwards on the robot, so the static transform
  needs yaw = `3.14159`, not `0` — this one silently wrecked every
  earlier map attempt until it was caught

## Task 3 — localization + navigation

On the Pi (after bringup):
```bash
roslaunch autonomous_navigation_uco rplidar_uco.launch
```

On the VM:
```bash
roslaunch autonomous_navigation_uco amcl_uco.launch
roslaunch autonomous_navigation_uco move_base_uco.launch
```

Set the initial pose in RViz (2D Pose Estimate), then send goals with
RViz's 2D Nav Goal tool.

Neither `amcl_demo.launch` nor `gmapping_demo.launch` (the stock
TurtleBot2 ones) work here — both assume an Orbbec camera that isn't
part of this setup. This repo's launch files replace them.

RViz doesn't show `Map`/`LaserScan` by default, add them manually
(Add → By topic).

Tested on the real robot, reaches goals and avoids obstacles.

## Task 4 — points of interest

`poi/points_of_interest.yaml` has named (x, y, yaw) spots on the map,
all in the same room for now since that's all the map currently covers.

Send the robot to one with:
```bash
rosrun autonomous_navigation_uco send_nav_goal.py <poi_id>
rosrun autonomous_navigation_uco send_nav_goal.py --list   # to see the ids
```

## Status

| Task | Status |
|---|---|
| 1 — ROS/TurtleBot setup | done |
| 2 — map building | done, map committed above |
| 3 — autonomous navigation | done, validated on the real robot |
| 4 — points of interest | done |
