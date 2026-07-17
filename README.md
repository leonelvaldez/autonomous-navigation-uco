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
directly (no launch file for this yet, was done via `rosrun`). Covers the
room, the corridor and the other rooms of the building. The older
single-room map is kept as `maps/map_room_only.*` for reference. A
stairwell that a 2D lidar can't see (it's a drop-off, no wall at scan
height) was blocked by hand in the map image so the robot won't try to
drive into it.

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

`poi/points_of_interest.yaml` has named (x, y, yaw) spots across the
building (entrances, the desk, corridor points, the stairs).

Send the robot to one with:
```bash
rosrun autonomous_navigation_uco send_nav_goal.py <poi_id>
rosrun autonomous_navigation_uco send_nav_goal.py --list   # to see the ids
```

## Task 5 — external events (presence detector)

`scripts/presence_detector.py` simulates a presence sensor. It publishes
JSON events on `/presence_events` saying a person was detected (or
cleared) in one of the POI zones. It can run on its own picking random
zones, or you can trigger events by hand:
```bash
rosrun autonomous_navigation_uco presence_detector.py
# manual:
rostopic pub /presence_events std_msgs/String \
  "data: '{\"zone_id\": \"my_desk\", \"event\": \"person_detected\"}'" --once
```

## Task 6 — dynamic costmap update

`scripts/dynamic_costmap_updater.py` listens to `/presence_events` and,
for each zone with a person in it, drops a patch of fake obstacle points
on `/presence_zone_obstacles`. That topic is a second observation source
in move_base's obstacle layer (see `config/costmap_common_params.yaml`),
so the planner routes around an active zone using the normal costmap
machinery — no custom plugin needed. When the zone clears it calls
`/move_base/clear_costmaps` to recompute.

## Extra — doors (not one of the assigned tasks)

The building has 4 doors that got mapped as open space but can actually
be open or closed. `scripts/door_state_publisher.py` keeps their state on
a latched `/door_states` topic (so it's known from the start, not only
once the robot is close), and `scripts/door_costmap_updater.py` marks any
closed door as an obstacle the same way Task 6 marks presence zones.
Change a door while everything's running with:
```bash
rostopic pub /set_door_state std_msgs/String \
  "data: '{\"door_id\": \"door1\", \"state\": \"closed\"}'" --once
```

## Simulation (Stage)

There's a 2D Stage simulation of the robot in `simulation/`, so the whole
navigation stack (and the presence/door features) can be tested without the
real robot. It uses the real map as the world, so everything runs unchanged.

```bash
sudo apt install -y ros-noetic-stage-ros
roslaunch autonomous_navigation_uco stage_nav.launch
```

See `simulation/README.md` for details. This covers the "test in simulation"
side of Task 7; it doesn't replace validating on the real robot.

## Status

| Task | Status |
|---|---|
| 1 — ROS/TurtleBot setup | done |
| 2 — map building | done, full-building map committed above |
| 3 — autonomous navigation | done, validated on the real robot |
| 4 — points of interest | done |
| 5 — external events | done |
| 6 — dynamic costmap update | done |
| extra — doors | done (self-initiated, beyond the assigned tasks) |
| simulation | Stage sim for robot-free testing (`simulation/`) |
