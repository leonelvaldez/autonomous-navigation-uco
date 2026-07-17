# Simulation (Stage)

A 2D simulated turtlebot in [Stage](http://wiki.ros.org/stage_ros), for
testing the whole navigation stack (and the presence/door features) without
the real robot. Dr. Olivares suggested this; it also covers the "test in
simulation" part of Task 7.

The simulated robot uses the **real map** (`../maps/map.pgm`) as its world, so
the laser scans the actual mapped building and `amcl` localizes against the
same map the real robot uses. Because Stage publishes the same topics/frames
the real robot does (`/odom`, `/scan`, the `odom -> base_footprint` tf chain,
listens on `cmd_vel`), the real `amcl`/`move_base` config and all the Task
4/5/6/door scripts run against it **unchanged**.

## Install

```bash
sudo apt update
sudo apt install -y ros-noetic-stage-ros
```

## Run

```bash
roslaunch autonomous_navigation_uco stage_nav.launch
```

That starts Stage, `map_server`, `amcl` and `move_base`. Open RViz
(`rosrun rviz rviz`), set Fixed Frame to `map`, add `/map`, `/scan` and the
costmaps, set a 2D Pose Estimate if needed, then send goals with 2D Nav Goal
or `send_nav_goal.py`. The presence detector, dynamic costmap updater and door
scripts all work the same as on the real robot.

## Notes

- No velocity mux and no `keyboard_teleop` in sim, so the Session 9
  teleop-blocks-navigation gotcha doesn't apply here.
- The simulated laser has no physical mounting offset, so the sim uses an
  identity `base_link -> laser_frame` transform (yaw 0), not the real robot's
  yaw = pi.
- If the simulated scan doesn't line up with the map walls in RViz, the Stage
  bitmap is flipped relative to `map_server` -- make a flipped copy and point
  the world file at it:
  `convert ../maps/map.pgm -flip map_stage.pgm`
- This is a testing/demo tool. It does **not** replace validating on the real
  robot.
