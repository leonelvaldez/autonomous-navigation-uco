#!/usr/bin/env python3
"""
door_costmap_updater.py

Extra feature on top of tasks 5+6 (added by choice, not one of the
official 8 tasks). Subscribes to /door_states (see
door_state_publisher.py) and, for every door currently marked "closed",
continuously publishes a disc of synthetic obstacle points centered on
that door's map-frame coordinates (from poi/doors.yaml) as a
sensor_msgs/PointCloud2 on the /door_obstacles topic.

/door_obstacles is registered as a third observation source in
move_base's obstacle_layer (alongside laser_scan_sensor and
presence_zone_sensor -- see config/costmap_common_params.yaml), so a
closed door is treated as a real obstacle by the costmap and global
planner, same as tasks 5+6's presence zones and the real LiDAR. This is
what lets move_base plan a route around a closed door from the very
first plan, not just react once it gets close enough to see it.

Because /door_states is latched, this node picks up whatever the current
door states already are as soon as it starts, even if door_state_publisher
was running before it -- so route planning accounts for closed doors from
the start, not just from the next state-change event.

Doors can change state at any time, including while the robot is
already mid-route:
  - closed -> open: this node calls /move_base/clear_costmaps to force a
    full recompute, same as tasks 5+6's person_cleared handling.
  - open -> closed: no special handling needed. The new obstacle points
    simply appear on the next publish cycle, and move_base's own global
    planner re-evaluates its plan against the costmap on every cycle --
    if the current plan now runs through a lethal cell, it replans
    automatically, even if the robot was already moving towards it.
"""

import json
import math
import os
import threading

import rospy
import rospkg
import yaml
from std_msgs.msg import String, Header
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2


class DoorCostmapUpdater(object):
    def __init__(self):
        rospy.init_node('door_costmap_updater')

        self.publish_rate = float(rospy.get_param('~publish_rate', 5.0))
        self.door_radius = float(rospy.get_param('~door_radius', 0.45))
        self.point_spacing = float(rospy.get_param('~point_spacing', 0.1))
        doors_file = rospy.get_param('~doors_file', self._default_doors_path())

        self.doors = self._load_doors(doors_file)
        self._door_states = {}
        self._lock = threading.Lock()

        self.cloud_pub = rospy.Publisher(
            '/door_obstacles', PointCloud2, queue_size=1
        )
        rospy.Subscriber('/door_states', String, self._on_door_states)

        rospy.loginfo(
            "door_costmap_updater started | %d doors loaded from %s | publish_rate=%.1fHz | door_radius=%.2fm",
            len(self.doors), doors_file, self.publish_rate, self.door_radius
        )

        self._clear_costmaps_srv = None  # resolved lazily on first use

        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish_cloud)

    def _default_doors_path(self):
        try:
            pkg_path = rospkg.RosPack().get_path('autonomous_navigation_uco')
            return os.path.join(pkg_path, 'poi', 'doors.yaml')
        except rospkg.ResourceNotFound:
            return ''

    def _load_doors(self, doors_file):
        doors = {}
        try:
            with open(doors_file, 'r') as f:
                data = yaml.safe_load(f) or {}
            for entry in (data.get('doors', []) or []):
                doors[entry['id']] = (float(entry['x']), float(entry['y']))
        except (IOError, OSError, KeyError, TypeError) as exc:
            rospy.logwarn("Could not load doors file '%s': %s. No doors registered.", doors_file, exc)
        return doors

    def _on_door_states(self, msg):
        try:
            new_states = json.loads(msg.data)
        except ValueError as exc:
            rospy.logwarn("Ignoring malformed /door_states message: %s (%s)", msg.data, exc)
            return

        with self._lock:
            old_states = self._door_states
            self._door_states = new_states

        # a door that just opened needs an explicit costmap recompute,
        # same reasoning as tasks 5+6's person_cleared handling. if
        # several doors open in one update we only need to recompute once.
        any_opened = False
        for door_id, state in new_states.items():
            was_closed = old_states.get(door_id) == 'closed'
            if was_closed and state == 'open':
                rospy.loginfo("Door '%s' opened.", door_id)
                any_opened = True
            elif state == 'closed' and old_states.get(door_id) != 'closed':
                rospy.loginfo("Door '%s' closed. Marking as obstacle.", door_id)

        if any_opened:
            rospy.loginfo("At least one door opened. Requesting costmap recompute.")
            self._call_clear_costmaps()

    def _call_clear_costmaps(self):
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=1.0)
            if self._clear_costmaps_srv is None:
                self._clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            self._clear_costmaps_srv()
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn("Could not call /move_base/clear_costmaps (is move_base running?): %s", exc)

    def _door_disc_points(self, cx, cy):
        points = []
        steps = max(1, int(self.door_radius / self.point_spacing))
        for i in range(-steps, steps + 1):
            for j in range(-steps, steps + 1):
                dx = i * self.point_spacing
                dy = j * self.point_spacing
                if math.sqrt(dx * dx + dy * dy) <= self.door_radius:
                    points.append((cx + dx, cy + dy, 0.0))
        return points

    def _publish_cloud(self, _event):
        with self._lock:
            closed_doors = [d for d, s in self._door_states.items() if s == 'closed']

        all_points = []
        for door_id in closed_doors:
            if door_id not in self.doors:
                continue
            cx, cy = self.doors[door_id]
            all_points.extend(self._door_disc_points(cx, cy))

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        cloud_msg = point_cloud2.create_cloud(header, fields, all_points)
        self.cloud_pub.publish(cloud_msg)


if __name__ == '__main__':
    try:
        DoorCostmapUpdater()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
