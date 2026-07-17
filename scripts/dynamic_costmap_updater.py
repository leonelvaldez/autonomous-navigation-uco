#!/usr/bin/env python3
"""
dynamic_costmap_updater.py

Task 6 -- Dynamic Map / Costmap Update.

Subscribes to /presence_events (see presence_detector.py for the message
format). Maintains the current set of "active" zones (zones where a
person is currently detected). For every active zone, continuously
publishes a disc of synthetic obstacle points centered on that zone's
map-frame coordinates (read from poi/points_of_interest.yaml) as a
sensor_msgs/PointCloud2 on the /presence_zone_obstacles topic.

/presence_zone_obstacles is registered as a second observation source
(alongside the real /scan) in move_base's obstacle_layer -- see
config/costmap_common_params.yaml. This means move_base's costmap and
DWA local planner automatically treat active zones as obstacles and
route around them, using the navigation stack's own existing, standard
machinery. No custom costmap_2d plugin or move_base source modification
is needed.

When a zone transitions from active to inactive (person_cleared), this
node calls the /move_base/clear_costmaps service. This forces move_base
to fully recompute both costmaps from all currently live observation
sources. Any zones that are still active are re-marked automatically on
the very next obstacle_layer update cycle from their continuously
published points, so clearing one zone does not meaningfully affect
other simultaneously active zones.
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


class DynamicCostmapUpdater(object):
    def __init__(self):
        rospy.init_node('dynamic_costmap_updater')

        self.publish_rate = float(rospy.get_param('~publish_rate', 5.0))
        self.zone_radius = float(rospy.get_param('~zone_radius', 0.5))
        self.point_spacing = float(rospy.get_param('~point_spacing', 0.1))
        poi_file = rospy.get_param('~poi_file', self._default_poi_path())

        self.zones = self._load_zones(poi_file)
        self._active_zones = set()
        self._lock = threading.Lock()

        self.cloud_pub = rospy.Publisher(
            '/presence_zone_obstacles', PointCloud2, queue_size=1
        )
        rospy.Subscriber('/presence_events', String, self._on_event)

        rospy.loginfo(
            "dynamic_costmap_updater started | %d zones loaded from %s | publish_rate=%.1fHz | zone_radius=%.2fm",
            len(self.zones), poi_file, self.publish_rate, self.zone_radius
        )

        self._clear_costmaps_srv = None  # resolved lazily on first use

        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish_cloud)

    def _default_poi_path(self):
        try:
            pkg_path = rospkg.RosPack().get_path('autonomous_navigation_uco')
            return os.path.join(pkg_path, 'poi', 'points_of_interest.yaml')
        except rospkg.ResourceNotFound:
            return ''

    def _load_zones(self, poi_file):
        zones = {}
        try:
            with open(poi_file, 'r') as f:
                data = yaml.safe_load(f) or {}
            for entry in (data.get('points_of_interest', []) or []):
                zones[entry['id']] = (float(entry['x']), float(entry['y']))
        except (IOError, OSError, KeyError, TypeError) as exc:
            rospy.logwarn("Could not load POI file '%s': %s. No zones registered until it is available.", poi_file, exc)
        return zones

    def _on_event(self, msg):
        try:
            payload = json.loads(msg.data)
            zone_id = payload['zone_id']
            event = payload['event']
        except (ValueError, KeyError) as exc:
            rospy.logwarn("Ignoring malformed /presence_events message: %s (%s)", msg.data, exc)
            return

        if zone_id not in self.zones:
            rospy.logwarn("Event for unknown zone_id '%s' (not in points_of_interest.yaml) -- ignored.", zone_id)
            return

        if event == 'person_detected':
            with self._lock:
                self._active_zones.add(zone_id)
            rospy.loginfo("Zone '%s' marked ACTIVE (obstacle).", zone_id)

        elif event == 'person_cleared':
            with self._lock:
                self._active_zones.discard(zone_id)
            rospy.loginfo("Zone '%s' marked CLEAR. Requesting costmap recompute.", zone_id)
            self._call_clear_costmaps()

        else:
            rospy.logwarn("Unknown event type '%s' -- ignored.", event)

    def _call_clear_costmaps(self):
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=1.0)
            if self._clear_costmaps_srv is None:
                self._clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            self._clear_costmaps_srv()
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn("Could not call /move_base/clear_costmaps (is move_base running?): %s", exc)

    def _zone_disc_points(self, cx, cy):
        points = []
        steps = max(1, int(self.zone_radius / self.point_spacing))
        for i in range(-steps, steps + 1):
            for j in range(-steps, steps + 1):
                dx = i * self.point_spacing
                dy = j * self.point_spacing
                if math.sqrt(dx * dx + dy * dy) <= self.zone_radius:
                    points.append((cx + dx, cy + dy, 0.0))
        return points

    def _publish_cloud(self, _event):
        with self._lock:
            active = list(self._active_zones)

        all_points = []
        for zone_id in active:
            cx, cy = self.zones[zone_id]
            all_points.extend(self._zone_disc_points(cx, cy))

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
        DynamicCostmapUpdater()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
