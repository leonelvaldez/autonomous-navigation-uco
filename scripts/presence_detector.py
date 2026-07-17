#!/usr/bin/env python3
"""
presence_detector.py

Task 5 -- External Event Integration.

Simulates a presence detector: publishes JSON-encoded person-detected /
person-cleared events for a configurable list of zones on the
/presence_events topic (std_msgs/String).

Two ways to generate events:
  1. Automatic mode (default, always running): at random intervals within
     [min_interval, max_interval] seconds, a random zone is chosen and a
     "person_detected" event is published for it. After event_duration
     seconds, a matching "person_cleared" event is published for that
     zone. Multiple zones can be simultaneously active.
  2. Manual mode: from any terminal on the ROS network, publish directly
     to /presence_events, e.g.:
       rostopic pub /presence_events std_msgs/String \
         "data: '{\\"zone_id\\": \\"my_desk\\", \\"event\\": \\"person_detected\\"}'"

Message format published on /presence_events (std_msgs/String, data field
is a JSON string):
  {
    "zone_id": "<string, matches an id in poi/points_of_interest.yaml>",
    "event": "person_detected" | "person_cleared",
    "timestamp": <float, seconds since epoch>
  }

To integrate a real sensor later instead of the simulator, replace the
body of _auto_event_loop() with a callback from the real sensor's driver
that calls _publish_event() directly -- the rest of the node (publisher,
message format, manual-trigger compatibility) does not need to change.
"""

import json
import random
import threading
import time

import rospy
from std_msgs.msg import String


class PresenceDetector(object):
    def __init__(self):
        rospy.init_node('presence_detector')

        self.zones = rospy.get_param('~zones', [
            'entrance1', 'entrance2', 'my_desk', 'middle_corridor', 'entrance_corridor'
        ])
        self.min_interval = float(rospy.get_param('~min_interval', 8.0))
        self.max_interval = float(rospy.get_param('~max_interval', 20.0))
        self.event_duration = float(rospy.get_param('~event_duration', 6.0))
        self.auto_mode = bool(rospy.get_param('~auto_mode', True))

        self.pub = rospy.Publisher('/presence_events', String, queue_size=10)
        self._active_zones = set()
        self._lock = threading.Lock()

        rospy.loginfo(
            "presence_detector started | zones=%s | auto_mode=%s | interval=[%.1f, %.1f]s | event_duration=%.1fs",
            self.zones, self.auto_mode, self.min_interval, self.max_interval, self.event_duration
        )

        if self.auto_mode:
            self._timer_thread = threading.Thread(target=self._auto_event_loop)
            self._timer_thread.daemon = True
            self._timer_thread.start()

    def _publish_event(self, zone_id, event):
        msg = String()
        msg.data = json.dumps({
            'zone_id': zone_id,
            'event': event,
            'timestamp': time.time()
        })
        self.pub.publish(msg)
        rospy.loginfo("Published event: zone=%s event=%s", zone_id, event)

    def _clear_zone_after_delay(self, zone_id, delay):
        rospy.sleep(delay)
        with self._lock:
            if zone_id in self._active_zones:
                self._active_zones.discard(zone_id)
        self._publish_event(zone_id, 'person_cleared')

    def _auto_event_loop(self):
        while not rospy.is_shutdown():
            wait_time = random.uniform(self.min_interval, self.max_interval)
            rospy.sleep(wait_time)
            if rospy.is_shutdown():
                break

            with self._lock:
                available = [z for z in self.zones if z not in self._active_zones]
                if not available:
                    continue
                zone_id = random.choice(available)
                self._active_zones.add(zone_id)

            self._publish_event(zone_id, 'person_detected')

            clear_thread = threading.Thread(
                target=self._clear_zone_after_delay,
                args=(zone_id, self.event_duration)
            )
            clear_thread.daemon = True
            clear_thread.start()


if __name__ == '__main__':
    try:
        PresenceDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
