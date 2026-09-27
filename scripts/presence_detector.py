#!/usr/bin/env python3
"""
presence_detector.py -- task 5, external event integration.

fake presence sensor. publishes json events on /presence_events saying a
person showed up (or left) in one of the poi zones.

runs on its own by default: picks a random zone every 8-20s, clears it
again ~6s later. multiple zones can be active at once. can also trigger
events by hand from any terminal on the network:
  rostopic pub /presence_events std_msgs/String \
    "data: '{\\"zone_id\\": \\"my_desk\\", \\"event\\": \\"person_detected\\"}'"

data field is json: {"zone_id": <poi id>, "event": "person_detected" or
"person_cleared", "timestamp": <epoch seconds>}

if a real sensor ever replaces this, just call _publish_event() from its
callback instead of _auto_event_loop() -- nothing else needs to change.
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
