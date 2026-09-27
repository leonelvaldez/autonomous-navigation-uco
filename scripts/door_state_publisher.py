#!/usr/bin/env python3
"""
door_state_publisher.py -- extra feature on top of tasks 5+6, not one of
the official 8 tasks.

the 4 real doors got mapped as open space but can actually be open or
closed. unlike presence_detector.py's events (a quick pulse: detected,
then cleared a few seconds later), a door's state sticks until someone
changes it. so this publishes on a LATCHED topic (/door_states, json
dict of all door states) -- anything that subscribes, even after
startup, gets the current state right away instead of waiting for a
change. that's what lets a closed door affect route planning from the
start, not just once the robot gets close.

change a door from any terminal on the network:
  rostopic pub /set_door_state std_msgs/String \
    "data: '{\\"door_id\\": \\"door1\\", \\"state\\": \\"closed\\"}'" --once

door_costmap_updater.py reacts live to whatever this publishes.

/door_states data field, json: {"door1": "open", "door2": "closed", ...}
"""

import json
import os
import threading

import rospy
import rospkg
import yaml
from std_msgs.msg import String


VALID_STATES = ('open', 'closed')


class DoorStatePublisher(object):
    def __init__(self):
        rospy.init_node('door_state_publisher')

        doors_file = rospy.get_param('~doors_file', self._default_doors_path())
        self._lock = threading.Lock()
        self.states = self._load_doors(doors_file)

        self.pub = rospy.Publisher('/door_states', String, queue_size=1, latch=True)
        rospy.Subscriber('/set_door_state', String, self._on_set_door_state)

        rospy.loginfo("door_state_publisher started | doors=%s | loaded from %s", self.states, doors_file)
        self._publish_states()

    def _default_doors_path(self):
        try:
            pkg_path = rospkg.RosPack().get_path('autonomous_navigation_uco')
            return os.path.join(pkg_path, 'poi', 'doors.yaml')
        except rospkg.ResourceNotFound:
            return ''

    def _load_doors(self, doors_file):
        states = {}
        try:
            with open(doors_file, 'r') as f:
                data = yaml.safe_load(f) or {}
            for entry in (data.get('doors', []) or []):
                states[entry['id']] = entry.get('default_state', 'open')
        except (IOError, OSError, KeyError, TypeError) as exc:
            rospy.logwarn("Could not load doors file '%s': %s. No doors registered.", doors_file, exc)
        return states

    def _publish_states(self):
        with self._lock:
            msg = String()
            msg.data = json.dumps(self.states)
            self.pub.publish(msg)

    def _on_set_door_state(self, msg):
        try:
            payload = json.loads(msg.data)
            door_id = payload['door_id']
            state = payload['state']
        except (ValueError, KeyError) as exc:
            rospy.logwarn("Ignoring malformed /set_door_state message: %s (%s)", msg.data, exc)
            return

        if door_id not in self.states:
            rospy.logwarn("Unknown door_id '%s' -- ignored.", door_id)
            return
        if state not in VALID_STATES:
            rospy.logwarn("Invalid state '%s' for door '%s' -- must be 'open' or 'closed'.", state, door_id)
            return

        with self._lock:
            self.states[door_id] = state
        rospy.loginfo("Door '%s' set to %s.", door_id, state)
        self._publish_states()


if __name__ == '__main__':
    try:
        DoorStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
