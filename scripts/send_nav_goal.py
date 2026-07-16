#!/usr/bin/env python3
"""
Sends a move_base goal to a named point of interest from
poi/points_of_interest.yaml.

Usage:
  rosrun autonomous_navigation_uco send_nav_goal.py <poi_id>
  rosrun autonomous_navigation_uco send_nav_goal.py --list
"""

import math
import os
import sys

import rospy
import rospkg
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def default_poi_path():
    try:
        pkg_path = rospkg.RosPack().get_path('autonomous_navigation_uco')
        return os.path.join(pkg_path, 'poi', 'points_of_interest.yaml')
    except rospkg.ResourceNotFound:
        return ''


def load_pois(poi_file):
    with open(poi_file, 'r') as f:
        data = yaml.safe_load(f) or {}
    return {entry['id']: entry for entry in (data.get('points_of_interest', []) or [])}


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    poi_file = default_poi_path()
    pois = load_pois(poi_file)

    if sys.argv[1] == '--list':
        print("Available points of interest (from %s):" % poi_file)
        for poi_id, entry in pois.items():
            print("  %-15s %s  (x=%.2f, y=%.2f, yaw=%.2f)" % (
                poi_id, entry.get('label', ''), entry['x'], entry['y'], entry.get('yaw', 0.0)
            ))
        sys.exit(0)

    poi_id = sys.argv[1]
    if poi_id not in pois:
        print("Unknown point of interest '%s'. Run with --list to see available options." % poi_id)
        sys.exit(1)

    entry = pois[poi_id]

    rospy.init_node('send_nav_goal', anonymous=True)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = entry['x']
    goal.target_pose.pose.position.y = entry['y']

    qx, qy, qz, qw = yaw_to_quaternion(entry.get('yaw', 0.0))
    goal.target_pose.pose.orientation.x = qx
    goal.target_pose.pose.orientation.y = qy
    goal.target_pose.pose.orientation.z = qz
    goal.target_pose.pose.orientation.w = qw

    rospy.loginfo("Sending goal: %s (%s) -> x=%.2f y=%.2f yaw=%.2f",
                   poi_id, entry.get('label', ''), entry['x'], entry['y'], entry.get('yaw', 0.0))
    client.send_goal(goal)
    client.wait_for_result()

    state = client.get_state()
    rospy.loginfo("Navigation finished with actionlib state: %d", state)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
