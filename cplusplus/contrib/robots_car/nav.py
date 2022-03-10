#!/usr/bin/env python
from __future__ import print_function

import sys
import json
import math
import numpy as np

import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from cv_bridge import CvBridge
import actionlib

import threading

first = [1.09197032452,0.778358817101,-0.0254044332957,0.0110283146296,0.744826288529,0.666683426344]

temperature = [1.45771980286,1.135487556,-0.0159205113377,-0.00778150882713,0.156708567164,0.987485903908]
#after_temperature = [1.26378452778, 0.938861548901, 0.000495287386352, -0.0309431035862, -0.706453932048, 0.707082117526]

_next1 = [-2.04224341869,0.25059089661,-0.00365901907001,0.02513902107,-0.9988949184126,0.03958822361246]
_next = [-2.0087779429,0.416883759673,-0.00831003222803,0.0292609043,0.7685677941185,0.6443532948]

home = [-0.606540799141,0.610840082169,-0.0169371503318,0.00306125779327,0.00702260271284,0.999827207417]


#waypoints = [first,  after_temperature, _next, home]
waypoints = [first, temperature, _next1,_next, home]

no_stop = [temperature,_next1]

def excute_move(client, goal):
  rospy.loginfo("Sending goal!")
  client.send_goal(goal, done_cb=final_cb, active_cb=start_cb, feedback_cb=fb_cb)
  client.wait_for_result()
  client.get_result()
  return

def start_cb():
  rospy.loginfo("Start Nav")
  return

def final_cb(status, result):
  rospy.loginfo("Get result: %d", status)
  if status == 3:
    rospy.loginfo('The robot has got the destination!')
  else:
    rospy.logwarn('The robot failed to get the goal!')
  return

def fb_cb(feedback):
  return

def main():
  # Init Node
  rospy.init_node("ultimate_nav_action_client")
  # Init Action Client
  client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  client.wait_for_server()
  # Nav
  while True:
    for waypoint in waypoints:
      print(waypoint)
      # Prepare Goal
      goal = MoveBaseGoal()
      goal.target_pose.header.frame_id = 'map'
      goal.target_pose.header.stamp = rospy.Time.now()
      goal.target_pose.pose.position.x = waypoint[0]
      goal.target_pose.pose.position.y = waypoint[1]
      goal.target_pose.pose.position.z = 0

      goal.target_pose.pose.orientation.x = 0
      goal.target_pose.pose.orientation.y = 0
      goal.target_pose.pose.orientation.z = waypoint[4]
      goal.target_pose.pose.orientation.w = waypoint[5]

      # Move
      rospy.loginfo("======== Move to waypoints========")
      excute_move(client, goal)

      if waypoint in no_stop:
        continue
      rospy.sleep(rospy.Duration(2.5))
    rospy.sleep(rospy.Duration(15))


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    sys.exit()

