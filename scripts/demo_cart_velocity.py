#! /usr/bin/env python
## \file
#  Demo for velocity control in cartesian space
#  \date 2012-06-21
#  \author(c) Nicolas Alt, TU München
#  
#  First uses Joint control to move to a defined starting position.
#  Then, different cartesian velocity commands are sent for certain periods to the controller.

import roslib; roslib.load_manifest('lmtlwr')
import rospy
import actionlib
import math
import numpy
import time
import sys
import yaml

from trajectory_msgs.msg import *
from control_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_msgs.msg import String


def go_start():
    client = actionlib.SimpleActionClient('/lwrcontroller/joint_trajectory_action', FollowJointTrajectoryAction)
    print 'Waiting for JTaction server...'
    client.wait_for_server()
    print 'ok'

    r = math.pi / 180
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['lwr_arm_0_joint', 'lwr_arm_1_joint', 'lwr_arm_2_joint', 'lwr_arm_3_joint', 'lwr_arm_4_joint', 'lwr_arm_5_joint', 'lwr_arm_6_joint']

    # Start point
    goal.trajectory.points.append(JointTrajectoryPoint([0, 60*r, 0, 120*r, 0, 60*r, 0], [], [], rospy.Duration(2)));
    goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)
    client.send_goal(goal)
    time.sleep(1)
    result = client.wait_for_result()
    print 'Done'


if __name__ == '__main__':
  rospy.init_node('test_pose')
  pub = rospy.Publisher('/lwrcontroller/velocity_cartesian', PoseStamped)

  go_start()

  pref = PoseStamped();
  pref.header.frame_id = "";
  h = pref.header;

  q0 = Quaternion(0, 0, 0, 1)
  # 9 deg-rotation around axis [1 -1 0]
  q1 = Quaternion(0.055479, -0.055479, 0, 0.99692)
  # Turn around [1 1 0]-axis (of the end-effector)
  q3 = Quaternion(0.055479, 0.055479, 0,  0.99692)
  # Definition of velocities
  p0 = PoseStamped(h, Pose(Point(0, 0, 0), q0))
  p1 = PoseStamped(h, Pose(Point(-0.05, -0.05, -0.02), q1))
  p2 = PoseStamped(h, Pose(Point(0.05, 0.0, -0.08), q1))
  p3 = PoseStamped(h, Pose(Point(-0.0, 0.0, 0), q3))
  p4 = PoseStamped(h, Pose(Point(0.05, 0.0, 0.03), q0))

  # Switch to CARTESIAN, but do not move...
  pub.publish(p0);
  rospy.sleep(5)

  print "pub..."
  for i in range(100):
    pub.publish(p1); rospy.sleep(0.1)
  for i in range(40):
    pub.publish(p2); rospy.sleep(0.1)
  for i in range(60):
    pub.publish(p3); rospy.sleep(0.1)
  for i in range(130):
    pub.publish(p4); rospy.sleep(0.1)
