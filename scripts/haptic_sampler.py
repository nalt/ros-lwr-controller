#! /usr/bin/env python
## \file
#  Simple haptic surface sampler with joint control and force stopping
#  \date 2012-04-25
#  \author (c) Nicolas Alt, TU München
#
#  Tool: Stabilo; operating just above ground


import roslib; roslib.load_manifest('lmtlwr')
import rospy
import actionlib
import math
import numpy
import time
import sys

from trajectory_msgs.msg import *
from control_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty


def frange(start, stop, step):
  while start <= stop:
	yield start
	start += step



if __name__ == '__main__':
  rospy.init_node('simple_trajectory')
  pub_zero = rospy.Publisher('/lwrcontroller/jr3_zero', Empty)
  pub_force = rospy.Publisher('/lwrcontroller/force_limit_element', PoseStamped)
  #rospy.Subscriber("/joint_states", JointState, onJoint)
  client = actionlib.SimpleActionClient('lwrcontroller/joint_trajectory_action', FollowJointTrajectoryAction)
  print 'Waiting for action server...'
  client.wait_for_server()
  print 'ok'

  r = math.pi / 180
  v = 10 * r;       v1 = [v, v, v, v, v, v, v];
  v = 90 * r;     v2 = [v, v, v, v, v, v, v];
  a = 160 * r;  avec = [a, a, a, a, a, a, a];
  t = 0
  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['lwr_arm_0_joint', 'lwr_arm_1_joint', 'lwr_arm_2_joint', 'lwr_arm_3_joint', 'lwr_arm_4_joint', 'lwr_arm_5_joint', 'lwr_arm_6_joint']
  force1 = PoseStamped(); force2 = PoseStamped();
  force1.header.frame_id = "/calib_lwr_arm_base_link"
  force2.header.frame_id = "/calib_lwr_arm_base_link"
  force1.pose.position.x = numpy.nan; force1.pose.position.y = numpy.nan; force1.pose.position.z = numpy.nan;
  force2.pose.position.x = numpy.nan; force2.pose.position.y = numpy.nan; force2.pose.position.z = -1;

  fh = open("data.ply", "w")
  fh.write("ply\nformat ascii 1.0\nelement vertex ##\n")
  fh.write("property float x\nproperty float y\nproperty float z\nend_header\n")

  jstart = [-142*r, -55*r, 0*r, 80*r, 0*r,  -45*r, 0*r];


  first = 1
  for xpos in frange(-4, 3, 0.25):
	for ypos in frange(-4, 5, 0.25):
	  j1 = jstart[:]
	  j1[0] += xpos*r
	  f = (40.0 / 39.0)*2
	  j1[1] += ypos*r;
	  h1 = 0.4 * math.cos(j1[1])     # Height of 4th joint above 2nd joint
	  h2 = +0.02                  # Requested height of hand above 2nd joint; -8cm: just above floor with stabilo and w/o JR3
	  beta = math.asin((h1-h2) / 0.39)
	  j1[3] = ((math.pi/2)-math.fabs(j1[1])+beta);
	  j1[5] = -(math.pi - math.fabs(j1[1]) - j1[3])
	  #print "J1: " + str(j1[1]) + ", J2: " + str(j1[3]) + " J3: " + str(j1[5])
	  #continue

	  j2 = j1[:]
	  d_a = -5
	  j2[1] += d_a*r; j2[5] -= d_a*r;

	  pub_force.publish(force1)
	  goal.trajectory.points = []
	  goal.trajectory.points.append(JointTrajectoryPoint(j1, v2, avec, rospy.Duration())); t += 1
	  client.send_goal(goal)
	  result = client.wait_for_result()
	  time.sleep(0.0)

	  # Zero force of JR3
	  if first:
		time.sleep(1.0)
		e = Empty()
		pub_zero.publish()
		first = 0

	  pub_force.publish(force2)
	  goal.trajectory.points = []
	  goal.trajectory.points.append(JointTrajectoryPoint(j2, v1, avec, rospy.Duration())); t += 1
	  #goal.trajectory.points.append(JointTrajectoryPoint(j1, vvec, avec, rospy.Duration())); t += 1
	  client.send_goal(goal)
	  result = client.wait_for_result()
	  forcestop = (client.get_state() == 2)
	  #print "Result:"; print client.get_result(); print client.get_state();

	  pub_force.publish(force1)
	  time.sleep(0.00)

	  # Get JointState
	  msg = rospy.wait_for_message("/joint_states", JointState)
	  z = 50*(msg.position[1] - j2[1])
	  if forcestop:
		 fh.write(str(xpos) + " " + str(ypos) + " " + str(z) + "\n")

	  goal.trajectory.points = []
	  goal.trajectory.points.append(JointTrajectoryPoint(j1, v2, avec, rospy.Duration())); t += 1
	  client.send_goal(goal)
	  result = client.wait_for_result()
	  #print "ResultUp:"; print client.get_result(); print client.get_state();

	  if forcestop: sys.stdout.write("+")
	  else: sys.stdout.write(".")
	  sys.stdout.flush()
	print

  fh.close



