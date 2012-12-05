#! /usr/bin/env python
# 2012-06-20
# (c) Nicolas Alt, TU München
# Save arm poses to a yaml file
#
# Use e.g. together with gravity mode. Press enter to save the current pose.
# File and frames defined in the code.

import roslib; roslib.load_manifest('lmtlwr')
import rospy
import tf
import math
import time
import yaml

from geometry_msgs.msg import *
from control_msgs.msg import *

if __name__ == '__main__':
  rospy.init_node('record_poses')
  tl = tf.TransformListener()

  frame = "/calib_lwr_arm_base_link"
  frame_base = "/lwr_tool_tcp"
  filename = "poses.yml"
  print "Saving poses of " + frame + " in frame " + frame_base + " to " + filename
  pose_list = []

  while not rospy.is_shutdown():
    c = raw_input("Press ENTER to save pose, q to quit ")
    if c == 'q':
        break
    try:
        tl.waitForTransform(frame, frame_base, rospy.Time(), rospy.Duration(0.1))
        (trans,rot) = tl.lookupTransform(frame, frame_base, rospy.Time(0))
        p = Pose(trans, rot)
        print p
        pose_list.append(p)
    except Exception as ex:
        print "Error while waiting for transform:"
        print ex


  stream = file(filename, 'w')
  yaml.dump(pose_list, stream)

