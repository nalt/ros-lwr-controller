#! /usr/bin/env python
## \file
#  Demo for cartesian impedance control
#  \date 2012-06-13
#  \author (c) Nicolas Alt, TU München
#  
#  Reads poses from a yaml file (that can be created with record_poses.py) and moves the robot along a path

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


def send_yaml_poses(yaml_file, publisher):
    stream = file('poses1.yml', 'r')
    poses = yaml.load(stream)
    stream.close()

    path = Path()
    pref = PoseStamped();
    pref.header.frame_id = "";
    h = pref.header;

    for p in poses:
        path.poses.append(PoseStamped(h, Pose(Point(*p.position), Quaternion(*p.orientation))))

    print path

    rospy.sleep(0.5)
    print "Publishing path with " + str(len(poses)) + " poses"
    publisher.publish(path);
    rospy.sleep(0.5)



if __name__ == '__main__':
  rospy.init_node('test_pose')
  pub = rospy.Publisher('/lwrcontroller/target_path', Path)

  send_yaml_poses('poses1.yml', pub)
  exit(0)

  path = Path()
  pref = PoseStamped();
  pref.header.frame_id = "";
  h = pref.header;

  #path.poses.append(PoseStamped(h, Pose(Point(0,0,1.3), Quaternion(0,0,0,1)) )) # Quaternion(x,y,z,w)
  q0 = Quaternion(0.543006298281,  -0.83969639568,  -0.000317475693053,  -0.00734998759112)
  q1 = Quaternion(0.573948216332,  -0.803399347087,  -0.153615950267,  0.0391800190502)
  q2 = Quaternion(0.434527902621,  -0.879976950847,  0.172859082092,  -0.0833414996333)
  q3 = Quaternion(0.42412312219,  -0.860583767633,  0.17705246832,  0.21947113608)
  q4 = Quaternion(0.587521054143,  -0.782881492102,  -0.0722604659308,  -0.191556793995)
  path.poses.append(PoseStamped(h, Pose(Point(-0.4, -0.4, 0.4), q0)))
  path.poses.append(PoseStamped(h, Pose(Point(-0.4, -0.4, 0.4), q1)))
  path.poses.append(PoseStamped(h, Pose(Point(-0.4, -0.4, 0.4), q2)))
  path.poses.append(PoseStamped(h, Pose(Point(-0.5, -0.5, 0.2), q3)))
  path.poses.append(PoseStamped(h, Pose(Point(-0.5, -0.5, 0.2), q1)))
  path.poses.append(PoseStamped(h, Pose(Point(-0.5, -0.5, 0.2), q2)))

  rospy.sleep(0.5)
  print "pub"
  pub.publish(path);
  rospy.sleep(0.5)
