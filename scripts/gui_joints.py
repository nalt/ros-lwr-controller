#! /usr/bin/env python
## \file
#  GUI to display arm joint positions with sliders and control in joint space
#  \date 2012-06-20
#  \author (c) Nicolas Alt, TU München
#  License: See License.txt. If missing, contact  n.alt -at- tum.de
#
#  Reads pre-defined poses from joint_positions.yml.
#  Listens on /joint_states, which is published by lmtlwr
#  TODO: Control robot as well

import wx
import sys
import math
import yaml
import time

import roslib;
roslib.load_manifest('lmtlwr')
import rospy
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import *
from control_msgs.msg import *

updateCnt = 0

def onJointState(data):
    global updateCnt
    #rospy.loginfo(rospy.get_name()+"I heard %s",data.position)
    app = wx.GetApp()
    for i in range(7):
        wx.CallAfter(app.frame.sliders[i].SetValue, round(180/math.pi * data.position[i]))
    updateCnt += 1
    wx.CallAfter(app.frame.textUpdate.SetLabel, "UPDATE " + ("."*(1 + (updateCnt % 50))) )

    
class MainWindow(wx.Frame):
    def __init__(self, parent, title):
        # Frame
        wx.Frame.__init__(self, parent, title = title, size = (200, 400))
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        # Joint state list box
        self.listbox = wx.Choice(self)
        stream = file('joint_positions.yml', 'r')
        self.joint_poses = yaml.load(stream)
        stream.close()
        self.listbox.Append("<Select pose)")
        for jp in self.joint_poses:
            self.listbox.Append(jp['name'])
        self.Bind(wx.EVT_CHOICE, self.onChoice, self.listbox)
        self.sizer.Add(self.listbox, 0, wx.EXPAND|wx.ALL,0)
        # Joint sliders
        self.sliders = [None]*7      
        for i in range(7):
            self.sliders[i] = wx.Slider(self, minValue=-180, maxValue=180, value=0, style=wx.EXPAND|wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS)
            self.sliders[i].SetToolTip(wx.ToolTip("Joint " + str(i)))
            self.sizer.Add(self.sliders[i], 0, wx.EXPAND|wx.ALL,0)
        self.textUpdate = wx.StaticText(self)
        self.sizer.Add(self.textUpdate, 0, wx.EXPAND|wx.ALL,0)
        self.SetSizer(self.sizer)
        self.Show(True)


    def onChoice(self, event):
        i = self.listbox.GetCurrentSelection()
        if i < 1 or i > len(self.joint_poses):
            return 0
        print self.joint_poses[i-1]['position']
        p = [ (x / 180.0 * math.pi) for x in self.joint_poses[i-1]['position'] ]
        if 'velocity' in self.joint_poses[i-1]:
            v = [ (x / 180.0 * math.pi) for x in self.joint_poses[i-1]['velocity'] ]
        else:
            v = []
        if 'acceleration' in self.joint_poses[i-1]:
            a = [ (x / 180.0 * math.pi) for x in self.joint_poses[i-1]['acceleration'] ]
        else:
            a = []
        print p
        # ROS publish
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['lwr_arm_0_joint', 'lwr_arm_1_joint', 'lwr_arm_2_joint', 'lwr_arm_3_joint', 'lwr_arm_4_joint', 'lwr_arm_5_joint', 'lwr_arm_6_joint']
        goal.trajectory.points.append(JointTrajectoryPoint(p,  v, a, rospy.Duration(5)))
        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)
        try:
            self.action_client
        except Exception:
            self.action_client = actionlib.SimpleActionClient('lwrcontroller/joint_trajectory_action', FollowJointTrajectoryAction)
            time.sleep(0.5)
        self.action_client.send_goal(goal)
    

if __name__ == "__main__":
    app = wx.App(False)
    app.frame = MainWindow(None, 'joint_display')
    rospy.init_node('joint_display')
    rospy.Subscriber("/joint_states", JointState, onJointState)
    
    try:
        app.MainLoop()
    except KeyboardInterrupt, e:
        pass
    print 'exiting'
