#! /usr/bin/env python
## \file
#  Inverse and forward kinematics using service from arm_kinematics package; arm control in joint space.
#  \date 2012-06-19
#  \author (c) Nicolas Alt, TU München
#
#  Partly taken from: http://hamm.cs.utexas.edu:1080/roswiki/doc/api/assistive_teleop/html/ik__testing_8py_source.html
#  The PR2 kinematic solver did not work


PKG = 'lmtlwr'

import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib
import math
import PyKDL
import tf_conversions.posemath as pm
import pprint
import time

from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
from kinematics_msgs.srv import *
from arm_navigation_msgs.srv import *
from trajectory_msgs.msg import *
from control_msgs.msg import *

x_steps = 2
y_steps = 2
z_steps = 2

class ik_test():

    def __init__(self):
        rospy.init_node('ik_test')
        
        self.get_fk()
        
        # ==== ROS Services ====
        self.ik_info = '/lwr_kinematics/get_ik_solver_info'
        print "waiting for ik service"
        rospy.wait_for_service(self.ik_info)
        self.ik_info_service = rospy.ServiceProxy(self.ik_info, GetKinematicSolverInfo)
        ik_info_req = GetKinematicSolverInfoRequest()
        self.ik_info = self.ik_info_service.call(ik_info_req)
        print "IK INFO:"
        print self.ik_info

        self.ik_service = rospy.ServiceProxy('/lwr_kinematics/get_ik', GetPositionIK, persistent=True)
        
        self.jt_client = actionlib.SimpleActionClient('lwrcontroller/joint_trajectory_action', FollowJointTrajectoryAction)
        print 'Waiting for action server...'
        self.jt_client.wait_for_server()

        # ==== Init the IK service ====
        self.ik_req = GetPositionIKRequest()
        self.ik_req.timeout = rospy.Duration(5)
        self.ik_req.ik_request.ik_link_name = 'lwr_arm_7_link' # 'lwr_arm_7_link'
        self.ik_req.ik_request.ik_seed_state.joint_state.name = self.ik_info.kinematic_solver_info.joint_names
        self.ik_req.ik_request.ik_seed_state.joint_state.position =  [0]*7
        # Init joints fotom current position
        print "Init joints from current position..."
        robot_js = JointState()
        msg = rospy.wait_for_message("/joint_states", JointState)
        self.ik_req.ik_request.ik_seed_state.joint_state.position = msg.position
        print "done"
        
        self.ps = PoseStamped()
        #self.ps.header.stamp = rospy.Time(0)
        self.ps.header.frame_id = '/calib_lwr_arm_base_link'
        self.ps.pose.position.x = self.ps.pose.position.y = self.ps.pose.position.z = 0
        self.ps.pose.orientation.x = self.ps.pose.orientation.y = self.ps.pose.orientation.z = 0
        self.ps.pose.orientation.w = 1

        #self.go_pose(PoseStamped(h, Pose(Point(0.1, 0.1, 0.9), Quaternion(0,0,0,1))))

    def pose_demo_stablio(self):
        h = self.ps.header;
        #self.go_pose(PoseStamped(h, Pose(Point(-0.4, -0.4, 0.5), Quaternion(0.96,0.14,-0.08, -0.23))))
        x = -0.4; y = -0.4; z = 0.20; zd = -0.057;
        schlaf = 1
        
        
        q0 = Quaternion(0.543006298281,  -0.83969639568,  -0.000317475693053,  -0.00734998759112)
        q1 = Quaternion(0.573948216332,  -0.803399347087,  -0.153615950267,  0.0391800190502)
        q2 = Quaternion(0.434527902621,  -0.879976950847,  0.172859082092,  -0.0833414996333)
        q3 = Quaternion(0.42412312219,  -0.860583767633,  0.17705246832,  0.21947113608)
        q4 = Quaternion(0.587521054143,  -0.782881492102,  -0.0722604659308,  -0.191556793995)
        self.go_pose(PoseStamped(h, Pose(Point(x, y, z), q0)))
        self.go_pose(PoseStamped(h, Pose(Point(x, y, z+zd), q0)))
        time.sleep(schlaf)
        self.go_pose(PoseStamped(h, Pose(Point(x, y, z), q1)))
        self.go_pose(PoseStamped(h, Pose(Point(x, y, z+zd), q1)))
        time.sleep(schlaf)
        self.go_pose(PoseStamped(h, Pose(Point(x, y, z), q3)))
        self.go_pose(PoseStamped(h, Pose(Point(x, y, z+zd), q3)))
        time.sleep(schlaf)
        self.go_pose(PoseStamped(h, Pose(Point(x, y, z), q2)))
        self.go_pose(PoseStamped(h, Pose(Point(x, y, z+zd), q2)))
        time.sleep(schlaf)
        self.go_pose(PoseStamped(h, Pose(Point(x, y, z), q4)))
        self.go_pose(PoseStamped(h, Pose(Point(x, y, z+zd), q4)))
        time.sleep(schlaf)
        self.go_pose(PoseStamped(h, Pose(Point(x, y, z+0.2), q4)))
        #self.go_pose(PoseStamped(h, Pose(Point(x, y, z), Quaternion(1.0,0.0,0.0,0.0))))
        #self.go_pose(PoseStamped(h, Pose(Point(x, y, z), Quaternion(0.9962,0.0,0.0,0.0872))))
        #self.go_pose(PoseStamped(h, Pose(Point(x, y, z), Quaternion(0.9962,0.0,0.0,-0.0872))))
        #self.go_pose(PoseStamped(h, Pose(Point(x, y, z+0.1), Quaternion(1.0,0.0,0.0,0.0))))
    
    ## Transforms pose from the TCP to the robot endeffector - currently hard-coded
    def transform_pose(self, p1):
        vd = PyKDL.Vector(-0.028, 0.0099, 0.202)
        f1 = pm.fromMsg(p1)
        vdr = f1.M*vd
        f1.p = f1.p - vdr
        return pm.toMsg(f1)
    
    ## Convert pose to joint space and send it to robot
    def go_pose(self, pose):
        pose.pose = self.transform_pose(pose.pose)
        self.ik_req.ik_request.pose_stamped = pose
        print pose
        #print self.ik_req
        ik_goal = self.ik_service(self.ik_req)
        #print ik_goal
        
        # Send goal to robot
        if ik_goal.error_code.val == 1:
            print "Sending JointTrajectoryGoal..."
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ik_goal.solution.joint_state.name
            j = ik_goal.solution.joint_state.position
            print j
            goal.trajectory.points.append(JointTrajectoryPoint(j, [], [], rospy.Duration(5)));
            goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)
            
            self.jt_client.send_goal(goal)
            result = self.jt_client.wait_for_result()
            print 'JointTrajectoryClient Result - state:'+repr(self.jt_client.get_state())
            
            # Save current config
            self.ik_req.ik_request.ik_seed_state.joint_state.position = j
    
    ## For testing forward kinematic
    def get_fk(self):
        print "FORWARD KINEMATIC get_fk:"
        rospy.wait_for_service('/lwr_kinematics/get_fk')
        rospy.wait_for_service('/lwr_kinematics/get_fk_solver_info')
        get_fk_proxy = rospy.ServiceProxy('/lwr_kinematics/get_fk', GetPositionFK, persistent=True)
        get_fk_solver_info_proxy = rospy.ServiceProxy('/lwr_kinematics/get_fk_solver_info', GetKinematicSolverInfo)
        solver_info = get_fk_solver_info_proxy()
        print solver_info
        
        joints = solver_info.kinematic_solver_info.joint_names

        request = GetPositionFKRequest()
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.header.frame_id = 'torso_link'
        request.robot_state.joint_state.name = joints
        request.robot_state.joint_state.position = [0]*len(joints)
        #request.robot_state.joint_state.position[1] = math.pi/4

        request.fk_link_names = list()
        request.fk_link_names.append("lwr_arm_7_link")
        #request.fk_link_names.append("lwr_tool_tcp")
        request.header.frame_id = "calib_lwr_arm_base_link"
        
        fk_response = get_fk_proxy(request)
        print fk_response
    
    
    def orig_run_poses(self):
        for i in xrange(x_steps):
            print i, "/50\n"
            self.ps.pose.position.x = float(i)/50
            for j in xrange(y_steps):
                #print j, "/ 100\n"
                self.ps.pose.position.y = -1 + float(j)/50
                for k in xrange(z_steps):
                    self.ps.pose.position.z = -1 + float(k)/50
                    print self.count
                    self.get_ik(self.ps, self.count)
                 

    def get_ik(self, ps, count):
        self.ik_req.ik_request.pose_stamped = ps
        ik_goal = self.ik_service(self.ik_req)
        self.results[0][count] = ik_goal.error_code.val
        self.results[1][count] = ps
        self.count += 1


                     
if __name__ == '__main__':
    ikt = ik_test()
    ikt.pose_demo_stablio()
