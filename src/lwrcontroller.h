#ifndef LWRCONTROLLER_H
#define LWRCONTROLLER_H
#include <cstdio>
#include "ros/ros.h"
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <boost/thread.hpp>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "friclient.h"

#define	SQR(x)	x*x

class LWRController
{
public:
    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> TASJT;

    LWRController(ros::NodeHandle n);
    /** Action client callback to setup points */
    void goalJointTrajectory(TASJT::GoalHandle gh);
    /** Action client callback to cancel */
	void cancelCB(TASJT::GoalHandle gh);
    /** Action client callback to send result */
    void sendResult(TASJT::GoalHandle gh);
    /** Thread to publish robot state */
    void pubStatus();
    /** Message callback to set stopping force */
	void onForceElement(const geometry_msgs::PoseStamped& f);
    /** Message callback for cartesian velocity command */
    void onCartVelocity(const geometry_msgs::PoseStamped& vel);
    /** Message callback for cartesion path (nav_msgs::Path) */
    void onPath(const nav_msgs::Path& path);
    /** Message callback to define current force as zero on the force of JR3 sensor */
	void onJR3zero(const std_msgs::Empty & m);

    FRIClient mFriClient;
    tf::TransformListener mTF;
    ros::NodeHandle mNode;
    std::string mTmp;
    ros::Subscriber mSubForceLimitElement, mSubJR3zero, mSubPath, mSubCartVel;
    TASJT mJTAction;
	boost::thread mThreadSend;
};

#endif // LWRCONTROLLER_H
