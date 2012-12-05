/** \file
  * \brief ROS node for robot control; communicates between ROS and friclient.cpp
  * \author (c) Nicolas Alt, TU München
  * License: See License.txt. If missing, contact  n.alt -at- tum.de
  */

#include "lwrcontroller.h"
bool is_running = false;

LWRController::LWRController(ros::NodeHandle n) :
    mJTAction(n, "joint_trajectory_action", boost::bind(&LWRController::goalJointTrajectory, this, _1),
			  boost::bind(&LWRController::cancelCB, this, _1), false)
{
	std::string init_fn;
	int freq;
	mNode = n;
	mNode.getParam("fri_init_file", init_fn);
	mNode.getParam("status_frequency", freq);
	mFriClient.start(init_fn, freq);
	mFriClient.JR3_open("/dev/jr3");
	mJTAction.start();
    mSubPath = mNode.subscribe("target_path", 3, &LWRController::onPath, this);
    mSubCartVel = mNode.subscribe("velocity_cartesian", 3, &LWRController::onCartVelocity, this);
	mSubForceLimitElement = mNode.subscribe("force_limit_element", 3, &LWRController::onForceElement, this);
	mSubJR3zero = mNode.subscribe("jr3_zero", 3, &LWRController::onJR3zero, this);
	mThreadSend = boost::thread(&LWRController::pubStatus, this);
}


void LWRController::onCartVelocity(const geometry_msgs::PoseStamped& vel)
{
    geometry_msgs::PoseStamped vel_base;
    if (vel.header.frame_id != "/calib_lwr_arm_base_link" &&vel.header.frame_id != "") {
        try	{ mTF.transformPose("/calib_lwr_arm_base_link", vel, vel_base); }
        catch (...) { ROS_ERROR_STREAM("onCartVelocity: Failed to transform pose");  return; }
        ROS_INFO_STREAM("onCartVelocity: Transformed force pose to current base coordinate system");
    } else vel_base = vel;
    ROS_DEBUG_STREAM("onForceElement: Setting new force limit");
    mFriClient.moveCartVelocity(vel_base.pose);
}


void LWRController::onForceElement(const geometry_msgs::PoseStamped& f)
{
    geometry_msgs::PoseStamped f_base;
    if (f.header.frame_id != "/calib_lwr_arm_base_link") {
        try	{ mTF.transformPose("/lwr_arm_7_link", f, f_base); }
        catch (...) {
            ROS_ERROR_STREAM("onForceElement: Failed to transform pose");
            return;
        }
        ROS_INFO_STREAM("onForceElement: Transformed force pose to current base coordinate system");
    } else f_base = f;
    ROS_DEBUG_STREAM("onForceElement: Setting new force limit");
    mFriClient.setStoppingForce(f_base.pose, 0);
}

void LWRController::onPath(const nav_msgs::Path& path)
{
     std::vector<geometry_msgs::PoseStamped> path2;
     path2.resize(path.poses.size());
    // Transform poses to arm coordinate system
    for (uint i=0; i<path.poses.size(); i++) {
        path2[i] = path.poses[i];
        if (path2[i].header.frame_id != "/calib_lwr_arm_base_link" && path2[i].header.frame_id != "") {
            try	{
                mTF.transformPose("/calib_lwr_arm_base_link", path.poses[i], path2[i]);
            } catch (...) {
                ROS_ERROR_STREAM("onPath: Failed to transform pose");
                return;
            }
        }
    }
    mFriClient.moveCartPath(path2);

    ROS_INFO_STREAM("Received Path with " << path.poses.size() << " points");
}

void LWRController::onJR3zero(const std_msgs::Empty & m)
{
    mFriClient.JR3_zeroforce();
}



void LWRController::goalJointTrajectory(TASJT::GoalHandle gh)
{
	boost::shared_ptr<const control_msgs::FollowJointTrajectoryGoal> goal = gh.getGoal();
	std::vector<trajectory_msgs::JointTrajectoryPoint> points = goal->trajectory.points;
	bool invalid = false;

	for (uint i=0; i<points.size(); i++) {
		if (points[i].positions.size() != 7) invalid = true;
	}

	ROS_INFO_STREAM("Received Joint Tracjectory Action with " << points.size() << " points. Valid: " << !invalid);

	if (!invalid) {
		// Setup FRIClient
        mFriClient.moveJointTrajectory(points);
		gh.setAccepted();
		boost::thread(boost::bind(&LWRController::sendResult, this, gh));
	} else gh.setRejected();
}


void LWRController::cancelCB(TASJT::GoalHandle gh)
{
    ROS_INFO_STREAM("Request to cancel Action received");
	mFriClient.cancelMotion();
}


/** Wait for mFriClient's result - started as seperate thread by goalCB */
void LWRController::sendResult(TASJT::GoalHandle gh)
{
	boost::unique_lock<boost::mutex> lock(mFriClient.mMutex);
	do {
		mFriClient.mCondResult.timed_wait(lock, boost::posix_time::milliseconds(150));

		switch (mFriClient.mCMDresult) {
		// TODO: Does not seem to work correctly...
		case FRIClient::CANCELED:
			gh.setCanceled(); break;
		case FRIClient::FORCE_EXCEEDED:
		{
			gh.setCanceled(TASJT::Result(), "FORCE_EXCEEDED"); break;
		}
		case FRIClient::FAILED:
			gh.setAborted(); break;
		case FRIClient::SUCCESS:
			gh.setSucceeded(); break;
		default: break;
		}
	} while (mFriClient.mCMDresult == FRIClient::UNDEF);
    ROS_INFO_STREAM("Action finished with state: " << mFriClient.mCMDresult);
}


void LWRController::pubStatus()
{
	ros::Publisher pub_js = mNode.advertise<sensor_msgs::JointState>("/joint_states", 1);
	ros::Publisher pub_force_vis = mNode.advertise<visualization_msgs::Marker>("force_visual", 1);
	ros::Publisher pub_force = mNode.advertise<geometry_msgs::PoseStamped>("force", 1);
    ros::Publisher pub_ptool = mNode.advertise<geometry_msgs::PoseStamped>("pose_tool", 1);
	std::string joint_names;
	mNode.getParam("joint_names", joint_names);

	while (is_running) {
		boost::unique_lock<boost::mutex> lock(mFriClient.mMutex);
		mFriClient.mCondSend.timed_wait(lock, boost::posix_time::milliseconds(150));
		// TODO: Should check for condition=true; measure runtime here!
		if (mFriClient.mSendStatus) {
			mFriClient.mSendStatus = false;

			// ==== JOINT STATES
			sensor_msgs::JointState js;
			js.header.stamp = ros::Time::now();
			js.name.resize(7); js.position.resize(7);
			for (uint i=0; i<7; i++) {
				char name[256];
				sprintf(name, joint_names.c_str(), i);
				js.name[i] = std::string(name);
                js.position[i] = mFriClient.mMeasuredJoint[i];
			}
			pub_js.publish(js);

            // ==== Cartesian Pose
            float *pval = mFriClient.mMeasuredCart;
            geometry_msgs::PoseStamped p_cart;
            mFriClient.pose_kuka2ros(pval, p_cart.pose, NULL, 1);
            p_cart.header.frame_id = "/calib_lwr_arm_base_link";
            pub_ptool.publish(p_cart);



			// ==== ESTIMATED FORCES
			// Looks like this pose is in the Base frame, and the z-axis is negated compared to ROS
			if (mFriClient.mForcesFrame > 0) {
				geometry_msgs::PoseStamped pF_base, pF_hand;
				pF_base.pose.position.x = mFriClient.mForces[0];
				pF_base.pose.position.y = mFriClient.mForces[1];
				pF_base.pose.position.z = mFriClient.mForces[2];
                // TODO: Quaternion!
				pF_base.pose.orientation.w = 1;
				pF_base.pose.orientation.x = 0;
				pF_base.pose.orientation.y = 0;
				pF_base.pose.orientation.z = 0;

				pF_base.header.stamp = ros::Time();
				if (mFriClient.mForcesFrame == 1) {
                    //pF_base.pose.position.z *= -1.0;
					pF_base.header.frame_id = "/calib_lwr_arm_base_link";
				}
				if (mFriClient.mForcesFrame == 2) {
					pF_base.header.frame_id = "/lwr_arm_7_link";
				}
				try {
					mTF.transformPose("/lwr_arm_7_link", pF_base, pF_hand);
					pub_force.publish(pF_hand);

					Eigen::Vector3d vF(pF_hand.pose.position.x, pF_hand.pose.position.y, pF_hand.pose.position.z);
					Eigen::Vector3d v_s(0,0,0.1);
					Eigen::Vector3d v_e = v_s + 0.1*vF;

					visualization_msgs::Marker marker;
					marker.header.frame_id = "/lwr_arm_7_link";
					marker.header.stamp = ros::Time();
					marker.ns = "my_namespace";
					marker.id = 0;
					marker.type = visualization_msgs::Marker::ARROW;
					marker.action = visualization_msgs::Marker::ADD;
					marker.points.resize(2);
					marker.points[0].x = v_s[0]; marker.points[0].y = v_s[1]; marker.points[0].z = v_s[2];
					marker.points[1].x = v_e[0]; marker.points[1].y = v_e[1]; marker.points[1].z = v_e[2];
					marker.scale.x = 0.01; marker.scale.y = 0.02; marker.scale.z = 0.02;
					marker.color.a = 1.0;
					marker.color.r = 1.0;
					marker.color.g = 0.0;
					marker.color.b = 1.0;
					pub_force_vis.publish(marker);
				} catch (...) {
					ROS_ERROR_STREAM("pubStatus: Failed to transform pose");
				}
			}
			//


		}
	}
	ROS_INFO_STREAM("LWRController::pubStatus: Exit");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "lwrcontroller");
	ros::NodeHandle n("~");
	is_running = true;
	LWRController lwr(n);
	ros::spin();
	std::cout << "About to exit...\n";
	is_running = false;
	usleep(2000000);
	return 0;
}
