#ifndef FRICLIENT_H
#define FRICLIENT_H
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <FastResearchInterface.h>
#include <TypeIRML.h>
#include "jr3pci-ioctl.h"


#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#define RAD(A)	((A) * M_PI / 180.0 )
#define DEG(A)	((A) * 180.0 / M_PI )
#define SGN(x)	((x > 0) - (x < 0))

extern bool is_running;

class FRIClient
{
public:  
	FRIClient();
	void start(std::string init_file, int send_frequency);
	bool JR3_open(std::string device);
	bool JR3_zeroforce();
    /** Move arm along joint trajectory, i.e. a list of joint states.
     *  Switches robot controller state mNextState to TRAJ_JOINT and KUKA robot to JOINT_POSITION_CONTROL mode.
     *  Intermediate states are calculated with TypeIRML.
     *  Locks the mutex.
     */
    bool moveJointTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint>& jointtrajectory);
    /** Move arm along a path in cartesian space.
     *  Switches robot controller state mNextState to CARTESIAN and KUKA robot to Cartesian Impedence mode.
     *  Locks the mutex.
     */
    bool moveCartPath(const std::vector<geometry_msgs::PoseStamped>& path);
    /** Move arm with velocity given in cartesian space.
     *  Switches robot controller state mNextState to VELOCITY_CART and KUKA robot to Cartesian Impedence mode.
     *  Robot stops if commands do not come in regularly.
     *  Locks the mutex.
     */
    bool moveCartVelocity(geometry_msgs::Pose velocity);
	void setStoppingForce(geometry_msgs::Pose force, int type);
	void cancelMotion();
	int getForces(float *forces, bool use_jr3);
    /** Helper function: Convert between pose formats from KUKA (in=1), ROS (in=2) */
    void pose_kuka2ros(float *p_kuka, geometry_msgs::Pose &p_ros, Eigen::Quaterniond *quaternion,  int in);



    enum CTRLSTATE { IDLE=0, NOCHANGE, BREAK, FORCES_EXCEEDED, TRAJ_JOINT, TRAJ_CARTESIAN, VELOCITY_CART };
	CTRLSTATE mNextState;
	FastResearchInterface *mFRI;

	boost::mutex mMutex;
	boost::condition_variable mCondSend, mCondResult;
    /** Tells the ROS controller that it is time to send a status update */
    bool mSendStatus;
	enum RESULT { UNDEF, SUCCESS, CANCELED, FORCE_EXCEEDED, FAILED };
	RESULT mCMDresult;

    float mForces[6], mMeasuredJoint[7], mMeasuredCart[12];
    /** See return type of getForces() */
	int mForcesFrame;

    /** Current robot control mode */
	FastResearchInterface::LWRControlModes mCMDmode;
    /** Saves commanded joint tractory; use moveJointTrajectory() */
	std::vector<trajectory_msgs::JointTrajectoryPoint> mCMDjointtrajectory;
    /** Saves commanded cartesian path; use moveCartPath() */
    std::vector<geometry_msgs::PoseStamped> mCMDpath;
    /** Saves commanded pose/velocity */
    geometry_msgs::Pose mCMDpose, mCMDvel_cart;


private:
    /** \brief Controller loop with state machine
     *  This routine runs synchronously with the FRI click tick and generates the FRI commands
     *  A state machine inside switches between the different operation modes and interpolates the trajectories.
     */
	void FRIspin();
	void FRIspin_old();
    /** \brief Stops robot and restarts it in requested mode */
	bool FRICheckRobot();
	void notifyResult(RESULT r);

	int mJR3_fd;
	bool mJR3_active;
	force_array mJR3_fullscale;

	boost::thread mThread;
	std::string mInitFile;
	int mStatusFrequency;
	int mCMDcounter;

	geometry_msgs::Pose mStopForce;
};


#endif // FRICLIENT_H
