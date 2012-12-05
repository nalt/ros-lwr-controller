#include "friclient.h"
/** \file
  * \brief Robot controller and interface to FRILibrary
  * \author (c) Nicolas Alt, TU München
  * License: See License.txt. If missing, contact  n.alt -at- tum.de
  */

FRIClient::FRIClient()
{
}

void FRIClient::start(std::string init_file, int status_frequency)
{
    mInitFile = init_file;
    mCMDresult = UNDEF;
    mCMDcounter = -1;
    mCMDmode = 0;
    mStatusFrequency = status_frequency;
    mSendStatus = false;
    mStopForce.position.x = NAN; mStopForce.position.y = NAN; mStopForce.position.z = NAN;
    mJR3_active = false;

    mThread = boost::thread(&FRIClient::FRIspin, this);
}

void FRIClient::FRIspin()
{
    // Open & init FRI interface
    ROS_INFO_STREAM("FRIClient: Opening FRI Interface...");
    mFRI = new FastResearchInterface(mInitFile.c_str());
    mFRI->WaitForKRCTick();
    usleep(100000);
    float JointStiffnessValues[LBR_MNJ], JointDampingValues[LBR_MNJ],
            CartStiffnessValues[FRI_CART_VEC], CartDampingValues[FRI_CART_VEC];
    for (uint i = 0; i < LBR_MNJ; i++)	{
        JointStiffnessValues	[i] =	(float)10.0;
        JointDampingValues		[i]	=	(float)0.7;
    }

    for (uint i = 0; i < 3; i++)
        CartStiffnessValues		[i]	=	(float)200.0;
    for (uint i = 3; i < FRI_CART_VEC; i++)
        CartStiffnessValues		[i]	=	(float)20.0;
    for (uint i = 0; i < FRI_CART_VEC; i++)	{
        CartDampingValues		[i]	=	(float)0.7;
    }
    mFRI->SetCommandedCartDamping(CartStiffnessValues);
    mFRI->SetCommandedCartStiffness(CartDampingValues);
    mFRI->SetCommandedJointDamping(JointDampingValues);
    mFRI->SetCommandedJointStiffness(JointStiffnessValues);
    float t_cycle = mFRI->GetFRICycleTime();
    ROS_INFO("FRIClient: FRI Interface opened with cyle time %5.1fms", t_cycle*1000);

    CTRLSTATE curstate=IDLE, nextstate = NOCHANGE;
    mNextState = NOCHANGE;
    int loop_counter = 0;
    TypeIRML rml(7, t_cycle);
    TypeIRMLInputParameters rmlIP(7);
    TypeIRMLOutputParameters rmlOP(7);
    geometry_msgs::Pose pose_start, pose_end;
    bool state_VELOCITY_CART_ok = false;

    while (is_running) {
        // General robot communication
        bool robot_ok = FRICheckRobot();
        if (mFRI->WaitForKRCTick(110000) != 0)
            continue;
        if (!robot_ok) {
            mCMDmode = 0;
            notifyResult(FAILED);// TODO: Check if action active!
            nextstate = IDLE;
            continue;
        }
        if (!mFRI->IsMachineOK() && curstate != IDLE)
            { nextstate = IDLE; }
        float cmd_joint[7];
        mFRI->GetMeasuredJointPositions(mMeasuredJoint);
        mFRI->GetMeasuredCartPose(mMeasuredCart);
        mFRI->GetCommandedJointPositions(cmd_joint);
        if (loop_counter % mStatusFrequency == 0) {
            mSendStatus = true;
            mCondSend.notify_one();
        }
        loop_counter++;

        // Force exceeded?
        bool force_exceeded = false;
        mForcesFrame = getForces(mForces, mJR3_active);
        if (mForcesFrame > 0) {
            // Check if any x/y/z element of the force exceeds the threshold
            double fm = mStopForce.position.x;
            if (!std::isnan(fm) && SGN(fm)*mForces[0] > SGN(fm)*fm) force_exceeded = true;
            fm = mStopForce.position.y;
            if (!std::isnan(fm) && SGN(fm)*mForces[1] > SGN(fm)*fm) force_exceeded = true;
            fm = mStopForce.position.z;
            if (!std::isnan(fm) && SGN(fm)*mForces[2] > SGN(fm)*fm) force_exceeded = true;
        }

        //ROS_ERROR("sgnfm: %d, fm: %.1f, f: %.1f", SGN(fm), fm, robot_forces[2] );

        // STATE DECISION
        bool is_enter = false;
        // nextstate is set by the state machine itself
        if (nextstate != NOCHANGE)
            { is_enter = true; curstate = nextstate; }
        nextstate = NOCHANGE;
        // mNextState is set by external routines
        mMutex.lock();
        if (mNextState != NOCHANGE)
            { is_enter = true; curstate = mNextState; }
        mNextState = NOCHANGE;
        mMutex.unlock();

        // STATE MACHINE
        std::stringstream text_enter;
        mMutex.lock();
        switch (curstate) {
            case IDLE:
                break;

            case FORCES_EXCEEDED:
                if (is_enter)
                    text_enter << "Force exceeded, BREAK. f = ("
                               << mForces[0] <<","<< mForces[1] <<","<< mForces[2] <<")";
                // continue to BREAK
            case BREAK: {
                bool allzero = true;
                /* Käse: for (uint i = 0; i < 7; i++) {
                    float amax = RAD(90) * t_cycle;
                    if (robot_joint_state[i] > amax) {
                        send_joint[i] = robot_joint_state[i] - amax;
                        allzero = false; }
                    else if (robot_joint_state[i] < amax) {
                        send_joint[i] = robot_joint_state[i] + amax;
                        allzero = false; }
                    else
                        send_joint[i] = 0.0;
                    mFRI->SetCommandedJointPositions(send_joint);
                } */
                if (allzero) nextstate = IDLE;
                break;
            }
            case VELOCITY_CART: {
                if (is_enter) {
                    if (mCMDmode != FastResearchInterface::CART_IMPEDANCE_CONTROL) {
                        // Change command mode and reenter
                        mCMDmode = FastResearchInterface::CART_IMPEDANCE_CONTROL;
                        nextstate = VELOCITY_CART;
                        state_VELOCITY_CART_ok = false;
                        break;
                    }
                    if (!state_VELOCITY_CART_ok) pose_kuka2ros(mMeasuredCart, pose_start, NULL, 1);
                }

                // Check if we should stop because of timeout
                if (mCMDcounter > 0.2 / t_cycle) {
                    nextstate = BREAK;
                    break;
                }
                mCMDcounter++;
                // Calulate new position
                pose_start.position.x += mCMDvel_cart.position.x * t_cycle;
                pose_start.position.y += mCMDvel_cart.position.y * t_cycle;
                pose_start.position.z += mCMDvel_cart.position.z * t_cycle;
                Eigen::Quaterniond q_start, q_cur;
                pose_kuka2ros(NULL, pose_start, &q_start, 2);
                pose_kuka2ros(NULL, mCMDvel_cart, &q_cur, 2);
                Eigen::AngleAxisd r_speed(q_cur);
                r_speed = Eigen::AngleAxisd(r_speed.angle()*t_cycle, r_speed.axis());
                q_cur = q_start * Eigen::Quaterniond(r_speed);
                // Send velocity to Kuka
                float p_kuka[12];
                pose_kuka2ros(p_kuka, pose_start, NULL, 2); // get translation speed
                pose_kuka2ros(p_kuka, pose_start, &q_cur, 3); // get rotatation speed
                ROS_INFO("TRAJ_CARTESIAN pose: T= %6.3f, %6.3f, %6.3f; Q= %6.3f, %6.3f, %6.3f",
                         p_kuka[3], p_kuka[7], p_kuka[11], pose_start.orientation.x, pose_start.orientation.y, pose_start.orientation.z);
                mFRI->SetCommandedCartPose(p_kuka);
                state_VELOCITY_CART_ok= true;
                break;
            }
            case TRAJ_JOINT: {
                // TODO: HACKY
                // IF force exceeded, go to last point
                // if (force_exceeded && mCMDcounter < (int)mCMDjointtrajectory.size()-1) {
                if (force_exceeded) {
                    notifyResult(FORCE_EXCEEDED);
                    nextstate = IDLE;
                    break;
                }

                if (is_enter) {
                    if (mCMDmode != FastResearchInterface::JOINT_POSITION_CONTROL) {
                        // Change command mode and reenter
                        mCMDmode = FastResearchInterface::JOINT_POSITION_CONTROL;
                        nextstate = TRAJ_JOINT;
                        break;
                    }
                    // Init RML: Set next target point
                    for (uint i = 0; i < 7; i++) {
                        trajectory_msgs::JointTrajectoryPoint cP = mCMDjointtrajectory[mCMDcounter];
                        rmlIP.CurrentPosition->VecData[i] =	(double)DEG(mMeasuredJoint[i]);
                        rmlIP.CurrentVelocity->VecData[i] =	(double)DEG(0);
                        rmlIP.TargetPosition->VecData[i]  =	(double)DEG(cP.positions[i]);
                        rmlIP.MaxVelocity->VecData[i]     = (i<cP.velocities.size())? DEG(cP.velocities[i]) : 30.0;
                        rmlIP.MaxAcceleration->VecData[i] =	(i<cP.accelerations.size())? DEG(cP.accelerations[i]) : 60.0;
                        rmlIP.SelectionVector->VecData[i] =	true;
                    }
                    text_enter << "for point " << mCMDcounter;
                    break;
                }

                // Control loop: Move robot along path
                int rmlResult = rml.GetNextMotionState_Position(rmlIP, &rmlOP);

                switch (rmlResult) {
                    case TypeIRML::RML_FINAL_STATE_REACHED:

                        if (mCMDcounter < (int)mCMDjointtrajectory.size()-1) {
                            mCMDcounter++;
                            nextstate = TRAJ_JOINT; // force re-entering
                        } else {
                            notifyResult(SUCCESS);
                            nextstate = IDLE;
                        }
                        // continue to RML_WORKING (?)
                        break;
                    case TypeIRML::RML_WORKING:
                        for (uint i = 0; i < 7; i++) {
                            cmd_joint[i] = RAD((double)(rmlOP.NewPosition->VecData[i]));
                            //printf("%7.4f ",cmd_joint[i]); if (i==6) printf("\n ",cmd_joint[i]);
                        }
                        mFRI->SetCommandedJointPositions(cmd_joint);
                        *(rmlIP.CurrentPosition) = *(rmlOP.NewPosition);
                        *(rmlIP.CurrentVelocity) = *(rmlOP.NewVelocity);
                        break;
                    default:
                        ROS_ERROR("FRIspin: ERROR during trajectory generation (%d)", rmlResult);
                        notifyResult(FAILED);
                        nextstate = IDLE;
                        break;
                }
                break;
                }
        case TRAJ_CARTESIAN: {
                if (is_enter) {
                    if (mCMDmode != FastResearchInterface::CART_IMPEDANCE_CONTROL) {
                        // Change command mode and reenter
                        mCMDmode = FastResearchInterface::CART_IMPEDANCE_CONTROL;
                        nextstate = TRAJ_CARTESIAN;
                        break;
                    }
                    // Init RML: Set next target point
                    // Initial position
                    pose_end = mCMDpath[mCMDcounter].pose;
                    pose_kuka2ros(mMeasuredCart, pose_start, NULL, 1);
                    // Start position for interpolation
                    rmlIP.CurrentPosition->VecData[0] = pose_start.position.x;
                    rmlIP.CurrentPosition->VecData[1] = pose_start.position.y;
                    rmlIP.CurrentPosition->VecData[2] = pose_start.position.z;
                    rmlIP.CurrentPosition->VecData[3] = 0;  // Alpha factor; rotation interpolation done with Eigen
                    // Target position
                    Eigen::Quaterniond q_start, q_end;
                    pose_kuka2ros(NULL, pose_start, &q_start, 2);
                    pose_kuka2ros(NULL, pose_end, &q_end, 2);
                    double dq = q_start.angularDistance(q_end);
                    if (dq <= 0) dq = 0.001;
                    rmlIP.TargetPosition->VecData[0] = pose_end.position.x;
                    rmlIP.TargetPosition->VecData[1] = pose_end.position.y;
                    rmlIP.TargetPosition->VecData[2] = pose_end.position.z;
                    rmlIP.TargetPosition->VecData[3] = dq;
                    for (uint i = 4; i < 7; i++) {
                        rmlIP.CurrentPosition->VecData[i] = 0;
                        rmlIP.TargetPosition->VecData[i] = 0;
                    }
                    // Velocity and acceleration limits


                    for (uint i = 0; i < 7; i++) {
                        rmlIP.CurrentVelocity->VecData[i] =	(double)DEG(0);
                        rmlIP.MaxVelocity->VecData[i]     = 0.1;
                        rmlIP.MaxAcceleration->VecData[i] =	0.5;
                        rmlIP.SelectionVector->VecData[i] =	true;
                    }
                    // TODO: Velocity for angle interpolation!
                    rmlIP.MaxVelocity->VecData[3]     = 20 * M_PI/180.0;
                    text_enter << "for point " << mCMDcounter;
                }

                // Control loop: Move robot along cartesion path
                int rmlResult = rml.GetNextMotionState_Position(rmlIP, &rmlOP);

                switch (rmlResult) {
                    case TypeIRML::RML_FINAL_STATE_REACHED:
                        if (mCMDcounter < (int)mCMDpath.size()-1) {
                            mCMDcounter++;
                            nextstate = TRAJ_CARTESIAN; // force re-entering
                        } else {
                            //notifyResult(SUCCESS);
                            nextstate = IDLE;
                        }
                        // continue to RML_WORKING (?)
                        break;
                    case TypeIRML::RML_WORKING: {
                        geometry_msgs::Pose pose_cur;
                        pose_cur.position.x = rmlOP.NewPosition->VecData[0];
                        pose_cur.position.y = rmlOP.NewPosition->VecData[1];
                        pose_cur.position.z = rmlOP.NewPosition->VecData[2];
                        double alpha = rmlOP.NewPosition->VecData[3] / rmlIP.TargetPosition->VecData[3];
                        // Slerp quaternion interpolation
                        Eigen::Quaterniond q_start, q_end, q_cur;
                        pose_kuka2ros(NULL, pose_start, &q_start, 2);
                        pose_kuka2ros(NULL, pose_end, &q_end, 2);
                        q_cur = q_start.slerp(alpha, q_end);
                        pose_cur.orientation.w = q_cur.w();
                        pose_cur.orientation.x = q_cur.x();
                        pose_cur.orientation.y = q_cur.y();
                        pose_cur.orientation.z = q_cur.z();

                        // Send pose to Kuka
                        float p_kuka[12];
                        pose_kuka2ros(p_kuka, pose_cur, NULL, 2);
                        ROS_INFO("TRAJ_CARTESIAN pose: T= %6.3f, %6.3f, %6.3f; Q= %6.3f, %6.3f, %6.3f",
                                 p_kuka[3], p_kuka[7], p_kuka[11], pose_cur.orientation.x, pose_cur.orientation.y, pose_cur.orientation.z);
                        mFRI->SetCommandedCartPose(p_kuka);
                        *(rmlIP.CurrentPosition) = *(rmlOP.NewPosition);
                        *(rmlIP.CurrentVelocity) = *(rmlOP.NewVelocity);
                        }
                        break;
                    default:
                        ROS_ERROR("FRIspin: ERROR during path generation (%d)", rmlResult);
                        //notifyResult(FAILED);
                        nextstate = IDLE;
                        break;
                }
                break;
                }
            default:
                nextstate = IDLE;
                break;

        }
        if (curstate != VELOCITY_CART) state_VELOCITY_CART_ok = false;
        mMutex.unlock();
        if (is_enter) {
            ROS_INFO_STREAM("FRIspin: Entered state " << curstate << " " << text_enter.str());
        }
    }  // big while-loop

    std::cout << "FRISpin: Stopping robot...\n";
    mFRI->StopRobot();

    if (mJR3_active) close(mJR3_fd);

    for (uint i = 0; i < ceil(0.5/t_cycle); i++) {
        // General robot communication
        if (mFRI->WaitForKRCTick(110000) != 0)
            continue;
    }
    std::cout << "FRISpin:  Done.\n";

    delete mFRI;
}



bool FRIClient::FRICheckRobot()
{
    if (mCMDmode != 0  && (mFRI->GetCurrentControlScheme() != mCMDmode || (!mFRI->IsMachineOK())))
    {
        float measured[12];
        mFRI->StopRobot();
        for (uint i=0; i<5; i++)
            mFRI->WaitForKRCTick(110000);
        if (mCMDmode != 0) {
            switch (mCMDmode) {
                case FastResearchInterface::CART_IMPEDANCE_CONTROL:
                    mFRI->GetMeasuredCartPose(measured);
                    mFRI->SetCommandedCartPose(measured);
                    ROS_INFO_STREAM("FRIClient: Restarting robot in CART_IMPEDANCE_CONTROL...");
                    break;
                case FastResearchInterface::JOINT_POSITION_CONTROL:
                    mFRI->GetMeasuredJointPositions(measured);
                    mFRI->SetCommandedJointPositions(measured);
                    ROS_INFO_STREAM("FRIClient: Restarting robot in JOINT_POSITION_CONTROL...");
                    break;
                default:
                    ROS_INFO_STREAM("FRIClient: Restarting robot...");
                    break;
            }

            int ResultValue	= mFRI->StartRobot(mCMDmode, 15);
            if ((ResultValue != 0) && (ResultValue != EALREADY)) {
                ROS_WARN_STREAM("FRIClient: Restarting Robot failed!");
                return false;
            }
            ROS_INFO_STREAM("FRIClient: Restarting Robot ok");
        }
        for (uint i=0; i<5; i++)
            mFRI->WaitForKRCTick(110000);
    }

    return true;
}

void FRIClient::notifyResult(RESULT r)
{
    mCMDresult = r;
    mCondResult.notify_all();
}

void FRIClient::cancelMotion()
{
    // TODO: slowly break robot!
    mMutex.lock();
    mNextState = BREAK;
    notifyResult(CANCELED);
    mMutex.unlock();
}


bool FRIClient::moveJointTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint>& jointtrajectory)
{
    mMutex.lock();
    mCMDjointtrajectory = jointtrajectory;
    mCMDresult = UNDEF;
    mCMDcounter = 0;
    mNextState = TRAJ_JOINT;
    mMutex.unlock();
    return true;
}

bool FRIClient::moveCartPath(const std::vector<geometry_msgs::PoseStamped>& path)
{
    mMutex.lock();
    mCMDpath = path;
    mCMDresult = UNDEF;
    mCMDcounter = 0;
    mNextState = TRAJ_CARTESIAN;
    mMutex.unlock();
    ROS_INFO("moveCartPose");
    return true;
}

bool FRIClient::moveCartVelocity(geometry_msgs::Pose velocity)
{
    mMutex.lock();
    mCMDvel_cart = velocity;
    mCMDresult = UNDEF;
    mCMDcounter = 0;
    mNextState = VELOCITY_CART;
    mMutex.unlock();
    ROS_INFO("moveCartVelocity");
    return true;
}

void FRIClient::setStoppingForce(geometry_msgs::Pose force, int type)
{
    mMutex.lock();
    mStopForce = force;
    //ROS_DEBUG("setStoppingForce: %.1f, %.1f, %.1f", force.position.x, force.position.y, force.position.z);
    mMutex.unlock();
}

bool FRIClient::JR3_open(std::string device)
{
    mJR3_active = false;
    if ((mJR3_fd = open(device.c_str(), O_RDWR)) < 0) {
        ROS_ERROR_STREAM("JR3_open: Can't open device " << device);
        return false;
    }

    int ret = ioctl(mJR3_fd, IOCTL0_JR3_GET_FULL_SCALES, &mJR3_fullscale);
    if (ret != -1) mJR3_active = true;
    ROS_INFO_STREAM("JR3_open: GET_FULL_SCALES, " << ((mJR3_active)?"":"not ") << "using JR3");
    return mJR3_active;
}

bool FRIClient::JR3_zeroforce()
{
    mMutex.lock();
    bool ret = ioctl(mJR3_fd, IOCTL0_JR3_ZEROOFFS) != -1;
    mMutex.unlock();
    return ret;
}

/** Read forces from KUKA or JR3. Beware of correct coordinate system!
 * Need to have mutex to call this.
 * \returns: 0:invalid, 1:base frame, 2:endeffector-frame
 */
int FRIClient::getForces(float *forces, bool use_jr3)
{
    if (!use_jr3) {
        mFRI->GetEstimatedExternalCartForcesAndTorques(forces);
        return 1;
    } else {
        six_axis_array force_jr3;
        int i;
        int ret = ioctl(mJR3_fd, IOCTL0_JR3_FILTER0, &force_jr3);
        if (ret == -1) return 0;
        for (i=0;i<3;i++) forces[i]   = (float)(force_jr3.f[i]*mJR3_fullscale.f[i])/16384.0;
        for (i=0;i<3;i++) forces[i+3] = (float)(force_jr3.m[i]*mJR3_fullscale.m[i])/16384.0;
        return 2;
    }
}


void FRIClient::pose_kuka2ros(float *p_kuka, geometry_msgs::Pose &p_ros, Eigen::Quaterniond *quaternion,  int in)
{
    switch (in) {
    case 1: {
        Eigen::Matrix3d rot;
        for (uint r=0; r<3; r++)
            for (uint c=0; c<3; c++)
                rot(r,c) = p_kuka[r*4+c];
        Eigen::Quaterniond quat(rot);
        if (quaternion != NULL) *quaternion = quat;

        p_ros.position.x = p_kuka[3];
        p_ros.position.y = p_kuka[7];
        p_ros.position.z = p_kuka[11];
        p_ros.orientation.w = quat.w();
        p_ros.orientation.x = quat.x();
        p_ros.orientation.y = quat.y();
        p_ros.orientation.z = quat.z();
        }
        break;
    case 2: {
        Eigen::Quaterniond quat(p_ros.orientation.w, p_ros.orientation.x, p_ros.orientation.y, p_ros.orientation.z);
        if (quaternion != NULL) *quaternion = quat;
        Eigen::Matrix3d rot = quat.toRotationMatrix();
        if (p_kuka != NULL) {
            for (uint r=0; r<3; r++)
                for (uint c=0; c<3; c++)
                    p_kuka[r*4+c] = rot(r,c);
            p_kuka[3] = p_ros.position.x;
            p_kuka[7] = p_ros.position.y;
            p_kuka[11] = p_ros.position.z;
        }
        }
        break;
    case 3: {
        Eigen::Matrix3d rot = quaternion->toRotationMatrix();
        if (p_kuka != NULL)
            for (uint r=0; r<3; r++)
                for (uint c=0; c<3; c++)
                    p_kuka[r*4+c] = rot(r,c);

        p_ros.orientation.w = quaternion->w();
        p_ros.orientation.x = quaternion->x();
        p_ros.orientation.y = quaternion->y();
        p_ros.orientation.z = quaternion->z();
        }
        break;
    default:
        ROS_ERROR_STREAM("pose_kuka2ros error");
    }
}






