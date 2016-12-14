/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef bimanual_catching_module_H_
#define bimanual_catching_module_H_

#include "RobotLib/RobotInterface.h"
#include "MathLib/IKGroupSolver.h"
#include "RobotLib/ForwardDynamics.h"
#include "RobotLib/InverseDynamics.h"
#include "RobotLib/KinematicChain.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

#include "sKinematics.h"
#include "CDDynamics.h"
#include "LWRRobot.h"

#include "bimanual_ds.h"

double cReady_left[]  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double cReady_right[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

enum ENUM_COMMAND {COMMAND_INITIAL, COMMAND_READY, COMMAND_CATCH, COMMAND_NONE};
enum ENUM_PLANNER {PLANNER_CARTESIAN, PLANNER_JOINT, PLANNER_NONE};
enum ENUM_Robot {Left, Right};
enum ENUM_AXIS {AXIS_X, AXIS_Y, AXIS_Z};
enum Command {Com_NONE, Com_INIT, Com_Throw};


class bimanual_catching_module : public RobotInterface
{
public:
            bimanual_catching_module();
    virtual ~bimanual_catching_module();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);

protected:

    void                        parameterInitialization();
    void                        topicInitialization();

    void                        initKinematics();
    void                        initKinematics_left();
    void                        initKinematics_right();

    void						solveInverseKinematics();

    void						sendPositionToRobot(ENUM_Robot Robot, Vector Position);
    void 						sendPredictionCommand(int _command);

    double						dt;

    int 						ROBOT_DOF;
    int							IK_CONSTRAINTS;
    int                         mEndEffectorId;


    Vector 						RPos_object;
    Vector 						DRPos_object;
    Vector 						DDRPos_object;

    Vector                      RPos_attractor;
    Vector                      DRPos_attractor;
    Vector                      DDRPos_attractor;

    Vector                      RPos_attractor_prev;

    Vector 						RPos_End_left;
    Vector 						DRPos_End_left;
    Vector 						DDRPos_End_left;

    Vector 						PosDesired_End_left;
    Vector						DPosDesired_End_left;
    Vector						DDPosDesired_End_left;

    Vector 						RPos_End_right;
    Vector 						DRPos_End_right;
    Vector 						DDRPos_End_right;

    Vector 						PosDesired_End_right;
    Vector 						DPosDesired_End_right;
    Vector 						DDPosDesired_End_right;

    Vector 						Position_VO;

    // joint
    Vector 						TJointPos_left;
    Vector						RJointPos_left;
    Vector						RJointPos_left_Error;

    Vector 						JointTargetPos_left;
    Vector 						JointFinalPos_left;
    Vector 						JointDesVel_left;

    Vector                      mJointVelLimitsUp_left;
    Vector                      mJointVelLimitsDn_left;

    Vector 						TJointPos_right;
    Vector						RJointPos_right;
    Vector						RJointPos_right_Error;

    Vector 						JointTargetPos_right;
    Vector 						JointFinalPos_right;
    Vector 						JointDesVel_right;

    Vector                      mJointVelLimitsUp_right;
    Vector                      mJointVelLimitsDn_right;

    Vector                      limLow_left;
    Vector                      limHigh_left;
    Vector                      limLow_right;
    Vector                      limHigh_right;



    Matrix						mJacobian3_left;
    Matrix						mJacobian9_left;

    Matrix						mJacobian3_right;
    Matrix						mJacobian9_right;

    Matrix						lJacobianDirY_left;
    Matrix						lJacobianDirZ_left;
    Matrix						lJacobianDirY_right;
    Matrix						lJacobianDirZ_right;

    Vector						lDirX_left;
    Vector						lDirY_left;
    Vector						lDirZ_left;

    Vector						lDirX_right;
    Vector						lDirY_right;
    Vector						lDirZ_right;

    Vector						lTargetDirY_left;
    Vector						lTargetDirZ_left;

    Vector						lTargetDirY_right;
    Vector						lTargetDirZ_right;


    Vector						lJoints_left;
    Vector						lJoints_right;

    Vector						mJointDesVel_left;
    Vector						mJointDesVel_right;

    Vector						mTargetVelocity_left;
    Vector						mTargetVelocity_right;


    Vector 						lJointWeight;

    IKGroupSolver               mIKSolver_left;
    IKGroupSolver               mIKSolver_right;

    KinematicChain              mKinematicChain;
    RevoluteJointSensorGroup    mSensorsGroup;
    RevoluteJointActuatorGroup  mActuatorsGroup;


    sKinematics                 *mSKinematicChain_left;
    sKinematics                 *mSKinematicChain_right;

    CDDynamics					*CDJointPlanner_left;
    CDDynamics					*CDJointPlanner_right;

    bimanual_ds					*force_dynamical_system;

    Vector                      d_L_d;
    Vector                      d_R_d;



    ENUM_COMMAND 				mCommand;
    ENUM_PLANNER 				mPlanner;

    int                         cnt_saving;
    double                      ttc;

    ofstream                    data_save;


    ros::Subscriber             sub_robot_left_jointPos_coman;
    ros::Subscriber             sub_robot_right_jointPos_coman;
    ros::Subscriber				sub_object_pos;
    ros::Subscriber				sub_object_vel;
    ros::Subscriber				sub_attractor_pos;
    ros::Subscriber				sub_attractor_vel;
    ros::Subscriber				sub_ttc;

    ros::Publisher 				pub_command;
    ros::Publisher  			pub_command_pos_left_coman;
    ros::Publisher 				pub_command_pos_right_coman;

    ros::Publisher  			pub_pos_END_left;
    ros::Publisher  			pub_pos_END_right;
    ros::Publisher              pub_pos_desired_END_left;
    ros::Publisher              pub_pos_desired_END_right;
    ros::Publisher              pub_virtual_object;

    geometry_msgs::Pose			Position_desired_end_left;
    geometry_msgs::Pose			Position_desired_end_right;

    geometry_msgs::Pose			Pos_Left_End;
    geometry_msgs::Pose			Pos_Right_End;

    geometry_msgs::Pose			Virtial_object;

    void						chatterCallback_robot_left_position(const sensor_msgs::JointState & msg);
    void						chatterCallback_robot_right_position(const sensor_msgs::JointState & msg);
    void 						chatterCallback_object_postion(const geometry_msgs::Pose & msg);
    void 						chatterCallback_object_velocity(const geometry_msgs::Pose & msg);
    void 						chatterCallback_attractor_postion(const geometry_msgs::Pose & msg);
    void 						chatterCallback_attractor_velocity(const geometry_msgs::Pose & msg);
    void 						chatterCallback_ttc(const std_msgs::String::ConstPtr& msg);

};


#endif 
