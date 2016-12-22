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

#include "bimanual_catching_module.h"


bimanual_catching_module::bimanual_catching_module()
:RobotInterface(){
}
bimanual_catching_module::~bimanual_catching_module(){
}



void bimanual_catching_module::chatterCallback_robot_left_position(const sensor_msgs::JointState & msg) {

    for(int i=0;i<7;i++)
        TJointPos_left(i) = msg.position[i];
}


void bimanual_catching_module::chatterCallback_robot_right_position(const sensor_msgs::JointState & msg) {

    for(int i=0;i<7;i++)
        TJointPos_right(i)=msg.position[i];
}


void bimanual_catching_module::chatterCallback_object_postion(const geometry_msgs::Pose & msg) {

    if (msg.position.x!=0)
    {
        double pos[3];

        pos[0] = msg.position.x;
        pos[1] = msg.position.y;
        pos[2] = msg.position.z;

        RPos_object.Set(pos, 3);
    }
}


void bimanual_catching_module::chatterCallback_object_velocity(const geometry_msgs::Pose & msg) {

    if (msg.position.x!=0)
    {
        double vel[3];

        vel[0] = msg.position.x;
        vel[1] = msg.position.y;
        vel[2] = msg.position.z;

        DRPos_object.Set(vel, 3);
    }
}


void bimanual_catching_module::chatterCallback_attractor_postion(const geometry_msgs::Pose & msg) {

    if (msg.position.x!=0)
    {
        double pos[3];

        if (msg.position.x > 0.3) {
            pos[0] = msg.position.x;
            pos[1] = msg.position.y;
            pos[2] = msg.position.z;

            RPos_attractor.Set(pos, 3);
            RPos_attractor_prev.Set(pos, 3);
        }
        else {
            RPos_attractor = RPos_attractor_prev;
        }
    }

}


void bimanual_catching_module::chatterCallback_attractor_velocity(const geometry_msgs::Pose & msg) {

    if (msg.position.x!=0)
    {
        double vel[3];

        if (msg.position.x > 0.3) {
            vel[0]=msg.position.x;
            vel[1]=msg.position.y;
            vel[2]=msg.position.z;
        }
        else {
            vel[0] = 0.0;
            vel[1] = 0.0;
            vel[2] = 0.0;
        }

        DRPos_attractor.Set(vel, 3);
    }

}


void bimanual_catching_module::chatterCallback_ttc(const std_msgs::String::ConstPtr& msg) {

    double ttc_in;
    sscanf(msg->data.c_str(), "%lf", &ttc_in);

    ttc = ttc_in;

}


void bimanual_catching_module::sendPredictionCommand(int _command)
{
    std_msgs::String msg;
    std::stringstream ss;
    ss << _command;

    cout << ss.str() << endl;
    msg.data = ss.str();
    pub_command.publish(msg);
}



void bimanual_catching_module::solveInverseKinematics() {

//    double lDirWeight = 1.0;
    double lVelMux = 1.0; //3.0;

    for(int ikframe=0; ikframe<1.0; ikframe++ ) {

        mSKinematicChain_left->getEndDirAxis(AXIS_X, lDirX_left.Array());
        mSKinematicChain_left->getEndDirAxis(AXIS_Y, lDirY_left.Array());
        mSKinematicChain_left->getEndDirAxis(AXIS_Z, lDirZ_left.Array());
        mSKinematicChain_left->getJoints(lJoints_left.Array());

        mSKinematicChain_right->getEndDirAxis(AXIS_X, lDirX_right.Array());
        mSKinematicChain_right->getEndDirAxis(AXIS_Y, lDirY_right.Array());
        mSKinematicChain_right->getEndDirAxis(AXIS_Z, lDirZ_right.Array());
        mSKinematicChain_right->getJoints(lJoints_right.Array());

        // set Jacobian
        mSKinematicChain_left->getJacobianPos(mJacobian3_left);
        mSKinematicChain_left->getJacobianDirection(AXIS_Y, lJacobianDirY_left);
        mSKinematicChain_left->getJacobianDirection(AXIS_Z, lJacobianDirZ_left);

        mSKinematicChain_right->getJacobianPos(mJacobian3_right);
        mSKinematicChain_right->getJacobianDirection(AXIS_Y, lJacobianDirY_right);
        mSKinematicChain_right->getJacobianDirection(AXIS_Z, lJacobianDirZ_right);


        // dealing with joint limits
        limLow_left.Resize(ROBOT_DOF);    limHigh_left.Resize(ROBOT_DOF);

        limLow_left(0)=-3.4034;     limHigh_left(0)=1.6581;
        limLow_left(1)=-0.31415;    limHigh_left(1)=2.094;
        limLow_left(2)=-1.5708;     limHigh_left(2)=1.5708;
        limLow_left(3)=-2.3562;     limHigh_left(3)=0.0;
        limLow_left(4)=-1.5708;     limHigh_left(4)=1.5708;
        limLow_left(5)=-0.524;      limHigh_left(5)=0.524;
        limLow_left(6)=-0.785375;   limHigh_left(6)=1.395;

        limLow_right.Resize(ROBOT_DOF);    limHigh_right.Resize(ROBOT_DOF);

        limLow_right(0)=-3.4034;     limHigh_right(0)=1.6581;
        limLow_right(1)=-2.094;      limHigh_right(1)=0.31415;
        limLow_right(2)=-1.5708;     limHigh_right(2)=1.5708;
        limLow_right(3)=-2.3562;     limHigh_right(3)=0.0;
        limLow_right(4)=-1.5708;     limHigh_right(4)=1.5708;
        limLow_right(5)=-0.524;      limHigh_right(5)=0.524;
        limLow_right(6)=-1.395;      limHigh_right(6)=0.785375;


        for(int i=0; i<ROBOT_DOF; i++) {
            // default: max
            mJointVelLimitsDn_left(i) = -mSKinematicChain_left->getMaxVel(i)*lVelMux;
            mJointVelLimitsUp_left(i) =  mSKinematicChain_left->getMaxVel(i)*lVelMux;

            mJointVelLimitsDn_right(i) = -mSKinematicChain_right->getMaxVel(i)*lVelMux;
            mJointVelLimitsUp_right(i) =  mSKinematicChain_right->getMaxVel(i)*lVelMux;

            // check for limits
//            double deltaLow_left = TJointPos_left(i) - limLow_left(i);       double deltaHigh_left = limHigh_left(i) - TJointPos_left(i);
//            double deltaLow_right = TJointPos_right(i) - limLow_right(i);     double deltaHigh_right = limHigh_right(i) - TJointPos_right(i);

//            if (deltaLow_left < 0.0)                    mJointVelLimitsDn_left *= 0.0;
//            else if (deltaLow_left < DEG2RAD(5.0))      mJointVelLimitsDn_left *= deltaLow_left /DEG2RAD(5.0);
//            if (deltaHigh_left < 0.0)                   mJointVelLimitsUp_left *= 0.0;
//            else if (deltaHigh_left < DEG2RAD(5.0))     mJointVelLimitsUp_left *= deltaHigh_left /DEG2RAD(5.0);

//            if (deltaLow_right < 0.0)                   mJointVelLimitsDn_right *= 0.0;
//            else if (deltaLow_right < DEG2RAD(5.0))     mJointVelLimitsDn_right *= deltaLow_right /DEG2RAD(5.0);
//            if (deltaHigh_right < 0.0)                  mJointVelLimitsUp_right *= 0.0;
//            else if (deltaHigh_right < DEG2RAD(5.0))    mJointVelLimitsUp_right *= deltaHigh_right /DEG2RAD(5.0);
        }

        mIKSolver_left.SetLimits(mJointVelLimitsDn_left, mJointVelLimitsUp_left);
        mIKSolver_right.SetLimits(mJointVelLimitsDn_right, mJointVelLimitsUp_right);


        for(int i=0; i<3; i++) {
            mJacobian9_left.SetRow(mJacobian3_left.GetRow(i)   , i  );
            mJacobian9_left.SetRow(lJacobianDirY_left.GetRow(i), i+3);
            mJacobian9_left.SetRow(lJacobianDirZ_left.GetRow(i), i+6);

            mJacobian9_right.SetRow(mJacobian3_right.GetRow(i)   , i  );
            mJacobian9_right.SetRow(lJacobianDirY_right.GetRow(i), i+3);
            mJacobian9_right.SetRow(lJacobianDirZ_right.GetRow(i), i+6);
        }

        // set desired target direction (to be changed)
        lTargetDirY_left.Zero();
        lTargetDirZ_left.Zero();
//        lTargetDirY_left(2) = 1;
//        lTargetDirZ_left(0) = -1;

        lTargetDirY_right.Zero();
        lTargetDirZ_right.Zero();
//        lTargetDirY_right(2) = 1;
//        lTargetDirZ_right(0) = -1;

        // left
        mTargetVelocity_left.SetSubVector(0, (PosDesired_End_left-RPos_End_left)*lVelMux / dt );
        mTargetVelocity_left.SetSubVector(3, (lTargetDirY_left-lDirY_left) / dt );
        mTargetVelocity_left.SetSubVector(6, (lTargetDirZ_left-lDirZ_left) / dt );

        mIKSolver_left.SetJacobian(mJacobian9_left);
        mIKSolver_left.SetTarget(mTargetVelocity_left, 0);

        mIKSolver_left.Solve();
        mJointDesVel_left = mIKSolver_left.GetOutput();    // ??

        JointTargetPos_left = lJoints_left + mJointDesVel_left*dt;

        // right
        mTargetVelocity_right.SetSubVector(0, (PosDesired_End_right-RPos_End_right)*lVelMux / dt );
        mTargetVelocity_right.SetSubVector(3, (lTargetDirY_right-lDirY_right) / dt );
        mTargetVelocity_right.SetSubVector(6, (lTargetDirZ_right-lDirZ_right) / dt );

        mIKSolver_right.SetJacobian(mJacobian9_right);
        mIKSolver_right.SetTarget(mTargetVelocity_right, 0);

        mIKSolver_right.Solve();
        mJointDesVel_right = mIKSolver_right.GetOutput();    // ??

        JointTargetPos_right = lJoints_right + mJointDesVel_right*dt;


        // set joint angles
        mSKinematicChain_left->setJoints(JointTargetPos_left.Array());
        mSKinematicChain_left->getEndPos(RPos_End_left.Array());

        mSKinematicChain_right->setJoints(JointTargetPos_right.Array());
        mSKinematicChain_right->getEndPos(RPos_End_right.Array());

    }

}



void bimanual_catching_module::sendPositionToRobot(ENUM_Robot Robot, Vector Position) {

    std_msgs::Float64MultiArray msg;

    msg.data.resize(ROBOT_DOF);

    for (int i=0; i<ROBOT_DOF;i=i+1)
        msg.data[i]  = Position(i);

    switch(Robot){
    case Left :
        pub_command_pos_left_coman.publish(msg);
        break;
    case Right :
        pub_command_pos_right_coman.publish(msg);
        break;
    }
}


void bimanual_catching_module::parameterInitialization() {
    // initilize parameters

    ROBOT_DOF = 7;
    dt = 0.002;
    IK_CONSTRAINTS = 9;

    RPos_object.Resize(3);
    DRPos_object.Resize(3);
    DDRPos_object.Resize(3);

    RPos_object.Zero();
    DRPos_object.Zero();
    DDRPos_object.Zero();

    RPos_attractor.Resize(3);
    DRPos_attractor.Resize(3);
    DDRPos_attractor.Resize(3);

    RPos_attractor_prev.Resize(3);

    RPos_attractor.Zero();
    DRPos_attractor.Zero();
    DDRPos_attractor.Zero();

    RPos_attractor(0)=0.3;         RPos_attractor(1)=0.0;         RPos_attractor(2)=0.0;
    RPos_attractor_prev(0)=0.3;    RPos_attractor_prev(1)=0.0;    RPos_attractor_prev(2)=0.0;

    PosDesired_End_left.Resize(3);
    DPosDesired_End_left.Resize(3);
    DDPosDesired_End_left.Resize(3);

    PosDesired_End_right.Resize(3);
    DPosDesired_End_right.Resize(3);
    DDPosDesired_End_right.Resize(3);



    // for inverse kinematics
    lDirX_left.Resize(3);
    lDirY_left.Resize(3);
    lDirZ_left.Resize(3);
    lJoints_left.Resize(ROBOT_DOF);

    lDirX_right.Resize(3);
    lDirY_right.Resize(3);
    lDirZ_right.Resize(3);
    lJoints_right.Resize(ROBOT_DOF);

    lTargetDirY_left.Resize(3);
    lTargetDirZ_left.Resize(3);

    lTargetDirY_right.Resize(3);
    lTargetDirZ_right.Resize(3);

    mJointDesVel_left.Resize(ROBOT_DOF);
    mJointDesVel_right.Resize(ROBOT_DOF);

    d_L_d.Resize(3);
    d_R_d.Resize(3);


    CDJointPlanner_left = new CDDynamics( ROBOT_DOF, dt, 0.5 );
    CDJointPlanner_right = new CDDynamics( ROBOT_DOF, dt, 0.5 );

    Vector lAccelLimits(ROBOT_DOF);
    lAccelLimits.One();
    lAccelLimits *= 35.0;

    CDJointPlanner_left->SetAccelLimits(lAccelLimits);
    CDJointPlanner_right->SetAccelLimits(lAccelLimits);

    force_dynamical_system = new bimanual_ds();

    force_dynamical_system->initialize(dt, 50.0, 500.0, 20.0, 20.0, 40.0, 400.0);     //



    // desired hand-object distance
    // ball radius 0.20m
    d_L_d(0) = -0.2;     d_L_d(1) = 0.2;     d_L_d(2) = -0.1;
    d_R_d(0) = -0.2;     d_R_d(1) = -0.2;     d_R_d(2) = -0.1;

    // ball radius 0.15m
//    d_L_d(0) = -0.2;     d_L_d(1) =  0.15;     d_L_d(2) = -0.1;
//    d_R_d(0) = -0.2;     d_R_d(1) = -0.15;     d_R_d(2) = -0.1;

    // ball radius 0.10m
//    d_L_d(0) = -0.2;     d_L_d(1) =  0.03;     d_L_d(2) = -0.1;
//    d_R_d(0) = -0.2;     d_R_d(1) = -0.03;     d_R_d(2) = -0.1;

    // ball radius 0.30m
//    d_L_d(0) = -0.2;     d_L_d(1) =  0.4;     d_L_d(2) = -0.1;
//    d_R_d(0) = -0.2;     d_R_d(1) = -0.4;     d_R_d(2) = -0.1;


    mCommand = COMMAND_NONE;
    mPlanner = PLANNER_NONE;

}


void bimanual_catching_module::initKinematics() {

    lJointWeight.Resize(ROBOT_DOF);

    lJointWeight(0) = 0.5678;
    lJointWeight(1) = 0.7768;
    lJointWeight(2) = 1.0462;
    lJointWeight(3) = 0.6331;
    lJointWeight(4) = 0.9335;
    lJointWeight(5) = 0.2607;
    lJointWeight(6) = 0.0540;


    mEndEffectorId = mRobot->GetLinksCount() - 1;
    mKinematicChain.SetRobot(mRobot);
    mKinematicChain.Create(0, 0, mEndEffectorId);

    // initialize sensor group
    mSensorsGroup.SetSensorsList(mRobot->GetSensors());
    mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());
    mSensorsGroup.ReadSensors();

}


void bimanual_catching_module::initKinematics_left() {

    TJointPos_left.Resize(ROBOT_DOF);
    RJointPos_left.Resize(ROBOT_DOF);
    RJointPos_left_Error.Resize(ROBOT_DOF);

    JointTargetPos_left.Resize(ROBOT_DOF);
    JointTargetPos_left.Zero();
    JointFinalPos_left.Resize(ROBOT_DOF);

    JointDesVel_left.Resize(ROBOT_DOF);
    mJointVelLimitsUp_left.Resize(ROBOT_DOF);
    mJointVelLimitsDn_left.Resize(ROBOT_DOF);

    RPos_End_left.Resize(3);
    DRPos_End_left.Resize(3);
    DDRPos_End_left.Resize(3);

    mSKinematicChain_left = new sKinematics(ROBOT_DOF, dt);


    // Coman                      i     a        d               alpha      theta0      1   min         max         maxvel
    mSKinematicChain_left->setDH(0,    0.0,     0.0733,        -M_PI_2,   -M_PI_2,     1,  -3.4034,    1.6581,     4.0);   // LShSag
    mSKinematicChain_left->setDH(1,    0.0,     0.0,           -M_PI_2,   -M_PI_2,     1,  -0.31415,   2.094,      4.0);   // LShLat
    mSKinematicChain_left->setDH(2,    0.015,  -0.0448-0.1352, -M_PI_2,   -M_PI_2,     1,  -1.5708,    1.5708,     4.0);   // LShYaw
    mSKinematicChain_left->setDH(3,   -0.015,   0.0,            M_PI_2,    0.0,        1,  -2.3562,    0.0,        4.0);   // LElbj
    mSKinematicChain_left->setDH(4,    0.0,    -0.1078-0.0869, -M_PI_2,    0.0,        1,  -1.5708,    1.5708,     4.0);   // LForearmPlate
    mSKinematicChain_left->setDH(5,    0.0,     0.0,           -M_PI_2,   -M_PI_2,     1,  -0.524,     0.524,      4.0);   // LWrj1
    mSKinematicChain_left->setDH(6,   -0.05,    0.0,            0.0,       0.0,        1,  -0.785375,  1.395,      4.0);   // LWrj2


    double T0[4][4];

    T0[0][0] = 1;   T0[0][1] = 0;   T0[0][2] = 0;   T0[0][3] = 0.04;
    T0[1][0] = 0;   T0[1][1] = 0;   T0[1][2] = 1;   T0[1][3] = 0.082;
    T0[2][0] = 0;   T0[2][1] = -1;  T0[2][2] = 0;   T0[2][3] = 0.324;
    T0[3][0] = 0;   T0[3][1] = 0;   T0[3][2] = 0;   T0[3][3] = 1;

    mSKinematicChain_left->setT0(T0);

    MathLib::Matrix3 mTF;
    double TF[4][4];
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            TF[i][j] = 0.0;
    TF[3][3] = 1;

    mTF = Matrix3::SRotationY(0.0);

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            TF[i][j] = mTF(i,j);

    mSKinematicChain_left->readyForKinematics();
    mSKinematicChain_left->setTF(TF);

    Vector lAccelLimits(ROBOT_DOF);
    lAccelLimits.One();
    lAccelLimits *= 35.0;

    Vector lMaxVel(ROBOT_DOF);
    mSKinematicChain_left->getMaxVel(lMaxVel.Array());

    CDJointPlanner_left->SetVelocityLimits(lMaxVel);

    // variable for ik
    mJacobian3_left.Resize(3,ROBOT_DOF);
    lJacobianDirY_left.Resize(3,ROBOT_DOF);
    lJacobianDirZ_left.Resize(3,ROBOT_DOF);
    mJacobian9_left.Resize(9,ROBOT_DOF);

    //Inverse Kinematic
    mIKSolver_left.SetSizes(ROBOT_DOF);  // Dof counts
    mIKSolver_left.AddSolverItem(IK_CONSTRAINTS);
    mIKSolver_left.SetVerbose(false);                // No comments
    mIKSolver_left.SetThresholds(0.00001,0.000001);    // Singularities thresholds
    mIKSolver_left.Enable(true,0);                   // Enable first solver
    mIKSolver_left.SetDofsIndices(mKinematicChain.GetJointMapping(),0); // Joint maps for first solver
    mIKSolver_left.SetDofsWeights(lJointWeight);
    mTargetVelocity_left.Resize(IK_CONSTRAINTS);

}


void bimanual_catching_module::initKinematics_right() {

    TJointPos_right.Resize(ROBOT_DOF);
    RJointPos_right.Resize(ROBOT_DOF);
    RJointPos_right_Error.Resize(ROBOT_DOF);

    JointTargetPos_right.Resize(ROBOT_DOF);
    JointTargetPos_right.Zero();
    JointFinalPos_right.Resize(ROBOT_DOF);

    JointDesVel_right.Resize(ROBOT_DOF);
    mJointVelLimitsUp_right.Resize(ROBOT_DOF);
    mJointVelLimitsDn_right.Resize(ROBOT_DOF);

    RPos_End_right.Resize(3);
    DRPos_End_right.Resize(3);
    DDRPos_End_right.Resize(3);

    mSKinematicChain_right = new sKinematics(ROBOT_DOF, dt);


    // Coman                      i     a        d               alpha      theta0      1   min         max         maxvel
    mSKinematicChain_right->setDH(0,    0.0,    -0.0733,        -M_PI_2,   -M_PI_2,     1,  -3.4034,    1.6581,     4.0);   // RShSag
    mSKinematicChain_right->setDH(1,    0.0,     0.0,           -M_PI_2,   -M_PI_2,     1,  -2.094,     0.31415,    4.0);   // RShLat
    mSKinematicChain_right->setDH(2,    0.015,  -0.0448-0.1352, -M_PI_2,   -M_PI_2,     1,  -1.5708,    1.5708,     4.0);   // RShYaw
    mSKinematicChain_right->setDH(3,   -0.015,   0.0,            M_PI_2,    0.0,        1,  -2.3562,    0.0,        4.0);   // RElbj
    mSKinematicChain_right->setDH(4,    0.0,    -0.1078-0.0869, -M_PI_2,    0.0,        1,  -1.5708,    1.5708,     4.0);   // RForearmPlate
    mSKinematicChain_right->setDH(5,    0.0,     0.0,           -M_PI_2,   -M_PI_2,     1,  -0.524,     0.524,      4.0);   // RWrj1
    mSKinematicChain_right->setDH(6,   -0.05,    0.0,            0.0,       0.0,        1,  -1.395,     0.785375,   4.0);   // RWrj2


    double T0[4][4];

    T0[0][0] = 1;   T0[0][1] = 0;   T0[0][2] = 0;   T0[0][3] = 0.04;
    T0[1][0] = 0;   T0[1][1] = 0;   T0[1][2] = 1;   T0[1][3] = -0.082; // -0.2194;
    T0[2][0] = 0;   T0[2][1] = -1;  T0[2][2] = 0;   T0[2][3] = 0.324;
    T0[3][0] = 0;   T0[3][1] = 0;   T0[3][2] = 0;   T0[3][3] = 1;

    mSKinematicChain_right->setT0(T0);

    MathLib::Matrix3 mTF;
    double TF[4][4];
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            TF[i][j] = 0.0;
    TF[3][3] = 1;

    mTF = Matrix3::SRotationY(0.0);

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            TF[i][j] = mTF(i,j);

    mSKinematicChain_right->readyForKinematics();
    mSKinematicChain_right->setTF(TF);

    Vector lAccelLimits(ROBOT_DOF);
    lAccelLimits.One();
    lAccelLimits *= 35.0;

    Vector lMaxVel(ROBOT_DOF);
    mSKinematicChain_right->getMaxVel(lMaxVel.Array());

    CDJointPlanner_right->SetVelocityLimits(lMaxVel);

    // variable for ik
    mJacobian3_right.Resize(3,ROBOT_DOF);
    lJacobianDirY_right.Resize(3,ROBOT_DOF);
    lJacobianDirZ_right.Resize(3,ROBOT_DOF);
    mJacobian9_right.Resize(9,ROBOT_DOF);

    //Inverse Kinematic
    mIKSolver_right.SetSizes(ROBOT_DOF);  // Dof counts
    mIKSolver_right.AddSolverItem(IK_CONSTRAINTS);
    mIKSolver_right.SetVerbose(false);                // No comments
    mIKSolver_right.SetThresholds(0.00001,0.000001);    // Singularities thresholds
    mIKSolver_right.Enable(true,0);                   // Enable first solver
    mIKSolver_right.SetDofsIndices(mKinematicChain.GetJointMapping(),0); // Joint maps for first solver
    mIKSolver_right.SetDofsWeights(lJointWeight);
    mTargetVelocity_right.Resize(IK_CONSTRAINTS);

}


void bimanual_catching_module::topicInitialization() {
    // initialize topics

    mRobot->SetControlMode(Robot::CTRLMODE_POSITION);
    ros::NodeHandle *n = mRobot->InitializeROS();

    // robot joint pos feedback (from gazebo)
    sub_robot_left_jointPos_coman = n->subscribe("/Coman/Left/in", 3, & bimanual_catching_module::chatterCallback_robot_left_position, this);
    sub_robot_right_jointPos_coman = n->subscribe("/Coman/Right/in", 3, & bimanual_catching_module::chatterCallback_robot_right_position, this);

    // object & attractor status from ball_motion_estimation
    sub_object_pos = n->subscribe("/catch/objpos", 3, & bimanual_catching_module::chatterCallback_object_postion, this);
    sub_object_vel = n->subscribe("/catch/objvel", 3, & bimanual_catching_module::chatterCallback_object_velocity, this);

    sub_attractor_pos = n->subscribe("/catch/attpos", 3, & bimanual_catching_module::chatterCallback_attractor_postion, this);
    sub_attractor_vel = n->subscribe("/catch/attvel", 3, & bimanual_catching_module::chatterCallback_attractor_velocity, this);

    sub_ttc = n->subscribe("/ttc", 3, & bimanual_catching_module::chatterCallback_ttc, this);


    pub_command = n->advertise<std_msgs::String>("/catch/command", 3);

    pub_command_pos_left_coman = n->advertise<std_msgs::Float64MultiArray>("/Coman/Left/in", 3);
    pub_command_pos_right_coman = n->advertise<std_msgs::Float64MultiArray>("/Coman/Right/in", 3);


    pub_pos_END_left = n->advertise<geometry_msgs::Pose>("/l_arm_END_pos", 3);
    pub_pos_END_right = n->advertise<geometry_msgs::Pose>("/r_arm_END_pos", 3);

    pub_pos_desired_END_left = n->advertise<geometry_msgs::Pose>("/desired_end_left", 3);
    pub_pos_desired_END_right = n->advertise<geometry_msgs::Pose>("/desired_end_right", 3);

    pub_virtual_object = n->advertise<geometry_msgs::Pose>("/object_virtual/position", 3);

    sendPredictionCommand(Com_NONE);
}






RobotInterface::Status bimanual_catching_module::RobotInit(){

    parameterInitialization();
    topicInitialization();

    initKinematics();
    initKinematics_left();
    initKinematics_right();

    AddConsoleCommand("init");
    AddConsoleCommand("ready");
    AddConsoleCommand("catch");

    cout<<"Initialization done. init/ready/catch"<<endl;
    cnt_saving = 0;

    return STATUS_OK;
}


RobotInterface::Status bimanual_catching_module::RobotFree(){
    return STATUS_OK;
}


RobotInterface::Status bimanual_catching_module::RobotStart(){

    ros::spinOnce();
    mSensorsGroup.ReadSensors();

    RJointPos_left = TJointPos_left;
    RJointPos_right = TJointPos_right;
    JointTargetPos_left = RJointPos_left;
    JointTargetPos_right = RJointPos_right;

    cout<<"Robot started."<<endl;

    return STATUS_OK;
}


RobotInterface::Status bimanual_catching_module::RobotStop(){
    return STATUS_OK;
}


RobotInterface::Status bimanual_catching_module::RobotUpdate(){

    ros::spinOnce();

    switch (mCommand) {

    case COMMAND_INITIAL:

        parameterInitialization();
        cnt_saving = cnt_saving + 1;

        DRPos_End_left.Zero();
        DDRPos_End_left.Zero();
        DRPos_End_right.Zero();
        DDRPos_End_right.Zero();

        mSKinematicChain_left->getEndPos(RPos_End_left.Array());
        mSKinematicChain_right->getEndPos(RPos_End_right.Array());

        force_dynamical_system->Set_Left_robot_state(RPos_End_left, DRPos_End_left, DDRPos_End_left);
        force_dynamical_system->Set_Right_robot_state(RPos_End_right, DRPos_End_right, DDRPos_End_right);

        cout<<"Dynamical system is initialized."<<endl;
        cout<<"left end pos :  "<<RPos_End_left(0)<<" "<<RPos_End_left(1)<<" "<<RPos_End_left(2)<<endl;
        cout<<"right end pos : "<<RPos_End_right(0)<<" "<<RPos_End_right(1)<<" "<<RPos_End_right(2)<<endl;


        data_save.close();

        char OutTEXTfile[256];

        sprintf(OutTEXTfile, "data_%d.txt", cnt_saving);
        data_save.open(OutTEXTfile);


        mPlanner = PLANNER_NONE;
        mCommand=COMMAND_NONE;

        sendPredictionCommand(Com_INIT);

        break;

    case COMMAND_READY:

        parameterInitialization();

        for (int i=0; i<7; i++){
            JointFinalPos_left(i) = cReady_left[i];
            JointFinalPos_right(i) = cReady_right[i];
        }

        RJointPos_left = TJointPos_left;
        RJointPos_right = TJointPos_right;

        CDJointPlanner_left->SetState(RJointPos_left);
        CDJointPlanner_right->SetState(RJointPos_right);

        CDJointPlanner_left->SetTarget(JointFinalPos_left);
        CDJointPlanner_right->SetTarget(JointFinalPos_right);


        mPlanner = PLANNER_JOINT;
        mCommand = COMMAND_NONE;

        sendPredictionCommand(Com_INIT);

        break;

    case COMMAND_CATCH:

        force_dynamical_system->Set_object_state(RPos_attractor, DRPos_attractor, DDRPos_attractor);

        cout<<"obj pos sent: "<<RPos_object(0)<<"  "<<RPos_object(1)<<"  "<<RPos_object(2)<<endl;

        force_dynamical_system->Set_Left_robot_state(RPos_End_left, DRPos_End_left, DDRPos_End_left);
        force_dynamical_system->Set_Right_robot_state(RPos_End_right, DRPos_End_right, DDRPos_End_right);

        force_dynamical_system->initialize_Virrtual_object(d_L_d, d_R_d);

        cout<<"Virtual object initialized."<<endl;

        mCommand = COMMAND_NONE;
        mPlanner = PLANNER_CARTESIAN;

        sendPredictionCommand(Com_Throw);

        break;
    }


    switch (mPlanner) {

    case PLANNER_JOINT:

        CDJointPlanner_left->Update();
        CDJointPlanner_right->Update();
        CDJointPlanner_left->GetState(JointTargetPos_left);
        CDJointPlanner_right->GetState(JointTargetPos_right);

        break;

    case PLANNER_CARTESIAN:

        mSKinematicChain_left->setJoints(RJointPos_left.Array());
        mSKinematicChain_right->setJoints(RJointPos_right.Array());

        mSKinematicChain_right->getEndPos(RPos_End_right.Array());
        mSKinematicChain_left->getEndPos(RPos_End_left.Array());

        force_dynamical_system->Set_object_state(RPos_attractor, DRPos_attractor, DDRPos_attractor);

        force_dynamical_system->Set_Left_robot_state(RPos_End_left, DRPos_End_left, DDRPos_End_left);
        force_dynamical_system->Set_Right_robot_state(RPos_End_right, DRPos_End_right, DDRPos_End_right);

        force_dynamical_system->Set_TTC(ttc);
        force_dynamical_system->Update();

        force_dynamical_system->Get_Left_robot_state(PosDesired_End_left, DPosDesired_End_left, DDPosDesired_End_left);
        force_dynamical_system->Get_Right_robot_state(PosDesired_End_right, DPosDesired_End_right, DDPosDesired_End_right);


        //
        Position_desired_end_left.position.x = PosDesired_End_left(0);
        Position_desired_end_left.position.y = PosDesired_End_left(1);
        Position_desired_end_left.position.z = PosDesired_End_left(2);
        Position_desired_end_right.position.x = PosDesired_End_right(0);
        Position_desired_end_right.position.y = PosDesired_End_right(1);
        Position_desired_end_right.position.z = PosDesired_End_right(2);

        pub_pos_desired_END_left.publish(Position_desired_end_left);
        pub_pos_desired_END_right.publish(Position_desired_end_right);


        Position_VO.Resize(3);
        Position_VO = force_dynamical_system->Get_virtual_object_pos();

        Virtial_object.position.x = Position_VO(0);
        Virtial_object.position.y = Position_VO(1);
        Virtial_object.position.z = Position_VO(2);

        pub_virtual_object.publish(Virtial_object);

        solveInverseKinematics();

        data_save <<RPos_object(0)<<" "<<RPos_object(1)<<" "<<RPos_object(2)<<" "<<Position_VO(0)<<" "<<Position_VO(1)<<" "<<Position_VO(2)<<" "<<RPos_End_left(0)<<" "<<RPos_End_left(1)<<" "<<RPos_End_left(2)<<" "<<RPos_End_right(0)<<" "<<RPos_End_right(1)<<" "<<RPos_End_right(2)<<" "<<ttc<<endl;

        break;

    }

    return STATUS_OK;
}


RobotInterface::Status bimanual_catching_module::RobotUpdateCore(){

    ros::spinOnce();

    sendPositionToRobot(Left, JointTargetPos_left);
    sendPositionToRobot(Right, JointTargetPos_right);

    mSKinematicChain_left->setJoints(JointTargetPos_left.Array());
    mSKinematicChain_right->setJoints(JointTargetPos_right.Array());

    mSKinematicChain_left->getEndPos(RPos_End_left.Array());
    mSKinematicChain_right->getEndPos(RPos_End_right.Array());

    if(mRobot->GetControlMode()!=Robot::CTRLMODE_POSITION)
        mRobot->SetControlMode(Robot::CTRLMODE_POSITION);

    mActuatorsGroup.SetJointAngles(JointTargetPos_left);
    mActuatorsGroup.WriteActuators();
    mKinematicChain.Update();
    mRobot->Update();

    mSensorsGroup.ReadSensors();


//    RJointPos_left = TJointPos_left;
//    RJointPos_right = TJointPos_right;
    RJointPos_left = JointTargetPos_left;
    RJointPos_right = JointTargetPos_right;

    Pos_Left_End.position.x = RPos_End_left(0);   Pos_Left_End.position.y = RPos_End_left(1);   Pos_Left_End.position.z = RPos_End_left(2);
    Pos_Right_End.position.x = RPos_End_right(0); Pos_Right_End.position.y = RPos_End_right(1); Pos_Right_End.position.z = RPos_End_right(2);

    pub_pos_END_left.publish(Pos_Left_End);
    pub_pos_END_right.publish(Pos_Right_End);

    return STATUS_OK;
}


int bimanual_catching_module::RespondToConsoleCommand(const string cmd, const vector<string> &args){

    if      (cmd=="init")       mCommand = COMMAND_INITIAL;
    else if (cmd=="ready")      mCommand = COMMAND_READY;
    else if (cmd=="catch")      mCommand = COMMAND_CATCH;

    return 0;
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    bimanual_catching_module* create(){return new bimanual_catching_module();}
    void destroy(bimanual_catching_module* module){delete module;}
}

