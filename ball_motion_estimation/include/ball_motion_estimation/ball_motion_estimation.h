
#ifndef ball_motion_estimation_H_
#define ball_motion_estimation_H_

#include "boost/date_time/posix_time/posix_time_types.hpp" //no i/o just types
#include <boost/timer.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include <sstream>
#include <iostream>
#include <math.h>

#include "Gaussians.h"
//#include "GraspPostureFinder.h"
//#include "CDDynamics.h"
//#include "filt.h"
//#include "Smoother.h"

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
//#include <ctime>
//#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
//#include "vector.h"
//#include "vector3.h"

#include <time.h>

using namespace std;


enum MOCAP_STATUS {MOCAP_IDLE, MOCAP_OBSERVING, MOCAP_TRACKING, MOCAP_SAVE};
enum TRACKING_STATUS{TRACKINGSTATUS_NONE, TRACKINGSTATUS_PREREADY, TRACKINGSTATUS_TRACKING, TRACKINGSTATUS_POSTREADY};
enum Command {Com_NONE,Com_INIT, Com_THROW};



ros::Subscriber	sub_object;
ros::Subscriber sub_command;

ros::Publisher pub_objpos;
ros::Publisher pub_objvel;
ros::Publisher pub_attpos;
ros::Publisher pub_attvel;
ros::Publisher pub_ttc;

Vector3 Object_pos_T;
Command COM;

typedef struct sMeasurement {
    double time;
    Vector3 vPos;
    Matrix3 vOri;
} TMeasurement;

TMeasurement mMeasurement;
TMeasurement mMeasurement_prev;

MOCAP_STATUS mMocapStatus;
TRACKING_STATUS mTrackingStatus;

Vector3 mFirstPosture;

int counter = 0;
int mObjTrajectoryCount = 0;
int mNbObservation = 0;

double mDT = 0.03;


geometry_msgs::Pose msg_objpos;
geometry_msgs::Pose msg_objvel;
geometry_msgs::Pose msg_attpos;
geometry_msgs::Pose msg_attvel;
std_msgs::String msg_ttc;


boost::posix_time::ptime mStartTime;

boost::posix_time::ptime lCurrentTime;
boost::posix_time::ptime lPrevTime;

bool mMeasurementIsUpdating = false;
bool mMeasurementIsNew = false;


#endif  // ball_motion_estimation
