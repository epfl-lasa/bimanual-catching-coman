#include "ball_motion_estimation.h"


void chatterCallback_object(const geometry_msgs::Pose & msg) {

    Object_pos_T(0) = msg.position.x;
    Object_pos_T(1) = msg.position.y;
    Object_pos_T(2) = msg.position.z;
}


void commandCallback(const std_msgs::String::ConstPtr& msg) {

    int command;
    sscanf(msg->data.c_str(), "%d", &command);
    ROS_INFO("msg: [%s]", msg->data.c_str());

    switch (command) {

    case Com_INIT:
        COM = Com_INIT;
        break;

    case Com_THROW:
        COM = Com_THROW;
        cout<<"Com_THROW "<<endl;
        mMocapStatus = MOCAP_IDLE;

        mNbObservation = 0;
        mStartTime = boost::posix_time::microsec_clock::local_time();

        mMocapStatus = MOCAP_TRACKING;
        mTrackingStatus = TRACKINGSTATUS_NONE;

        break;

    }
}


void *MotionCaptureReading(void *ptr) {
    // mocap reading thread

    Vector3 pos, pos_old;
    pos_old.Zero();

    boost::posix_time::ptime lCurrentTime;

    mMeasurementIsUpdating = false;
    mMeasurementIsNew = false;
    mStartTime = boost::posix_time::microsec_clock::local_time();
    bool first_time = false;

    while(ros::ok())
    {
        while(mMeasurementIsUpdating == true)   usleep(1.);

        pos = Object_pos_T;
        lCurrentTime = boost::posix_time::microsec_clock::local_time();

        if(( (pos-pos_old).Norm() < 0.01 )) {        //same data
            continue;
        }
        else {
//            if (mTrackingStatus == TRACKINGSTATUS_PREREADY)
//                first_time=false;

//            if (mTrackingStatus == TRACKINGSTATUS_TRACKING) {
//                if (first_time==false) {
//                    cout<<"The_first_time "<<first_time<<endl;
//                    next_sample = pos(1);
//                    Filter_Follower->Initialization(next_sample);
//                    filtered_sample=Filter_Follower->Smooting(next_sample,mDT);
//                    pos(1)=filtered_sample;

//                }
//                else {
//                    next_sample=pos(1);
//                    filtered_sample=Filter_Follower->Smooting(next_sample,mDT);
//                    pos(1)=filtered_sample;
//                }
//                The_first_time=true;
//            }

            mMeasurementIsUpdating = true;
            mMeasurement.time = (double)(lCurrentTime- mStartTime).total_milliseconds() / 1000.;
            mMeasurement.vPos = pos;
//            mMeasurement.vOri=Rotation_Matrix;
            mMeasurementIsNew = true;
            mMeasurementIsUpdating = false;
            pos_old = pos;

        }
    }

}


void *EstimateTTC(void *ptr) {
    // ttc estimating thread

    TMeasurement lMeasurement;

    ros::Rate loop_rate(500);

    Vector lVel(3);
    double ttc;

    lVel.Zero();

    char buffer[1024];

    while(ros::ok()) {
        lCurrentTime = boost::posix_time::microsec_clock::local_time();
//        lPrevTime = boost::posix_time::microsec_clock::local_time();

        switch (mMocapStatus) {

        case MOCAP_IDLE:
            usleep(100.);
            break;

        case MOCAP_TRACKING:
            if( mMeasurementIsNew ){
                while(mMeasurementIsUpdating == true)    usleep(2.);

                lMeasurement = mMeasurement;
                mMeasurementIsNew = false;

                if (mTrackingStatus==TRACKINGSTATUS_NONE) {
                    cout << "Current distance : " << lMeasurement.vPos(0) << endl;
                    counter = 1;

                    if( (lMeasurement.vPos(0) < -1.2) ) {
                        mFirstPosture = lMeasurement.vPos;
                        mFirstPosture.Print("mFirstPosture");
                        mTrackingStatus = TRACKINGSTATUS_PREREADY;
                    }

                }
                else if (mTrackingStatus==TRACKINGSTATUS_PREREADY) {
                    counter = 0;

                    if( lMeasurement.vPos(0) > (mFirstPosture(0) + 0.1) ) {
//                        mMotionStartTime = lCurrentTime;
                        mObjTrajectoryCount = 0;
//                        mCatchingPostureCount =0;
                        mTrackingStatus = TRACKINGSTATUS_TRACKING;

                        cout<< "tracking started" << endl;
                    }

                }
                else if (mTrackingStatus==TRACKINGSTATUS_TRACKING) {
//                    mNbObservation++;
                    counter++;

                    if (counter==1) {
                        continue;
                    }
                    else {
                        double lDuration = (double)(lCurrentTime-lPrevTime).total_milliseconds()/1000.;

                        lVel(0) = ( lMeasurement.vPos(0)-mMeasurement_prev.vPos(0) ) / lDuration;
                        lVel(1) = ( lMeasurement.vPos(1)-mMeasurement_prev.vPos(1) ) / lDuration;
                        lVel(2) = ( lMeasurement.vPos(2)-mMeasurement_prev.vPos(2) ) / lDuration;
                    }

                    if (lMeasurement.vPos(0) < 0.0) {
                        if (lVel(0)!=0.0)   ttc = abs(lMeasurement.vPos(0) / lVel(0));
                        else                ttc = 5.0;
                    }
                    else {
                        ttc = 5.0;
                    }

                    sprintf(buffer, "%lf ", ttc);
                    msg_ttc.data = buffer;
                    pub_ttc.publish(msg_ttc);

//                    cout<<"lVel: "<<lVel(0)<<" "<<lVel(1)<<" "<<lVel(2)<<"    ttc : "<<ttc<<endl;
//                    cout<<"time : "<<(double)(lCurrentTime-lPrevTime).total_milliseconds()/1000.<<endl;

                    mMeasurement_prev = lMeasurement;
                    lPrevTime = lCurrentTime;


                    msg_objpos.position.x = lMeasurement.vPos(0);
                    msg_objpos.position.y = lMeasurement.vPos(1);
                    msg_objpos.position.z = lMeasurement.vPos(2);

                    msg_objvel.position.x = lVel(0);
                    msg_objvel.position.y = lVel(1);
                    msg_objvel.position.z = lVel(2);

//                    msg_attpos.position.x = lMeasurement.vPos(0) +0.3;
//                    msg_attpos.position.y = lMeasurement.vPos(1);
//                    msg_attpos.position.z = lMeasurement.vPos(2) -0.2;

                    msg_attpos.position.x = lMeasurement.vPos(0);
                    msg_attpos.position.y = lMeasurement.vPos(1);
                    msg_attpos.position.z = lMeasurement.vPos(2);

                    msg_attvel.position.x = lVel(0);
                    msg_attvel.position.y = lVel(1);
                    msg_attvel.position.z = lVel(2);


                    pub_objpos.publish(msg_objpos);
                    pub_objvel.publish(msg_objvel);
                    pub_attpos.publish(msg_attpos);
                    pub_attvel.publish(msg_attvel);

                }
//                else if (mTrackingStatus==TRACKINGSTATUS_POSTREADY) {

//                }


            }
            else {  // no new measurement
                usleep(10.);
            }





            break;

        }  // switch

        loop_rate.sleep();

    } // while


}






int main(int argc, char** argv) {

    ros::init(argc, argv, "ballMotionEstimation");
    ros::NodeHandle n;

    sub_object = n.subscribe("/object", 3, chatterCallback_object);
    sub_command = n.subscribe("/catch/command", 3, commandCallback);

    pub_objpos = n.advertise<geometry_msgs::Pose>("/catch/objpos", 3);
    pub_objvel =  n.advertise<geometry_msgs::Pose>("/catch/objvel", 3);

    pub_attpos = n.advertise<geometry_msgs::Pose>("/catch/attpos", 3);
    pub_attvel = n.advertise<geometry_msgs::Pose>("/catch/attvel", 3);

    pub_ttc = n.advertise<std_msgs::String>("/ttc", 3);

    mStartTime = boost::posix_time::microsec_clock::local_time();

    cout<<"hello"<<endl;


    mMocapStatus = MOCAP_IDLE;
    COM = Com_NONE;

    // motion capture reading thread
    pthread_t lTrackingThread;
    while((COM!=Com_NONE)&&(ros::ok))        ros::spinOnce();
    pthread_create( &lTrackingThread, NULL, MotionCaptureReading, NULL);

    // Finding best catching posture thread
    pthread_t lEstimatingThread;
    pthread_create( &lEstimatingThread, NULL, EstimateTTC, NULL);



    ros::spin();

    return 0;
}
