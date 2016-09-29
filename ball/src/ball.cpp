/*
  Copyright (c) 2013 Sina Mirrazavi,
  LASA Lab, EPFL, CH-1015 Lausanne, Switzerland,
  http://lasa.epfl.ch
  The program is free for non-commercial academic use.
  Please acknowledge the authors in any academic publications that have
  made use of this code or part of it.
  }
 */
#include "ball.h"

Command COM;
ENUM_State State;
double Postion_VO[3];

void chatterCallback_Command(const std_msgs::String::ConstPtr& msg)
{
    int command;

    sscanf(msg->data.c_str(), "%d", &command);

    switch(command){
    case Com_INIT:
        COM=Com_INIT;
        break;
    case Com_Throw:
        COM=Com_Throw;
        break;
    }

}
double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void chatterCallback_State(const std_msgs::String::ConstPtr& msg)
{
    int State_h;

    sscanf(msg->data.c_str(), "%d", &State_h);

    switch(State_h){
    case Com_Stop:
        State=Com_Stop;
        break;
    case Com_Break:
        State=Com_Break;
        break;
    case Com_Safe:
        State=Com_Safe;
        break;
    }

}

void chatterCallback_VO(const std_msgs::Float32MultiArray& msg)
{
    Postion_VO[0]=msg.data[0];
    Postion_VO[1]=msg.data[1];
    Postion_VO[2]=msg.data[2];
    cout<<"Postion_VO "<<Postion_VO[0]<<" "<<Postion_VO[1]<<" "<<Postion_VO[2]<<endl;

}


double step=0.0001;

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub;
    ros::Publisher chatter_pub_object_left;
    ros::Publisher chatter_pub_object_right;
    ros::Publisher chatter_pub_real;
    ros::Subscriber sub_command;
    ros::Subscriber sub_state;
    ros::Subscriber sub_VO;
    geometry_msgs::Pose Object;
    geometry_msgs::Pose Object_real;
    geometry_msgs::Pose Object_left;
    geometry_msgs::Pose Object_right;

    chatter_pub = n.advertise<geometry_msgs::Pose>("object", 3);
    chatter_pub_object_left = n.advertise<geometry_msgs::Pose>("/object/Shift/Left", 3);
    chatter_pub_object_right = n.advertise<geometry_msgs::Pose>("/object/Shift/Right", 3);
    chatter_pub_real=n.advertise<geometry_msgs::Pose>("object_real_position", 3);
    sub_command = n.subscribe("/catch/command", 3, chatterCallback_Command);
    sub_state = n.subscribe("/catch/state", 3, chatterCallback_State);
    sub_VO= n.subscribe("/catch/VO", 3, chatterCallback_VO);

    double P_I[3];
    double DP_I[3];
    P_O.Resize(3);
    Shift_left_P_O.Resize(3);Shift_left_P_O.Zero();
    Shift_right_P_O.Resize(3);Shift_right_P_O.Zero();
    DP_O.Resize(3);
    DDP_O.Resize(3);
    dt=0.001;

    Shift_left_P_O(1)=-0.40;
    Shift_right_P_O(1)=0.40;

    P_I[0]=-3.5;P_I[1]=-0.5;P_I[2]=-0.0;
    DP_I[0]=1.2;DP_I[1]=0.0;DP_I[2]=1.0; //DP_I[0]=1.2;DP_I[1]=0.0;DP_I[2]=1.1;

    P_O(0)=P_I[0];P_O(1)=P_I[1];P_O(2)=P_I[2];
    DP_O(0)=DP_I[0];DP_O(1)=DP_I[1];DP_O(2)=DP_I[2];
    DDP_O(0)=0.0;DDP_O(1)=0.0;DDP_O(2)=-0.5;//-0.5

    Vector DDhandle;
    DDhandle.Resize(3);

    Vector Dhandle;
    Dhandle.Resize(3);

    Vector handle;
    handle.Resize(3);

    ros::Rate r(1000);
    COM=Com_NONE;
    State=Com_Safe;
    while ((COM==Com_NONE)||(COM==Com_INIT))
    {
        ros::spinOnce();
        if (COM==Com_INIT)
        {
            Object.position.x=P_O(0);
            Object.position.y=P_O(1);
            Object.position.z=P_O(2);

            Object.orientation.x=DP_O(0);
            Object.orientation.y=DP_O(1);
            Object.orientation.z=DP_O(2);
            chatter_pub.publish(Object);
            chatter_pub.publish(Object);
        }
    }
    double count=0;
    double count_global=0;
    double count_object=0;
    while ((ros::ok()))
    {
        if ((COM==Com_Throw)&&(P_O(2)>-1.0))
        {

                DDP_O.Mult(dt,DDhandle);
                DP_O=DP_O+DDhandle;
                DP_O.Mult(dt,Dhandle);
                P_O=P_O+Dhandle;


            Object_real.position.x=P_O(0);
            Object_real.position.y=P_O(1);
            Object_real.position.z=P_O(2);

//            Object.position.x=P_O(0)+fRand(0,0.05);
//            Object.position.y=P_O(1)+fRand(0,0.05);
//            Object.position.z=P_O(2)+fRand(0,0.05);


            Object.position.x=P_O(0);
            Object.position.y=P_O(1);
            Object.position.z=P_O(2);

            Object.orientation.x=DP_O(0);
            Object.orientation.y=DP_O(1);
            Object.orientation.z=DP_O(2);

//            Object_left.position.x=P_O(0)+Shift_left_P_O(0);
  //          Object_left.position.y=P_O(1)+Shift_left_P_O(1);
    //        Object_left.position.z=P_O(2)+Shift_left_P_O(2);

            Object_left.position.x=Shift_left_P_O(0);
            Object_left.position.y=Shift_left_P_O(1);
            Object_left.position.z=Shift_left_P_O(2);

//            Object_right.position.x=P_O(0)+Shift_right_P_O(0);
//            Object_right.position.y=P_O(1)+Shift_right_P_O(1);
//            Object_right.position.z=P_O(2)+Shift_right_P_O(2);

            Object_right.position.x=Shift_right_P_O(0);
            Object_right.position.y=Shift_right_P_O(1);
            Object_right.position.z=Shift_right_P_O(2);
            count=count+1;
            count_global=count_global+1;
//            cout<<"count_global "<<count_global<<" "<<count_object<<endl;
            chatter_pub_real.publish(Object_real);
            if (count>30) {
                count=0;
                count_object=count_object+1;
                chatter_pub.publish(Object);
                chatter_pub_object_left.publish(Object_left);
                chatter_pub_object_right.publish(Object_right);
            }
        }
        if (COM==Com_INIT)
        {
            double AA=0.1;
            P_O(0)=P_I[0]+fRand(-AA,AA);P_O(1)=P_I[1]+fRand(-AA,AA);P_O(2)=P_I[2]+fRand(-AA,AA);
            P_O.Print("The initial position");
            DP_O(0)=DP_I[0]+fRand(-AA,AA);DP_O(1)=DP_I[1]+fRand(-AA,AA);DP_O(2)=DP_I[2]+fRand(0-AA,AA);
            DP_O.Print("The initial velocity");
            DDP_O(0)=0.0;DDP_O(1)=0.0;DDP_O(2)=-0.5;//-0.5
            Object.position.x=P_O(0);
            Object.position.y=P_O(1);
            Object.position.z=P_O(2);

            Object.orientation.x=DP_O(0);
            Object.orientation.y=DP_O(1);
            Object.orientation.z=DP_O(2);
            count=0;
            count_global=0;
            count_object=0;
            State=Com_Safe;

        }

        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
