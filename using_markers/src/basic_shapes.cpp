#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Pose.h"


visualization_msgs::Marker marker;
visualization_msgs::Marker marker_att;
visualization_msgs::Marker marker_virtual;

visualization_msgs::Marker marker_desired_left;
visualization_msgs::Marker marker_desired_right;

ros::Publisher marker_pub;
ros::Publisher marker_att_pub;
ros::Publisher marker_virtual_pub;

ros::Publisher marker_desired_left_pub;
ros::Publisher marker_desired_right_pub;

double scaleFactor=0.6;

void chatterCallback_object(const geometry_msgs::Pose & msg)
{

	marker.header.stamp = ros::Time::now();

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

	marker.pose.position.x = msg.position.x;
	marker.pose.position.y = msg.position.y;
    marker.pose.position.z = msg.position.z;
	marker.pose.orientation.w = msg.orientation.w;
	marker.pose.orientation.x = msg.orientation.x;
	marker.pose.orientation.y = msg.orientation.y;
	marker.pose.orientation.z = msg.orientation.z;
	marker_pub.publish(marker);
}


void chatterCallback_attractor(const geometry_msgs::Pose & msg)
{

    marker_att.header.stamp = ros::Time::now();

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

    marker_att.pose.position.x = msg.position.x;
    marker_att.pose.position.y = msg.position.y;
    marker_att.pose.position.z = msg.position.z;
    marker_att.pose.orientation.w = msg.orientation.w;
    marker_att.pose.orientation.x = msg.orientation.x;
    marker_att.pose.orientation.y = msg.orientation.y;
    marker_att.pose.orientation.z = msg.orientation.z;
    marker_att_pub.publish(marker_att);
}


void chatterCallback_object_virtual(const geometry_msgs::Pose & msg)
{

	marker_virtual.header.stamp = ros::Time::now();

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

	marker_virtual.pose.position.x = msg.position.x;
	marker_virtual.pose.position.y = msg.position.y;
	marker_virtual.pose.position.z = msg.position.z;
	marker_virtual.pose.orientation.w = msg.orientation.w;
	marker_virtual.pose.orientation.x = msg.orientation.x;
	marker_virtual.pose.orientation.y = msg.orientation.y;
	marker_virtual.pose.orientation.z = msg.orientation.z;
    marker_virtual_pub.publish(marker_virtual);
}


void chatterCallback_desired_left(const geometry_msgs::Pose & msg)
{

    marker_desired_left.header.stamp = ros::Time::now();

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

    marker_desired_left.pose.position.x = msg.position.x;
    marker_desired_left.pose.position.y = msg.position.y;
    marker_desired_left.pose.position.z = msg.position.z;
    marker_desired_left.pose.orientation.w = msg.orientation.w;
    marker_desired_left.pose.orientation.x = msg.orientation.x;
    marker_desired_left.pose.orientation.y = msg.orientation.y;
    marker_desired_left.pose.orientation.z = msg.orientation.z;
    marker_desired_left_pub.publish(marker_desired_left);
}

void chatterCallback_desired_right(const geometry_msgs::Pose & msg)
{

    marker_desired_left.header.stamp = ros::Time::now();

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

    marker_desired_right.pose.position.x = msg.position.x;
    marker_desired_right.pose.position.y = msg.position.y;
    marker_desired_right.pose.position.z = msg.position.z;
    marker_desired_right.pose.orientation.w = msg.orientation.w;
    marker_desired_right.pose.orientation.x = msg.orientation.x;
    marker_desired_right.pose.orientation.y = msg.orientation.y;
    marker_desired_right.pose.orientation.z = msg.orientation.z;
    marker_desired_right_pub.publish(marker_desired_right);
}







int main( int argc, char** argv )
{
	ros::init(argc, argv, "basic_shapes");
	ros::NodeHandle n;
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_ball", 0);
    marker_att_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_att", 0);
    marker_virtual_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_vir", 0);

    marker_desired_left_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_desired_L", 0);
    marker_desired_right_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_desired_R", 0);

    ros::Subscriber	sub_object = n.subscribe("/catch/objpos", 3, chatterCallback_object);
//    ros::Subscriber	sub_object = n.subscribe("/object", 3, chatterCallback_object);
    ros::Subscriber	sub_attractor = n.subscribe("/catch/attpos", 3, chatterCallback_attractor);
	ros::Subscriber	sub_object_virtual = n.subscribe("/object_virtual/position", 3, chatterCallback_object_virtual);

    ros::Subscriber	sub_desired_left = n.subscribe("/desired_end_left", 3, chatterCallback_desired_left);
    ros::Subscriber	sub_desired_right = n.subscribe("/desired_end_right", 3, chatterCallback_desired_right);

//    scaleFactor= scaleFactor * 0.4 / abs(marker_desired_left.pose.position.x-marker_desired_right.pose.position.x);

    // Set our initial shape type to be a cube
    uint32_t   shape = visualization_msgs::Marker::SPHERE;

	marker.header.frame_id = "/world_frame";
	marker.action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = shape;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	double scale=0.5;
	marker.scale.x = 0.6;
	marker.scale.y = 0.6;
    marker.scale.z = 0.6;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
	marker.color.a = 1.0;
	marker.pose.position.x = 0.0 ;

    marker_att.header.frame_id = "/world_frame";
    marker_att.action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker_att.ns = "basic_shapes";
    marker_att.id = 0;
    marker_att.type = shape;
    marker_att.pose.orientation.x = 0.0;
    marker_att.pose.orientation.y = 0.0;
    marker_att.pose.orientation.z = 0.0;
    marker_att.pose.orientation.w = 1.0;
    marker_att.scale.x = 0.6;
    marker_att.scale.y = 0.6;
    marker_att.scale.z = 0.6;
    marker_att.color.r = 0.0f;
    marker_att.color.g = 0.5f;
    marker_att.color.b = 0.5f;
    marker_att.color.a = 1.0;
    marker_att.pose.position.x = 0.0 ;

	marker_virtual.header.frame_id = "/world_frame";
	marker_virtual.action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker_virtual.ns = "basic_shapes";
	marker_virtual.id = 1;
	marker_virtual.type = shape;
	marker_virtual.pose.orientation.x = 0.0;
	marker_virtual.pose.orientation.y = 0.0;
	marker_virtual.pose.orientation.z = 0.0;
	marker_virtual.pose.orientation.w = 1.0;
    marker_virtual.scale.x = scaleFactor;
    marker_virtual.scale.y = scaleFactor;
    marker_virtual.scale.z = scaleFactor;
    marker_virtual.color.r = 0.5f;
    marker_virtual.color.g = 0.5f;
	marker_virtual.color.b = 0.0f;
	marker_virtual.color.a = 1.0;
	marker_virtual.pose.position.x = 0.0 ;

    marker_desired_left.header.frame_id = "/world_frame";
    marker_desired_left.action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker_desired_left.ns = "basic_shapes";
    marker_desired_left.id = 1;
    marker_desired_left.type = shape;
    marker_desired_left.pose.orientation.x = 0.0;
    marker_desired_left.pose.orientation.y = 0.0;
    marker_desired_left.pose.orientation.z = 0.0;
    marker_desired_left.pose.orientation.w = 1.0;
    marker_desired_left.scale.x = 0.1;
    marker_desired_left.scale.y = 0.1;
    marker_desired_left.scale.z = 0.1;
    marker_desired_left.color.r = 1.0f;
    marker_desired_left.color.g = 0.0f;
    marker_desired_left.color.b = 0.0f;
    marker_desired_left.color.a = 1.0;
    marker_desired_left.pose.position.x = 0.0 ;

    marker_desired_right.header.frame_id = "/world_frame";
    marker_desired_right.action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker_desired_right.ns = "basic_shapes";
    marker_desired_right.id = 1;
    marker_desired_right.type = shape;
    marker_desired_right.pose.orientation.x = 0.0;
    marker_desired_right.pose.orientation.y = 0.0;
    marker_desired_right.pose.orientation.z = 0.0;
    marker_desired_right.pose.orientation.w = 1.0;
    marker_desired_right.scale.x = 0.1;
    marker_desired_right.scale.y = 0.1;
    marker_desired_right.scale.z = 0.1;
    marker_desired_right.color.r = 1.0f;
    marker_desired_right.color.g = 0.0f;
    marker_desired_right.color.b = 0.0f;
    marker_desired_right.color.a = 1.0;
    marker_desired_right.pose.position.x = 0.0 ;

//    marker_pub.publish(marker);
//    marker_att_pub.publish(marker_att);
//    marker_virtual_pub.publish(marker_virtual);


	ros::spin();
}
