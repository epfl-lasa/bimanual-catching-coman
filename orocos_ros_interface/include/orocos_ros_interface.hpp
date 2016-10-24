#ifndef OROCOS_OROCOS_ROS_INTERFACE_HPP
#define OROCOS_OROCOS_ROS_INTERFACE_HPP

#include <ros/ros.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/robot/JointState.hpp>

#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"

// for gazebo-rtt interface
#include <rtt/RTT.hpp>
#include <rtt/os/Semaphore.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
//#include <gazebo_msgs/SetModelState.h>
#include <gazebo/math/gzmath.hh>
#include <boost/shared_ptr.hpp>


using namespace std;

#define COMAN_ARM_DOF_SIZE 7
class Orocos_ros_interface : public RTT::TaskContext{
public:
    Orocos_ros_interface(std::string const& name);
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();
    void WorldUpdateBegin();
    void WorldUpdateEnd();

private:
	double prop_counter_step;
	double prop_service_call_counter;

	std_msgs::Float64MultiArray q_left_Arm;
	std_msgs::Float64MultiArray q_right_Arm;

	std_msgs::Float64MultiArray q_left_Arm_current;
	std_msgs::Float64MultiArray q_right_Arm_current;

	std_msgs::Float64MultiArray Dq_left_Arm_current;
	std_msgs::Float64MultiArray Dq_right_Arm_current;

	std_msgs::Float64MultiArray T_left_Arm_current;
	std_msgs::Float64MultiArray T_right_Arm_current;

	RTT::InputPort<std_msgs::Float64MultiArray > inport_left;
	RTT::InputPort<std_msgs::Float64MultiArray > inport_right;

	RTT::OutputPort<std_msgs::Float64MultiArray> outport_q_left;
	RTT::OutputPort<std_msgs::Float64MultiArray> outport_q_right;

	RTT::OutputPort<std_msgs::Float64MultiArray> outport_Dq_left;
	RTT::OutputPort<std_msgs::Float64MultiArray> outport_Dq_right;

	RTT::OutputPort<std_msgs::Float64MultiArray> outport_T_left;
	RTT::OutputPort<std_msgs::Float64MultiArray> outport_T_right;

	// Declare ports and their datatypes
	RTT::OutputPort<rstrt::kinematics::JointAngles> joint_position_left_arm_output_port;
	RTT::OutputPort<rstrt::kinematics::JointAngles> joint_position_right_arm_output_port;

	RTT::InputPort<rstrt::robot::JointState> joint_position_left_arm_input_port;
	RTT::InputPort<rstrt::robot::JointState> joint_position_right_arm_input_port;

	// Actuall joint command to be sent over port:
	rstrt::kinematics::JointAngles joint_position_left_arm_command;
	rstrt::kinematics::JointAngles joint_position_right_arm_command;

	rstrt::robot::JointState joint_position_left_arm;
	rstrt::robot::JointState joint_position_right_arm;


    // for gazebo interface
    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr world_begin;
    gazebo::event::ConnectionPtr world_end;

    gazebo::physics::Joint_V gazebo_joints_;
    gazebo::physics::Link_V model_links_;

    bool getModel(const std::string& model_name);
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model);

    bool _models_loaded;
    bool is_configured;

    RTT::OutputPort<std_msgs::Float64MultiArray> outport_obj_pos;
    std_msgs::Float64MultiArray obj_pos;
    gazebo::math::Pose obj_pos_input;


};
#endif
