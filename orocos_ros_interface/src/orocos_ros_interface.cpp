#include "orocos_ros_interface.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace RTT;

Orocos_ros_interface::Orocos_ros_interface(std::string const& name) : TaskContext(name)
, inport_left("Left_arm_in"), inport_right("Right_arm_in")
, outport_q_left("Left_q_arm_out"), outport_q_right("Right_q_arm_out")
, outport_Dq_left("Left_Dq_arm_out"), outport_Dq_right("Right_Dq_arm_out")
, outport_T_left("Left_T_arm_out"), outport_T_right("Right_T_arm_out")
, is_configured(false), _models_loaded(false), outport_obj_pos("Object_pos_out") {

    prop_counter_step = 0.001;
    prop_service_call_counter = 0.001;
	this->addEventPort(inport_left).doc("Receiving the joint's position of the left arm and they will wake up this component.");
	this->addEventPort(inport_right).doc("Receiving the joint's position of the right arm and they will wake up this component.");

	this->addPort(outport_q_left).doc("Sends out the actual joints' position of the left arm ");
	this->addPort(outport_q_right).doc("Sends out the actual joints' position of the right arm");

	this->addPort(outport_Dq_left).doc("Sends out the actual joints' velocity of the left arm ");
	this->addPort(outport_Dq_right).doc("Sends out the actual joints' velocity of the right arm");

	this->addPort(outport_T_left).doc("Sends out the actual joints' torque of the left arm ");
	this->addPort(outport_T_right).doc("Sends out the actual joints' torque of the right arm");


	joint_position_left_arm_command = rstrt::kinematics::JointAngles(COMAN_ARM_DOF_SIZE);
	joint_position_left_arm_command.angles.setZero();

	joint_position_right_arm_command= rstrt::kinematics::JointAngles(COMAN_ARM_DOF_SIZE);
	joint_position_right_arm_command.angles.setZero();

	joint_position_left_arm = rstrt::robot::JointState(COMAN_ARM_DOF_SIZE);
	joint_position_left_arm.angles.setZero();

	joint_position_right_arm= rstrt::robot::JointState(COMAN_ARM_DOF_SIZE);
	joint_position_right_arm.angles.setZero();

	joint_position_left_arm_output_port.setName("JointPositionOutputPort_left_arm");
	joint_position_left_arm_output_port.setDataSample(joint_position_left_arm_command);

	this->addPort(joint_position_left_arm_output_port).doc("Output port for sending left arm reference joint values");

	joint_position_right_arm_output_port.setName("JointPositionOutputPort_right_arm");
	joint_position_right_arm_output_port.setDataSample(joint_position_right_arm_command);

	this->addPort(joint_position_right_arm_output_port).doc("Output port for sending right arm reference joint values");


	joint_position_left_arm_input_port.setName("JointPositionInputPort_left_arm");

	this->addPort(joint_position_left_arm_input_port).doc("Input port for receiving left arm actual joint values");

	joint_position_right_arm_input_port.setName("JointPositionInputPort_right_arm");

	this->addPort(joint_position_right_arm_input_port).doc("Input port for receiving right arm actual joint values");


	q_left_Arm_current.data.resize(COMAN_ARM_DOF_SIZE);
	q_right_Arm_current.data.resize(COMAN_ARM_DOF_SIZE);

	Dq_left_Arm_current.data.resize(COMAN_ARM_DOF_SIZE);
	Dq_right_Arm_current.data.resize(COMAN_ARM_DOF_SIZE);

	T_left_Arm_current.data.resize(COMAN_ARM_DOF_SIZE);
	T_right_Arm_current.data.resize(COMAN_ARM_DOF_SIZE);


    this->addOperation("getModel", &Orocos_ros_interface::getModel, this, RTT::ClientThread);
    this->addPort(outport_obj_pos).doc("Sends out the object position");
    obj_pos.data.resize(6);
}



bool Orocos_ros_interface::getModel(const std::string& model_name) {
    if (model) {
        log(Warning) << "Model [" << model_name << "] already loaded !"
                << endlog();
        return true;
    }
    gazebo::printVersion();
    if (!gazebo::physics::get_world()) {
        log(Error) << "getWorldPtr does not seem to exists" << endlog();
        return false;
    }

    model = gazebo::physics::get_world()->GetModel(model_name);

    if (model) {
        log(Info) << "Model [" << model_name << "] successfully loaded !"<< endlog();
        return true;
    }
    return bool(model);
}


bool Orocos_ros_interface::gazeboConfigureHook(gazebo::physics::ModelPtr model) {
    if (model.get() == NULL) {
        RTT::log(RTT::Error) << "No model could be loaded" << RTT::endlog();
        return false;
    }

    // Get the joints
    gazebo_joints_ = model->GetJoints();
    model_links_ = model->GetLinks();

    RTT::log(RTT::Info) << "Model name "<< model->GetName() << RTT::endlog();
    RTT::log(RTT::Info) << "Model has " << gazebo_joints_.size() << " joints" << RTT::endlog();
    RTT::log(RTT::Info) << "Model has " << model_links_.size() << " links" << RTT::endlog();
    RTT::log(RTT::Info) << "Model pos " << model->GetWorldPose() <<RTT::endlog();


    RTT::log(RTT::Warning) << "Done configuring component" << RTT::endlog();
    return true;
}



bool Orocos_ros_interface::configureHook(){
	if (!joint_position_left_arm_output_port.connected())
		return false;
	else
		return true;
	if (!joint_position_right_arm_output_port.connected())
		return false;
	else
		return true;

    std::cout << "Orocos_ros_interface configured !" <<std::endl;

    this->is_configured = gazeboConfigureHook(model);
    return is_configured;
}


bool Orocos_ros_interface::startHook(){
    std::cout << "Orocos_ros_interface started !" <<std::endl;
	return true;
}

void Orocos_ros_interface::updateHook(){
	inport_left.read(q_left_Arm);
	inport_right.read(q_right_Arm);
	joint_position_left_arm_input_port.read(joint_position_left_arm);
	joint_position_right_arm_input_port.read(joint_position_right_arm);
	std::vector<double> Dp_message_left = q_left_Arm.data;
	std::vector<double> Dp_message_right = q_right_Arm.data;

	if(inport_left.read(q_left_Arm)){
		for(int i=0; i<COMAN_ARM_DOF_SIZE; ++i)
		{
			joint_position_left_arm_command.angles(i) = Dp_message_left[i];
		}
		joint_position_left_arm_output_port.write(joint_position_left_arm_command);
	}

	if(inport_right.read(q_right_Arm)){
		for(int i=0; i<COMAN_ARM_DOF_SIZE; ++i)
		{
			joint_position_right_arm_command.angles(i) = Dp_message_right[i];
		}
		joint_position_right_arm_output_port.write(joint_position_right_arm_command);
	}


	if (joint_position_left_arm_input_port.read(joint_position_left_arm))
	{
        for(int i=0; i<COMAN_ARM_DOF_SIZE; ++i) {
			q_left_Arm_current.data[i]=joint_position_left_arm.angles(i);
			Dq_left_Arm_current.data[i]=joint_position_left_arm.velocities(i);
			T_left_Arm_current.data[i]=joint_position_left_arm.torques(i);
		}
		outport_q_left.write(q_left_Arm_current);
		outport_Dq_left.write(Dq_left_Arm_current);
		outport_T_left.write(T_left_Arm_current);
	}

	if (joint_position_right_arm_input_port.read(joint_position_right_arm))
	{
		for(int i=0; i<COMAN_ARM_DOF_SIZE; ++i)
		{
			q_right_Arm_current.data[i]=joint_position_right_arm.angles(i);
			Dq_right_Arm_current.data[i]=joint_position_right_arm.velocities(i);
			T_right_Arm_current.data[i]=joint_position_right_arm.torques(i);
		}
		outport_q_right.write(q_right_Arm_current);
		outport_Dq_right.write(Dq_right_Arm_current);
		outport_T_right.write(T_right_Arm_current);
	}

    // object position from gazebo
    obj_pos.data[0] = model->GetWorldPose().pos.x;
    obj_pos.data[1] = model->GetWorldPose().pos.y;
    obj_pos.data[2] = model->GetWorldPose().pos.z - 0.4268;
    obj_pos.data[3] = model->GetWorldPose().rot.x;
    obj_pos.data[4] = model->GetWorldPose().rot.y;
    obj_pos.data[5] = model->GetWorldPose().rot.z;

    outport_obj_pos.write(obj_pos);

}


void Orocos_ros_interface::stopHook() {
    std::cout << "Orocos_ros_interface executes stopping !" <<std::endl;
}


void Orocos_ros_interface::cleanupHook() {
    std::cout << "Orocos_ros_interface cleaning up !" <<std::endl;
}


ORO_CREATE_COMPONENT(Orocos_ros_interface)
