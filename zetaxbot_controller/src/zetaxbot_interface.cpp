/*
  zetaxbot - zetaxbot_interface

  This script implements the Hardware Interface that enables the communication
  between the ROS environment and the real robot.
  It uses the ros_control package that allows to abstract the hardware from the software 
  and from the driver required in order to communicate with the hardware.

  In this case, the hardware is an Arduino Board and the driver interface with the hardware is 
  simply composed of a publisher that publishes the desired joint angles of each servo motor.
  Via rooserial_arduino, the Arduino Board will subscribe to this topic and will command the
  servo motor to perorm the desired mouvements. 

  Copyright (c) 2021 Antonio Brandi.  All right reserved.
*/

#include "zetaxbot_controller/zetaxbot_interface.h"
#include <std_msgs/UInt16MultiArray.h>
#include "zetaxbot_controller/AnglesConverter.h"


zetaxbotInterface::zetaxbotInterface(ros::NodeHandle& nh) : nh_(nh), 
            pnh_("~"),
            pos_(22, 0),
            vel_(22, 0),
            eff_(22, 0),
            cmd_(22, 0),
            names_{ "J1" ,"J2","J3","J4","J5","J7","J10",
                    "J13","J16","J19",
                    "J24","J25","J26",
                    "J27","J28","J29",
                    "J32","J35","J38",
                    "J42","J46","J47"}
{
    // Read from the param server
    pnh_.param("joint_names", names_, names_);

    // Init the publisher with the hardware
    hardware_pub_ = pnh_.advertise<std_msgs::UInt16MultiArray>("/arduino/arm_actuate", 1000);
    hardware_srv_ = pnh_.serviceClient<zetaxbot_controller::AnglesConverter>("/radians_to_degrees");
    
    ROS_INFO("Starting zetaxbot Hardware Interface...");

    // connect and register joint state interface
    // hardware_interface::JointStateHandle state_handle1(names_.at(0), &pos_.at(0), &vel_.at(0), &eff_.at(0));
    // joint_state_interface_.registerHandle(state_handle1);
    // hardware_interface::JointStateHandle state_handle2(names_.at(1), &pos_.at(1), &vel_.at(1), &eff_.at(1));
    // joint_state_interface_.registerHandle(state_handle2);
    // hardware_interface::JointStateHandle state_handle3(names_.at(2), &pos_.at(2), &vel_.at(2), &eff_.at(2));
    // joint_state_interface_.registerHandle(state_handle3);
    // hardware_interface::JointStateHandle state_handle4(names_.at(3), &pos_.at(3), &vel_.at(3), &eff_.at(3));
    // joint_state_interface_.registerHandle(state_handle4);

    hardware_interface::JointStateHandle state_handle1(names_.at(0), &pos_.at(0), &vel_.at(0), &eff_.at(0));
    hardware_interface::JointStateHandle state_handle2(names_.at(1), &pos_.at(1), &vel_.at(1), &eff_.at(1));
    hardware_interface::JointStateHandle state_handle3(names_.at(2), &pos_.at(2), &vel_.at(2), &eff_.at(2));
    hardware_interface::JointStateHandle state_handle4(names_.at(3), &pos_.at(3), &vel_.at(3), &eff_.at(3));
    hardware_interface::JointStateHandle state_handle5(names_.at(4), &pos_.at(4), &vel_.at(4), &eff_.at(4));
    hardware_interface::JointStateHandle state_handle6(names_.at(5), &pos_.at(5), &vel_.at(5), &eff_.at(5));
    hardware_interface::JointStateHandle state_handle7(names_.at(6), &pos_.at(6), &vel_.at(6), &eff_.at(6));
    hardware_interface::JointStateHandle state_handle8(names_.at(7), &pos_.at(7), &vel_.at(7), &eff_.at(7));
    hardware_interface::JointStateHandle state_handle9(names_.at(8), &pos_.at(8), &vel_.at(8), &eff_.at(8));
    hardware_interface::JointStateHandle state_handle10(names_.at(9), &pos_.at(9), &vel_.at(9), &eff_.at(9));
    hardware_interface::JointStateHandle state_handle11(names_.at(10), &pos_.at(10), &vel_.at(10), &eff_.at(10));
    hardware_interface::JointStateHandle state_handle12(names_.at(11), &pos_.at(11), &vel_.at(11), &eff_.at(11));
    hardware_interface::JointStateHandle state_handle13(names_.at(12), &pos_.at(12), &vel_.at(12), &eff_.at(12));
    hardware_interface::JointStateHandle state_handle14(names_.at(13), &pos_.at(13), &vel_.at(13), &eff_.at(13));
    hardware_interface::JointStateHandle state_handle15(names_.at(14), &pos_.at(14), &vel_.at(14), &eff_.at(14));
    hardware_interface::JointStateHandle state_handle16(names_.at(15), &pos_.at(15), &vel_.at(15), &eff_.at(15));
    hardware_interface::JointStateHandle state_handle17(names_.at(16), &pos_.at(16), &vel_.at(16), &eff_.at(16));
    hardware_interface::JointStateHandle state_handle18(names_.at(17), &pos_.at(17), &vel_.at(17), &eff_.at(17));
    hardware_interface::JointStateHandle state_handle19(names_.at(18), &pos_.at(18), &vel_.at(18), &eff_.at(18));
    hardware_interface::JointStateHandle state_handle20(names_.at(19), &pos_.at(19), &vel_.at(19), &eff_.at(19));
    hardware_interface::JointStateHandle state_handle21(names_.at(20), &pos_.at(20), &vel_.at(20), &eff_.at(20));
    hardware_interface::JointStateHandle state_handle22(names_.at(21), &pos_.at(21), &vel_.at(21), &eff_.at(21));


    joint_state_interface_.registerHandle(state_handle1);
    joint_state_interface_.registerHandle(state_handle2);
    joint_state_interface_.registerHandle(state_handle3);
    joint_state_interface_.registerHandle(state_handle4);
    joint_state_interface_.registerHandle(state_handle5);
    joint_state_interface_.registerHandle(state_handle6);
    joint_state_interface_.registerHandle(state_handle7);
    joint_state_interface_.registerHandle(state_handle8);
    joint_state_interface_.registerHandle(state_handle9);
    joint_state_interface_.registerHandle(state_handle10);
    joint_state_interface_.registerHandle(state_handle11);
    joint_state_interface_.registerHandle(state_handle12);
    joint_state_interface_.registerHandle(state_handle13);
    joint_state_interface_.registerHandle(state_handle14);
    joint_state_interface_.registerHandle(state_handle15);
    joint_state_interface_.registerHandle(state_handle16);
    joint_state_interface_.registerHandle(state_handle17);
    joint_state_interface_.registerHandle(state_handle18);
    joint_state_interface_.registerHandle(state_handle19);
    joint_state_interface_.registerHandle(state_handle20);
    joint_state_interface_.registerHandle(state_handle21);
    joint_state_interface_.registerHandle(state_handle22);

    registerInterface(&joint_state_interface_);

    // connect and register joint position interface
    // the motors accept position inputs
    // hardware_interface::JointHandle position_handle1(joint_state_interface_.getHandle(names_.at(0)), &cmd_.at(0));
    // joint_position_interface_.registerHandle(position_handle1);
    // hardware_interface::JointHandle position_handle2(joint_state_interface_.getHandle(names_.at(1)), &cmd_.at(1));
    // joint_position_interface_.registerHandle(position_handle2);
    // hardware_interface::JointHandle position_handle3(joint_state_interface_.getHandle(names_.at(2)), &cmd_.at(2));
    // joint_position_interface_.registerHandle(position_handle3);
    // hardware_interface::JointHandle position_handle4(joint_state_interface_.getHandle(names_.at(3)), &cmd_.at(3));
    // joint_position_interface_.registerHandle(position_handle4);

    hardware_interface::JointHandle position_handle1(joint_state_interface_.getHandle(names_.at(0)), &cmd_.at(0));
    hardware_interface::JointHandle position_handle2(joint_state_interface_.getHandle(names_.at(1)), &cmd_.at(1));
    hardware_interface::JointHandle position_handle3(joint_state_interface_.getHandle(names_.at(2)), &cmd_.at(2));
    hardware_interface::JointHandle position_handle4(joint_state_interface_.getHandle(names_.at(3)), &cmd_.at(3));
    hardware_interface::JointHandle position_handle5(joint_state_interface_.getHandle(names_.at(4)), &cmd_.at(4));
    hardware_interface::JointHandle position_handle6(joint_state_interface_.getHandle(names_.at(5)), &cmd_.at(5));
    hardware_interface::JointHandle position_handle7(joint_state_interface_.getHandle(names_.at(6)), &cmd_.at(6));
    hardware_interface::JointHandle position_handle8(joint_state_interface_.getHandle(names_.at(7)), &cmd_.at(7));
    hardware_interface::JointHandle position_handle9(joint_state_interface_.getHandle(names_.at(8)), &cmd_.at(8));
    hardware_interface::JointHandle position_handle10(joint_state_interface_.getHandle(names_.at(9)), &cmd_.at(9));
    hardware_interface::JointHandle position_handle11(joint_state_interface_.getHandle(names_.at(10)), &cmd_.at(10));
    hardware_interface::JointHandle position_handle12(joint_state_interface_.getHandle(names_.at(11)), &cmd_.at(11));
    hardware_interface::JointHandle position_handle13(joint_state_interface_.getHandle(names_.at(12)), &cmd_.at(12));
    hardware_interface::JointHandle position_handle14(joint_state_interface_.getHandle(names_.at(13)), &cmd_.at(13));
    hardware_interface::JointHandle position_handle15(joint_state_interface_.getHandle(names_.at(14)), &cmd_.at(14));
    hardware_interface::JointHandle position_handle16(joint_state_interface_.getHandle(names_.at(15)), &cmd_.at(15));
    hardware_interface::JointHandle position_handle17(joint_state_interface_.getHandle(names_.at(16)), &cmd_.at(16));
    hardware_interface::JointHandle position_handle18(joint_state_interface_.getHandle(names_.at(17)), &cmd_.at(17));
    hardware_interface::JointHandle position_handle19(joint_state_interface_.getHandle(names_.at(18)), &cmd_.at(18));
    hardware_interface::JointHandle position_handle20(joint_state_interface_.getHandle(names_.at(19)), &cmd_.at(19));
    hardware_interface::JointHandle position_handle21(joint_state_interface_.getHandle(names_.at(20)), &cmd_.at(20));
    hardware_interface::JointHandle position_handle22(joint_state_interface_.getHandle(names_.at(21)), &cmd_.at(21));
    
    joint_position_interface_.registerHandle(position_handle1);
    joint_position_interface_.registerHandle(position_handle2);
    joint_position_interface_.registerHandle(position_handle3);
    joint_position_interface_.registerHandle(position_handle4);
    joint_position_interface_.registerHandle(position_handle5);
    joint_position_interface_.registerHandle(position_handle6);
    joint_position_interface_.registerHandle(position_handle8);
    joint_position_interface_.registerHandle(position_handle9);
    joint_position_interface_.registerHandle(position_handle10);
    joint_position_interface_.registerHandle(position_handle11);
    joint_position_interface_.registerHandle(position_handle12);
    joint_position_interface_.registerHandle(position_handle13);
    joint_position_interface_.registerHandle(position_handle14);
    joint_position_interface_.registerHandle(position_handle15);
    joint_position_interface_.registerHandle(position_handle16);
    joint_position_interface_.registerHandle(position_handle17);
    joint_position_interface_.registerHandle(position_handle18);
    joint_position_interface_.registerHandle(position_handle19);
    joint_position_interface_.registerHandle(position_handle20);
    joint_position_interface_.registerHandle(position_handle21);
    joint_position_interface_.registerHandle(position_handle22);
    
    registerInterface(&joint_position_interface_);

    ROS_INFO("Interfaces registered.");


    ROS_INFO("Preparing the Controller Manager");

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    update_freq_ = ros::Duration(0.1);
    looper_ = nh_.createTimer(update_freq_, &zetaxbotInterface::update, this);
    
    ROS_INFO("Ready to execute the control loop");
}

void zetaxbotInterface::update(const ros::TimerEvent& e)
{
    // This function is called periodically in order to update the controller
    // manager about the progress in the execution of the goal of the hardware
    // ROS_INFO("Update Event");
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void zetaxbotInterface::read()
{
    // Reads the current status of the Hardware (Arduino)
    // Open Loop Control - no sensor available on the robot taht detects the effective
    // angle of totation of each joint. Suppose that the motors are always able to follow
    // the position command
    for (int i = 0; i < 22; i++)
    {
        pos_.at(i) = cmd_.at(i);
    }
    
    // pos_.at(0) = cmd_.at(0);
    // pos_.at(1) = cmd_.at(1);
    // pos_.at(2) = cmd_.at(2);
    // pos_.at(3) = cmd_.at(3);
}
  
void zetaxbotInterface::write(ros::Duration elapsed_time)
{    
    // Send the command to the Hardware (Arduino)
    // First converts the angle from the moveit/urdf convention 
    // to the Arduino convention and then publishes the converted angles
    zetaxbot_controller::AnglesConverter srv;
    srv.request.rshoulder = cmd_.at(0);
    srv.request.rshoulderRot = cmd_.at(1);
    srv.request.rbicep = cmd_.at(2);
    srv.request.relbow = cmd_.at(3);
    srv.request.rwrist  = cmd_.at(4);
    srv.request.rthumb = cmd_.at(5);
    srv.request.rpind  = cmd_.at(6);
    srv.request.rmaj = cmd_.at(7);
    srv.request.rarc = cmd_.at(8);
    srv.request.rring = cmd_.at(9);
    srv.request.lshoulder = cmd_.at(10);
    srv.request.lshoulderRot = cmd_.at(11);
    srv.request.lbicep = cmd_.at(12);
    srv.request.lelbow = cmd_.at(13);
    srv.request.lwrist  = cmd_.at(14);
    srv.request.lthumb = cmd_.at(15);
    srv.request.lpind  = cmd_.at(16);
    srv.request.lmaj = cmd_.at(17);
    srv.request.lring = cmd_.at(18);
    srv.request.larc = cmd_.at(19);
    srv.request.headPitch = cmd_.at(20);
    srv.request.headYaw = cmd_.at(21);
    // srv.request.base = cmd_.at(0);
    // srv.request.shoulder = cmd_.at(1);
    // srv.request.elbow = cmd_.at(2);
    // srv.request.gripper = cmd_.at(3);
    int a =hardware_srv_.waitForExistence();
    // if(a)ROS_INFO("success on waiting for service");
    // else ROS_INFO("failed on waiting for service");
    
    
    // Call the service and show the response of the service
    if (hardware_srv_.call(srv))
    {
        // compose the array message
        std::vector<unsigned int> angles_deg;
        angles_deg.push_back(srv.response.rshoulder);
        // ROS_INFO("srv.response.rshoulder:"+str(srv.response.rshoulder));
        // ROS_INFO("srv.response.rshoulder_converted:"+str(srv.response.rshoulder));
        angles_deg.push_back(srv.response.rshoulderRot);
        angles_deg.push_back(srv.response.rbicep);
        angles_deg.push_back(srv.response.relbow);
        angles_deg.push_back(srv.response.rwrist);
        angles_deg.push_back(srv.response.rthumb);
        angles_deg.push_back(srv.response.rpind);
        angles_deg.push_back(srv.response.rmaj);
        angles_deg.push_back(srv.response.rarc);
        angles_deg.push_back(srv.response.rring);
        angles_deg.push_back(srv.response.lshoulder);
        angles_deg.push_back(srv.response.lshoulderRot);
        angles_deg.push_back(srv.response.lbicep);
        angles_deg.push_back(srv.response.lelbow);
        angles_deg.push_back(srv.response.lwrist);
        angles_deg.push_back(srv.response.lthumb);
        angles_deg.push_back(srv.response.lpind);
        angles_deg.push_back(srv.response.lmaj);
        angles_deg.push_back(srv.response.lring);
        angles_deg.push_back(srv.response.larc);
        angles_deg.push_back(srv.response.headPitch);
        angles_deg.push_back(srv.response.headYaw);
        // angles_deg.push_back(srv.response.base);
        // angles_deg.push_back(srv.response.shoulder);
        // angles_deg.push_back(srv.response.elbow);
        // angles_deg.push_back(srv.response.gripper);

        std_msgs::UInt16MultiArray msg;

        // set up dimensions
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size = angles_deg.size();
        msg.layout.dim[0].stride = 1;

        // copy in the data
        msg.data.clear();
        msg.data.insert(msg.data.end(), angles_deg.begin(), angles_deg.end());

        // publish the array message to the defined topic
        hardware_pub_.publish(msg);
    }
    else
    {
        ROS_ERROR("Failed to call service radians_to_degrees");
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "zetaxbot_interface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    zetaxbotInterface robot(nh);

    // Keep ROS up and running
    spinner.spin();
    return 0;
}