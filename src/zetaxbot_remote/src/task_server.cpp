/*
  zetaxbot - task_server

  This script implements an Action Server that manages the execution
  of goals of the robot interfacing with moveit.
  Given a goal, it sends and execute a moveit trajectory

  Copyright (c) 2021 Antonio Brandi.  All right reserved.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <zetaxbot_remote/ZetaxbotTaskAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>


class TaskServer
{
private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<zetaxbot_remote::ZetaxbotTaskAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to publish result
  zetaxbot_remote::ZetaxbotTaskResult result_;
  std::vector<double> right_arm_goal_;
  std::vector<double> head_goal_;
  std::vector<double> gripper_goal_;
  moveit::planning_interface::MoveGroupInterface right_arm_move_group_;
  moveit::planning_interface::MoveGroupInterface gripper_move_group_;

public:

  // Constructor
  // function that inizialize the ZetaxbotTaskAction class and creates 
  // a Simple Action Server from the library actionlib
  TaskServer(std::string name) :
    as_(nh_, name, boost::bind(&TaskServer::execute_cb, this, _1), false)
    , action_name_(name)
    , right_arm_move_group_("right_arm")
    , gripper_move_group_("head")
  {
    as_.start();
  }

  void execute_cb(const zetaxbot_remote::ZetaxbotTaskGoalConstPtr &goal)
  {
    bool success = true;

    // start executing the action
    // based on the goal id received, send a different goal 
    // to the robot
    if (goal->task_number == 0)
    {
      right_arm_goal_ = {0.0, 0.0, 0.0 , 0.0, 0.0 };
      gripper_goal_ = {-0.7, 0.7};
    }
    else if (goal->task_number == 1)
    {
      right_arm_goal_ = {-1.14, -0.6, -0.07 , 1.0 , 1.0};
      gripper_goal_ = {0.5, 0.6};
    }
    else if (goal->task_number == 2)
    {
      right_arm_goal_ = {1.1,0.0,1.0,0.5,0.7};
      gripper_goal_ = {0.2, 0.7};
    }
    else
    {
      ROS_ERROR("Invalid goal");
      return;
    }

    // Sends a goal to the moveit API
    right_arm_move_group_.setJointValueTarget(right_arm_goal_);
    gripper_move_group_.setJointValueTarget(gripper_goal_);

    // blocking functions below, will return after the execution
    right_arm_move_group_.move();
    gripper_move_group_.move();

    // Make sure that no residual movement remains
    right_arm_move_group_.stop();
    gripper_move_group_.stop();

    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      success = false;
    }

    // check if the goal request has been executed correctly
    if(success)
    {
      result_.success = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  // Inizialize a ROS node called task_server
  ros::init(argc, argv, "task_server");
  TaskServer server("task_server");

  // keeps the node up and running
  ros::spin();
  return 0;
}