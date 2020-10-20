#ifndef _COMPONENT_SORTING__COMPONENT_SORTING_H_
#define _COMPONENT_SORTING__COMPONENT_SORTING_H_

#include <ros/ros.h>

#include <rcomponent/rcomponent.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>

#include <component_sorting_msgs/PickupFromAction.h>
#include <component_sorting_msgs/PlaceOnAction.h>

#include <warehouse_ros/database_connection.h>
#include <moveit/warehouse/constraints_storage.h>

#include <moveit_msgs/Constraints.h>

#include <string>

class ComponentSorting : public rcomponent::RComponent
{
public:
  ComponentSorting(ros::NodeHandle h);
  ~ComponentSorting();

protected:
  // RComponent stuff

  //! Setups all the ROS' stuff
  virtual int rosSetup();
  //! Shutdowns all the ROS' stuff
  virtual int rosShutdown();
  //! Reads params from params server
  virtual void rosReadParams();

  virtual int setup();

  // States
  virtual void standbyState();
  virtual void readyState();

  // ROS stuff
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

  // MoveIt stuff
  void chain_movement(std::string pre_position, std::string position);

  std::string group_name_;
  ros::WallDuration move_group_timeout_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan;



  std::string action_;
  actionlib::SimpleActionServer<component_sorting_msgs::PickupFromAction>::GoalConstPtr pickup_from_goal_;
  std::shared_ptr<actionlib::SimpleActionServer<component_sorting_msgs::PickupFromAction>> pickup_from_as_;
  std::shared_ptr<actionlib::SimpleActionServer<component_sorting_msgs::PlaceOnAction>> place_on_as_;

  component_sorting_msgs::PickupFromFeedback feedback_;
  component_sorting_msgs::PickupFromResult result_;

  warehouse_ros::DatabaseConnection::Ptr conn_;

  bool success_plan;
  bool success_move;
  bool success_execute;
  
  std::string host;
  int port;
  float connection_timeout;
  int connection_retries;
  

  // in case we contact MoveIt through actionlib
  // std::shared_ptr<actionlib::SimpleActionServer<moveit_msgs::PickupAction>> pickup_as_;
  // std::shared_ptr<actionlib::SimpleActionServer<moveit_msgs::PlaceAction>> place_as_;

  // std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction>> pickup_ac_;
  // std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction>> place_ac_;

  virtual void goalCB(const std::string& action);
  virtual void preemptCB();
};

#endif  // _COMPONENT_SORTING__COMPONENT_SORTING_H_
