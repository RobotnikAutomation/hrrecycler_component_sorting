#ifndef _COMPONENT_SORTING__COMPONENT_SORTING_H_
#define _COMPONENT_SORTING__COMPONENT_SORTING_H_

#include <ros/ros.h>

#include <rcomponent/rcomponent.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>

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
  std::string group_name_;
  ros::WallDuration move_group_timeout_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;

  std::shared_ptr<actionlib::SimpleActionServer<moveit_msgs::PickupAction>> pickup_as_;
  std::shared_ptr<actionlib::SimpleActionServer<moveit_msgs::PlaceAction>> place_as_;

  std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction>> pickup_ac_;
  std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction>> place_ac_;

  virtual void goalCB();
  virtual void preemptCB();
};

#endif  // _COMPONENT_SORTING__COMPONENT_SORTING_H_
