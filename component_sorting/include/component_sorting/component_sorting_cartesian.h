#ifndef _COMPONENT_SORTING__COMPONENT_SORTING_H_
#define _COMPONENT_SORTING__COMPONENT_SORTING_H_

#include <ros/ros.h>

#include <rcomponent/rcomponent.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <geometric_shapes/shape_operations.h>

#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>

#include <component_sorting_msgs/PickupFromAction.h>
#include <component_sorting_msgs/PlaceOnAction.h>

#include <warehouse_ros/database_connection.h>
#include <moveit/warehouse/constraints_storage.h>

#include <moveit_msgs/Constraints.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

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
  void pick_chain_movement(geometry_msgs::Pose pre_position, geometry_msgs::Pose position, std::string box_id);
  void place_chain_movement(geometry_msgs::Pose pre_position, geometry_msgs::Pose position);
  void create_planning_scene();

  std::string group_name_;
  ros::WallDuration move_group_timeout_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::CollisionObject co_1;
  moveit_msgs::CollisionObject co_2;
  moveit_msgs::CollisionObject co_3;
  moveit_msgs::CollisionObject co_4;
  moveit_msgs::CollisionObject co_5;
  moveit_msgs::CollisionObject co_6;

  std::string action_;
  actionlib::SimpleActionServer<component_sorting_msgs::PickupFromAction>::GoalConstPtr pickup_from_goal_;
  actionlib::SimpleActionServer<component_sorting_msgs::PlaceOnAction>::GoalConstPtr place_on_goal_;
  std::shared_ptr<actionlib::SimpleActionServer<component_sorting_msgs::PickupFromAction>> pickup_from_as_;
  std::shared_ptr<actionlib::SimpleActionServer<component_sorting_msgs::PlaceOnAction>> place_on_as_;

  component_sorting_msgs::PlaceOnFeedback place_feedback_;
  component_sorting_msgs::PlaceOnResult place_result_;
  component_sorting_msgs::PickupFromFeedback pick_feedback_;
  component_sorting_msgs::PickupFromResult pick_result_;

  warehouse_ros::DatabaseConnection::Ptr conn_;

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose current_cartesian_pose;
  geometry_msgs::Pose waypoint_cartesian_pose;
  moveit_msgs::RobotTrajectory trajectory;
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  const double jump_threshold = 0.0;
  const double eef_step = 0.001;

  bool success_plan;
  bool success_move;
  bool success_execute;
  
  std::string host;
  int port;
  float connection_timeout;
  int connection_retries;

  //Poses kairos
  geometry_msgs::Pose pre_kairos_center_pose;
  geometry_msgs::Point pre_kairos_center_position;
  geometry_msgs::Quaternion pre_kairos_center_orientation;

  geometry_msgs::Pose pre_kairos_right_pose;

  geometry_msgs::Pose pre_kairos_left_pose;

  geometry_msgs::Pose kairos_right_pose;

  geometry_msgs::Pose kairos_center_pose;

  geometry_msgs::Pose kairos_left_pose;

  
  //Poses table
  geometry_msgs::Pose pre_table_center_pose;
  geometry_msgs::Point pre_table_center_position;
  geometry_msgs::Quaternion pre_table_center_orientation;

  geometry_msgs::Pose pre_table_right_pose;

  geometry_msgs::Pose pre_table_left_pose;

  geometry_msgs::Pose table_right_pose;

  geometry_msgs::Pose table_center_pose;

  geometry_msgs::Pose table_left_pose;


  // in case we contact MoveIt through actionlib
  // std::shared_ptr<actionlib::SimpleActionServer<moveit_msgs::PickupAction>> pickup_as_;
  // std::shared_ptr<actionlib::SimpleActionServer<moveit_msgs::PlaceAction>> place_as_;

  // std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction>> pickup_ac_;
  // std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction>> place_ac_;

  virtual void goalCB(const std::string& action);
  virtual void preemptCB();
};

#endif  // _COMPONENT_SORTING__COMPONENT_SORTING_H_
