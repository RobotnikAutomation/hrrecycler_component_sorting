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
#include <iostream>

#include <ur_msgs/SetIO.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

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
  void pick_chain_movement(geometry_msgs::PoseStamped pre_position, geometry_msgs::PoseStamped position);
  void place_chain_movement(geometry_msgs::PoseStamped pre_position, geometry_msgs::PoseStamped position);
  void create_planning_scene();
  void gripper_on();
  void gripper_off();

  std::string group_name_;
  ros::WallDuration move_group_timeout_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector< std::string > objects;
  
  moveit_msgs::CollisionObject co_1;
  moveit_msgs::CollisionObject co_2;
  moveit_msgs::CollisionObject co_3;
  moveit_msgs::CollisionObject co_4;
  moveit_msgs::CollisionObject co_5;
  moveit_msgs::CollisionObject co_6;
  moveit_msgs::CollisionObject co_7;
  moveit_msgs::CollisionObject co_8;
  moveit_msgs::CollisionObject co_9;
  moveit_msgs::CollisionObject co_10;
  moveit_msgs::CollisionObject object_move;

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
  const double eef_step = 0.01;

  double success_cartesian_plan;
  double allowed_fraction_success = 0.95;
  bool success_plan;
  bool success_move;
  bool success_execute;

  const moveit::core::JointModelGroup* joint_model_group;
  robot_state::RobotStatePtr robot_state_;
 // moveit_visual_tools::MoveItVisualTools visual_tools_("robot_base_footprint");
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  
  std::string host;
  int port;
  float connection_timeout;
  int connection_retries;

  //Poses kairos
  geometry_msgs::PoseStamped pre_kairos_center_pose;
  geometry_msgs::Point pre_kairos_center_position;
  geometry_msgs::Quaternion pre_kairos_center_orientation;
  std_msgs::Header kairos_frame;


  geometry_msgs::PoseStamped pre_kairos_right_pose;

  geometry_msgs::PoseStamped pre_kairos_left_pose;

  geometry_msgs::PoseStamped kairos_right_pose;

  geometry_msgs::PoseStamped kairos_center_pose;

  geometry_msgs::PoseStamped kairos_left_pose;

  
  //Poses table
  geometry_msgs::PoseStamped pre_table_center_pose;
  geometry_msgs::Point pre_table_center_position;
  geometry_msgs::Quaternion pre_table_center_orientation;
  std_msgs::Header table_qr_frame;

  geometry_msgs::PoseStamped pre_table_right_pose;

  geometry_msgs::PoseStamped pre_table_left_pose;

  geometry_msgs::PoseStamped table_right_pose;

  geometry_msgs::PoseStamped table_center_pose;

  geometry_msgs::PoseStamped table_left_pose;

  geometry_msgs::PoseStamped box_grab_pose;

  ur_msgs::SetIO srv;
  gazebo_ros_link_attacher::Attach gazebo_link_attacher_msg;
  ros::ServiceClient client;
  ros::ServiceClient gazebo_link_attacher_client;
  ros::ServiceClient gazebo_link_detacher_client;

  //double dock_dist_table = 0.115;
  double table_length = 0.63;
  double table_width = 0.63;
  double table_height = 0.75;
  double holder_width = 0.228;
  double holder_length = 0.30; 
  double qr_height = 0.375;
  double box_width = 0.20;
  double box_length = 0.28;

  object_recognition_msgs::ObjectType  allowed_movement;
  std::vector< std::string > types = {"allowed_movement"};

  std::string identified_box;
  std::string identified_handle;


  // in case we contact MoveIt through actionlib
  // std::shared_ptr<actionlib::SimpleActionServer<moveit_msgs::PickupAction>> pickup_as_;
  // std::shared_ptr<actionlib::SimpleActionServer<moveit_msgs::PlaceAction>> place_as_;

  // std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction>> pickup_ac_;
  // std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction>> place_ac_;

  virtual void goalCB(const std::string& action);
  virtual void preemptCB();

  void tfLatchTimerCallback();
  std::vector <tf::StampedTransform> latched_tf;
  ros::Timer tf_latch_timer;
   tf::TransformBroadcaster tf_broadcaster;

  void tfListenerTimerCallback(std::string frame_name);
   tf::TransformListener tf_listener;
  tf::StampedTransform transform;
};

#endif  // _COMPONENT_SORTING__COMPONENT_SORTING_H_
