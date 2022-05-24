#ifndef _COMPONENT_SORTING_MANIPULATION__COMPONENT_SORTING_MANIPULATION_H_
#define _COMPONENT_SORTING_MANIPULATION__COMPONENT_SORTING_MANIPULATION_H_

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// CPP
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Byte.h>
#include <string>
#include <iostream>
#include <map>
#include <algorithm>
#include <vector>
#include <thread>

// UTILS
#include <rcomponent/rcomponent.h>
#include <gazebo_ros_link_attacher/Attach.h>

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <warehouse_ros/database_connection.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// CUSTOM
#include <component_sorting_msgs/PlaceObjectAction.h>
#include <component_sorting_msgs/PickObjectAction.h>

// SCENE MANAGER
#include "scene_manager/scene_manager.h"

#include <ur_msgs/SetIO.h>



class ManipulationApp : public rcomponent::RComponent
{
public:
  ManipulationApp(ros::NodeHandle h);
  ~ManipulationApp();

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
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  geometry_msgs::TransformStamped transform_stamped;

  // Scene Manager
  std::unique_ptr<SceneManager> scene_manager_;

  // MoveIt Init
  std::string group_name_;
  ros::WallDuration move_group_timeout_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  collision_detection::AllowedCollisionMatrix acm_;
  moveit_msgs::PlanningScene planning_scene_msg;


  // MoveIt database
  warehouse_ros::DatabaseConnection::Ptr conn_;
  std::string host;
  int port;
  float connection_timeout;
  int connection_retries;

  // Moveit cartesian
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose current_cartesian_pose;
  geometry_msgs::Pose waypoint_cartesian_pose;
  moveit_msgs::RobotTrajectory trajectory;
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  // Moveit Check
  double success_cartesian_plan;
  double allowed_fraction_success = 0.80;
  bool success_plan;
  bool success_move;
  bool success_execute;

  // Moveit Config params
  double scale_vel_;
  double scale_acc_;
  std::string end_effector_link_, robot_base_link_;

  std::string moveit_constraint;
  moveit_msgs::Constraints current_constraint;

  // Action servers
  virtual void goalCB(const std::string& action);
  virtual void preemptCB();
  std::string action_;

  actionlib::SimpleActionServer<component_sorting_msgs::PlaceObjectAction>::GoalConstPtr place_object_goal_;
  actionlib::SimpleActionServer<component_sorting_msgs::PickObjectAction>::GoalConstPtr pick_object_goal_;

  std::shared_ptr<actionlib::SimpleActionServer<component_sorting_msgs::PlaceObjectAction>> place_object_as_;
  std::shared_ptr<actionlib::SimpleActionServer<component_sorting_msgs::PickObjectAction>> pick_object_as_;
  
  // Action msgs
  component_sorting_msgs::PlaceObjectFeedback place_object_feedback_;
  component_sorting_msgs::PlaceObjectResult place_object_result_;
  component_sorting_msgs::PickObjectFeedback pick_object_feedback_;
  component_sorting_msgs::PickObjectResult pick_object_result_;

  // Action functions
  void placeObject(const std::string& object, const std::string& surface);
  void pickObject(const std::string& object);

  // Services
  ros::ServiceServer emergency_stop_trigger_;
  bool emergency_stop_cb(std_srvs::Trigger::Request &req, std_srvs::TriggerResponse &res);

  // Service Clients
  ros::ServiceClient gazebo_link_attacher_client;
  ros::ServiceClient gazebo_link_detacher_client;
  gazebo_ros_link_attacher::Attach gazebo_link_attacher_msg;

  // Threads
  boost::thread place_object_thread_;
  boost::thread pick_object_thread_;
  bool thread_active_flag_;
  bool action_finished_flag_;

};

#endif  // _COMPONENT_SORTING_MANIPULATION__COMPONENT_SORTING_MANIPULATION_H_
