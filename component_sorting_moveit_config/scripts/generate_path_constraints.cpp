#include <ros/ros.h>

#include <warehouse_ros/database_connection.h>
#include <moveit/warehouse/constraints_storage.h>

#include <moveit_msgs/Constraints.h>
constexpr char LOGNAME[] = "generate_path_constraints_database";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "generate_path_constraints_database", ros::init_options::AnonymousName);

  std::string host = "localhost";
  int port = 33829;
  std::unique_ptr<moveit_warehouse::ConstraintsStorage> constraints_storage_;
  // Set up db
  try
  {
    warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase();
    conn->setParams(host, port);
    if (conn->connect())
    {
      constraints_storage_.reset(new moveit_warehouse::ConstraintsStorage(conn));
    }
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED(LOGNAME, "%s", ex.what());
  }

  std::string robot = "component_sorting";
  std::string group = "arm"; 
  std::vector<std::string> names;
  constraints_storage_->getKnownConstraints(names, robot, group);
  ROS_INFO_STREAM("Has1 " << names.size());
  for (auto n : names)
    ROS_INFO_STREAM(n);

  
    moveit_msgs::Constraints constraints;
    //constraints.header.frame_id = "robot_base_link";
    moveit_msgs::OrientationConstraint orientation;
    orientation.header.frame_id = "robot_base_link";
    orientation.link_name = "robot_arm_tool0";
    orientation.orientation.w = 1;
    orientation.absolute_x_axis_tolerance = 0.1;
    orientation.absolute_y_axis_tolerance = 0.1;
    orientation.absolute_z_axis_tolerance = 3.15;
    orientation.weight = 100;
    constraints.name = "upright";
    constraints.orientation_constraints.clear();
    constraints.orientation_constraints.push_back(orientation);
    constraints_storage_->addConstraints(constraints, robot, group);
  
  
 //   moveit_msgs::Constraints constraints;
    constraints = moveit_msgs::Constraints();
    //constraints.header.frame_id = "robot_base_link";
   // moveit_msgs::OrientationConstraint orientation;
    orientation.header.frame_id = "robot_base_link";
    orientation.link_name = "robot_arm_tool0";
    orientation.orientation.x = 1;
    orientation.orientation.w = 0;
    orientation.absolute_x_axis_tolerance = 0.1;
    orientation.absolute_y_axis_tolerance = 0.1;
    orientation.absolute_z_axis_tolerance = 3.15;
    orientation.weight = 100;
    constraints.name = "downright";
    constraints.orientation_constraints.clear();
    constraints.orientation_constraints.push_back(orientation);
    constraints_storage_->addConstraints(constraints, robot, group);
    ROS_INFO("Adding");
    ROS_INFO_STREAM(constraints);
 //   i
 //   moveit_msgs::Constraints constraints;
    constraints = moveit_msgs::Constraints();
    //constraints.header.frame_id = "robot_base_link";
   // moveit_msgs::OrientationConstraint orientation;
    orientation.header.frame_id = "robot_base_link";
    orientation.link_name = "robot_arm_tool0";
    orientation.orientation.x = 1;
    orientation.orientation.w = 0;
    orientation.absolute_x_axis_tolerance = 0.3;
    orientation.absolute_y_axis_tolerance = 0.3;
    orientation.absolute_z_axis_tolerance = 3.15;
    orientation.weight = 100;
    constraints.name = "soft_downright";
    constraints.orientation_constraints.clear();
    constraints.orientation_constraints.push_back(orientation);
    constraints_storage_->addConstraints(constraints, robot, group);
    ROS_INFO("Adding");
    ROS_INFO_STREAM(constraints);

  //   moveit_msgs::Constraints constraints;
    constraints = moveit_msgs::Constraints();
    moveit_msgs::JointConstraint joint;
    //constraints.header.frame_id = "robot_base_link";
   // moveit_msgs::OrientationConstraint orientation;
    orientation.header.frame_id = "robot_base_link";
    orientation.link_name = "robot_arm_tool0";
/*     orientation.orientation.x=  0;
    orientation.orientation.y = 0;
    orientation.orientation.z = 0; */
    orientation.orientation.x = 1;
    orientation.orientation.w = 0;
    orientation.absolute_x_axis_tolerance = 0.5;
    orientation.absolute_y_axis_tolerance = 0.5;
    orientation.absolute_z_axis_tolerance = 6.28318531;
    orientation.weight = 100;
    constraints.name = "elbow_up";
    constraints.orientation_constraints.clear();
    constraints.orientation_constraints.push_back(orientation);
    constraints.joint_constraints.clear();
    joint.joint_name = "robot_arm_elbow_joint";
    joint.position = 0;
    joint.tolerance_below = 0.20;
    joint.tolerance_above = 3.1415;
    joint.weight = 100;
    constraints.joint_constraints.push_back(joint);
    joint.joint_name = "robot_arm_shoulder_lift_joint";
    joint.position = -1.6057;
    joint.tolerance_below = 0.6981; //0.78
    joint.tolerance_above = 0.6981; //0.78
    joint.weight = 100;
    constraints.joint_constraints.push_back(joint);
    constraints_storage_->addConstraints(constraints, robot, group);
    
    



  //constraints_storage_->getKnownConstraints(names);
  constraints_storage_->getKnownConstraints(names, robot, group);
  ROS_INFO_STREAM("Has 221" << names.size());
  for (auto n : names)
    ROS_INFO_STREAM(n);
  //    initializing_constraints_ = false;
  return 0;
}
