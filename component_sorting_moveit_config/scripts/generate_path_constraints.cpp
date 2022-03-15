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
  ROS_INFO_STREAM("Number of stored constraints: " << names.size());

  for (auto n : names)
    ROS_INFO_STREAM(n);

  
  moveit_msgs::Constraints constraints;
  constraints.name = "upright";
  moveit_msgs::OrientationConstraint orientation;
  orientation.header.frame_id = "robot_base_link";
  orientation.link_name = "robot_arm_tool0";
  orientation.orientation.w = 1;
  orientation.absolute_x_axis_tolerance = 0.1;
  orientation.absolute_y_axis_tolerance = 0.1;
  orientation.absolute_z_axis_tolerance = 3.15;
  orientation.weight = 100;
  constraints.orientation_constraints.push_back(orientation);
  constraints_storage_->addConstraints(constraints, robot, group);


  constraints = moveit_msgs::Constraints();
  constraints.name = "downright";
  orientation = moveit_msgs::OrientationConstraint();
  orientation.header.frame_id = "robot_base_link";
  orientation.link_name = "robot_arm_tool0";
  orientation.orientation.x = 1;
  orientation.orientation.w = 0;
  orientation.absolute_x_axis_tolerance = 0.1;
  orientation.absolute_y_axis_tolerance = 0.1;
  orientation.absolute_z_axis_tolerance = 3.15;
  orientation.weight = 100;
  constraints.orientation_constraints.push_back(orientation);
  constraints_storage_->addConstraints(constraints, robot, group);


  constraints = moveit_msgs::Constraints();
  constraints.name = "soft_downright";
  orientation = moveit_msgs::OrientationConstraint();
  orientation.header.frame_id = "robot_base_link";
  orientation.link_name = "robot_arm_tool0";
  orientation.orientation.x = 1;
  orientation.orientation.w = 0;
  orientation.absolute_x_axis_tolerance = 0.3;
  orientation.absolute_y_axis_tolerance = 0.3;
  orientation.absolute_z_axis_tolerance = 3.15;
  orientation.weight = 100;
  constraints.orientation_constraints.push_back(orientation);
  constraints_storage_->addConstraints(constraints, robot, group);


  constraints = moveit_msgs::Constraints();
  constraints.name = "move_parallel";
  orientation = moveit_msgs::OrientationConstraint();
  orientation.header.frame_id = "robot_base_link";
  orientation.link_name = "robot_arm_tool0";
  orientation.orientation.x = 1;
  orientation.orientation.w = 0;
  orientation.absolute_x_axis_tolerance = 0.4;
  orientation.absolute_y_axis_tolerance = 0.4;
  orientation.absolute_z_axis_tolerance = 6.28318531;
  orientation.weight = 100;
  constraints.orientation_constraints.push_back(orientation);
  moveit_msgs::JointConstraint joint;
  joint.joint_name = "robot_arm_elbow_joint";
  joint.position = 0;
  joint.tolerance_below = 0.20;
  joint.tolerance_above = 3.1415;
  joint.weight = 100;
//  constraints.joint_constraints.push_back(joint);
  joint.joint_name = "robot_arm_shoulder_lift_joint";
  joint.position = -1.6057;
  joint.tolerance_below = 0.6981; //0.78
  joint.tolerance_above = 0.6981; //0.78
  joint.weight = 100;
//  constraints.joint_constraints.push_back(joint);
  constraints_storage_->addConstraints(constraints, robot, group);


  constraints = moveit_msgs::Constraints();
  constraints.name = "elbow_up";
  orientation = moveit_msgs::OrientationConstraint();
  orientation.header.frame_id = "robot_base_link";
  orientation.link_name = "robot_arm_tool0";
  orientation.orientation.x = 1;
  orientation.orientation.w = 0;
  orientation.absolute_x_axis_tolerance = 1.57;
  orientation.absolute_y_axis_tolerance = 1.57;
  orientation.absolute_z_axis_tolerance = 6.28318531;
  orientation.weight = 100;
  constraints.orientation_constraints.push_back(orientation);
  joint = moveit_msgs::JointConstraint();
  joint.joint_name = "robot_arm_elbow_joint";
  joint.position = 0;
  joint.tolerance_below = 0.20;
  joint.tolerance_above = 3.1415;
  joint.weight = 100;
  constraints.joint_constraints.push_back(joint);
  joint.joint_name = "robot_arm_shoulder_lift_joint";
  joint.position = -1.6057;
  joint.tolerance_below = 1.22; //0.78
  joint.tolerance_above = 0.7; //0.78
  joint.weight = 100;
  constraints.joint_constraints.push_back(joint);
  constraints_storage_->addConstraints(constraints, robot, group);
    
    
  constraints_storage_->getKnownConstraints(names, robot, group);
  ROS_INFO_STREAM("Updated number of constraints: " << names.size());
  for (auto n : names)
    ROS_INFO_STREAM(n);
  return 0;
}
