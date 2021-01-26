#ifndef _COMPONENT_SORTING__OBJECT_H_
#define _COMPONENT_SORTING__OBJECT_H_

#include <ros/ros.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit_msgs/Constraints.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>

class Object
{
  public:
    
    Object(ros::NodeHandle pnh, std::string id);

    virtual ~Object();

    std::string get_id();

    std::string get_frame_id();

    bool has_mesh();

    bool has_primitive();

    geometry_msgs::Pose get_pose();

    shape_msgs::Mesh get_mesh();

    shape_msgs::SolidPrimitive get_primitive();




  private:

    // Variables which hold config parameter yaml info

    ros::NodeHandle pnh_; // object paramenter node handle
    std::string id_; // Object id
    std::string frame_id_; // Frame used for object relative positioning
    XmlRpc::XmlRpcValue geometry_; // Stores object geometry parameter
    XmlRpc::XmlRpcValue pose_; // Stores object pose parameter
    
    // Variables created in Object class after processing parameter info 

    geometry_msgs::Pose pose_msg; // MoveIt Object Pose message
    shape_msgs::Mesh mesh; // Will be filled in only for mesh objects
    shape_msgs::SolidPrimitive primitive; // Will be filled in only for primitive shape objects
    
    bool check_mesh = false; // Checks if object is defined using mesh
    bool check_primitive = false; // Checks if object is defined using primitive shape
      
      
};

#endif // _COMPONENT_SORTING__OBJECT_H_