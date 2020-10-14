//Import the header
#include "ros/ros.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_collision_object");
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Params read from launch: the name of the mesh and the position.
    std::string mesh_name;
    pnh.getParam("mesh_name", mesh_name);      
    if (!pnh.hasParam("mesh_name"))
    {
      ROS_WARN("You forgot to set mesh_name param with the .stl file that you want to publish");
    }
    
    int mesh_position_x;
    pnh.param("mesh_position_x", mesh_position_x, 1);

    int mesh_position_y;
    pnh.param("mesh_position_y", mesh_position_y, 1);      

    int mesh_position_z;
    pnh.param("mesh_position_z", mesh_position_z, 1);

    std::string object_id;
    pnh.getParam("object_id", object_id);      

       
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    sleep(2.0);
    
    //Vector to scale 3D file units (to convert from mm to meters for example)
    //Vector3d vectorScale(0.001, 0.001, 0.001);
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject co;

    // The id of the object is used to identify it.
    co.id = object_id;
    co.header.frame_id = "robot_base_footprint";

    std::string mesh_path = "package://component_sorting_description/meshes/box/" + mesh_name + ".stl";

    //Path where the .dae or .stl object is located
    //shapes::Mesh* m = shapes::createMeshFromResource("package://component_sorting_description/meshes/box/box_handle.stl", vectorScale); 
    shapes::Mesh* m = shapes::createMeshFromResource(mesh_path); 
    ROS_INFO("Your mesh was loaded");
    
    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;  
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
    
    //Define a pose for your mesh (specified relative to frame_id)
    geometry_msgs::Pose obj_pose;
    obj_pose.position.x = mesh_position_x;
    obj_pose.position.y = mesh_position_y;
    obj_pose.position.z = mesh_position_z;
    
    // Add the mesh to the Collision object message 
    co.meshes.push_back(mesh);
    co.mesh_poses.push_back(obj_pose);
    co.operation = co.ADD;
    
    
    //Publish object in monitored planning scene
    
    // Create vector of collision objects to add 
    std::vector<moveit_msgs::CollisionObject> object;
    object.push_back(co);
    
    // Add the collision object into the world
    planning_scene_interface.addCollisionObjects(object);
}