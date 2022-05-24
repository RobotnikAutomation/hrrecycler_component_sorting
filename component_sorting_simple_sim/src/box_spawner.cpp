#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetModelState.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "box_spawner_node"); // Node setup
    ros::NodeHandle nh("box_spawner_node");

    std::string box_description_;

    if (nh.getParam("/box_description", box_description_))
    {
      ROS_INFO_STREAM("Retrieved box description from parameter server.");
    }else{
      ROS_INFO_STREAM("Could not find box description on server. Aborting.");  
      return 0;
    }

    double box_width_, box_height_, box_length_, table_base_height_, table_base_width_, table_base_length_;
    int table_floors_, table_x_, table_y_; 
    std::string table_name_;
  

    nh.getParam("box/geometry/box/width", box_width_);
    nh.getParam("box/geometry/box/height", box_height_);
    nh.getParam("box/geometry/box/length", box_length_);
    nh.getParam("table/geometry/box/height", table_base_height_);
    nh.getParam("table/geometry/box/width", table_base_width_);
    nh.getParam("table/geometry/box/length", table_base_length_);
    nh.getParam("box/frame_id", table_name_);
    nh.getParam("box/layout/z", table_floors_);
    nh.getParam("box/layout/x", table_x_);
    nh.getParam("box/layout/y", table_y_);

    
    table_base_width_ = box_width_*table_y_;
    table_base_length_ = box_length_*table_x_;

    ros::service::waitForService("/gazebo/spawn_urdf_model");
    ros::ServiceClient spawn = nh.serviceClient< gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");

    ros::service::waitForService("/gazebo/get_model_state");
    ros::ServiceClient model_state = nh.serviceClient< gazebo_msgs::GetModelState> ("/gazebo/get_model_state");

    // Check if model exists
    bool table_spawned = false;
    gazebo_msgs::GetModelState table_status;
    table_status.request.model_name = table_name_;

    while (table_spawned == false){
     model_state.call(table_status);
     table_spawned = table_status.response.success;
    }

    // Boxes common info
    gazebo_msgs::SpawnModel box;
    box.request.model_xml = box_description_;
    box.request.robot_namespace = "";
    geometry_msgs::Pose pose_;
    pose_.orientation = tf::createQuaternionMsgFromYaw(0.0);
    box.request.reference_frame = table_name_ + "_base_link";

    // table configuration
    int boxes_floor_ = table_x_*table_y_;
    int id = 1;

    for(int z = 0; z <= (table_floors_-1)*boxes_floor_; z+=boxes_floor_){

      pose_.position.z = table_base_height_ + box_height_*z/boxes_floor_;

      for(int y = 0; y<table_y_; y++){
        for(int x = 0; x<table_x_; x++){
        box.request.model_name = "box_" + std::to_string(id);
        pose_.position.x = -table_base_width_/2 + box_width_/2 + x*box_width_;
        pose_.position.y =  -table_base_length_/2 + box_length_/2 + y*box_length_;
        box.request.initial_pose = pose_;
        spawn.call(box);
        id += 1;
        }
      }
    }  


    ros::shutdown();
}